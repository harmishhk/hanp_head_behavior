/*/
 * Copyright (c) 2015 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *                                  Harmish Khambhaita on Sat Sep 12 2015
 */

// defining constants
#define NODE_NAME "hanp_head_behavior"

#define LOCAL_PLAN_SUB_TOPIC "local_plan"
#define HUMAN_SUB_TOPIC "humans"
#define POINT_HEAD_PUB_TOPIC "point_head"
#define ROBOT_BASE_FRAME "base_link"
#define HEAD_PAN_FRAME "head_pan_link"
#define PUBLISH_RATE 10
#define VISIBILITY_ANGLE 0.087 //radian (5 degrees)

#include <signal.h>

#include <hanp_head_behavior/hanp_head_behavior.h>

namespace hanp_head_behavior
{
    // empty constructor and destructor
    HANPHeadBehavior::HANPHeadBehavior() {}
    HANPHeadBehavior::~HANPHeadBehavior() {}

    PathCostFunc::PathCostFunc() {}
    HumanCostFunc::HumanCostFunc() {}

    void HANPHeadBehavior::initialize()
    {
        // get private node handle
        ros::NodeHandle private_nh("~/");

        // get parameters
        private_nh.param("local_plan_sub_topic", local_plan_sub_topic_, std::string(LOCAL_PLAN_SUB_TOPIC));
        private_nh.param("human_sub_topic", human_sub_topic_, std::string(HUMAN_SUB_TOPIC));
        private_nh.param("point_head_pub_topic", point_head_pub_topic_, std::string(POINT_HEAD_PUB_TOPIC));

        private_nh.param("robot_base_frame", robot_base_frame_, std::string(ROBOT_BASE_FRAME));
        private_nh.param("head_pan_frame", head_pan_frame_, std::string(HEAD_PAN_FRAME));

        private_nh.param("publish_rate", publish_rate_, PUBLISH_RATE);

        // initialize subscribers and publishers
        local_plan_sub_ = private_nh.subscribe(local_plan_sub_topic_, 1, &HANPHeadBehavior::localPlanCB, this);
        humans_sub_ = private_nh.subscribe(human_sub_topic_, 1, &HANPHeadBehavior::trackedHumansCB, this);
        point_head_pub_ = private_nh.advertise<geometry_msgs::PointStamped>(point_head_pub_topic_, 1);

        // initialize publish timer
        // create a publish timer
        if(publish_rate_ > 0.0)
        {
            publish_timer_ = private_nh.createTimer(ros::Duration(1.0 / publish_rate_),
                &HANPHeadBehavior::publishPointHead, this);
            ROS_INFO_NAMED(NODE_NAME, "publishing: %s at %d hz",
                point_head_pub_topic_.c_str(), publish_rate_);
        }
        else
        {
            ROS_ERROR_NAMED(NODE_NAME, "%s: publish rate cannot be < 0,"
                " nothing will be published", std::string(NODE_NAME).c_str());
        }

        // initialize cost functions
        path_cost_func_ = new hanp_head_behavior::PathCostFunc();
        cost_functions_.push_back(path_cost_func_);
        human_cost_func_ = new hanp_head_behavior::HumanCostFunc();
        cost_functions_.push_back(human_cost_func_);

        // set-up dynamic reconfigure
        dsrv_ = new dynamic_reconfigure::Server<HANPHeadBehaviorConfig>(private_nh);
        dynamic_reconfigure::Server<HANPHeadBehaviorConfig>::CallbackType cb = boost::bind(&HANPHeadBehavior::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void HANPHeadBehavior::reconfigureCB(HANPHeadBehaviorConfig &config, uint32_t level)
    {
        robot_base_frame_ = config.robot_base_frame;
        head_pan_frame_ = config.head_pan_frame;

        point_head_height_ = config.point_head_height;
        ttc_collision_radius_ = config.ttc_collision_dist/2;
        visibility_angle_ = config.visibility_angle;

        publish_rate_ = config.publish_rate;

        path_cost_func_->cost = config.path_func_cost;
        human_cost_func_->cost = config.human_func_cost;
    }

    void HANPHeadBehavior::localPlanCB(const nav_msgs::Path& local_plan)
    {
        geometry_msgs::PointStamped point_head;
        point_head.header.stamp = ros::Time::now();

        // look at the end of the local plan
        if(local_plan.poses.size() > 0)
        {
            tf::Pose local_plan_end;
            tf::poseMsgToTF(local_plan.poses.back().pose, local_plan_end);
            auto local_plan_end_extended = local_plan_end(tf::Vector3(0.5,0,0)); //TODO: make this configurable
            point_head.header.frame_id = local_plan.poses.back().header.frame_id;
            point_head.point.x = local_plan_end_extended.x();
            point_head.point.y = local_plan_end_extended.y();
            point_head.point.z = point_head_height_;

            ROS_DEBUG_NAMED(NODE_NAME, "head pointing to path");
        }
        // look in the front
        else
        {
            point_head.header.frame_id = robot_base_frame_;
            point_head.point.x = 1.0;
            point_head.point.y = 0.0;
            point_head.point.z = point_head_height_;

            ROS_DEBUG_NAMED(NODE_NAME, "head pointing in the front");
        }

        path_cost_func_->cost = 0.9;
        path_cost_func_->point = point_head;
    }

    void HANPHeadBehavior::trackedHumansCB(const hanp_msgs::TrackedHumans& tracked_humans)
    {
        bool transform_found = false;
        tf::StampedTransform head_pan_to_humans_transform;

        // check if we are already looking at human
        if(human_cost_func_->looking_at_someone)
        {
            // get the person we are looking at
            hanp_msgs::TrackedHuman looking_at;
            for(auto tracked_human : tracked_humans.tracks)
            {
                if(tracked_human.track_id == human_cost_func_->looking_at_id)
                {
                    looking_at = tracked_human;
                    break;
                }
            }
            // to whom we were looking at is still detected
            if(looking_at.track_id != 0)
            {
                // get transform between head_pan and humans frame
                int res;
                try
                {
                    std::string error_msg;
                    res = tf_.waitForTransform(head_pan_frame_, tracked_humans.header.frame_id,
                        ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &error_msg);
                    tf_.lookupTransform(head_pan_frame_, tracked_humans.header.frame_id,
                        ros::Time(0), head_pan_to_humans_transform);
                    transform_found = true;
                }
                catch(const tf::ExtrapolationException &ex)
                {
                    ROS_DEBUG("context_cost_function: cannot extrapolate transform");
                }
                catch(const tf::TransformException &ex)
                {
                    ROS_ERROR("context_cost_function: transform failure (%d): %s", res, ex.what());
                }

                if(transform_found)
                {
                    // get human whom we are looking at in head_pan frame
                    tf::Pose transformed_human_tf;
                    tf::poseMsgToTF(looking_at.pose.pose, transformed_human_tf);
                    auto human_pose_in_head_pan = (head_pan_to_humans_transform * transformed_human_tf).getOrigin();

                    // check if human is already in visibility range
                    auto head_pan_to_human_angle = atan2(human_pose_in_head_pan.getX(), human_pose_in_head_pan.getY());
                    if(fabs(head_pan_to_human_angle) < visibility_angle_)
                    {
                        human_cost_func_->cost = 0.0;
                        human_cost_func_->looking_at_someone = false;
                    }
                    else
                    {
                        // update the looking point with new human position
                        geometry_msgs::PointStamped human_cost_point;
                        human_cost_point.header.stamp = ros::Time::now();
                        human_cost_point.header.frame_id = head_pan_frame_;
                        human_cost_point.point.x = human_pose_in_head_pan.getX();
                        human_cost_point.point.y = human_pose_in_head_pan.getY();

                        human_cost_func_->cost = 1.0;
                        human_cost_func_->point = human_cost_point;
                        return;
                    }
                }
            }
            // we lost the human whom we were looking at
            else
            {
                //TODO: dedice whether to keep looking at persons last knows position until we finish, and implement here
            }
        }

        // we are her if we are currently not looking at anyone or we lost whom we were looking at

        // get robot pose in frame of humans
        tf::StampedTransform robot_to_human_transform;
        geometry_msgs::Twist robot_twist_in_humans_frame;

        int res;
        try
        {
            std::string error_msg;
            res = tf_.waitForTransform(robot_base_frame_, tracked_humans.header.frame_id,
                ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &error_msg);
            tf_.lookupTransform(robot_base_frame_, tracked_humans.header.frame_id,
                ros::Time(0), robot_to_human_transform);
            tf_.lookupTwist(robot_base_frame_, tracked_humans.header.frame_id,
                ros::Time(0), ros::Duration(0.1), robot_twist_in_humans_frame);
            transform_found = true;
        }
        catch(const tf::ExtrapolationException &ex)
        {
            ROS_DEBUG("context_cost_function: cannot extrapolate transform");
        }
        catch(const tf::TransformException &ex)
        {
            ROS_ERROR("context_cost_function: transform failure (%d): %s", res, ex.what());
        }

        if(transform_found)
        {
            // get robot entity in humans frame
            hanp_head_behavior::Entity robot({robot_to_human_transform.getOrigin().getX(),
                robot_to_human_transform.getOrigin().getY(),
                robot_twist_in_humans_frame.linear.x, robot_twist_in_humans_frame.linear.y});

            // caculate person with lowest time-to-collision with the robot
            double min_ttc = std::numeric_limits<double>::infinity();
            hanp_msgs::TrackedHuman human_with_min_ttc;
            for(auto tracked_human : tracked_humans.tracks)
            {
                hanp_head_behavior::Entity human({tracked_human.pose.pose.position.x,
                    tracked_human.pose.pose.position.y, tracked_human.twist.twist.linear.x,
                    tracked_human.twist.twist.linear.y});

                auto ttc = timeToCollision(human, robot);
                if(ttc < min_ttc)
                {
                    min_ttc = ttc;
                    human_with_min_ttc = tracked_human;
                }
            }

            if(min_ttc < std::numeric_limits<double>::infinity())
            {
                // we found someone to look at
                geometry_msgs::PointStamped human_cost_point;
                human_cost_point.header.stamp = ros::Time::now();
                human_cost_point.header.frame_id = tracked_humans.header.frame_id;
                human_cost_point.point = human_with_min_ttc.pose.pose.position;

                human_cost_func_->cost = 1.0;
                human_cost_func_->point = human_cost_point;
                human_cost_func_->looking_at_id = human_with_min_ttc.track_id;
                human_cost_func_->looking_at_someone = true;
            }
            else
            {
                // there is no one to look at :(
                human_cost_func_->cost = 0.0;
                human_cost_func_->looking_at_someone = false;
            }
        }
    }

    void HANPHeadBehavior::publishPointHead(const ros::TimerEvent& event)
    {
        // get the fucntion with maximum cost
        double max_cost = 0;
        HANPHeadBehaviorCostFunc* max_cost_function = nullptr;
        for (auto cost_funtion : cost_functions_)
        {
            if(cost_funtion->cost > max_cost)
            {
                max_cost = cost_funtion->cost;
                max_cost_function = cost_funtion;
            }
        }

        // get the point_head point for function with maximum cost
        auto& point_head = max_cost_function->point;

        ROS_DEBUG("heading point: x=%f, y=%f, frame=%s", point_head.point.x,
            point_head.point.y, point_head.header.frame_id.c_str());
        point_head_pub_.publish(point_head);
    }

    // calculates time-to-collision from entity robot to human
    double HANPHeadBehavior::timeToCollision(hanp_head_behavior::Entity human, hanp_head_behavior::Entity robot)
    {
        auto r = ttc_collision_radius_;

        // c is vector from robot to human
        auto c = Eigen::Vector2d(robot.x - human.x, robot.y - human.y);

        // check for already in collision
        if (c.norm() < 2*r)
        {
            ROS_DEBUG_NAMED(NODE_NAME, "time-to-collision: human and robot are already in collision");
            return 0.0;
        }

        // v is the velocity of p1 considering p2 steady, in global co-ordinate frame
        auto v = Eigen::Vector2d(human.vx - robot.vx, human.vy - robot.vy);

        auto c_dot_v = c.dot(v);
        auto c_sq = c.squaredNorm();
        auto v_sq = v.squaredNorm();
        auto r_sq = r * r;
        auto f = (c_dot_v * c_dot_v) - (v_sq * (c_sq - r_sq));

        if(f > 0)
        {
            // ttc is calculated using t = ( V.C - sqrt((V.C)^2 - ||V||^2 (||C||^2 - r^2)) ) / ||V||^2
            auto ttc = (c_dot_v - std::sqrt(f)) / v_sq;
            if(ttc > 0)
            {
                return ttc;
            }
        }

        return std::numeric_limits<double>::infinity();
    }
}

// handler for something to do before killing the node
void sigintHandler(int sig)
{
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "node will now shutdown");

    // the default sigint handler, it calls shutdown() on node
    ros::shutdown();
}

// the main method starts a rosnode and initializes the optotrack_person class
int main(int argc, char **argv)
{
    // starting the optotrack_person node
    ros::init(argc, argv, NODE_NAME);
    ROS_DEBUG_STREAM_NAMED(NODE_NAME, "started " << NODE_NAME << " node");

    // initiazling HANPHeadBehavior class
    hanp_head_behavior::HANPHeadBehavior HANPHeadBehavior;
    HANPHeadBehavior.initialize();

    // look for sigint and start spinning the node
    signal(SIGINT, sigintHandler);
    ros::spin();

    return 0;
}
