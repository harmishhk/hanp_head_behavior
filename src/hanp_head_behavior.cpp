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
#define THROTTLE_TIME 4.0

#define LOCAL_PLAN_SUB_TOPIC "local_plan"
#define HUMAN_SUB_TOPIC "humans"
#define POINT_HEAD_PUB_TOPIC "point_head"
#define ROBOT_BASE_FRAME "base_link"
#define HEAD_PAN_FRAME "head_pan_link"
#define PUBLISH_RATE 10
#define VISIBILITY_ANGLE 0.087 //radian (5 degrees)
#define LOCAL_PLAN_MAX_DELAY 4.0 // seconds
#define LOCAL_PLAN_END_EXTEND 0.5 // meters

#include <signal.h>

#include <hanp_head_behavior/hanp_head_behavior.h>

namespace hanp_head_behavior
{
    // empty constructor and destructor
    HANPHeadBehavior::HANPHeadBehavior() {}
    HANPHeadBehavior::~HANPHeadBehavior() {}

    PathUtilFunc::PathUtilFunc() {}
    HumanUtilFunc::HumanUtilFunc() {}

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

        double local_plan_max_delay_time;
        private_nh.param("local_plan_max_delay", local_plan_max_delay_time, LOCAL_PLAN_MAX_DELAY);
        local_plan_max_delay_ = ros::Duration(local_plan_max_delay_time);
        last_plan_recieve_time_ = ros::Time::now() - ros::Duration(local_plan_max_delay_time);

        // initialize subscribers and publishers
        local_plan_sub_ = private_nh.subscribe(local_plan_sub_topic_, 1, &HANPHeadBehavior::localPlanCB, this);
        humans_sub_ = private_nh.subscribe(human_sub_topic_, 1, &HANPHeadBehavior::trackedHumansCB, this);
        point_head_pub_ = private_nh.advertise<geometry_msgs::PointStamped>(point_head_pub_topic_, 1);

        // initialize publish timer
        if(publish_rate_ > 0.0)
        {
            publish_timer_ = private_nh.createTimer(ros::Duration(1.0 / publish_rate_),
                &HANPHeadBehavior::publishPointHead, this);
            ROS_INFO_NAMED(NODE_NAME, "%s: will publish %s at %d hz", NODE_NAME,
                point_head_pub_topic_.c_str(), publish_rate_);
        }
        else
        {
            ROS_ERROR_NAMED(NODE_NAME, "%s: publish rate (asked: %f hz) cannot be < 0, nothing will be published",
                NODE_NAME, publish_rate_);
        }

        // initialize utility functions
        path_util_func_ = new hanp_head_behavior::PathUtilFunc();
        util_funcs_.push_back(path_util_func_);
        human_util_func_ = new hanp_head_behavior::HumanUtilFunc();
        util_funcs_.push_back(human_util_func_);

        // set-up dynamic reconfigure
        dsrv_ = new dynamic_reconfigure::Server<HANPHeadBehaviorConfig>(private_nh);
        dynamic_reconfigure::Server<HANPHeadBehaviorConfig>::CallbackType cb = boost::bind(&HANPHeadBehavior::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        ROS_DEBUG_NAMED(NODE_NAME, "%s: node initialized", NODE_NAME);
    }

    void HANPHeadBehavior::reconfigureCB(HANPHeadBehaviorConfig &config, uint32_t level)
    {
        // ROS_DEBUG_NAMED(NODE_NAME, "%s: reconguring variables", NODE_NAME);

        robot_base_frame_ = config.robot_base_frame;
        head_pan_frame_ = config.head_pan_frame;

        publish_rate_ = config.publish_rate;

        path_util_func_->weight = config.path_func_weight;
        human_util_func_->weight = config.human_func_weight;

        point_head_height_ = config.point_head_height;
        ttc_collision_radius_ = config.ttc_collision_dist/2;
        visibility_angle_ = config.visibility_angle;
        local_plan_max_delay_ = ros::Duration(config.local_plan_max_delay);
        max_ttc_looking_ = config.max_ttc;
        max_gma_ = config.max_gma;
    }

    void HANPHeadBehavior::localPlanCB(const nav_msgs::Path& local_plan)
    {
        // ROS_DEBUG_NAMED(NODE_NAME, "%s: received local plan", NODE_NAME);

        last_plan_recieve_time_ = ros::Time::now();

        hanp_head_behavior::Point point_head;
        point_head.point.header.stamp = ros::Time::now();

        // look at the end of the local plan
        if(local_plan.poses.size() > 0)
        {
            tf::Pose local_plan_end;
            tf::poseMsgToTF(local_plan.poses.back().pose, local_plan_end);
            auto local_plan_end_extended = local_plan_end(tf::Vector3(LOCAL_PLAN_END_EXTEND,0.0,0.0));
            point_head.point.header.frame_id = local_plan.poses.back().header.frame_id;
            point_head.point.point.x = local_plan_end_extended.x();
            point_head.point.point.y = local_plan_end_extended.y();
            point_head.point.point.z = point_head_height_;
            point_head.utility = 1.0;

            // ROS_DEBUG_NAMED(NODE_NAME, "%s: head pointing to path", NODE_NAME);
        }
        // look in the front
        else
        {
            point_head.point.header.frame_id = robot_base_frame_;
            point_head.point.point.x = LOCAL_PLAN_END_EXTEND;
            point_head.point.point.y = 0.0;
            point_head.point.point.z = point_head_height_;
            point_head.utility = 1.0;

            // ROS_DEBUG_NAMED(NODE_NAME, "%s: head pointing in the front", NODE_NAME);
        }

        path_util_func_->point = point_head;
        path_util_func_->enable = true;
    }

    void HANPHeadBehavior::trackedHumansCB(const hanp_msgs::TrackedHumans& tracked_humans)
    {
        // ROS_DEBUG_NAMED(NODE_NAME, "%s: received tracked humans", NODE_NAME);

        // don't care about humans when we are not moving
        if(!path_util_func_->enable)
        {
            return;
        }

        // get transforms between head_pan, robot_base and humans frame
        tf::StampedTransform head_pan_to_humans_transform;
        tf::StampedTransform robot_base_to_human_transform;
        geometry_msgs::Twist robot_twist_in_humans_frame;
        int res;
        std::string error_msg;
        bool transforms_found = false;
        try
        {
            res = tf_.waitForTransform(head_pan_frame_, tracked_humans.header.frame_id,
                tracked_humans.header.stamp, ros::Duration(0.5), ros::Duration(0.01), &error_msg);
            tf_.lookupTransform(head_pan_frame_, tracked_humans.header.frame_id,
                tracked_humans.header.stamp, head_pan_to_humans_transform);
            res = tf_.waitForTransform(robot_base_frame_, tracked_humans.header.frame_id,
                tracked_humans.header.stamp, ros::Duration(0.5), ros::Duration(0.01), &error_msg);
            tf_.lookupTransform(robot_base_frame_, tracked_humans.header.frame_id,
                tracked_humans.header.stamp, robot_base_to_human_transform);
            tf_.lookupTwist(robot_base_frame_, tracked_humans.header.frame_id,
                tracked_humans.header.stamp, ros::Duration(0.1), robot_twist_in_humans_frame);
            transforms_found = true;
        }
        catch(const tf::ExtrapolationException &ex)
        {
            ROS_DEBUG_NAMED(NODE_NAME, "%s: cannot extrapolate transform from %s to %s, error: %d",
                NODE_NAME, tracked_humans.header.frame_id.c_str(), head_pan_frame_.c_str(), res);
        }
        catch(const tf::TransformException &ex)
        {
            ROS_ERROR_NAMED(NODE_NAME, "%s: transform failure (%d): %s", NODE_NAME, res, ex.what());
        }

        if(!transforms_found)
        {
            //TODO: what to do when transform are not found
            return;
        }

        // check if we are already looking at human
        if(human_util_func_->enable)
        {
            // get the person we are looking at
            hanp_msgs::TrackedHuman looking_at;
            looking_at.track_id = 0;
            for(auto tracked_human : tracked_humans.tracks)
            {
                if(tracked_human.track_id == human_util_func_->looking_at_id)
                {
                    looking_at = tracked_human;
                    break;
                }
            }

            // to whom we were looking at is still detected
            if(looking_at.track_id != 0)
            {
                // get human whom we are looking at in head_pan frame
                tf::Pose transformed_human_tf;
                tf::poseMsgToTF(looking_at.pose.pose, transformed_human_tf);
                auto human_pose_in_head_pan = (head_pan_to_humans_transform
                    * transformed_human_tf).getOrigin();
                auto head_pan_to_human_angle = atan2(human_pose_in_head_pan.getY(),
                    human_pose_in_head_pan.getX());
                auto human_pose_in_robot_base = (robot_base_to_human_transform
                    * transformed_human_tf).getOrigin();
                auto robot_base_to_human_angle = atan2(human_pose_in_robot_base.getY(),
                    human_pose_in_robot_base.getX());

                if(fabs(head_pan_to_human_angle) < visibility_angle_)
                {
                    // we have seen the human
                    human_util_func_->enable = false;
                    ROS_DEBUG_NAMED(NODE_NAME, "%s: we have seen human %d, angle: %f",
                        NODE_NAME, looking_at.track_id, head_pan_to_human_angle);
                }
                else if(fabs(robot_base_to_human_angle) > max_gma_)
                {
                    // we won't be able to see human anymore / doesn't matter any more to look that human
                    human_util_func_->enable = false;
                    ROS_DEBUG_NAMED(NODE_NAME, "%s: we won't look at human %d anymore, angle: %f",
                        NODE_NAME, looking_at.track_id, robot_base_to_human_angle);
                }
                else
                {
                    // update the looking point with new human position
                    hanp_head_behavior::Point human_point;
                    human_point.point.header.stamp = ros::Time::now();
                    human_point.point.header.frame_id = head_pan_frame_;
                    human_point.point.point.x = human_pose_in_head_pan.getX();
                    human_point.point.point.y = human_pose_in_head_pan.getY();
                    human_point.utility = 1.0;

                    human_util_func_->point = human_point;
                    human_util_func_->enable = true;

                    ROS_DEBUG_NAMED(NODE_NAME, "%s: still looking at human %d, \npan-human angle: %f, \nbase-human angle: %f",
                        NODE_NAME, looking_at.track_id, head_pan_to_human_angle, robot_base_to_human_angle);
                    return;
                }
            }
            else
            {
                // we lost the human whom we were looking at
                ROS_DEBUG_NAMED(NODE_NAME, "%s: we lost human %d, whom we were looking at",
                    NODE_NAME, looking_at.track_id);
                //TODO: dedice whether to keep looking at persons last knows position until we finish, and implement here
            }
        }

        // we are her if we are currently not looking at anyone or we lost whom we were looking at

        // get robot entity in humans frame
        hanp_head_behavior::Entity robot({robot_base_to_human_transform.getOrigin().getX(),
            robot_base_to_human_transform.getOrigin().getY(),
            robot_twist_in_humans_frame.linear.x, robot_twist_in_humans_frame.linear.y});

        // caculate person with lowest time-to-collision with the robot
        double min_ttc = std::numeric_limits<double>::infinity();
        hanp_msgs::TrackedHuman human_with_min_ttc;
        for(auto tracked_human : tracked_humans.tracks)
        {
            // nothing to do if we have already looked at this person
            if(std::find(already_looked_at_.begin(), already_looked_at_.end(),
                tracked_human.track_id) != already_looked_at_.end())
            {
                ROS_DEBUG_NAMED(NODE_NAME, "%s: we have already looked at human %d, continueing",
                    NODE_NAME, tracked_human.track_id);
                continue;
            }

            hanp_head_behavior::Entity human({tracked_human.pose.pose.position.x,
                tracked_human.pose.pose.position.y, tracked_human.twist.twist.linear.x,
                tracked_human.twist.twist.linear.y});

            auto ttc = timeToCollision(human, robot);
            ttc = 1.0; //TODO: remove this after tests
            // ROS_DEBUG_NAMED(NODE_NAME, "%s: ttc for human %d: %f", NODE_NAME,
            //     tracked_human.track_id, ttc);
            if(ttc < min_ttc)
            {
                min_ttc = ttc;
                human_with_min_ttc = tracked_human;
            }
        }

        if(min_ttc < max_ttc_looking_)
        {
            // we found someone to look at
            hanp_head_behavior::Point human_point;
            human_point.point.header.stamp = ros::Time::now();
            human_point.point.header.frame_id = tracked_humans.header.frame_id;
            human_point.point.point = human_with_min_ttc.pose.pose.position;
            human_point.utility = 1.0;

            human_util_func_->point = human_point;
            human_util_func_->looking_at_id = human_with_min_ttc.track_id;
            human_util_func_->enable = true;

            already_looked_at_.push_back(human_with_min_ttc.track_id);

            // ROS_DEBUG_NAMED(NODE_NAME, "%s: looking at human %d", NODE_NAME,
            //     human_with_min_ttc.track_id);
        }
        else
        {
            // there is no one to look at :(
            ROS_DEBUG_THROTTLE_NAMED(THROTTLE_TIME, NODE_NAME, "%s: there is no one to look at :(",
                NODE_NAME);
            human_util_func_->enable = false;
        }
    }

    void HANPHeadBehavior::publishPointHead(const ros::TimerEvent& event)
    {
        // nothing to do when not moving
        if(!path_util_func_->enable)
        {
            return;
        }

        // disable publishing if no plan has been received for long
        auto now = ros::Time::now();
        if((now - last_plan_recieve_time_) > local_plan_max_delay_)
        {
            already_looked_at_.clear();
            path_util_func_->enable = false;

            geometry_msgs::PointStamped point_head;
            point_head.header.stamp = now;
            point_head.header.frame_id = robot_base_frame_;
            point_head.point.x = LOCAL_PLAN_END_EXTEND;
            point_head.point.y = 0.0;
            point_head.point.z = point_head_height_;
            point_head_pub_.publish(point_head);

            ROS_INFO_NAMED(NODE_NAME, "%s: we are not moving anymore, disabling head movements", NODE_NAME);
            return;
        }

        // ROS_DEBUG_NAMED(NODE_NAME, "%s: going to publish point head", NODE_NAME);

        // get the fucntion with maximum weight
        double max_weight = 0;
        HANPHeadBehaviorUtilFunc* max_weight_func = nullptr;
        for (auto util_func : util_funcs_)
        {
            if(util_func->enable && (util_func->weight > max_weight))
            {
                max_weight = util_func->weight;
                max_weight_func = util_func;
            }
        }

        // get the point_head point for function with maximum weight
        if(max_weight_func)
        {
            auto& point_head = max_weight_func->point.point;

            // get pointing angle in base
            int res;
            std::string error_msg;
            try
            {
                geometry_msgs::PointStamped point_head_in_base;
                res = tf_.waitForTransform(robot_base_frame_, point_head.header.frame_id,
                    point_head.header.stamp, ros::Duration(0.5), ros::Duration(0.01), &error_msg);
                tf_.transformPoint(robot_base_frame_, point_head, point_head_in_base);

                // check for gma limits
                auto point_head_angle = atan2(point_head_in_base.point.y, point_head_in_base.point.x);
                ROS_DEBUG_NAMED(NODE_NAME, "%s: calculated point head angle: %f", NODE_NAME, point_head_angle);
                if(fabs(point_head_angle) > max_gma_)
                {
                    point_head_in_base.point.x = cos(max_gma_);
                    point_head_in_base.point.y = std::copysign(sin(max_gma_), point_head_angle);
                    point_head = point_head_in_base;
                }
                ROS_DEBUG_NAMED(NODE_NAME, "%s: heading point: x=%f, y=%f, frame=%s",
                    NODE_NAME, point_head.point.x, point_head.point.y,
                    point_head.header.frame_id.c_str());
                    point_head_pub_.publish(point_head);
            }
            catch(const tf::ExtrapolationException &ex)
            {
                ROS_DEBUG_NAMED(NODE_NAME, "%s: cannot extrapolate transform from %s to %s, error %d",
                    NODE_NAME, point_head.header.frame_id.c_str(), robot_base_frame_.c_str(), res);
            }
            catch(const tf::TransformException &ex)
            {
                ROS_ERROR_NAMED(NODE_NAME, "%s: transform failure (%d): %s", NODE_NAME, res, ex.what());
            }
        }
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
            ROS_DEBUG_NAMED(NODE_NAME, "%s: time-to-collision: human and robot are "
                "already in collision", NODE_NAME);
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
    ROS_DEBUG_NAMED(NODE_NAME, "%s: node will now shutdown", NODE_NAME);

    // the default sigint handler, it calls shutdown() on node
    ros::shutdown();
}

// the main method starts a rosnode and initializes the optotrack_person class
int main(int argc, char **argv)
{
    // starting the optotrack_person node
    ros::init(argc, argv, NODE_NAME);
    ROS_DEBUG_NAMED(NODE_NAME, "started %s node", NODE_NAME);

    // initiazling HANPHeadBehavior class
    hanp_head_behavior::HANPHeadBehavior HANPHeadBehavior;
    HANPHeadBehavior.initialize();

    // look for sigint and start spinning the node
    signal(SIGINT, sigintHandler);
    ros::spin();

    return 0;
}
