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

 #include <signal.h>

 #include <hanp_head_behavior/hanp_head_behavior.h>

 namespace hanp_head_behavior
 {
     // empty constructor and destructor
     HANPHeadBehavior::HANPHeadBehavior() {}
     HANPHeadBehavior::~HANPHeadBehavior() {}

    void HANPHeadBehavior::initialize()
    {
        // get private node handle
        ros::NodeHandle private_nh("~/");

        // get parameters
        private_nh.param("local_plan_sub_topic", local_plan_sub_topic_, std::string(LOCAL_PLAN_SUB_TOPIC));
        private_nh.param("human_sub_topic", human_sub_topic_, std::string(HUMAN_SUB_TOPIC));
        private_nh.param("point_head_pub_topic", point_head_pub_topic_, std::string(POINT_HEAD_PUB_TOPIC));
        private_nh.param("robot_base_frame", robot_base_frame_, std::string(ROBOT_BASE_FRAME));

        // initialize subscribers and publishers
        local_plan_sub_ = private_nh.subscribe(local_plan_sub_topic_, 1, &HANPHeadBehavior::localPlanCB, this);
        humans_sub_ = private_nh.subscribe(human_sub_topic_, 1, &HANPHeadBehavior::trackedHumansCB, this);
        point_head_pub_ = private_nh.advertise<geometry_msgs::PointStamped>(point_head_pub_topic_, 1);

        // set-up dynamic reconfigure
        dsrv_ = new dynamic_reconfigure::Server<HANPHeadBehaviorConfig>(private_nh);
        dynamic_reconfigure::Server<HANPHeadBehaviorConfig>::CallbackType cb = boost::bind(&HANPHeadBehavior::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void HANPHeadBehavior::reconfigureCB(HANPHeadBehaviorConfig &config, uint32_t level)
    {
        point_head_height_ = config.point_head_height;
        ttc_collision_radius_ = config.ttc_collision_dist/2;
    }

    void HANPHeadBehavior::localPlanCB(const nav_msgs::Path& local_plan)
    {
        geometry_msgs::PointStamped point_head;
        point_head.header.stamp = ros::Time::now();

        // first check if we are looking at human
        if(human_cost_point_)
        {
            point_head.header.frame_id = human_cost_point_->header.frame_id;
            point_head.point = human_cost_point_->pose.position;

            ROS_DEBUG_NAMED(NODE_NAME, "head pointing to a human");
        }
        // look at the end of the local plan
        else if(local_plan.poses.size() > 0)
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

        publishPointHead(point_head);
    }

    void HANPHeadBehavior::trackedHumansCB(const hanp_msgs::TrackedHumans& tracked_humans)
    {
        // get robot pose in frame of humans
        bool transform_found = false;
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
            hanp_head_behavior::Entity robot({robot_to_human_transform.getOrigin().getX(),
                robot_to_human_transform.getOrigin().getY(),
                robot_twist_in_humans_frame.linear.x, robot_twist_in_humans_frame.linear.y});

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
                human_cost_point_ = new geometry_msgs::PoseStamped();
                human_cost_point_->header.stamp = ros::Time::now();
                human_cost_point_->header.frame_id = tracked_humans.header.frame_id;
                human_cost_point_->pose = human_with_min_ttc.pose.pose;
            }
            else
            {
                human_cost_point_ = nullptr;
            }
        }
    }

    void HANPHeadBehavior::publishPointHead(geometry_msgs::PointStamped& point_head)
    {
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
