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
    }

    void HANPHeadBehavior::localPlanCB(const nav_msgs::Path& local_plan)
    {
        // look at the end of the local plan
        if(local_plan.poses.size() > 0)
        {
            tf::Pose local_plan_end;
            tf::poseMsgToTF(local_plan.poses.back().pose, local_plan_end);
            auto local_plan_end_extended = local_plan_end(tf::Vector3(0.5,0,0)); //TODO: make this configurable
            geometry_msgs::PointStamped point_head;
            point_head.header.stamp = ros::Time::now();
            point_head.header.frame_id = local_plan.poses.back().header.frame_id;
            point_head.point.x = local_plan_end_extended.x();
            point_head.point.y = local_plan_end_extended.y();
            point_head.point.z = point_head_height_;
            publishPointHead(point_head);
        }
        else
        {
            geometry_msgs::PointStamped point_head;
            point_head.header.stamp = ros::Time::now();
            point_head.header.frame_id = robot_base_frame_;
            point_head.point.x = 1.0;
            point_head.point.y = 0.0;
            point_head.point.z = point_head_height_;
            publishPointHead(point_head);
        }
    }

    void HANPHeadBehavior::trackedHumansCB(const hanp_msgs::TrackedHumans& tracked_humans)
    {

    }

    void HANPHeadBehavior::publishPointHead(geometry_msgs::PointStamped& point_head)
    {
        ROS_DEBUG("heading point: x=%f, y=%f, frame=%s", point_head.point.x,
            point_head.point.y, point_head.header.frame_id.c_str());
        point_head_pub_.publish(point_head);
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
