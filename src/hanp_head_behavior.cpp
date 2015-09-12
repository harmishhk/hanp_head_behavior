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

        // initialize subscribers and publishers
        local_plan_sub_ = private_nh.subscribe(LOCAL_PLAN_SUB_TOPIC, 1, &HANPHeadBehavior::localPlanCB, this);
        humans_sub_ = private_nh.subscribe(HUMAN_SUB_TOPIC, 1, &HANPHeadBehavior::trackedHumansCB, this);
        point_head_pub_ = private_nh.advertise<geometry_msgs::PointStamped>(POINT_HEAD_PUB_TOPIC, 1);

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

    }

    void HANPHeadBehavior::trackedHumansCB(const hanp_msgs::TrackedHumans& tracked_humans)
    {

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
    hanp_head_behavior::HANPHeadBehavior HANPHeadBehavior();

    // look for sigint and start spinning the node
    signal(SIGINT, sigintHandler);
    ros::spin();

    return 0;
}
