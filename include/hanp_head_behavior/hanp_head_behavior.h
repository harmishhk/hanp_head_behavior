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

#ifndef HANP_HEAD_BEHAVIOR_H_
#define HANP_HEAD_BEHAVIOR_H_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <hanp_head_behavior/HANPHeadBehaviorConfig.h>

#include <hanp_msgs/TrackedHumans.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

namespace hanp_head_behavior
{
    class HANPHeadBehavior
    {
    public:
        HANPHeadBehavior();
        ~HANPHeadBehavior();

        void initialize();

    private:
        // ros subscribers and publishers
        ros::Subscriber local_plan_sub_, humans_sub_;
        ros::Publisher point_head_pub_;

        // dynamic reconfigure variables
        dynamic_reconfigure::Server<HANPHeadBehaviorConfig> *dsrv_;
        void reconfigureCB(HANPHeadBehaviorConfig &config, uint32_t level);

        // subscriber callbacks
        void localPlanCB(const nav_msgs::Path& local_plan);
        void trackedHumansCB(const hanp_msgs::TrackedHumans& tracked_humans);

        std::string local_plan_sub_topic_, human_sub_topic_, point_head_pub_topic_, robot_base_frame_;

        double point_head_height_;

        void publishPointHead(geometry_msgs::PointStamped& point_head);
    };
}

#endif // HANP_HEAD_BEHAVIOR_H_
