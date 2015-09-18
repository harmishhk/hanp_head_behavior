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

#include <hanp_head_behavior/hanp_head_behavior_cost_func.h>

#include <dynamic_reconfigure/server.h>
#include <hanp_head_behavior/HANPHeadBehaviorConfig.h>

#include <hanp_msgs/TrackedHumans.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>

namespace hanp_head_behavior
{
    struct Entity
    {
    private:
        Eigen::Vector4d entity;
    public:
        double& x;
        double& y;
        double& vx;
        double& vy;

        Entity(Eigen::Vector4d entity) : entity(entity), x(entity[0]), y(entity[1]),
            vx(entity[2]), vy(entity[3]) {};
    };

    class PathCostFunc : public HANPHeadBehaviorCostFunc
    {
    public:
        PathCostFunc();
    };

    class HumanCostFunc : public HANPHeadBehaviorCostFunc
    {
    public:
        HumanCostFunc();

        int looking_at_id;
    };

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

        std::string local_plan_sub_topic_, human_sub_topic_, point_head_pub_topic_,
            robot_base_frame_, head_pan_frame_;

        tf::TransformListener tf_;
        ros::Timer publish_timer_;
        int publish_rate_;
        double ttc_collision_radius_, point_head_height_, visibility_angle_;

        hanp_head_behavior::PathCostFunc* path_cost_func_;
        hanp_head_behavior::HumanCostFunc* human_cost_func_;
        std::vector<hanp_head_behavior::HANPHeadBehaviorCostFunc*> cost_functions_;
        ros::Time last_plan_recieve_time_;
        ros::Duration local_plan_max_delay_;
        std::vector<int> already_looked_at_;
        double max_ttc_looking_, max_gma_;

        void publishPointHead(const ros::TimerEvent& event);
        double timeToCollision(hanp_head_behavior::Entity robot, hanp_head_behavior::Entity human);
    };
}

#endif // HANP_HEAD_BEHAVIOR_H_
