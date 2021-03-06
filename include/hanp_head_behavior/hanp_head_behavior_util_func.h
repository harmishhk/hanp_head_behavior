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
 *                                  Harmish Khambhaita on Tue Sep 15 2015
 */

#ifndef HANP_HEAD_BEHAVIOR_FUNC_H_
#define HANP_HEAD_BEHAVIOR_FUNC_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

namespace hanp_head_behavior
{
    enum class SocialContextType : int {Aliveness, Interaction, SocialAttention, IntentProjection};
    const SocialContextType SocialContext[] =
    {
        SocialContextType::Aliveness,
        SocialContextType::Interaction,
        SocialContextType::SocialAttention,
        SocialContextType::IntentProjection
    };

    struct Point
    {
        std::map<hanp_head_behavior::SocialContextType, double> criteria_scores_;
        geometry_msgs::PointStamped point_;
    };

    class HANPHeadBehaviorFunc
    {
    public:
        HANPHeadBehaviorFunc()
        {
            for(auto context : hanp_head_behavior::SocialContext)
            {
                point_.criteria_scores_[context] = 0.0;
            }
        };
        ~HANPHeadBehaviorFunc();

        bool enable_ = false;
        hanp_head_behavior::Point point_;
    };
}

#endif // HANP_HEAD_BEHAVIOR_FUNC_H_
