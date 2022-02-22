/*
Created by Konstantin Maunter-Lassnig on 23.11.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_STANLEY_CONTROL_STANLEY_CONTROL_PATH_FOLLOWER_H
#define ARTI_STANLEY_CONTROL_STANLEY_CONTROL_PATH_FOLLOWER_H

#include <arti_nav_core/base_path_follower.h>
#include <arti_public_stanley_control/stanley_control.h>
#include <boost/optional.hpp>

namespace arti_public_stanley_control
{

class StanleyControlPathFollower : public arti_nav_core::BasePathFollower
{
public:
  StanleyControlPathFollower();

  void initialize(
    std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) override;

  BasePathFollowerErrorEnum computeVelocityCommands(geometry_msgs::Twist& next_command) override;

  bool isGoalReached() override;

protected:
  boost::optional<StanleyControl> stanley_control_;
};

}

#endif //ARTI_STANLEY_CONTROL_STANLEY_CONTROL_PATH_FOLLOWER_H
