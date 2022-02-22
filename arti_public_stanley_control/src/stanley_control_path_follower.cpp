/*
Created by Konstantin Mautner-Lassnig on 23.11.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_public_stanley_control/stanley_control_path_follower.h>
#include <pluginlib/class_list_macros.h>

namespace arti_public_stanley_control
{

StanleyControlPathFollower::StanleyControlPathFollower() = default;

void StanleyControlPathFollower::initialize(
  std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* /*costmap_ros*/)
{
  stanley_control_.emplace(ros::NodeHandle("~/" + name), transformer);
}

bool StanleyControlPathFollower::setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  if (!stanley_control_)
  {
    ROS_ERROR_STREAM("called setTrajectory before initialize");
    return false;
  }

  return stanley_control_->setTrajectory(trajectory);
}

arti_nav_core::BasePathFollower::BasePathFollowerErrorEnum StanleyControlPathFollower::computeVelocityCommands(
  geometry_msgs::Twist& next_command)
{
  next_command = geometry_msgs::Twist();

  if (!stanley_control_)
  {
    ROS_ERROR_STREAM("called computeVelocityCommands before initialize");
    return BasePathFollowerErrorEnum::NO_COMMAND_POSSIBLE;
  }

  const StanleyControl::Result result = stanley_control_->computeVelocityCommand();
  next_command.linear.x = result.velocity_command.linear_velocity;
  next_command.angular.z = result.velocity_command.angular_velocity;

  switch (result.status)
  {
    case StanleyControl::Result::Status::GOAL_REACHED:
      return BasePathFollowerErrorEnum::GOAL_REACHED;

    case StanleyControl::Result::Status::COMMAND_FOUND:
      return BasePathFollowerErrorEnum::COMMAND_FOUND;

    case StanleyControl::Result::Status::NO_COMMAND_POSSIBLE:
      return BasePathFollowerErrorEnum::NO_COMMAND_POSSIBLE;
  }

  ROS_ERROR_STREAM("invalid result status, this should never happen");
  return BasePathFollowerErrorEnum::NO_COMMAND_POSSIBLE;
}

bool StanleyControlPathFollower::isGoalReached()
{
  if (!stanley_control_)
  {
    ROS_ERROR_STREAM("called isGoalReached before initialize");
    return false;
  }

  return stanley_control_->isGoalReached();
}

}

PLUGINLIB_EXPORT_CLASS(arti_public_stanley_control::StanleyControlPathFollower, arti_nav_core::BasePathFollower)
