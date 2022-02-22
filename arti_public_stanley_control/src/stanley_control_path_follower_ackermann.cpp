/// \file
/// \author Alexander Buchegger
/// \date 2021-06-17
/// \copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
#include <arti_public_stanley_control/stanley_control_path_follower_ackermann.h>
#include <pluginlib/class_list_macros.h>

namespace arti_public_stanley_control
{

void StanleyControlPathFollowerAckermann::initialize(
  std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* /*costmap_ros*/)
{
  stanley_control_.emplace(ros::NodeHandle("~/" + name), transformer);
}

bool StanleyControlPathFollowerAckermann::setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  if (!stanley_control_)
  {
    ROS_ERROR_STREAM("called setTrajectory before initialize");
    return false;
  }

  return stanley_control_->setTrajectory(trajectory);
}

arti_nav_core::BasePathFollowerAckermann::BasePathFollowerAckermannErrorEnum
StanleyControlPathFollowerAckermann::computeVelocityCommands(ackermann_msgs::AckermannDrive& next_command)
{
  next_command = ackermann_msgs::AckermannDrive();

  if (!stanley_control_)
  {
    ROS_ERROR_STREAM("called computeVelocityCommands before initialize");
    return BasePathFollowerAckermannErrorEnum::NO_COMMAND_POSSIBLE;
  }

  const StanleyControl::Result result = stanley_control_->computeVelocityCommand();
  next_command.speed
    = static_cast<ackermann_msgs::AckermannDrive::_speed_type>(result.velocity_command.linear_velocity);
  next_command.steering_angle
    = static_cast<ackermann_msgs::AckermannDrive::_steering_angle_type>(result.velocity_command.steering_angle);

  switch (result.status)
  {
    case StanleyControl::Result::Status::GOAL_REACHED:
      return BasePathFollowerAckermannErrorEnum::GOAL_REACHED;

    case StanleyControl::Result::Status::COMMAND_FOUND:
      return BasePathFollowerAckermannErrorEnum::COMMAND_FOUND;

    case StanleyControl::Result::Status::NO_COMMAND_POSSIBLE:
      return BasePathFollowerAckermannErrorEnum::NO_COMMAND_POSSIBLE;
  }

  ROS_ERROR_STREAM("invalid result status, this should never happen");
  return BasePathFollowerAckermannErrorEnum::NO_COMMAND_POSSIBLE;
}

bool StanleyControlPathFollowerAckermann::isGoalReached()
{
  if (!stanley_control_)
  {
    ROS_ERROR_STREAM("called isGoalReached before initialize");
    return false;
  }

  return stanley_control_->isGoalReached();
}

}

PLUGINLIB_EXPORT_CLASS(arti_public_stanley_control::StanleyControlPathFollowerAckermann,
                       arti_nav_core::BasePathFollowerAckermann)
