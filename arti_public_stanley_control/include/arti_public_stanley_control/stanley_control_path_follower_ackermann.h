/// \file
/// \author Alexander Buchegger
/// \date 2021-06-17
/// \copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
#ifndef ARTI_STANLEY_CONTROL_STANLEY_CONTROL_PATH_FOLLOWER_ACKERMANN_H
#define ARTI_STANLEY_CONTROL_STANLEY_CONTROL_PATH_FOLLOWER_ACKERMANN_H

#include <arti_nav_core/base_path_follower_ackermann.h>
#include <arti_public_stanley_control/stanley_control.h>
#include <boost/optional.hpp>

namespace arti_public_stanley_control
{

class StanleyControlPathFollowerAckermann : public arti_nav_core::BasePathFollowerAckermann
{
public:
  void initialize(
    std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) override;

  BasePathFollowerAckermannErrorEnum computeVelocityCommands(ackermann_msgs::AckermannDrive& next_command) override;

  bool isGoalReached() override;

protected:
  boost::optional<StanleyControl> stanley_control_;
};

}

#endif // ARTI_STANLEY_CONTROL_STANLEY_CONTROL_PATH_FOLLOWER_ACKERMANN_H
