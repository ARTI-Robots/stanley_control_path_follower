/// \file
/// \author Alexander Buchegger
/// \date 2021-06-17
/// \copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
#ifndef ARTI_STANLEY_CONTROL_STANLEY_CONTROL_H
#define ARTI_STANLEY_CONTROL_STANLEY_CONTROL_H

#include <arti_nav_core/transformer.h>
#include <arti_nav_core_msgs/Pose2DWithLimits.h>
#include <arti_nav_core_msgs/Trajectory2DWithLimits.h>
#include <arti_nav_core_msgs/Twist2DWithLimits.h>
#include <arti_nav_core_msgs/ValueWithLimits.h>
#include <arti_nav_core_utils/visualization.h>
#include <arti_public_stanley_control/StanleyControlConfig.h>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <dynamic_reconfigure/server.h>
#include <limits>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_msgs/Float32.h>
#include <string>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace arti_public_stanley_control
{

class StanleyControl
{
public:
  struct VelocityCommand
  {
    double linear_velocity;
    double angular_velocity;
    double steering_angle;
  };

  struct Result
  {
    enum class Status
    {
      GOAL_REACHED,
      COMMAND_FOUND,
      NO_COMMAND_POSSIBLE
    };

    Status status;
    VelocityCommand velocity_command;
  };

  StanleyControl(const ros::NodeHandle& node_handle, arti_nav_core::Transformer* transformer);

  bool setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory);

  Result computeVelocityCommand();

  bool isGoalReached();

protected:
  struct State
  {
    double x;
    double y;
    double yaw;
    double v;
  };

  struct TrajectoryPosition
  {
    size_t segment_index{0};
    double segment_length{0.0};
    double parallel_distance{0.0};
    double normal_distance{0.0};
  };

  struct Target
  {
    size_t segment_index{0};
    double position_in_segment{0.0};
    arti_nav_core_msgs::Movement2DWithLimits movement;
    bool reverse{false};
    double lookahead{0.0};
    double crosstrack_error{0.0};
    double heading_error{0.0};
  };

  void reconfigure(const arti_public_stanley_control::StanleyControlConfig& new_config);
  void processInputPose(const nav_msgs::OdometryConstPtr& input_pose);
  boost::optional<State> getState() const;

  double computeSteeringAngle(
    double heading_error, double crosstrack_error, bool reverse, double lookahead, double linear_velocity) const;

  TrajectoryPosition determineTrajectoryPosition(
    size_t start_segment_index, double start_parallel_distance, double x, double y) const;
  Target determineTarget(const State& state) const;

  VelocityCommand computeCommand(double linear_velocity, double steering_angle) const;
  void publishTrajectory();

  static arti_nav_core_msgs::Pose2DWithLimits combinePosesWithLimits(
    const arti_nav_core_msgs::Pose2DWithLimits& a, const arti_nav_core_msgs::Pose2DWithLimits& b, double f);
  static arti_nav_core_msgs::Twist2DWithLimits combineTwistsWithLimits(
    const arti_nav_core_msgs::Twist2DWithLimits& a, const arti_nav_core_msgs::Twist2DWithLimits& b, double f);
  static arti_nav_core_msgs::ValueWithLimits combineValuesWithLimits(
    const arti_nav_core_msgs::ValueWithLimits& a, const arti_nav_core_msgs::ValueWithLimits& b, double f,
    bool is_angle = false);

  bool xyWithinTolerance(double current_value, const arti_nav_core_msgs::ValueWithLimits& goal) const;
  bool thetaWithinTolerance(double current_value, const arti_nav_core_msgs::ValueWithLimits& goal) const;
  static bool isForwardSegment(
    const arti_nav_core_msgs::Pose2DWithLimits& a, const arti_nav_core_msgs::Pose2DWithLimits& b);
  static std_msgs::Float32 makeFloat32(std_msgs::Float32::_data_type data);

  ros::NodeHandle private_nh_;
  arti_nav_core::Transformer* transformer_;

  dynamic_reconfigure::Server<arti_public_stanley_control::StanleyControlConfig> cfg_server_;
  arti_public_stanley_control::StanleyControlConfig cfg_;
  double max_steering_angle_{0.0};

  ros::NodeHandle debug_nh_;
  ros::Publisher path_publisher_;
  arti_nav_core_utils::TrajectoryWithVelocityMarkersPublisher path_with_velocity_publisher_;
  ros::Publisher lookahead_pose_publisher_;
  ros::Publisher target_publisher_;
  ros::Publisher speed_error_publisher_;
  ros::Publisher heading_error_publisher_;
  ros::Publisher crosstrack_error_compensation_publisher_;
  ros::Publisher crosstrack_error_publisher_;
  ros::Publisher goal_distance_publisher_;
  ros::Publisher goal_distance_x_publisher_;
  ros::Publisher goal_distance_y_publisher_;
  ros::Publisher goal_distance_theta_publisher_;
  ros::Subscriber input_pose_subscriber_;

  nav_msgs::OdometryConstPtr input_pose_;

  bool path_processing_{false};
  arti_nav_core_msgs::Trajectory2DWithLimits current_trajectory_;
  size_t current_segment_index_{0};
  double current_position_in_segment_{0.0};

  double goal_distance_{std::numeric_limits<double>::max()};
  ros::Time last_valid_velocity_time_{0};

  VelocityCommand latest_velocity_command_{0.0, 0.0, 0.0};

  bool reached_x_{false};
  bool reached_y_{false};
  bool reached_theta_{false};
  bool goal_reached_{false};
  void setVelocity(boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& anOptional) const;
};

}

#endif // ARTI_STANLEY_CONTROL_STANLEY_CONTROL_H
