/// \file
/// \author Alexander Buchegger
/// \date 2021-06-17
/// \copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
#include <arti_public_stanley_control/stanley_control.h>
#include <algorithm>
#include <angles/angles.h>
#include <arti_nav_core_utils/conversions.h>
#include <arti_nav_core_utils/tf2_arti_nav_core_msgs.h>
#include <arti_nav_core_utils/transformer.h>
#include <cmath>
#include <functional>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>


namespace arti_public_stanley_control
{

StanleyControl::StanleyControl(
  const ros::NodeHandle& node_handle, arti_nav_core::Transformer* transformer)
  : private_nh_(node_handle), transformer_(transformer), cfg_server_(private_nh_),
    debug_nh_(private_nh_, "/stanley_control"),
    path_publisher_(debug_nh_.advertise<nav_msgs::Path>("path", 1, true)),
    path_with_velocity_publisher_(debug_nh_, "path_with_velocity"),
    lookahead_pose_publisher_(debug_nh_.advertise<geometry_msgs::PoseStamped>("lookahead_pose", 1)),
    target_publisher_(debug_nh_.advertise<geometry_msgs::PoseStamped>("target", 1)),
    speed_error_publisher_(debug_nh_.advertise<std_msgs::Float32>("speed_error", 1)),
    heading_error_publisher_(debug_nh_.advertise<std_msgs::Float32>("heading_error", 1)),
    crosstrack_error_compensation_publisher_(debug_nh_.advertise<std_msgs::Float32>("cross_track_error", 1)),
    crosstrack_error_publisher_(debug_nh_.advertise<std_msgs::Float32>("axle_error", 1)),
    goal_distance_publisher_(debug_nh_.advertise<std_msgs::Float32>("goal_distance", 1)),
    goal_distance_x_publisher_(debug_nh_.advertise<std_msgs::Float32>("goal_distance_x", 1)),
    goal_distance_y_publisher_(debug_nh_.advertise<std_msgs::Float32>("goal_distance_y", 1)),
    goal_distance_theta_publisher_(debug_nh_.advertise<std_msgs::Float32>("goal_distance_theta", 1))
{
  cfg_server_.setCallback(std::bind(&StanleyControl::reconfigure, this, std::placeholders::_1));
}

bool StanleyControl::setTrajectory(const arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits> trajectory_tfd;
  if (trajectory.header.frame_id == cfg_.target_frame)
  {
    // same frame, no transformation required
    trajectory_tfd = trajectory;
  }
  else
  {
    std::string error_message;
    trajectory_tfd = arti_nav_core_utils::tryToTransform(*transformer_, trajectory, cfg_.target_frame,
                                                         ros::Duration(cfg_.transformation_timeout), &error_message);
    if (!trajectory_tfd)
    {
      ROS_ERROR_STREAM("failed to transform trajectory: " << error_message);
      return false;
    }
  }

  setVelocity(trajectory_tfd);
  current_trajectory_ = trajectory_tfd.value();

  path_processing_ = true;
  current_segment_index_ = 0;
  current_position_in_segment_ = 0.0;

  goal_distance_ = std::numeric_limits<double>::max();

  reached_x_ = false;
  reached_y_ = false;
  reached_theta_ = false;
  goal_reached_ = false;

  last_valid_velocity_time_ = ros::Time::now();

  return true;
}

StanleyControl::Result StanleyControl::computeVelocityCommand()
{
  if (!path_processing_)
  {
    return {(goal_reached_ ? Result::Status::GOAL_REACHED : Result::Status::NO_COMMAND_POSSIBLE), {0.0, 0.0, 0.0}};
  }

  const boost::optional<State> optional_state = getState();
  if (!optional_state)
  {
    ROS_ERROR_STREAM("cannot compute velocity command without a recent input pose");
    return {Result::Status::NO_COMMAND_POSSIBLE, {0.0, 0.0, 0.0}};
  }
  const State state = optional_state.value();

  const Target target = determineTarget(state);
  ROS_DEBUG_STREAM(
    "target:\n"
      << "segment_index: " << target.segment_index << ", position_in_segment: " << target.position_in_segment
      << ", reverse: " << target.reverse << ", lookahead: " << target.lookahead
      << ", crosstrack_error: " << target.crosstrack_error << ", heading_error: " << target.heading_error);

  current_segment_index_ = target.segment_index;
  current_position_in_segment_ = target.position_in_segment;

  crosstrack_error_publisher_.publish(makeFloat32(static_cast<std_msgs::Float32::_data_type>(target.crosstrack_error)));
  heading_error_publisher_.publish(makeFloat32(static_cast<std_msgs::Float32::_data_type>(target.heading_error)));

  geometry_msgs::PoseStamped lookahead_pose;
  lookahead_pose.header = input_pose_->header;
  lookahead_pose.pose.position.x = state.x + target.lookahead * std::cos(state.yaw);
  lookahead_pose.pose.position.y = state.y + target.lookahead * std::sin(state.yaw);
  lookahead_pose.pose.orientation = tf::createQuaternionMsgFromYaw(state.yaw);
  lookahead_pose_publisher_.publish(lookahead_pose);

  geometry_msgs::PoseStamped target_msg;
  target_msg.header = input_pose_->header;
  target_msg.pose = arti_nav_core_utils::convertToPose(target.movement.pose,
                                                       arti_nav_core_utils::non_finite_values::REPLACE_BY_0);
  target_publisher_.publish(target_msg);

  double target_speed = target.movement.twist.x.value;
  ROS_DEBUG_STREAM("target speed: " << target_speed);

  if (!std::isfinite(target_speed))
  {
    // This means there is no velocity set point or limit, so we default to the maximum velocity:
    target_speed = target.reverse ? -cfg_.max_velocity : cfg_.max_velocity;
  }
  else if (std::abs(target_speed) > cfg_.max_velocity)
  {
    ROS_WARN_STREAM_THROTTLE(5.0, "target speed in trajectory ("
      << target_speed << ") is higher than configured maximum (" << cfg_.max_velocity << ")");
    target_speed = std::copysign(cfg_.max_velocity, target_speed);
  }

  const double speed_error = target_speed - state.v;
  speed_error_publisher_.publish(makeFloat32(static_cast<std_msgs::Float32::_data_type>(speed_error)));

  const double speed_gain = target.reverse ? cfg_.speed_proportional_gain_kp_negative : cfg_.speed_proportional_gain_kp;

  double linear_velocity = state.v + speed_gain * speed_error;

  double abs_max_velocity = cfg_.max_velocity;

  // Limit linear velocity to absolute minimum and maximum limits:
  if (abs_max_velocity < std::abs(linear_velocity))
  {
    linear_velocity = std::copysign(abs_max_velocity, linear_velocity);
  }
  else if (std::abs(linear_velocity) < cfg_.min_linear_x)
  {
    // Use target.reverse to prevent vehicle to drive into wrong direction on trajectory start or direction change:
    linear_velocity = target.reverse ? -cfg_.min_linear_x : cfg_.min_linear_x;
  }
  const double steering_angle = computeSteeringAngle(target.heading_error, target.crosstrack_error, target.reverse,
                                                     target.lookahead, target_speed);

  VelocityCommand velocity_command = computeCommand(linear_velocity, steering_angle);

  if (!(std::isfinite(velocity_command.linear_velocity) && std::isfinite(velocity_command.angular_velocity)
        && std::isfinite(velocity_command.steering_angle)))
  {
    if (!std::isfinite(velocity_command.linear_velocity))
    {
      ROS_ERROR_STREAM(
        "computation of linear velocity yielded non-finite value " << velocity_command.linear_velocity
                                                                   << ", replacing with 0");
      velocity_command.linear_velocity = 0.0;
    }

    if (!std::isfinite(velocity_command.angular_velocity))
    {
      ROS_ERROR_STREAM(
        "computation of angular velocity yielded non-finite value " << velocity_command.angular_velocity
                                                                    << ", replacing with 0");
      velocity_command.angular_velocity = 0.0;
    }

    if (!std::isfinite(velocity_command.steering_angle))
    {
      ROS_ERROR_STREAM(
        "computation of steering angle yielded non-finite value " << velocity_command.steering_angle
                                                                  << ", replacing with 0");
      velocity_command.steering_angle = 0.0;
    }

    ROS_INFO_STREAM("crosstrack error: " << target.crosstrack_error << ", heading error: " << target.heading_error
                                         << ", target speed: " << target_speed);
  }

  publishTrajectory();

  latest_velocity_command_ = velocity_command;

  // check the duration since a non-zero velocity has been received from the robot
  if (cfg_.enable_check_blocked_duration && last_valid_velocity_time_ != ros::Time(0))
  {
    ros::Duration time_diff = ros::Time::now() - last_valid_velocity_time_;

    ROS_DEBUG_STREAM("Time difference since the last valid velocity: " << time_diff.toSec() << "s");
    if (time_diff > ros::Duration(cfg_.max_blocked_duration))
    {
      ROS_ERROR_STREAM("Time difference since the last valid velocity ( " << time_diff.toSec() << "s), is too high!");
      return {Result::Status::NO_COMMAND_POSSIBLE, latest_velocity_command_};
    }
  }

  return {Result::Status::COMMAND_FOUND, latest_velocity_command_};
}

bool StanleyControl::isGoalReached()
{
  if (!path_processing_)
  {
    return goal_reached_;
  }

  const boost::optional<State> optional_state = getState();
  if (!optional_state)
  {
    ROS_ERROR_STREAM("cannot check whether goal is reached without a recent input pose");
    return goal_reached_;
  }
  const State state = optional_state.value();

  const arti_nav_core_msgs::Pose2DWithLimits& goal_pose = current_trajectory_.movements.back().pose;
  bool losing_goal = false;

  const double current_goal_distance = std::hypot(state.x - goal_pose.point.x.value,
                                                  state.y - goal_pose.point.y.value);

  const double current_goal_distance_x = std::abs(state.x - goal_pose.point.x.value);
  const double current_goal_distance_y = std::abs(state.y - goal_pose.point.y.value);

  goal_distance_x_publisher_.publish(makeFloat32(static_cast<std_msgs::Float32::_data_type>(current_goal_distance_x)));
  goal_distance_y_publisher_.publish(makeFloat32(static_cast<std_msgs::Float32::_data_type>(current_goal_distance_y)));
  goal_distance_publisher_.publish(makeFloat32(static_cast<std_msgs::Float32::_data_type>(current_goal_distance)));

  // check if we are loosing the goal - stop driving if this happens
  if ((current_segment_index_ + 2) >= current_trajectory_.movements.size())
  {
    // TODO THIS IS NEVER REACHED!?

    // as long as we are driving closer to the goal we are fine
    ROS_DEBUG_STREAM("goal_distance_: " << goal_distance_ << " current_goal_distance: " << current_goal_distance);
    if (current_goal_distance < goal_distance_)
    {
      goal_distance_ = current_goal_distance;
    }
    else if (std::abs(current_goal_distance - goal_distance_) > cfg_.loosing_goal_distance)
    {
      ROS_WARN_STREAM_THROTTLE(5.0, "missed the goal");
      losing_goal = true;
    }

    // reached_x_current & reached_y_current can be true only if we are at the end of the trajectory
    const bool reached_x_current = losing_goal || xyWithinTolerance(state.x, goal_pose.point.x);
    const bool reached_y_current = losing_goal || xyWithinTolerance(state.y, goal_pose.point.y);

    // if we reached x or y we stick to it. - we could possible lose it in the future
    if (reached_x_current)
    {
      reached_x_ = true;
    }
    if (reached_y_current)
    {
      reached_y_ = true;
    }
  }
  if (!losing_goal)
  {
    const double x_diff = state.x - goal_pose.point.x.value;
    ROS_DEBUG_STREAM(
      "reached_x: " << reached_x_ << ", x_diff: " << x_diff << ", goal_x.lower_limit: " << goal_pose.point.x.lower_limit
                    << " goal_x.upper_limit: " << goal_pose.point.x.upper_limit);
    const double y_diff = state.y - goal_pose.point.y.value;
    ROS_DEBUG_STREAM(
      "reached_y: " << reached_y_ << ", y_diff: " << y_diff << ", goal_y.lower_limit: " << goal_pose.point.y.lower_limit
                    << " goal_y.upper_limit: " << goal_pose.point.y.upper_limit);
  }

  if (cfg_.rotate_on_place && std::isfinite(goal_pose.theta.value))
  {
    if (reached_x_ && reached_y_)
    {
      const double theta_diff = angles::normalize_angle(goal_pose.theta.value - state.yaw);

      goal_distance_theta_publisher_.publish(
        makeFloat32(static_cast<std_msgs::Float32::_data_type>(angles::to_degrees(theta_diff))));

      bool reached_theta_current = thetaWithinTolerance(state.yaw, goal_pose.theta);
      // fixed theta reached - current could be usefull for output
      if (reached_theta_current)
      {
        reached_theta_ = true;
      }

      ROS_DEBUG_STREAM("reached_theta: " << reached_theta_current << ", theta_diff: " << theta_diff);
    }
  }
  else
  {
    reached_theta_ = true;
  }

  if (reached_x_ && reached_y_ && reached_theta_)
  {
    path_processing_ = false;
    goal_reached_ = true;
  }

  return goal_reached_;
}

void StanleyControl::reconfigure(const StanleyControlConfig& new_config)
{
  if (!input_pose_subscriber_ || new_config.input_pose_topic != cfg_.input_pose_topic)
  {
    input_pose_subscriber_
      = private_nh_.subscribe(new_config.input_pose_topic, 1, &StanleyControl::processInputPose, this);
    input_pose_.reset();
  }

  cfg_ = new_config;
  max_steering_angle_ = angles::from_degrees(cfg_.max_steering_angle);
}

void StanleyControl::processInputPose(const nav_msgs::OdometryConstPtr& input_pose)
{
  if (input_pose->header.frame_id == cfg_.target_frame)
  {
    input_pose_ = input_pose;

    // save last time when the robot moved at a nonzero-velocity
    if (cfg_.enable_check_blocked_duration)
    {
      const double v = std::round(input_pose_->twist.twist.linear.x * 1000.0) / 1000.0;

      if (std::abs(v) > 0.01)
      {
        last_valid_velocity_time_ = ros::Time::now();
      }
    }

  }
  else
  {
    ROS_ERROR_STREAM_THROTTLE(10.0, "input pose frame differs from target frame");
  }

}

boost::optional<StanleyControl::State> StanleyControl::getState() const
{
  if (!input_pose_ || (input_pose_->header.stamp + ros::Duration(cfg_.transformation_timeout)) < ros::Time::now())
  {
    return boost::none;
  }

  const double v = std::round(input_pose_->twist.twist.linear.x * 1000.0) / 1000.0;

  return {{
            std::round(input_pose_->pose.pose.position.x * 1000.0) / 1000.0,
            std::round(input_pose_->pose.pose.position.y * 1000.0) / 1000.0,
            std::round(tf::getYaw(input_pose_->pose.pose.orientation) * 1000.0) / 1000.0,
            (std::abs(v) < 0.01) ? 0.0 : v
          }};
}

double StanleyControl::computeSteeringAngle(
  const double heading_error, const double crosstrack_error, const bool reverse, const double lookahead,
  const double linear_velocity) const
{
  double determined_control_gain = cfg_.control_gain;
  ROS_DEBUG_STREAM("linear_velocity: " << linear_velocity);
  ROS_DEBUG_STREAM("determined_control_gain: " << determined_control_gain);

  // atan2(y,x) -> jumpy if x == 0
  const double crosstrack_error_compensation
    = -std::atan2(determined_control_gain * crosstrack_error,
                  std::max(std::abs(linear_velocity), cfg_.cross_track_min_linear_x));

  crosstrack_error_compensation_publisher_.publish(
    makeFloat32(static_cast<std_msgs::Float32::_data_type>(crosstrack_error_compensation)));

  const double steering_angle = heading_error * cfg_.heading_error_gain + crosstrack_error_compensation;

  // Convert lookahead-based steering angle to vehicle steering angle:
  const double vehicle_steering_angle
    = (lookahead != 0.0) ? std::atan2(std::sin(steering_angle) * cfg_.wheel_base / lookahead, std::cos(steering_angle))
                         : (reverse ? -1.0 : 1.0) * steering_angle;

  return std::min(std::max(vehicle_steering_angle, -max_steering_angle_), max_steering_angle_);
}

StanleyControl::TrajectoryPosition StanleyControl::determineTrajectoryPosition(
  const size_t start_segment_index, const double start_parallel_distance, const double x, const double y) const
{
  constexpr double INF = std::numeric_limits<double>::infinity();

  TrajectoryPosition closest_position;
  closest_position.segment_index = start_segment_index;
  closest_position.segment_length = 0.0;
  closest_position.parallel_distance = start_parallel_distance;
  closest_position.normal_distance = INF;
  double min_distance = INF;

  for (size_t i = start_segment_index; (i + 1) < current_trajectory_.movements.size(); ++i)
  {
    const arti_nav_core_msgs::Point2DWithLimits& p0 = current_trajectory_.movements[i].pose.point;
    const arti_nav_core_msgs::Point2DWithLimits& p1 = current_trajectory_.movements[i + 1].pose.point;
    const double segment_dx = p1.x.value - p0.x.value;
    const double segment_dy = p1.y.value - p0.y.value;
    const double dx = x - p0.x.value;
    const double dy = y - p0.y.value;

    TrajectoryPosition position;
    position.segment_index = i;
    position.segment_length = std::hypot(segment_dx, segment_dy);
    // Compute point's parallel and normal distance from segment vector (inner and outer vector products):
    position.parallel_distance = (segment_dx * dx + segment_dy * dy) / position.segment_length;
    position.normal_distance = (segment_dx * dy - segment_dy * dx) / position.segment_length;

    if (0.0 < position.segment_length && position.parallel_distance <= position.segment_length)
    {
      if (0.0 <= position.parallel_distance && std::abs(position.normal_distance) <= min_distance)
      {
        min_distance = std::abs(position.normal_distance);
        closest_position = position;
      }
      else
      {
        // Also check distance to point between segments, which is relevant if we're on the outside of a trajectory
        // "knee":
        const double p0_distance = std::hypot(dx, dy);
        if (p0_distance <= min_distance)
        {
          min_distance = p0_distance;
          closest_position = position;
        }
      }
    }
    else if ((i + 2) >= current_trajectory_.movements.size())
    {
      // If this is the last segment, also check the distance to the last point:
      const double p1_distance = std::hypot(x - p1.x.value, y - p1.y.value);
      if (p1_distance <= min_distance)
      {
        min_distance = p1_distance;
        if (0.0 < position.segment_length)
        {
          closest_position = position;
        }
        else
        {
          const double p1_theta = current_trajectory_.movements[i + 1].pose.theta.value;
          closest_position.segment_index = i;
          closest_position.segment_length = 1.0;
          closest_position.parallel_distance = 1.0;
          closest_position.normal_distance = std::cos(p1_theta) * dy - std::sin(p1_theta) * dx;
        }
      }
    }
  }

  if (closest_position.segment_index == start_segment_index)
  {
    closest_position.parallel_distance = std::max(closest_position.parallel_distance, start_parallel_distance);
  }

  return closest_position;
}

StanleyControl::Target StanleyControl::determineTarget(const State& state) const
{
  const TrajectoryPosition base_position
    = determineTrajectoryPosition(current_segment_index_, current_position_in_segment_, state.x, state.y);
  const arti_nav_core_msgs::Movement2DWithLimits& bpm0 = current_trajectory_.movements[base_position.segment_index];
  const arti_nav_core_msgs::Movement2DWithLimits& bpm1 = current_trajectory_.movements[std::min(base_position
                                                                                                  .segment_index + 1,
                                                                                                current_trajectory_.movements.size()
                                                                                                - 1)];

  double determined_lookahead = cfg_.lookahead;

  ROS_DEBUG_STREAM("state.v: " << state.v);
  ROS_DEBUG_STREAM("determined_lookahead: " << determined_lookahead);

  Target target;
  target.segment_index = base_position.segment_index;
  target.position_in_segment = base_position.parallel_distance;
  target.reverse = cfg_.enable_reverse_driving && (bpm0.twist.x.value < 0.0 || !isForwardSegment(bpm0.pose, bpm1.pose));
  target.lookahead = target.reverse ? -determined_lookahead : determined_lookahead;

  const double lookahead_x = state.x + target.lookahead * std::cos(state.yaw);
  const double lookahead_y = state.y + target.lookahead * std::sin(state.yaw);

  const TrajectoryPosition lookahead_position
    = determineTrajectoryPosition(base_position.segment_index, base_position.parallel_distance,
                                  lookahead_x, lookahead_y);

  target.crosstrack_error = lookahead_position.normal_distance;

  const arti_nav_core_msgs::Movement2DWithLimits& m0 = current_trajectory_.movements[lookahead_position.segment_index];
  const arti_nav_core_msgs::Movement2DWithLimits& m1
    = current_trajectory_.movements[std::min(lookahead_position.segment_index + 1,
                                             current_trajectory_.movements.size() - 1)];
  double f
    = std::min(std::max(lookahead_position.parallel_distance / lookahead_position.segment_length, 0.0), 1.0);
  if (!std::isfinite(f)) // In case of NAN value set f to 1
  {
    f = 1.0;
  }

  target.movement.pose = combinePosesWithLimits(m0.pose, m1.pose, f);

  if (cfg_.target_speed_from_nearest_point)
  {
    const double bpf = std::min(std::max(base_position.parallel_distance / base_position.segment_length, 0.0), 1.0);
    target.movement.twist = combineTwistsWithLimits(bpm0.twist, bpm1.twist, bpf);
  }
  else
  {
    target.movement.twist = combineTwistsWithLimits(m0.twist, m1.twist, f);
  }

  target.heading_error = angles::normalize_angle(target.movement.pose.theta.value - state.yaw);
  return target;
}

StanleyControl::VelocityCommand StanleyControl::computeCommand(
  const double linear_velocity, const double steering_angle) const
{
  VelocityCommand velocity_command{linear_velocity, 0.0, 0.0};

  // Limit steering angle to +-max_steering_angle_:
  velocity_command.steering_angle = std::min(std::max(steering_angle, -max_steering_angle_), max_steering_angle_);

  // Compute angular velocity from steering angle:
  velocity_command.angular_velocity = linear_velocity * std::tan(velocity_command.steering_angle) / cfg_.wheel_base;

  return velocity_command;
}

void StanleyControl::publishTrajectory()
{
  path_publisher_.publish(
    arti_nav_core_utils::convertToPath(current_trajectory_, arti_nav_core_utils::non_finite_values::REPLACE_BY_0));
  path_with_velocity_publisher_.publish(current_trajectory_, current_segment_index_);
}

arti_nav_core_msgs::Pose2DWithLimits StanleyControl::combinePosesWithLimits(
  const arti_nav_core_msgs::Pose2DWithLimits& a, const arti_nav_core_msgs::Pose2DWithLimits& b, double f)
{
  arti_nav_core_msgs::Pose2DWithLimits result;
  result.point.x = combineValuesWithLimits(a.point.x, b.point.x, f);
  result.point.y = combineValuesWithLimits(a.point.y, b.point.y, f);
  result.theta = combineValuesWithLimits(a.theta, b.theta, f, true);
  return result;
}

arti_nav_core_msgs::Twist2DWithLimits StanleyControl::combineTwistsWithLimits(
  const arti_nav_core_msgs::Twist2DWithLimits& a, const arti_nav_core_msgs::Twist2DWithLimits& b, double f)
{
  arti_nav_core_msgs::Twist2DWithLimits result;
  result.x = combineValuesWithLimits(a.x, b.x, f);
  result.y = combineValuesWithLimits(a.y, b.y, f);
  result.theta = combineValuesWithLimits(a.theta, b.theta, f);
  return result;
}

arti_nav_core_msgs::ValueWithLimits StanleyControl::combineValuesWithLimits(
  const arti_nav_core_msgs::ValueWithLimits& a, const arti_nav_core_msgs::ValueWithLimits& b, const double f,
  const bool is_angle)
{
  const double f_1 = 1.0 - f;

  arti_nav_core_msgs::ValueWithLimits result;

  if (std::isfinite(a.value) && std::isfinite(b.value))
  {
    if (is_angle)
    {
      result.value = a.value + angles::normalize_angle(b.value - a.value) * f;
    }
    else
    {
      result.value = a.value * f_1 + b.value * f;
    }
  }
  else if (std::isfinite(a.value))
  {
    result.value = a.value;
  }
  else if (std::isfinite(b.value))
  {
    result.value = b.value;
  }
  else
  {
    result.value = std::numeric_limits<double>::quiet_NaN();
  }

  if (a.has_limits && b.has_limits)
  {
    result.has_limits = true;
    result.upper_limit = a.upper_limit * f_1 + b.upper_limit * f;
    result.lower_limit = a.lower_limit * f_1 + b.lower_limit * f;
  }
  else if (a.has_limits)
  {
    result.has_limits = true;
    result.upper_limit = a.upper_limit;
    result.lower_limit = a.lower_limit;
  }
  else if (b.has_limits)
  {
    result.has_limits = true;
    result.upper_limit = b.upper_limit;
    result.lower_limit = b.lower_limit;
  }

  return result;
}

bool StanleyControl::xyWithinTolerance(
  double current_value, const arti_nav_core_msgs::ValueWithLimits& goal) const
{
  if (!std::isfinite(goal.value))
  {
    return true;
  }

  const double distance = current_value - goal.value;

  if (goal.has_limits)
  {
    return goal.lower_limit <= distance && distance <= goal.upper_limit;
  }
  else
  {
    return -cfg_.default_trans_limit <= distance && distance <= cfg_.default_trans_limit;
  }
}

bool StanleyControl::thetaWithinTolerance(
  double current_value, const arti_nav_core_msgs::ValueWithLimits& goal) const
{
  if (!std::isfinite(goal.value))
  {
    return true;
  }

  const double distance = angles::normalize_angle(current_value - goal.value);

  if (goal.has_limits)
  {
    return goal.lower_limit <= distance && distance <= goal.upper_limit;
  }
  else
  {
    return -cfg_.default_theta_limit <= distance && distance <= cfg_.default_theta_limit;
  }
}

bool StanleyControl::isForwardSegment(
  const arti_nav_core_msgs::Pose2DWithLimits& a, const arti_nav_core_msgs::Pose2DWithLimits& b)
{
  const double angle_between_points = std::atan2(b.point.y.value - a.point.y.value, b.point.x.value - a.point.x.value);

  return std::abs(angles::normalize_angle(b.theta.value - angle_between_points))
         <= (M_PI - std::abs(angles::normalize_angle(angle_between_points - a.theta.value)));
}

std_msgs::Float32 StanleyControl::makeFloat32(std_msgs::Float32::_data_type data)
{
  std_msgs::Float32 result;
  result.data = data;
  return result;
}

void StanleyControl::setVelocity(boost::optional<arti_nav_core_msgs::Trajectory2DWithLimits>& trajectory_tfd) const
{
  ROS_DEBUG_STREAM("SetVelocit for trajectory");
  if (!trajectory_tfd)
  {
    return;
  }

  for (u_int i = 0; i < trajectory_tfd->movements.size(); ++i)
  {
    arti_nav_core_msgs::Movement2DWithLimits& m = trajectory_tfd->movements.at(i);
    arti_nav_core_msgs::Movement2DWithLimits* m_next;
    if (trajectory_tfd->movements.size() == 1)
    {
      return;
    }
    if (i + 1 < trajectory_tfd->movements.size())
    {
      m_next = &trajectory_tfd->movements.at(i + 1);
    }
    else
    {
      m_next = &trajectory_tfd->movements.at(i - 1);
    }
    bool reverse = cfg_.enable_reverse_driving && !isForwardSegment(m.pose, m_next->pose);
    if (!std::isfinite(m.twist.x.value))
    {
      // This means there is no velocity set point or limit, so we default to the maximum velocity:
      m.twist.x.value = reverse ? -cfg_.max_velocity : cfg_.max_velocity;
    }
  }
}

}
