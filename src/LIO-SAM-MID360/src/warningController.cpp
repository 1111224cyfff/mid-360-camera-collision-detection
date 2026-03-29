#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <lio_sam/ControlCommand.h>
#include <lio_sam/WarningState.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace
{
double vectorNorm(const geometry_msgs::Vector3& value)
{
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

float clampRatio(double value)
{
  return static_cast<float>(std::max(0.0, std::min(1.0, value)));
}

std::string actionToText(uint8_t action)
{
  switch (action) {
    case lio_sam::ControlCommand::ACTION_OBSERVE:
      return "observe";
    case lio_sam::ControlCommand::ACTION_LIMIT_MOTION:
      return "limit_motion";
    case lio_sam::ControlCommand::ACTION_HOLD_POSITION:
      return "hold_position";
    case lio_sam::ControlCommand::ACTION_EMERGENCY_STOP:
      return "emergency_stop";
    default:
      return "clear";
  }
}

geometry_msgs::Vector3 zeroVector()
{
  geometry_msgs::Vector3 value;
  value.x = 0.0;
  value.y = 0.0;
  value.z = 0.0;
  return value;
}

}  // namespace

class WarningControllerNode
{
public:
  WarningControllerNode()
  : pnh_("~")
  {
    pnh_.param<std::string>("warning_topic", warning_topic_, std::string("/warning/state"));
    pnh_.param<std::string>("command_topic", command_topic_, std::string("/warning/control_command"));
    pnh_.param<bool>("latch_output", latch_output_, true);
    pnh_.param<double>("warning_speed_limit_ratio", warning_speed_limit_ratio_, 0.35);
    pnh_.param<double>("hold_speed_limit_ratio", hold_speed_limit_ratio_, 0.0);
    pnh_.param<double>("emergency_speed_limit_ratio", emergency_speed_limit_ratio_, 0.0);
    pnh_.param<double>("min_direction_speed", min_direction_speed_, 0.08);
    pnh_.param<bool>("reverse_escape_direction", reverse_escape_direction_, true);
    pnh_.param<bool>("require_operator_ack_for_warning", require_operator_ack_for_warning_, true);
    pnh_.param<bool>("require_operator_ack_for_emergency", require_operator_ack_for_emergency_, true);
    pnh_.param<bool>("hold_on_static_warning", hold_on_static_warning_, false);
    pnh_.param<bool>("hold_on_combined_warning", hold_on_combined_warning_, false);

    sub_warning_ = nh_.subscribe(warning_topic_, 5, &WarningControllerNode::warningCallback, this);
    pub_command_ = nh_.advertise<lio_sam::ControlCommand>(command_topic_, 1, latch_output_);

    ROS_INFO_STREAM("[warning_controller] warning_topic=" << warning_topic_
                    << " command_topic=" << command_topic_
                    << " latch_output=" << (latch_output_ ? "true" : "false"));
  }

private:
  void warningCallback(const lio_sam::WarningStateConstPtr& msg)
  {
    lio_sam::ControlCommand command;
    command.header = msg->header;
    command.warning_level = msg->active_level;
    command.source = msg->source;
    command.reason = msg->reason;
    command.monitored_class_name = msg->monitored_class_name;
    command.monitored_class_id = msg->monitored_class_id;
    command.dynamic_track_id = msg->dynamic_track_id;
    command.speed_limit_ratio = 1.0f;
    command.hold_position = false;
    command.emergency_stop = false;
    command.operator_ack_required = false;
    command.system_ready = msg->system_ready;
    command.recommended_direction = buildRecommendedDirection(*msg);
    command.monitored_pose = msg->monitored_pose;
    command.monitored_velocity = msg->monitored_velocity;

    if (!msg->system_ready) {
      command.action = lio_sam::ControlCommand::ACTION_CLEAR;
      command.action_text = actionToText(command.action);
      command.recommended_direction = zeroVector();
      logIfChanged(command);
      pub_command_.publish(command);
      return;
    }

    switch (msg->active_level) {
      case lio_sam::WarningState::LEVEL_NOTICE:
        command.action = lio_sam::ControlCommand::ACTION_OBSERVE;
        break;

      case lio_sam::WarningState::LEVEL_WARNING:
        if ((hold_on_static_warning_ && msg->source == "static")
            || (hold_on_combined_warning_ && msg->source == "combined")) {
          command.action = lio_sam::ControlCommand::ACTION_HOLD_POSITION;
          command.speed_limit_ratio = clampRatio(hold_speed_limit_ratio_);
          command.hold_position = true;
        } else {
          command.action = lio_sam::ControlCommand::ACTION_LIMIT_MOTION;
          command.speed_limit_ratio = clampRatio(warning_speed_limit_ratio_);
        }
        command.operator_ack_required = require_operator_ack_for_warning_;
        break;

      case lio_sam::WarningState::LEVEL_EMERGENCY:
        command.action = lio_sam::ControlCommand::ACTION_EMERGENCY_STOP;
        command.speed_limit_ratio = clampRatio(emergency_speed_limit_ratio_);
        command.hold_position = true;
        command.emergency_stop = true;
        command.operator_ack_required = require_operator_ack_for_emergency_;
        break;

      case lio_sam::WarningState::LEVEL_NONE:
      default:
        command.action = lio_sam::ControlCommand::ACTION_CLEAR;
        command.recommended_direction = zeroVector();
        break;
    }

    if (command.action == lio_sam::ControlCommand::ACTION_CLEAR
        || command.action == lio_sam::ControlCommand::ACTION_OBSERVE) {
      command.recommended_direction = zeroVector();
    }

    command.action_text = actionToText(command.action);
    logIfChanged(command);
    pub_command_.publish(command);
  }

  geometry_msgs::Vector3 buildRecommendedDirection(const lio_sam::WarningState& warning) const
  {
    geometry_msgs::Vector3 direction = zeroVector();
    if (!warning.system_ready || warning.active_level < lio_sam::WarningState::LEVEL_WARNING) {
      return direction;
    }

    const double speed = vectorNorm(warning.monitored_velocity);
    if (speed < min_direction_speed_) {
      return direction;
    }

    const double sign = reverse_escape_direction_ ? -1.0 : 1.0;
    direction.x = sign * warning.monitored_velocity.x / speed;
    direction.y = sign * warning.monitored_velocity.y / speed;
    direction.z = sign * warning.monitored_velocity.z / speed;
    return direction;
  }

  void logIfChanged(const lio_sam::ControlCommand& command)
  {
    const bool changed = !has_last_command_
      || command.action != last_action_
      || command.warning_level != last_warning_level_
      || command.reason != last_reason_
      || command.source != last_source_
      || std::fabs(command.speed_limit_ratio - last_speed_limit_ratio_) > 1e-3f
      || command.hold_position != last_hold_position_
      || command.emergency_stop != last_emergency_stop_;

    if (changed) {
      ROS_INFO_STREAM("[warning_controller] action=" << command.action_text
                      << " level=" << static_cast<int>(command.warning_level)
                      << " source=" << command.source
                      << " reason=" << command.reason
                      << " speed_limit=" << command.speed_limit_ratio
                      << " hold=" << (command.hold_position ? "true" : "false")
                      << " estop=" << (command.emergency_stop ? "true" : "false"));
    }

    has_last_command_ = true;
    last_action_ = command.action;
    last_warning_level_ = command.warning_level;
    last_reason_ = command.reason;
    last_source_ = command.source;
    last_speed_limit_ratio_ = command.speed_limit_ratio;
    last_hold_position_ = command.hold_position;
    last_emergency_stop_ = command.emergency_stop;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_warning_;
  ros::Publisher pub_command_;

  std::string warning_topic_;
  std::string command_topic_;

  bool latch_output_{true};
  double warning_speed_limit_ratio_{0.35};
  double hold_speed_limit_ratio_{0.0};
  double emergency_speed_limit_ratio_{0.0};
  double min_direction_speed_{0.08};
  bool reverse_escape_direction_{true};
  bool require_operator_ack_for_warning_{true};
  bool require_operator_ack_for_emergency_{true};
  bool hold_on_static_warning_{false};
  bool hold_on_combined_warning_{false};

  bool has_last_command_{false};
  uint8_t last_action_{lio_sam::ControlCommand::ACTION_CLEAR};
  uint8_t last_warning_level_{lio_sam::WarningState::LEVEL_NONE};
  std::string last_reason_;
  std::string last_source_;
  float last_speed_limit_ratio_{1.0f};
  bool last_hold_position_{false};
  bool last_emergency_stop_{false};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "warning_controller");
  WarningControllerNode node;
  ros::spin();
  return 0;
}