#include <ros/ros.h>

#include <XmlRpcValue.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <lio_sam/DynamicTrack.h>
#include <lio_sam/DynamicTrackArray.h>
#include <lio_sam/SegmentedObjectState.h>
#include <lio_sam/SegmentedObjectStateArray.h>
#include <lio_sam/WarningState.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct DynamicRiskMetrics
{
  int track_id{-1};
  double current_clearance{std::numeric_limits<double>::infinity()};
  double predicted_clearance{std::numeric_limits<double>::infinity()};
  double relative_speed{0.0};
  double ttc{std::numeric_limits<double>::infinity()};
  bool head_on{false};
  uint8_t level{lio_sam::WarningState::LEVEL_NONE};
  geometry_msgs::Point track_position;
};

struct ObjectMotionState
{
  bool has_previous{false};
  geometry_msgs::Point last_position;
  ros::Time last_stamp;
  geometry_msgs::Vector3 velocity;
};

enum class MonitoredSourceMode
{
  Visual,
  Cylinder,
  Both
};

enum class ObjectSourceKind
{
  Visual,
  Cylinder
};

struct StaticSupportMetrics
{
  int notice_point_count{0};
  int warning_point_count{0};
  int emergency_point_count{0};
};

struct SourceEvaluation
{
  bool valid{false};
  ObjectSourceKind source_kind{ObjectSourceKind::Visual};
  std::string reason;
  std::string risk_source{"clear"};
  lio_sam::SegmentedObjectState selected_object;
  geometry_msgs::Pose monitored_pose_output;
  geometry_msgs::Vector3 monitored_velocity;
  double static_clearance{std::numeric_limits<double>::infinity()};
  StaticSupportMetrics static_support;
  DynamicRiskMetrics dynamic_risk;
  uint8_t static_level{lio_sam::WarningState::LEVEL_NONE};
  uint8_t dynamic_level{lio_sam::WarningState::LEVEL_NONE};
  uint8_t desired_level{lio_sam::WarningState::LEVEL_NONE};
};

struct CylinderJudgmentState
{
  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 velocity;
};

struct VisualLiftDebugInfo
{
  bool gate_enabled{false};
  bool context_available{false};
  bool passed{false};
  double lift_height_m{std::numeric_limits<double>::quiet_NaN()};
  double threshold_m{0.0};
  double support_z_output{std::numeric_limits<double>::quiet_NaN()};
  double object_bottom_z_output{std::numeric_limits<double>::quiet_NaN()};
};

struct StaticEmergencyDebugInfo
{
  bool clearance_valid{false};
  bool has_emergency_points{false};
  std::string frame_id;
  ros::Time stamp;
  StaticSupportMetrics support_metrics;
  geometry_msgs::Point object_position;
  geometry_msgs::Point nearest_point;
  geometry_msgs::Point centroid;
  double nearest_clearance{std::numeric_limits<double>::infinity()};
  std::vector<geometry_msgs::Point> emergency_points;
};

double clampValue(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

std::string levelToText(uint8_t level)
{
  switch (level) {
    case lio_sam::WarningState::LEVEL_NOTICE:
      return "notice";
    case lio_sam::WarningState::LEVEL_WARNING:
      return "warning";
    case lio_sam::WarningState::LEVEL_EMERGENCY:
      return "emergency";
    default:
      return "clear";
  }
}

std_msgs::ColorRGBA colorForLevel(uint8_t level)
{
  std_msgs::ColorRGBA color;
  color.a = 0.9f;
  if (level == lio_sam::WarningState::LEVEL_NOTICE) {
    color.r = 1.0f;
    color.g = 0.85f;
    color.b = 0.2f;
    return color;
  }
  if (level == lio_sam::WarningState::LEVEL_WARNING) {
    color.r = 1.0f;
    color.g = 0.5f;
    color.b = 0.1f;
    return color;
  }
  if (level == lio_sam::WarningState::LEVEL_EMERGENCY) {
    color.r = 1.0f;
    color.g = 0.15f;
    color.b = 0.15f;
    return color;
  }
  color.r = 0.2f;
  color.g = 0.8f;
  color.b = 0.2f;
  return color;
}

double quietNaN()
{
  return std::numeric_limits<double>::quiet_NaN();
}

geometry_msgs::Vector3 zeroVector()
{
  geometry_msgs::Vector3 value;
  value.x = 0.0;
  value.y = 0.0;
  value.z = 0.0;
  return value;
}

std::string toLowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

std::string sourceKindToText(ObjectSourceKind source_kind)
{
  return source_kind == ObjectSourceKind::Cylinder ? "cylinder" : "visual";
}

std::string prefixReason(ObjectSourceKind source_kind, const std::string& reason)
{
  return sourceKindToText(source_kind) + "/" + reason;
}

double pointToVerticalCylinderDistance(
  const Eigen::Vector3d& point,
  const Eigen::Vector3d& top_center,
  double radius,
  double height)
{
  const double bounded_radius = std::max(0.0, radius);
  const double bounded_height = std::max(1e-3, height);
  const double dx = point.x() - top_center.x();
  const double dy = point.y() - top_center.y();
  const double radial_distance = std::sqrt(dx * dx + dy * dy);
  const double radial_outside = std::max(0.0, radial_distance - bounded_radius);

  const double top_z = top_center.z();
  const double bottom_z = top_z - bounded_height;
  const double vertical_above = std::max(0.0, point.z() - top_z);
  const double vertical_below = std::max(0.0, bottom_z - point.z());
  const double vertical_outside = vertical_above > 0.0 ? vertical_above : vertical_below;

  if (vertical_outside > 0.0) {
    return std::sqrt(radial_outside * radial_outside + vertical_outside * vertical_outside);
  }
  return radial_outside;
}

double pointToVerticalCylinderDistance(
  const pcl::PointXYZI& point,
  const Eigen::Vector3d& top_center,
  double radius,
  double height)
{
  return pointToVerticalCylinderDistance(
    Eigen::Vector3d(point.x, point.y, point.z),
    top_center,
    radius,
    height);
}

double bestClearanceMetric(const SourceEvaluation& evaluation)
{
  double best = std::numeric_limits<double>::infinity();
  if (std::isfinite(evaluation.static_clearance)) {
    best = std::min(best, evaluation.static_clearance);
  }
  if (std::isfinite(evaluation.dynamic_risk.predicted_clearance)) {
    best = std::min(best, evaluation.dynamic_risk.predicted_clearance);
  }
  if (std::isfinite(evaluation.dynamic_risk.current_clearance)) {
    best = std::min(best, evaluation.dynamic_risk.current_clearance);
  }
  return best;
}

double pointToCircleDistance2D(
  const Eigen::Vector2d& point,
  const Eigen::Vector2d& center,
  double radius)
{
  return std::max(0.0, (point - center).norm() - std::max(0.0, radius));
}

}  // namespace

class WarningEvaluatorNode
{
public:
  WarningEvaluatorNode()
  : pnh_("~")
  , tf_listener_(tf_buffer_)
  , static_cloud_(new pcl::PointCloud<pcl::PointXYZI>())
  {
    pnh_.param<std::string>("visual_objects_topic", visual_objects_topic_, std::string("/segment/object_states"));
    pnh_.param<std::string>("segment_objects_topic", visual_objects_topic_, visual_objects_topic_);
    pnh_.param<std::string>("cylinder_objects_topic", cylinder_objects_topic_, std::string("/warning/cylinder_object_states"));
    pnh_.param<std::string>("dynamic_tracks_topic", dynamic_tracks_topic_, std::string("/dynamic_tracker/track_states"));
    pnh_.param<std::string>("static_cloud_topic", static_cloud_topic_, std::string("/lio_sam/mapping/map_local"));
    pnh_.param<std::string>("warning_topic", warning_topic_, std::string("/warning/state"));
    pnh_.param<std::string>("marker_topic", marker_topic_, std::string("/warning/markers"));
    pnh_.param<std::string>("visual_object_marker_topic", visual_object_marker_topic_, std::string("/warning/visual_object_markers"));
    pnh_.param<std::string>("output_frame", output_frame_, std::string("map"));
    pnh_.param<std::string>("monitored_source_mode", monitored_source_mode_raw_, std::string("visual"));
    pnh_.param<std::string>("cylinder_class_name", cylinder_class_name_, std::string("synthetic_cylinder"));

    pnh_.param<double>("object_stale_timeout", object_stale_timeout_sec_, 0.5);
    pnh_.param<double>("visual_hold_timeout", visual_hold_timeout_sec_, 1.0);
    pnh_.param<double>("track_stale_timeout", track_stale_timeout_sec_, 1.0);
    pnh_.param<double>("static_cloud_stale_timeout", static_cloud_stale_timeout_sec_, 1.0);
    pnh_.param<double>("evaluation_rate_hz", evaluation_rate_hz_, 10.0);
    pnh_.param<bool>("enable_static_warning", enable_static_warning_, true);
    pnh_.param<double>("transform_timeout_sec", transform_timeout_sec_, 0.05);
    pnh_.param<double>("min_object_confidence", min_object_confidence_, 0.35);
    pnh_.param<int>("min_object_points", min_object_points_, 25);
    pnh_.param<bool>("allow_tentative_tracks", allow_tentative_tracks_, false);
    pnh_.param<double>("assumed_dynamic_track_radius", assumed_dynamic_track_radius_, 0.4);
    pnh_.param<double>("prediction_horizon", prediction_horizon_sec_, 2.0);
    pnh_.param<double>("dynamic_prediction_sample_dt", dynamic_prediction_sample_dt_, 0.1);
    pnh_.param<double>("min_closing_speed", min_closing_speed_, 0.05);
    pnh_.param<double>("head_on_closing_speed", head_on_closing_speed_, 0.4);
    pnh_.param<double>("head_on_cosine_threshold", head_on_cosine_threshold_, -0.2);
    pnh_.param<double>("object_velocity_alpha", object_velocity_alpha_, 0.6);
    pnh_.param<double>("max_object_velocity_dt", max_object_velocity_dt_sec_, 0.5);
    pnh_.param<double>("downgrade_hold_sec", downgrade_hold_sec_, 1.0);
    pnh_.param<bool>("publish_markers", publish_markers_, true);
    pnh_.param<double>("visual_object_marker_lifetime_sec", visual_object_marker_lifetime_sec_, 0.0);
    pnh_.param<double>("visual_object_envelope_fill_alpha", visual_object_envelope_fill_alpha_, 0.28);
      pnh_.param<bool>("visual_object_envelope_draw_outline", visual_object_envelope_draw_outline_, false);
    pnh_.param<double>("visual_object_envelope_outline_alpha", visual_object_envelope_outline_alpha_, 0.95);
    pnh_.param<double>("visual_object_envelope_outline_width_m", visual_object_envelope_outline_width_m_, 0.08);
    pnh_.param<int>("visual_object_envelope_outline_segments", visual_object_envelope_outline_segments_, 72);
    pnh_.param<double>("visual_object_radius_margin_m", visual_object_radius_margin_m_, 0.15);
    pnh_.param<double>("static_warning_min_lift_height_m", static_warning_min_lift_height_m_, 1.0);

    pnh_.param<double>("static_notice_clearance", static_notice_clearance_, 3.0);
    pnh_.param<double>("static_warning_clearance", static_warning_clearance_, 2.0);
    pnh_.param<double>("static_emergency_clearance", static_emergency_clearance_, 1.0);
    pnh_.param<int>("static_notice_min_points", static_notice_min_points_, 1);
    pnh_.param<int>("static_warning_min_points", static_warning_min_points_, 1);
    pnh_.param<int>("static_emergency_min_points", static_emergency_min_points_, 1);

    pnh_.param<double>("dynamic_notice_clearance", dynamic_notice_clearance_, 2.5);
    pnh_.param<double>("dynamic_warning_clearance", dynamic_warning_clearance_, 1.5);
    pnh_.param<double>("dynamic_emergency_clearance", dynamic_emergency_clearance_, 0.8);

    pnh_.param<double>("notice_ttc", notice_ttc_sec_, 4.0);
    pnh_.param<double>("warning_ttc", warning_ttc_sec_, 2.5);
    pnh_.param<double>("emergency_ttc", emergency_ttc_sec_, 1.0);

    pnh_.param<double>("default_cylinder_radius", default_cylinder_radius_, 2.0);
    pnh_.param<double>("default_cylinder_height", default_cylinder_height_, 3.0);
    pnh_.param<double>("cylinder_ground_contact_tolerance", cylinder_ground_contact_tolerance_, 0.15);
    pnh_.param<double>("cylinder_judgment_z_offset_m", cylinder_judgment_z_offset_m_, -5.0);
    pnh_.param<bool>("ignore_dynamic_tracks_inside_cylinder", ignore_dynamic_tracks_inside_cylinder_, true);
    pnh_.param<double>("inside_cylinder_ignore_epsilon", inside_cylinder_ignore_epsilon_, 1e-3);
    nh_.param<bool>("/use_sim_time", use_sim_time_, false);

    monitored_source_mode_ = parseMonitoredSourceMode(monitored_source_mode_raw_);
    loadMonitoredClassNames();

    if (!visual_objects_topic_.empty()) {
      sub_visual_objects_ = nh_.subscribe(visual_objects_topic_, 3, &WarningEvaluatorNode::visualObjectsCallback, this);
    }
    if (!cylinder_objects_topic_.empty()) {
      sub_cylinder_objects_ = nh_.subscribe(cylinder_objects_topic_, 3, &WarningEvaluatorNode::cylinderObjectsCallback, this);
    }
    sub_tracks_ = nh_.subscribe(dynamic_tracks_topic_, 3, &WarningEvaluatorNode::tracksCallback, this);
    sub_static_cloud_ = nh_.subscribe(static_cloud_topic_, 1, &WarningEvaluatorNode::staticCloudCallback, this);

    pub_warning_ = nh_.advertise<lio_sam::WarningState>(warning_topic_, 1, true);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
    pub_visual_object_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(visual_object_marker_topic_, 1);

    evaluation_timer_ = nh_.createTimer(
      ros::Duration(1.0 / std::max(1.0, evaluation_rate_hz_)),
      &WarningEvaluatorNode::evaluationTimerCallback,
      this);

    ROS_INFO_STREAM("[warning_evaluator] mode=" << monitored_source_mode_raw_
                    << " visual_objects_topic=" << visual_objects_topic_
                    << " cylinder_objects_topic=" << cylinder_objects_topic_
                    << " tracks_topic=" << dynamic_tracks_topic_
                    << " static_cloud_topic=" << static_cloud_topic_
                    << " visual_object_marker_topic=" << visual_object_marker_topic_
                    << " output_frame=" << output_frame_
                    << " evaluation_rate_hz=" << evaluation_rate_hz_
                    << " visual_hold_timeout=" << visual_hold_timeout_sec_);
  }

private:
  MonitoredSourceMode parseMonitoredSourceMode(const std::string& raw_mode) const
  {
    const std::string normalized = toLowerCopy(raw_mode);
    if (normalized == "visual") {
      return MonitoredSourceMode::Visual;
    }
    if (normalized == "cylinder") {
      return MonitoredSourceMode::Cylinder;
    }
    if (normalized == "both") {
      return MonitoredSourceMode::Both;
    }

    ROS_WARN("[warning_evaluator] Unknown monitored_source_mode '%s'; falling back to visual.", raw_mode.c_str());
    return MonitoredSourceMode::Visual;
  }

  void loadMonitoredClassNames()
  {
    XmlRpc::XmlRpcValue class_names_param;
    if (!pnh_.getParam("monitored_class_names", class_names_param)) {
      return;
    }
    if (class_names_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("[warning_evaluator] monitored_class_names must be a YAML list.");
      return;
    }

    monitored_class_names_.clear();
    for (int index = 0; index < class_names_param.size(); ++index) {
      if (class_names_param[index].getType() != XmlRpc::XmlRpcValue::TypeString) {
        continue;
      }
      monitored_class_names_.push_back(static_cast<std::string>(class_names_param[index]));
    }
  }

  void visualObjectsCallback(const lio_sam::SegmentedObjectStateArrayConstPtr& msg)
  {
    if (!msg) {
      return;
    }

    latest_visual_objects_ = *msg;
    latest_visual_receive_time_ = ros::Time::now();
    has_visual_objects_ = true;
  }

  void cylinderObjectsCallback(const lio_sam::SegmentedObjectStateArrayConstPtr& msg)
  {
    latest_cylinder_objects_ = *msg;
    latest_cylinder_receive_time_ = ros::Time::now();
    has_cylinder_objects_ = true;
  }

  void tracksCallback(const lio_sam::DynamicTrackArrayConstPtr& msg)
  {
    latest_tracks_ = *msg;
    latest_tracks_receive_time_ = ros::Time::now();
    has_tracks_ = true;
  }

  void staticCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    latest_static_cloud_msg_ = *msg;
    latest_static_cloud_receive_time_ = ros::Time::now();
    has_static_cloud_ = true;

    pcl::PointCloud<pcl::PointXYZI>::Ptr parsed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *parsed_cloud);
    static_cloud_ = parsed_cloud;
    static_kdtree_.setInputCloud(static_cloud_);
  }

  void evaluationTimerCallback(const ros::TimerEvent&)
  {
    evaluateAndPublish(resolveStamp(currentEvaluationStamp()));
  }

  ros::Time currentEvaluationStamp() const
  {
    const ros::Time now = ros::Time::now();
    if (!now.isZero()) {
      return now;
    }

    ros::Time latest_stamp;
    const auto update_latest = [&latest_stamp](const ros::Time& candidate) {
      if (!candidate.isZero() && (latest_stamp.isZero() || candidate > latest_stamp)) {
        latest_stamp = candidate;
      }
    };
    update_latest(latest_visual_objects_.header.stamp);
    update_latest(latest_cylinder_objects_.header.stamp);
    update_latest(latest_tracks_.header.stamp);
    update_latest(latest_static_cloud_msg_.header.stamp);
    return latest_stamp;
  }

  bool isVisualStateUsable(const ros::Time& evaluation_stamp) const
  {
    if (!shouldEvaluateSource(ObjectSourceKind::Visual) || !has_visual_objects_) {
      return false;
    }

    if (!isInputStale(
          latest_visual_objects_.header.stamp,
          latest_visual_receive_time_,
          object_stale_timeout_sec_,
          evaluation_stamp)) {
      return true;
    }

    return visual_hold_timeout_sec_ > object_stale_timeout_sec_
      && !isInputStale(
        latest_visual_objects_.header.stamp,
        latest_visual_receive_time_,
        visual_hold_timeout_sec_,
        evaluation_stamp);
  }

  bool isVisualMarkerUsable(const ros::Time& evaluation_stamp) const
  {
    if (!has_visual_objects_) {
      return false;
    }

    if (!isInputStale(
          latest_visual_objects_.header.stamp,
          latest_visual_receive_time_,
          object_stale_timeout_sec_,
          evaluation_stamp)) {
      return true;
    }

    return visual_hold_timeout_sec_ > object_stale_timeout_sec_
      && !isInputStale(
        latest_visual_objects_.header.stamp,
        latest_visual_receive_time_,
        visual_hold_timeout_sec_,
        evaluation_stamp);
  }

  void evaluateAndPublish(const ros::Time& evaluation_stamp)
  {
    lio_sam::WarningState out;
    out.header.stamp = evaluation_stamp;
    out.header.frame_id = output_frame_;
    out.active_level = lio_sam::WarningState::LEVEL_NONE;
    out.desired_level = lio_sam::WarningState::LEVEL_NONE;
    out.level_text = levelToText(out.active_level);
    out.source = "system";
    out.reason = "waiting_for_inputs";
    out.monitored_class_id = -1;
    out.dynamic_track_id = -1;
    out.static_clearance = quietNaN();
    out.dynamic_clearance = quietNaN();
    out.relative_speed = quietNaN();
    out.ttc = quietNaN();
    out.predicted_clearance = quietNaN();
    out.head_on = false;
    out.system_ready = false;
    out.monitored_pose.orientation.w = 1.0;
    out.monitored_velocity = zeroVector();

    geometry_msgs::Pose default_pose;
    default_pose.orientation.w = 1.0;
    DynamicRiskMetrics empty_dynamic_risk;
    StaticSupportMetrics empty_static_support;

    publishVisualObjectMarkers(evaluation_stamp);

    const bool visual_ready = isVisualStateUsable(evaluation_stamp);
    const bool cylinder_ready = shouldEvaluateSource(ObjectSourceKind::Cylinder)
      && has_cylinder_objects_
      && !isInputStale(
        latest_cylinder_objects_.header.stamp,
        latest_cylinder_receive_time_,
        object_stale_timeout_sec_,
        evaluation_stamp);
    const bool track_ready = has_tracks_
      && !isInputStale(
        latest_tracks_.header.stamp,
        latest_tracks_receive_time_,
        track_stale_timeout_sec_,
        evaluation_stamp);
    const bool static_ready = has_static_cloud_
      && !isInputStale(
        latest_static_cloud_msg_.header.stamp,
        latest_static_cloud_receive_time_,
        static_cloud_stale_timeout_sec_,
        evaluation_stamp)
      && static_cloud_
      && !static_cloud_->empty();
    const bool static_warning_ready = enable_static_warning_ && static_ready;

    std::vector<SourceEvaluation> evaluations;
    bool any_source_ready = false;

    if (visual_ready) {
      any_source_ready = true;
      evaluations.push_back(evaluateSource(
        ObjectSourceKind::Visual,
        latest_visual_objects_,
        evaluation_stamp,
        static_warning_ready,
        track_ready));
    }
    if (cylinder_ready) {
      any_source_ready = true;
      evaluations.push_back(evaluateSource(
        ObjectSourceKind::Cylinder,
        latest_cylinder_objects_,
        evaluation_stamp,
        static_warning_ready,
        track_ready));
    }

    if (!any_source_ready) {
      publishWarningState(out, publish_markers_, default_pose, zeroVector(), empty_dynamic_risk, empty_static_support);
      resetLevelState();
      return;
    }

    const SourceEvaluation* best_evaluation = nullptr;
    for (const auto& evaluation : evaluations) {
      if (!evaluation.valid) {
        continue;
      }
      if (!best_evaluation || isHigherRisk(evaluation, *best_evaluation)) {
        best_evaluation = &evaluation;
      }
    }

    if (!best_evaluation) {
      for (const auto& evaluation : evaluations) {
        if (!evaluation.reason.empty()) {
          out.reason = evaluation.reason;
          break;
        }
      }
      if (out.reason.empty()) {
        out.reason = "no_monitored_object";
      }
      publishWarningState(out, publish_markers_, default_pose, zeroVector(), empty_dynamic_risk, empty_static_support);
      resetLevelState();
      return;
    }

    out.monitored_pose = best_evaluation->monitored_pose_output;
    out.monitored_velocity = best_evaluation->monitored_velocity;
    out.monitored_class_id = best_evaluation->selected_object.class_id;
    out.monitored_class_name = best_evaluation->selected_object.class_name;
    out.desired_level = best_evaluation->desired_level;
    out.reason = best_evaluation->reason;

    bool has_static_contributor = false;
    bool has_dynamic_contributor = false;
    for (const auto& evaluation : evaluations) {
      if (!evaluation.valid) {
        continue;
      }
      has_static_contributor = has_static_contributor
        || evaluation.static_level > lio_sam::WarningState::LEVEL_NONE;
      has_dynamic_contributor = has_dynamic_contributor
        || evaluation.dynamic_level > lio_sam::WarningState::LEVEL_NONE;
    }

    if (has_static_contributor && has_dynamic_contributor) {
      out.source = "combined";
    } else {
      out.source = best_evaluation->risk_source;
    }

    if (std::isfinite(best_evaluation->static_clearance)) {
      out.static_clearance = static_cast<float>(best_evaluation->static_clearance);
    }
    if (std::isfinite(best_evaluation->dynamic_risk.current_clearance)) {
      out.dynamic_clearance = static_cast<float>(best_evaluation->dynamic_risk.current_clearance);
    }
    if (std::isfinite(best_evaluation->dynamic_risk.relative_speed)) {
      out.relative_speed = static_cast<float>(best_evaluation->dynamic_risk.relative_speed);
    }
    if (std::isfinite(best_evaluation->dynamic_risk.ttc)) {
      out.ttc = static_cast<float>(best_evaluation->dynamic_risk.ttc);
    }
    if (std::isfinite(best_evaluation->dynamic_risk.predicted_clearance)) {
      out.predicted_clearance = static_cast<float>(best_evaluation->dynamic_risk.predicted_clearance);
    }
    out.dynamic_track_id = best_evaluation->dynamic_risk.track_id;
    out.head_on = best_evaluation->dynamic_risk.head_on;
    out.system_ready = true;

    if (!has_static_contributor && !has_dynamic_contributor) {
      out.source = "clear";
      if (out.reason.empty() || out.reason == "visual/clear" || out.reason == "cylinder/clear") {
        out.reason = "clear";
      }
    }

    out.active_level = applyLevelHysteresis(out.desired_level, evaluation_stamp);
    out.level_text = levelToText(out.active_level);
    publishWarningState(
      out,
      publish_markers_,
      out.monitored_pose,
      out.monitored_velocity,
      best_evaluation->dynamic_risk,
      best_evaluation->static_support);
  }

  bool shouldEvaluateSource(ObjectSourceKind source_kind) const
  {
    switch (monitored_source_mode_) {
      case MonitoredSourceMode::Visual:
        return source_kind == ObjectSourceKind::Visual;
      case MonitoredSourceMode::Cylinder:
        return source_kind == ObjectSourceKind::Cylinder;
      case MonitoredSourceMode::Both:
        return true;
      default:
        return source_kind == ObjectSourceKind::Visual;
    }
  }

  SourceEvaluation evaluateSource(
    ObjectSourceKind source_kind,
    const lio_sam::SegmentedObjectStateArray& objects_msg,
    const ros::Time& evaluation_stamp,
    bool static_ready,
    bool track_ready)
  {
    SourceEvaluation evaluation;
    evaluation.source_kind = source_kind;
    evaluation.reason = prefixReason(source_kind, "no_monitored_object");
    evaluation.monitored_pose_output.orientation.w = 1.0;
    evaluation.monitored_velocity = zeroVector();

    bool selected = false;
    if (source_kind == ObjectSourceKind::Visual) {
      selected = selectVisualObject(objects_msg, &evaluation.selected_object);
    } else {
      selected = selectCylinderObject(objects_msg, &evaluation.selected_object);
    }
    if (!selected) {
      return evaluation;
    }

    if (!transformPose(
          evaluation.selected_object.pose,
          objects_msg.header.frame_id,
          output_frame_,
          resolveStamp(objects_msg.header.stamp),
          &evaluation.monitored_pose_output)) {
      evaluation.reason = prefixReason(source_kind, "object_tf_failed");
      return evaluation;
    }

    evaluation.monitored_velocity = estimateObjectVelocity(
      source_kind,
      evaluation.monitored_pose_output.position,
      resolveStamp(objects_msg.header.stamp));

    const double selected_cylinder_radius = cylinderRadius(evaluation.selected_object);

    if (source_kind == ObjectSourceKind::Visual) {
      if (!static_ready) {
        evaluation.reason = prefixReason(source_kind, "missing_static_context");
        return evaluation;
      }

      const double inflated_object_radius = inflatedVisualEnvelopeRadius(evaluation.selected_object);
      if (static_warning_min_lift_height_m_ > 1e-6) {
        VisualLiftDebugInfo lift_debug_info;
        if (!buildVisualLiftDebugInfo(
              evaluation.selected_object,
              evaluation.monitored_pose_output,
              evaluation_stamp,
              &lift_debug_info)) {
          evaluation.valid = true;
          evaluation.reason = prefixReason(source_kind, "missing_lift_context");
          evaluation.risk_source = "clear";
          return evaluation;
        }
        if (!lift_debug_info.passed) {
          evaluation.valid = true;
          evaluation.reason = prefixReason(source_kind, "below_static_lift_gate");
          evaluation.risk_source = "clear";
          return evaluation;
        }
      }

      StaticEmergencyDebugInfo static_debug_info;
      evaluation.static_clearance = computeStaticClearance(
        evaluation.monitored_pose_output,
        inflated_object_radius,
        evaluation_stamp,
        &static_debug_info);
      evaluation.static_support = static_debug_info.support_metrics;
    } else {
      if (!track_ready) {
        evaluation.reason = prefixReason(source_kind, "missing_dynamic_context");
        return evaluation;
      }

      evaluation.dynamic_risk = evaluateCylinderPlanarDynamicRisk(
        evaluation.monitored_pose_output,
        evaluation.monitored_velocity,
        selected_cylinder_radius);
    }

    evaluation.static_level = classifyStaticLevel(evaluation.static_clearance, evaluation.static_support);
    evaluation.dynamic_level = evaluation.dynamic_risk.level;
    evaluation.desired_level = std::max(evaluation.static_level, evaluation.dynamic_level);
    evaluation.risk_source = classifySource(evaluation.static_level, evaluation.dynamic_level);
    evaluation.reason = prefixReason(
      source_kind,
      buildReason(
        evaluation.static_level,
        evaluation.dynamic_level,
        evaluation.static_clearance,
        evaluation.static_support,
        evaluation.dynamic_risk));
    evaluation.valid = true;
    return evaluation;
  }

  bool selectVisualObject(
    const lio_sam::SegmentedObjectStateArray& objects_msg,
    lio_sam::SegmentedObjectState* selected_object) const
  {
    if (!selected_object) {
      return false;
    }

    const lio_sam::SegmentedObjectState* best = nullptr;
    for (const auto& object : objects_msg.objects) {
      if (object.confidence < min_object_confidence_) {
        continue;
      }
      if (static_cast<int>(object.point_count) < min_object_points_) {
        continue;
      }
      if (!monitored_class_names_.empty()) {
        const bool matches = std::find(
          monitored_class_names_.begin(),
          monitored_class_names_.end(),
          object.class_name) != monitored_class_names_.end();
        if (!matches) {
          continue;
        }
      }

      if (!best) {
        best = &object;
        continue;
      }

      if (object.nearest_range < best->nearest_range - 1e-3f) {
        best = &object;
        continue;
      }

      if (std::fabs(object.nearest_range - best->nearest_range) <= 1e-3f && object.confidence > best->confidence) {
        best = &object;
      }
    }

    if (!best) {
      return false;
    }

    *selected_object = *best;
    return true;
  }

  bool selectCylinderObject(
    const lio_sam::SegmentedObjectStateArray& objects_msg,
    lio_sam::SegmentedObjectState* selected_object) const
  {
    if (!selected_object) {
      return false;
    }

    const lio_sam::SegmentedObjectState* best = nullptr;
    for (const auto& object : objects_msg.objects) {
      if (!cylinder_class_name_.empty() && object.class_name != cylinder_class_name_) {
        continue;
      }
      if (!best
          || object.confidence > best->confidence
          || (std::fabs(object.confidence - best->confidence) <= 1e-3f && object.point_count > best->point_count)) {
        best = &object;
      }
    }

    if (!best && !objects_msg.objects.empty()) {
      best = &objects_msg.objects.front();
    }
    if (!best) {
      return false;
    }

    *selected_object = *best;
    return true;
  }

  bool resolveVisualMarkerAnchorPose(
    const ros::Time& evaluation_stamp,
    geometry_msgs::Pose* anchor_pose_output)
  {
    if (!anchor_pose_output) {
      return false;
    }

    lio_sam::SegmentedObjectState visual_object;
    if (!selectVisualObject(latest_visual_objects_, &visual_object)) {
      return false;
    }

    return transformPose(
      visual_object.pose,
      latest_visual_objects_.header.frame_id,
      output_frame_,
      resolveStamp(latest_visual_objects_.header.stamp),
      anchor_pose_output);
  }

  bool transformPose(
    const geometry_msgs::Pose& source_pose,
    const std::string& source_frame,
    const std::string& target_frame,
    const ros::Time& stamp,
    geometry_msgs::Pose* transformed_pose)
  {
    if (!transformed_pose) {
      return false;
    }
    if (source_frame.empty() || target_frame.empty() || source_frame == target_frame) {
      *transformed_pose = source_pose;
      return true;
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = source_frame;
    pose_stamped.pose = source_pose;

    try {
      const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        target_frame,
        source_frame,
        stamp,
        ros::Duration(transform_timeout_sec_));
      tf2::doTransform(pose_stamped, pose_stamped, transform);
      *transformed_pose = pose_stamped.pose;
      return true;
    } catch (const tf2::TransformException&) {
    }

    try {
      const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        target_frame,
        source_frame,
        ros::Time(0),
        ros::Duration(transform_timeout_sec_));
      tf2::doTransform(pose_stamped, pose_stamped, transform);
      *transformed_pose = pose_stamped.pose;
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "[warning_evaluator] Failed to transform pose from %s to %s: %s",
                        source_frame.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  geometry_msgs::Vector3 estimateObjectVelocity(
    ObjectSourceKind source_kind,
    const geometry_msgs::Point& current_position,
    const ros::Time& stamp)
  {
    ObjectMotionState& motion_state = motionStateForSource(source_kind);
    geometry_msgs::Vector3 velocity = motion_state.velocity;

    if (motion_state.has_previous && !motion_state.last_stamp.isZero()) {
      const double dt = (stamp - motion_state.last_stamp).toSec();
      if (dt > 1e-3 && dt <= max_object_velocity_dt_sec_) {
        geometry_msgs::Vector3 raw_velocity;
        raw_velocity.x = (current_position.x - motion_state.last_position.x) / dt;
        raw_velocity.y = (current_position.y - motion_state.last_position.y) / dt;
        raw_velocity.z = (current_position.z - motion_state.last_position.z) / dt;

        velocity.x = object_velocity_alpha_ * motion_state.velocity.x + (1.0 - object_velocity_alpha_) * raw_velocity.x;
        velocity.y = object_velocity_alpha_ * motion_state.velocity.y + (1.0 - object_velocity_alpha_) * raw_velocity.y;
        velocity.z = object_velocity_alpha_ * motion_state.velocity.z + (1.0 - object_velocity_alpha_) * raw_velocity.z;
      }
    }

    motion_state.last_position = current_position;
    motion_state.last_stamp = stamp;
    motion_state.velocity = velocity;
    motion_state.has_previous = true;
    return velocity;
  }

  ObjectMotionState& motionStateForSource(ObjectSourceKind source_kind)
  {
    return source_kind == ObjectSourceKind::Cylinder ? cylinder_motion_state_ : visual_motion_state_;
  }

  CylinderJudgmentState buildCylinderJudgmentState(
    const geometry_msgs::Pose& monitored_pose_output,
    const geometry_msgs::Vector3& monitored_velocity,
    double cylinder_height) const
  {
    CylinderJudgmentState state;
    state.pose = monitored_pose_output;
    state.velocity = monitored_velocity;

    const double bounded_height = std::max(1e-3, cylinder_height);
    const double bottom_z = monitored_pose_output.position.z - bounded_height;
    const double offset_z = monitored_pose_output.position.z + cylinder_judgment_z_offset_m_;
    state.pose.position.z = std::max(offset_z, bottom_z);
    return state;
  }

  double computeStaticClearance(
    const geometry_msgs::Pose& monitored_pose_output,
    double object_radius,
    const ros::Time& evaluation_stamp,
    StaticEmergencyDebugInfo* debug_info = nullptr)
  {
    if (!static_cloud_ || static_cloud_->empty()) {
      return std::numeric_limits<double>::infinity();
    }

    const ros::Time static_cloud_stamp = latest_static_cloud_msg_.header.stamp;
    geometry_msgs::Pose object_pose_cloud = monitored_pose_output;
    if (!transformPose(
          monitored_pose_output,
          output_frame_,
          latest_static_cloud_msg_.header.frame_id,
          static_cloud_stamp,
          &object_pose_cloud)) {
      return std::numeric_limits<double>::infinity();
    }

    if (debug_info) {
      *debug_info = StaticEmergencyDebugInfo();
      debug_info->frame_id = latest_static_cloud_msg_.header.frame_id;
      debug_info->stamp = static_cloud_stamp;
      debug_info->object_position = object_pose_cloud.position;
    }

    pcl::PointXYZI search_point;
    search_point.x = object_pose_cloud.position.x;
    search_point.y = object_pose_cloud.position.y;
    search_point.z = object_pose_cloud.position.z;
    search_point.intensity = 0.0f;

    double nearest_distance_sq = std::numeric_limits<double>::infinity();
    bool found_candidate = false;
    Eigen::Vector3d centroid_accumulator = Eigen::Vector3d::Zero();
    const double bounded_object_radius = std::max(0.0, object_radius);
    const double bounded_object_radius_sq = bounded_object_radius * bounded_object_radius;
    for (const auto& point : static_cloud_->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (static_cast<double>(point.z) > object_pose_cloud.position.z) {
        continue;
      }

      const double dx = static_cast<double>(point.x) - object_pose_cloud.position.x;
      const double dy = static_cast<double>(point.y) - object_pose_cloud.position.y;
      const double dz = static_cast<double>(point.z) - object_pose_cloud.position.z;
      const double distance_sq = dx * dx + dy * dy + dz * dz;
      if (distance_sq <= bounded_object_radius_sq) {
        continue;
      }
      if (distance_sq < nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        if (debug_info) {
          debug_info->nearest_point.x = point.x;
          debug_info->nearest_point.y = point.y;
          debug_info->nearest_point.z = point.z;
        }
      }
      if (debug_info) {
        const double point_clearance = std::sqrt(distance_sq) - bounded_object_radius;
        if (point_clearance <= static_notice_clearance_) {
          ++debug_info->support_metrics.notice_point_count;
        }
        if (point_clearance <= static_warning_clearance_) {
          ++debug_info->support_metrics.warning_point_count;
        }
        if (point_clearance <= static_emergency_clearance_) {
          ++debug_info->support_metrics.emergency_point_count;
          geometry_msgs::Point debug_point;
          debug_point.x = point.x;
          debug_point.y = point.y;
          debug_point.z = point.z;
          debug_info->emergency_points.push_back(debug_point);
          centroid_accumulator += Eigen::Vector3d(point.x, point.y, point.z);
        }
      }
      found_candidate = true;
    }

    if (!found_candidate) {
      return std::numeric_limits<double>::infinity();
    }

    const double clearance = std::sqrt(nearest_distance_sq) - bounded_object_radius;
    if (debug_info) {
      debug_info->clearance_valid = true;
      debug_info->nearest_clearance = clearance;
      debug_info->has_emergency_points = !debug_info->emergency_points.empty();
      if (debug_info->has_emergency_points) {
        const double count = static_cast<double>(debug_info->emergency_points.size());
        debug_info->centroid.x = centroid_accumulator.x() / count;
        debug_info->centroid.y = centroid_accumulator.y() / count;
        debug_info->centroid.z = centroid_accumulator.z() / count;
      }
    }
    return clearance;
  }

  double computeCylinderStaticClearance(
    const geometry_msgs::Pose& monitored_pose_output,
    double cylinder_radius,
    double cylinder_height,
    const ros::Time& evaluation_stamp)
  {
    if (!static_cloud_ || static_cloud_->empty()) {
      return std::numeric_limits<double>::infinity();
    }

    geometry_msgs::Pose cylinder_pose_cloud = monitored_pose_output;
    if (!transformPose(monitored_pose_output, output_frame_, latest_static_cloud_msg_.header.frame_id, evaluation_stamp, &cylinder_pose_cloud)) {
      return std::numeric_limits<double>::infinity();
    }

    const Eigen::Vector3d top_center(
      cylinder_pose_cloud.position.x,
      cylinder_pose_cloud.position.y,
      cylinder_pose_cloud.position.z);
    const double bounded_radius = std::max(0.0, cylinder_radius);
    const double bounded_height = std::max(1e-3, cylinder_height);
    const double bottom_z = top_center.z() - bounded_height;

    double min_clearance = std::numeric_limits<double>::infinity();
    bool found_candidate = false;

    for (const auto& point : static_cloud_->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      const double dx = static_cast<double>(point.x) - top_center.x();
      const double dy = static_cast<double>(point.y) - top_center.y();
      const double radial_distance = std::sqrt(dx * dx + dy * dy);
      if (point.z <= bottom_z + cylinder_ground_contact_tolerance_
          && radial_distance <= bounded_radius + cylinder_ground_contact_tolerance_) {
        continue;
      }

      const double clearance = pointToVerticalCylinderDistance(point, top_center, bounded_radius, bounded_height);
      min_clearance = std::min(min_clearance, clearance);
      found_candidate = true;
    }

    if (!found_candidate) {
      return std::numeric_limits<double>::infinity();
    }
    return min_clearance;
  }

  DynamicRiskMetrics evaluateSphereDynamicRisk(
    const geometry_msgs::Pose& monitored_pose_output,
    const geometry_msgs::Vector3& monitored_velocity,
    double object_radius) const
  {
    DynamicRiskMetrics best_risk;
    const Eigen::Vector3d object_position(
      monitored_pose_output.position.x,
      monitored_pose_output.position.y,
      monitored_pose_output.position.z);
    const Eigen::Vector3d object_velocity_vector(
      monitored_velocity.x,
      monitored_velocity.y,
      monitored_velocity.z);
    const double object_speed = object_velocity_vector.norm();

    for (const auto& track : latest_tracks_.tracks) {
      if (!allow_tentative_tracks_ && track.state != lio_sam::DynamicTrack::STATE_CONFIRMED) {
        continue;
      }

      const Eigen::Vector3d track_position(
        track.pose.position.x,
        track.pose.position.y,
        track.pose.position.z);

      const Eigen::Vector3d track_velocity(track.velocity.x, track.velocity.y, track.velocity.z);

      const Eigen::Vector3d relative_position = track_position - object_position;
      const Eigen::Vector3d relative_velocity = track_velocity - object_velocity_vector;

      const double distance = relative_position.norm();
      const double clearance = std::max(0.0, distance - object_radius - assumed_dynamic_track_radius_);

      double relative_speed = 0.0;
      if (distance > 1e-4) {
        relative_speed = -relative_velocity.dot(relative_position / distance);
      }

      double ttc = std::numeric_limits<double>::infinity();
      if (clearance <= 0.0) {
        ttc = 0.0;
      } else if (relative_speed > min_closing_speed_) {
        ttc = clearance / relative_speed;
      }

      double predicted_clearance = clearance;
      const double relative_speed_sq = relative_velocity.squaredNorm();
      if (relative_speed_sq > 1e-4) {
        const double tau = clampValue(
          -relative_position.dot(relative_velocity) / relative_speed_sq,
          0.0,
          prediction_horizon_sec_);
        predicted_clearance = std::max(
          0.0,
          (relative_position + relative_velocity * tau).norm() - object_radius - assumed_dynamic_track_radius_);
      }

      bool head_on = false;
      const double track_speed = track_velocity.norm();
      if (object_speed > min_closing_speed_ && track_speed > min_closing_speed_) {
        const double cosine = object_velocity_vector.normalized().dot(track_velocity.normalized());
        head_on = relative_speed > head_on_closing_speed_ && cosine < head_on_cosine_threshold_;
      } else {
        head_on = relative_speed > head_on_closing_speed_ && predicted_clearance < dynamic_warning_clearance_;
      }

      uint8_t level = classifyDynamicLevel(clearance, predicted_clearance, ttc);
      if (head_on && level > lio_sam::WarningState::LEVEL_NONE) {
        level = std::min<uint8_t>(lio_sam::WarningState::LEVEL_EMERGENCY, level + 1);
      }

      const bool better_level = level > best_risk.level;
      const bool better_gap = level == best_risk.level && predicted_clearance < best_risk.predicted_clearance;
      const bool better_ttc = level == best_risk.level
        && std::fabs(predicted_clearance - best_risk.predicted_clearance) < 1e-3
        && ttc < best_risk.ttc;
      if (!better_level && !better_gap && !better_ttc) {
        continue;
      }

      best_risk.track_id = track.track_id;
      best_risk.current_clearance = clearance;
      best_risk.predicted_clearance = predicted_clearance;
      best_risk.relative_speed = relative_speed;
      best_risk.ttc = ttc;
      best_risk.head_on = head_on;
      best_risk.level = level;
      best_risk.track_position = track.pose.position;
    }

    return best_risk;
  }

  DynamicRiskMetrics evaluateCylinderPlanarDynamicRisk(
    const geometry_msgs::Pose& monitored_pose_output,
    const geometry_msgs::Vector3& monitored_velocity,
    double cylinder_radius) const
  {
    DynamicRiskMetrics best_risk;

    const Eigen::Vector2d object_position_xy(
      monitored_pose_output.position.x,
      monitored_pose_output.position.y);
    const Eigen::Vector2d object_velocity_xy(
      monitored_velocity.x,
      monitored_velocity.y);
    const double object_speed = object_velocity_xy.norm();
    const double bounded_radius = std::max(0.0, cylinder_radius);
    const double sample_dt = std::max(0.02, dynamic_prediction_sample_dt_);

    for (const auto& track : latest_tracks_.tracks) {
      if (!allow_tentative_tracks_ && track.state != lio_sam::DynamicTrack::STATE_CONFIRMED) {
        continue;
      }

      const Eigen::Vector2d track_position_xy(
        track.pose.position.x,
        track.pose.position.y);
      if (ignore_dynamic_tracks_inside_cylinder_) {
        const double distance_to_cylinder_surface = pointToCircleDistance2D(
          track_position_xy,
          object_position_xy,
          bounded_radius);
        if (distance_to_cylinder_surface <= inside_cylinder_ignore_epsilon_) {
          continue;
        }
      }

      const Eigen::Vector2d track_velocity_xy(track.velocity.x, track.velocity.y);

      auto clearanceAtTime = [&](double time_sec) {
        const Eigen::Vector2d object_at_time = object_position_xy + object_velocity_xy * time_sec;
        const Eigen::Vector2d track_at_time = track_position_xy + track_velocity_xy * time_sec;
        return std::max(0.0, (track_at_time - object_at_time).norm() - bounded_radius - assumed_dynamic_track_radius_);
      };

      const double current_clearance = clearanceAtTime(0.0);
      double predicted_clearance = current_clearance;
      double ttc = current_clearance <= 0.0 ? 0.0 : std::numeric_limits<double>::infinity();

      for (double time_sec = sample_dt; time_sec <= prediction_horizon_sec_ + 1e-6; time_sec += sample_dt) {
        const double bounded_time = std::min(time_sec, prediction_horizon_sec_);
        const double clearance = clearanceAtTime(bounded_time);
        predicted_clearance = std::min(predicted_clearance, clearance);
        if (!std::isfinite(ttc) && clearance <= 0.0) {
          ttc = bounded_time;
        }
      }

      const double speed_dt = std::max(1e-3, std::min(prediction_horizon_sec_, sample_dt));
      const double relative_speed = (current_clearance - clearanceAtTime(speed_dt)) / speed_dt;

      bool head_on = false;
      const double track_speed = track_velocity_xy.norm();
      if (object_speed > min_closing_speed_ && track_speed > min_closing_speed_) {
        const double cosine = object_velocity_xy.normalized().dot(track_velocity_xy.normalized());
        head_on = relative_speed > head_on_closing_speed_ && cosine < head_on_cosine_threshold_;
      } else {
        head_on = relative_speed > head_on_closing_speed_ && predicted_clearance < dynamic_warning_clearance_;
      }

      uint8_t level = classifyDynamicLevel(current_clearance, predicted_clearance, ttc);
      if (head_on && level > lio_sam::WarningState::LEVEL_NONE) {
        level = std::min<uint8_t>(lio_sam::WarningState::LEVEL_EMERGENCY, level + 1);
      }

      const bool better_level = level > best_risk.level;
      const bool better_gap = level == best_risk.level && predicted_clearance < best_risk.predicted_clearance;
      const bool better_ttc = level == best_risk.level
        && std::fabs(predicted_clearance - best_risk.predicted_clearance) < 1e-3
        && ttc < best_risk.ttc;
      if (!better_level && !better_gap && !better_ttc) {
        continue;
      }

      best_risk.track_id = track.track_id;
      best_risk.current_clearance = current_clearance;
      best_risk.predicted_clearance = predicted_clearance;
      best_risk.relative_speed = relative_speed;
      best_risk.ttc = ttc;
      best_risk.head_on = head_on;
      best_risk.level = level;
      best_risk.track_position = track.pose.position;
    }

    return best_risk;
  }

  double cylinderRadius(const lio_sam::SegmentedObjectState& object) const
  {
    if (object.footprint_radius > 1e-3f) {
      return static_cast<double>(object.footprint_radius);
    }
    const double from_size = 0.5 * std::max(static_cast<double>(object.size.x), static_cast<double>(object.size.y));
    if (from_size > 1e-3) {
      return from_size;
    }
    if (object.bounding_radius > 1e-3f) {
      return std::min(static_cast<double>(object.bounding_radius), default_cylinder_radius_);
    }
    return default_cylinder_radius_;
  }

  double cylinderHeight(const lio_sam::SegmentedObjectState& object) const
  {
    if (object.size.z > 1e-3f) {
      return static_cast<double>(object.size.z);
    }
    return default_cylinder_height_;
  }

  int staticSupportPointCountForLevel(uint8_t level, const StaticSupportMetrics& static_support) const
  {
    switch (level) {
      case lio_sam::WarningState::LEVEL_EMERGENCY:
        return static_support.emergency_point_count;
      case lio_sam::WarningState::LEVEL_WARNING:
        return static_support.warning_point_count;
      case lio_sam::WarningState::LEVEL_NOTICE:
        return static_support.notice_point_count;
      default:
        return 0;
    }
  }

  int staticSupportMinPointsForLevel(uint8_t level) const
  {
    switch (level) {
      case lio_sam::WarningState::LEVEL_EMERGENCY:
        return std::max(1, static_emergency_min_points_);
      case lio_sam::WarningState::LEVEL_WARNING:
        return std::max(1, static_warning_min_points_);
      case lio_sam::WarningState::LEVEL_NOTICE:
        return std::max(1, static_notice_min_points_);
      default:
        return 0;
    }
  }

  bool hasSufficientStaticSupport(uint8_t level, const StaticSupportMetrics& static_support) const
  {
    return staticSupportPointCountForLevel(level, static_support) >= staticSupportMinPointsForLevel(level);
  }

  uint8_t classifyStaticLevel(double clearance, const StaticSupportMetrics& static_support) const
  {
    if (!std::isfinite(clearance)) {
      return lio_sam::WarningState::LEVEL_NONE;
    }
    if (clearance <= static_emergency_clearance_
        && hasSufficientStaticSupport(lio_sam::WarningState::LEVEL_EMERGENCY, static_support)) {
      return lio_sam::WarningState::LEVEL_EMERGENCY;
    }
    if (clearance <= static_warning_clearance_
        && hasSufficientStaticSupport(lio_sam::WarningState::LEVEL_WARNING, static_support)) {
      return lio_sam::WarningState::LEVEL_WARNING;
    }
    if (clearance <= static_notice_clearance_
        && hasSufficientStaticSupport(lio_sam::WarningState::LEVEL_NOTICE, static_support)) {
      return lio_sam::WarningState::LEVEL_NOTICE;
    }
    return lio_sam::WarningState::LEVEL_NONE;
  }

  uint8_t classifyDynamicLevel(double current_clearance, double predicted_clearance, double ttc) const
  {
    if ((std::isfinite(current_clearance) && current_clearance <= dynamic_emergency_clearance_)
        || (std::isfinite(predicted_clearance) && predicted_clearance <= dynamic_emergency_clearance_)
        || (std::isfinite(ttc) && ttc <= emergency_ttc_sec_)) {
      return lio_sam::WarningState::LEVEL_EMERGENCY;
    }
    if ((std::isfinite(current_clearance) && current_clearance <= dynamic_warning_clearance_)
        || (std::isfinite(predicted_clearance) && predicted_clearance <= dynamic_warning_clearance_)
        || (std::isfinite(ttc) && ttc <= warning_ttc_sec_)) {
      return lio_sam::WarningState::LEVEL_WARNING;
    }
    if ((std::isfinite(current_clearance) && current_clearance <= dynamic_notice_clearance_)
        || (std::isfinite(predicted_clearance) && predicted_clearance <= dynamic_notice_clearance_)
        || (std::isfinite(ttc) && ttc <= notice_ttc_sec_)) {
      return lio_sam::WarningState::LEVEL_NOTICE;
    }
    return lio_sam::WarningState::LEVEL_NONE;
  }

  uint8_t applyLevelHysteresis(uint8_t desired_level, const ros::Time& stamp)
  {
    if (desired_level > active_level_) {
      active_level_ = desired_level;
      pending_lower_level_ = desired_level;
      pending_lower_since_ = stamp;
      return active_level_;
    }

    if (desired_level == active_level_) {
      pending_lower_level_ = desired_level;
      pending_lower_since_ = stamp;
      return active_level_;
    }

    if (pending_lower_level_ != desired_level) {
      pending_lower_level_ = desired_level;
      pending_lower_since_ = stamp;
      return active_level_;
    }

    if ((stamp - pending_lower_since_).toSec() >= downgrade_hold_sec_) {
      active_level_ = desired_level;
    }
    return active_level_;
  }

  std::string classifySource(uint8_t static_level, uint8_t dynamic_level) const
  {
    if (static_level > lio_sam::WarningState::LEVEL_NONE && dynamic_level > lio_sam::WarningState::LEVEL_NONE) {
      return "combined";
    }
    if (dynamic_level > lio_sam::WarningState::LEVEL_NONE) {
      return "dynamic";
    }
    if (static_level > lio_sam::WarningState::LEVEL_NONE) {
      return "static";
    }
    return "clear";
  }

  std::string buildReason(
    uint8_t static_level,
    uint8_t dynamic_level,
    double static_clearance,
    const StaticSupportMetrics& static_support,
    const DynamicRiskMetrics& dynamic_risk) const
  {
    if (static_level == lio_sam::WarningState::LEVEL_NONE && dynamic_level == lio_sam::WarningState::LEVEL_NONE) {
      if (std::isfinite(static_clearance)
          && static_clearance <= static_notice_clearance_
          && staticSupportPointCountForLevel(lio_sam::WarningState::LEVEL_NOTICE, static_support) > 0
          && !hasSufficientStaticSupport(lio_sam::WarningState::LEVEL_NOTICE, static_support)) {
        return "static_sparse_points";
      }
      return "clear";
    }
    if (dynamic_level > static_level) {
      if (dynamic_risk.head_on) {
        return "dynamic_head_on_upgrade";
      }
      if (std::isfinite(dynamic_risk.ttc) && dynamic_risk.ttc <= emergency_ttc_sec_) {
        return "dynamic_ttc_emergency";
      }
      return "dynamic_predicted_clearance";
    }
    if (static_level > lio_sam::WarningState::LEVEL_NONE && std::isfinite(static_clearance)) {
      return "static_clearance_threshold";
    }
    return "dynamic_current_clearance";
  }

  bool isHigherRisk(const SourceEvaluation& lhs, const SourceEvaluation& rhs) const
  {
    if (lhs.desired_level != rhs.desired_level) {
      return lhs.desired_level > rhs.desired_level;
    }

    const double lhs_clearance = bestClearanceMetric(lhs);
    const double rhs_clearance = bestClearanceMetric(rhs);
    if (std::isfinite(lhs_clearance) && std::isfinite(rhs_clearance) && std::fabs(lhs_clearance - rhs_clearance) > 1e-3) {
      return lhs_clearance < rhs_clearance;
    }
    if (std::isfinite(lhs_clearance) != std::isfinite(rhs_clearance)) {
      return std::isfinite(lhs_clearance);
    }

    if (std::isfinite(lhs.dynamic_risk.ttc) && std::isfinite(rhs.dynamic_risk.ttc)
        && std::fabs(lhs.dynamic_risk.ttc - rhs.dynamic_risk.ttc) > 1e-3) {
      return lhs.dynamic_risk.ttc < rhs.dynamic_risk.ttc;
    }
    if (std::isfinite(lhs.dynamic_risk.ttc) != std::isfinite(rhs.dynamic_risk.ttc)) {
      return std::isfinite(lhs.dynamic_risk.ttc);
    }

    return false;
  }

  double visualEnvelopeRadius(const lio_sam::SegmentedObjectState& object) const
  {
    if (object.bounding_radius > 1e-3f) {
      return static_cast<double>(object.bounding_radius);
    }

    if (object.footprint_radius > 1e-3f) {
      return static_cast<double>(object.footprint_radius);
    }

    const double from_size = 0.5 * std::hypot(
      static_cast<double>(object.size.x),
      static_cast<double>(object.size.y));
    if (from_size > 1e-3) {
      return from_size;
    }

    return 0.5;
  }

  double inflatedVisualEnvelopeRadius(const lio_sam::SegmentedObjectState& object) const
  {
    return visualEnvelopeRadius(object) + std::max(0.0, visual_object_radius_margin_m_);
  }

  double visualVerticalHalfExtent(
    const lio_sam::SegmentedObjectState& object,
    double fallback_radius) const
  {
    if (object.size.z > 1e-3f) {
      return 0.5 * static_cast<double>(object.size.z);
    }
    return std::max(0.05, fallback_radius);
  }

  bool buildVisualLiftDebugInfo(
    const lio_sam::SegmentedObjectState& object,
    const geometry_msgs::Pose& monitored_pose_output,
    const ros::Time& evaluation_stamp,
    VisualLiftDebugInfo* debug_info)
  {
    if (!debug_info) {
      return false;
    }

    *debug_info = VisualLiftDebugInfo();
    debug_info->gate_enabled = static_warning_min_lift_height_m_ > 1e-6;
    debug_info->threshold_m = static_warning_min_lift_height_m_;
    if (!has_cylinder_objects_
        || isInputStale(
          latest_cylinder_objects_.header.stamp,
          latest_cylinder_receive_time_,
          object_stale_timeout_sec_,
          evaluation_stamp)) {
      return false;
    }

    lio_sam::SegmentedObjectState cylinder_object;
    if (!selectCylinderObject(latest_cylinder_objects_, &cylinder_object)) {
      return false;
    }

    geometry_msgs::Pose cylinder_pose_output;
    if (!transformPose(
          cylinder_object.pose,
          latest_cylinder_objects_.header.frame_id,
          output_frame_,
          resolveStamp(latest_cylinder_objects_.header.stamp),
          &cylinder_pose_output)) {
      return false;
    }

    const double support_z = cylinder_pose_output.position.z - cylinderHeight(cylinder_object);
    const double object_bottom_z = monitored_pose_output.position.z
      - visualVerticalHalfExtent(object, inflatedVisualEnvelopeRadius(object));
    debug_info->support_z_output = support_z;
    debug_info->object_bottom_z_output = object_bottom_z;
    debug_info->lift_height_m = object_bottom_z - support_z;
    debug_info->context_available = std::isfinite(debug_info->lift_height_m);
    debug_info->passed = !debug_info->gate_enabled || debug_info->lift_height_m >= static_warning_min_lift_height_m_;
    return debug_info->context_available;
  }

  void publishVisualObjectMarkers(const ros::Time& evaluation_stamp)
  {
    if (!publish_markers_) {
      return;
    }

    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker clear_marker;
    clear_marker.header.stamp = evaluation_stamp;
    clear_marker.header.frame_id = output_frame_;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    if (!isVisualMarkerUsable(evaluation_stamp)) {
      pub_visual_object_markers_.publish(markers);
      return;
    }

    lio_sam::SegmentedObjectState selected_object;
    if (!selectVisualObject(latest_visual_objects_, &selected_object)) {
      pub_visual_object_markers_.publish(markers);
      return;
    }

    geometry_msgs::Pose transformed_pose;
    if (!resolveVisualMarkerAnchorPose(evaluation_stamp, &transformed_pose)) {
      pub_visual_object_markers_.publish(markers);
      return;
    }

    const double radius = std::max(0.05, inflatedVisualEnvelopeRadius(selected_object));
    VisualLiftDebugInfo lift_debug_info;
    const bool has_lift_debug = buildVisualLiftDebugInfo(
      selected_object,
      transformed_pose,
      evaluation_stamp,
      &lift_debug_info);
    StaticEmergencyDebugInfo static_debug_info;
    const bool has_static_debug = has_static_cloud_
      && !isInputStale(
        latest_static_cloud_msg_.header.stamp,
        latest_static_cloud_receive_time_,
        static_cloud_stale_timeout_sec_,
        evaluation_stamp)
      && computeStaticClearance(
        transformed_pose,
        radius,
        evaluation_stamp,
        &static_debug_info) <= static_emergency_clearance_;

    visualization_msgs::Marker envelope_sphere;
    envelope_sphere.header.stamp = evaluation_stamp;
    envelope_sphere.header.frame_id = output_frame_;
    envelope_sphere.ns = "visual_object_envelope_sphere";
    envelope_sphere.id = 1;
    envelope_sphere.type = visualization_msgs::Marker::SPHERE;
    envelope_sphere.action = visualization_msgs::Marker::ADD;
    envelope_sphere.pose = transformed_pose;
    envelope_sphere.scale.x = 2.0 * radius;
    envelope_sphere.scale.y = 2.0 * radius;
    envelope_sphere.scale.z = 2.0 * radius;
    envelope_sphere.color.r = 0.06f;
    envelope_sphere.color.g = 0.62f;
    envelope_sphere.color.b = 0.96f;
    envelope_sphere.color.a = static_cast<float>(clampValue(visual_object_envelope_fill_alpha_, 0.0, 1.0));
    envelope_sphere.lifetime = ros::Duration(visual_object_marker_lifetime_sec_);
    markers.markers.push_back(envelope_sphere);

    if (visual_object_envelope_draw_outline_) {
      const int outline_segments = std::max(24, visual_object_envelope_outline_segments_);
      const double outline_width = std::max(0.01, visual_object_envelope_outline_width_m_);
      const float outline_alpha = static_cast<float>(clampValue(visual_object_envelope_outline_alpha_, 0.0, 1.0));
      const double kTwoPi = 2.0 * std::acos(-1.0);
      auto append_ring_marker = [&](int id, const std::string& ns, int plane) {
        visualization_msgs::Marker ring;
        ring.header.stamp = evaluation_stamp;
        ring.header.frame_id = output_frame_;
        ring.ns = ns;
        ring.id = id;
        ring.type = visualization_msgs::Marker::LINE_STRIP;
        ring.action = visualization_msgs::Marker::ADD;
        ring.pose.position = transformed_pose.position;
        ring.pose.orientation.w = 1.0;
        ring.scale.x = outline_width;
        ring.color.r = 0.1f;
        ring.color.g = 0.92f;
        ring.color.b = 1.0f;
        ring.color.a = outline_alpha;
        ring.lifetime = ros::Duration(visual_object_marker_lifetime_sec_);
        ring.points.reserve(static_cast<size_t>(outline_segments) + 1U);

        for (int index = 0; index <= outline_segments; ++index) {
          const double angle = kTwoPi * static_cast<double>(index) / static_cast<double>(outline_segments);
          geometry_msgs::Point point;
          if (plane == 0) {
            point.x = radius * std::cos(angle);
            point.y = radius * std::sin(angle);
            point.z = 0.0;
          } else if (plane == 1) {
            point.x = radius * std::cos(angle);
            point.y = 0.0;
            point.z = radius * std::sin(angle);
          } else {
            point.x = 0.0;
            point.y = radius * std::cos(angle);
            point.z = radius * std::sin(angle);
          }
          ring.points.push_back(point);
        }

        markers.markers.push_back(ring);
      };
      append_ring_marker(4, "visual_object_envelope_outline", 0);
      append_ring_marker(5, "visual_object_envelope_outline", 1);
      append_ring_marker(6, "visual_object_envelope_outline", 2);
    }

    visualization_msgs::Marker center_marker;
    center_marker.header.stamp = evaluation_stamp;
    center_marker.header.frame_id = output_frame_;
    center_marker.ns = "visual_object_envelope_center";
    center_marker.id = 3;
    center_marker.type = visualization_msgs::Marker::SPHERE;
    center_marker.action = visualization_msgs::Marker::ADD;
    center_marker.pose = transformed_pose;
    center_marker.scale.x = 0.22;
    center_marker.scale.y = 0.22;
    center_marker.scale.z = 0.22;
    center_marker.color.r = 0.02f;
    center_marker.color.g = 1.0f;
    center_marker.color.b = 1.0f;
    center_marker.color.a = 0.95f;
    center_marker.lifetime = ros::Duration(visual_object_marker_lifetime_sec_);
    markers.markers.push_back(center_marker);

    visualization_msgs::Marker label_marker;
    label_marker.header.stamp = evaluation_stamp;
    label_marker.header.frame_id = output_frame_;
    label_marker.ns = "visual_object_envelope_label";
    label_marker.id = 2;
    label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label_marker.action = visualization_msgs::Marker::ADD;
    label_marker.pose.orientation.w = 1.0;
    label_marker.pose.position = transformed_pose.position;
    label_marker.pose.position.z += radius + 0.25;
    label_marker.scale.z = 0.22;
    label_marker.color.r = 0.95f;
    label_marker.color.g = 0.98f;
    label_marker.color.b = 1.0f;
    label_marker.color.a = 0.95f;
    std::ostringstream label_stream;
    label_stream << std::fixed << std::setprecision(2);
    label_stream << selected_object.class_name
                 << " cls=" << selected_object.class_id
                 << " r=" << radius;
    if (static_debug_info.clearance_valid) {
      label_stream << "\nstatic_pts n/w/e="
                   << static_debug_info.support_metrics.notice_point_count << "/"
                   << static_debug_info.support_metrics.warning_point_count << "/"
                   << static_debug_info.support_metrics.emergency_point_count;
    }
    if (lift_debug_info.gate_enabled) {
      label_stream << "\nlift_gate=";
      if (has_lift_debug && lift_debug_info.context_available) {
        label_stream << (lift_debug_info.passed ? "pass" : "block")
                     << " " << lift_debug_info.lift_height_m
                     << "/" << lift_debug_info.threshold_m << "m";
      } else {
        label_stream << "missing";
      }
    }
    label_marker.text = label_stream.str();
    label_marker.lifetime = ros::Duration(visual_object_marker_lifetime_sec_);
    markers.markers.push_back(label_marker);

    if (lift_debug_info.gate_enabled) {
      visualization_msgs::Marker lift_text_marker;
      lift_text_marker.header.stamp = evaluation_stamp;
      lift_text_marker.header.frame_id = output_frame_;
      lift_text_marker.ns = "visual_object_lift_gate_text";
      lift_text_marker.id = 7;
      lift_text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      lift_text_marker.action = visualization_msgs::Marker::ADD;
      lift_text_marker.pose.orientation.w = 1.0;
      lift_text_marker.pose.position = transformed_pose.position;
      lift_text_marker.pose.position.z += radius + 0.65;
      lift_text_marker.scale.z = 0.18;
      lift_text_marker.lifetime = ros::Duration(visual_object_marker_lifetime_sec_);
      if (has_lift_debug && lift_debug_info.context_available) {
        lift_text_marker.color.r = lift_debug_info.passed ? 0.2f : 1.0f;
        lift_text_marker.color.g = lift_debug_info.passed ? 0.95f : 0.3f;
        lift_text_marker.color.b = 0.25f;
        lift_text_marker.color.a = 0.98f;
        std::ostringstream lift_text_stream;
        lift_text_stream << std::fixed << std::setprecision(2)
                         << "static_lift "
                         << (lift_debug_info.passed ? "PASS" : "BLOCK")
                         << " " << lift_debug_info.lift_height_m
                         << "/" << lift_debug_info.threshold_m << "m";
        lift_text_marker.text = lift_text_stream.str();
      } else {
        lift_text_marker.color.r = 1.0f;
        lift_text_marker.color.g = 0.95f;
        lift_text_marker.color.b = 0.2f;
        lift_text_marker.color.a = 0.98f;
        lift_text_marker.text = "static_lift MISSING";
      }
      markers.markers.push_back(lift_text_marker);
    }

    if (has_lift_debug && lift_debug_info.context_available) {
      visualization_msgs::Marker lift_line_marker;
      lift_line_marker.header.stamp = evaluation_stamp;
      lift_line_marker.header.frame_id = output_frame_;
      lift_line_marker.ns = "visual_object_lift_gate_line";
      lift_line_marker.id = 8;
      lift_line_marker.type = visualization_msgs::Marker::LINE_LIST;
      lift_line_marker.action = visualization_msgs::Marker::ADD;
      lift_line_marker.pose.orientation.w = 1.0;
      lift_line_marker.scale.x = 0.05;
      lift_line_marker.color.r = lift_debug_info.passed ? 0.2f : 1.0f;
      lift_line_marker.color.g = lift_debug_info.passed ? 0.95f : 0.3f;
      lift_line_marker.color.b = 0.25f;
      lift_line_marker.color.a = 0.95f;
      geometry_msgs::Point support_point = transformed_pose.position;
      support_point.z = lift_debug_info.support_z_output;
      geometry_msgs::Point object_bottom_point = transformed_pose.position;
      object_bottom_point.z = lift_debug_info.object_bottom_z_output;
      lift_line_marker.points.push_back(support_point);
      lift_line_marker.points.push_back(object_bottom_point);
      lift_line_marker.lifetime = ros::Duration(visual_object_marker_lifetime_sec_);
      markers.markers.push_back(lift_line_marker);
    }

    if (has_static_debug && static_debug_info.has_emergency_points) {
      visualization_msgs::Marker static_points_marker;
      static_points_marker.header.stamp = static_debug_info.stamp;
      static_points_marker.header.frame_id = static_debug_info.frame_id;
      static_points_marker.ns = "static_emergency_points";
      static_points_marker.id = 9;
      static_points_marker.type = visualization_msgs::Marker::POINTS;
      static_points_marker.action = visualization_msgs::Marker::ADD;
      static_points_marker.pose.orientation.w = 1.0;
      static_points_marker.scale.x = 0.5;
      static_points_marker.scale.y = 0.5;
      static_points_marker.color.r = 1.0f;
      static_points_marker.color.g = 0.2f;
      static_points_marker.color.b = 0.75f;
      static_points_marker.color.a = 0.95f;
      static_points_marker.points = static_debug_info.emergency_points;
      static_points_marker.lifetime = ros::Duration(visual_object_marker_lifetime_sec_);
      markers.markers.push_back(static_points_marker);

      visualization_msgs::Marker static_centroid_marker;
  static_centroid_marker.header.stamp = static_debug_info.stamp;
      static_centroid_marker.header.frame_id = static_debug_info.frame_id;
      static_centroid_marker.ns = "static_emergency_centroid";
      static_centroid_marker.id = 10;
      static_centroid_marker.type = visualization_msgs::Marker::SPHERE;
      static_centroid_marker.action = visualization_msgs::Marker::ADD;
      static_centroid_marker.pose.orientation.w = 1.0;
      static_centroid_marker.pose.position = static_debug_info.centroid;
      static_centroid_marker.scale.x = 0.35;
      static_centroid_marker.scale.y = 0.35;
      static_centroid_marker.scale.z = 0.35;
      static_centroid_marker.color.r = 1.0f;
      static_centroid_marker.color.g = 0.6f;
      static_centroid_marker.color.b = 0.9f;
      static_centroid_marker.color.a = 0.98f;
      static_centroid_marker.lifetime = ros::Duration(visual_object_marker_lifetime_sec_);
      markers.markers.push_back(static_centroid_marker);

      visualization_msgs::Marker static_lines_marker;
  static_lines_marker.header.stamp = static_debug_info.stamp;
      static_lines_marker.header.frame_id = static_debug_info.frame_id;
      static_lines_marker.ns = "static_emergency_centroid_links";
      static_lines_marker.id = 11;
      static_lines_marker.type = visualization_msgs::Marker::LINE_LIST;
      static_lines_marker.action = visualization_msgs::Marker::ADD;
      static_lines_marker.pose.orientation.w = 1.0;
      static_lines_marker.scale.x = 0.03;
      static_lines_marker.color.r = 1.0f;
      static_lines_marker.color.g = 0.45f;
      static_lines_marker.color.b = 0.85f;
      static_lines_marker.color.a = 0.6f;
      static_lines_marker.points.reserve(static_debug_info.emergency_points.size() * 2U);
      for (const auto& emergency_point : static_debug_info.emergency_points) {
        static_lines_marker.points.push_back(static_debug_info.centroid);
        static_lines_marker.points.push_back(emergency_point);
      }
      static_lines_marker.lifetime = ros::Duration(visual_object_marker_lifetime_sec_);
      markers.markers.push_back(static_lines_marker);
    }

    pub_visual_object_markers_.publish(markers);
  }

  void publishWarningState(
    lio_sam::WarningState& message,
    bool publish_markers,
    const geometry_msgs::Pose& monitored_pose_output,
    const geometry_msgs::Vector3& monitored_velocity,
    const DynamicRiskMetrics& dynamic_risk,
    const StaticSupportMetrics& static_support)
  {
    message.level_text = levelToText(message.active_level);
    message.monitored_pose = monitored_pose_output;
    message.monitored_velocity = monitored_velocity;
    pub_warning_.publish(message);

    if (!publish_markers) {
      return;
    }
    publishMarkers(message, dynamic_risk, static_support);
  }

  void publishMarkers(
    const lio_sam::WarningState& message,
    const DynamicRiskMetrics& dynamic_risk,
    const StaticSupportMetrics& static_support)
  {
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker clear_marker;
    clear_marker.header = message.header;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    if (message.reason == "waiting_for_inputs"
        || message.reason.find("no_monitored_object") != std::string::npos
        || message.reason.find("object_tf_failed") != std::string::npos) {
      pub_markers_.publish(markers);
      return;
    }

    visualization_msgs::Marker sphere;
    sphere.header = message.header;
    sphere.ns = "warning_target";
    sphere.id = 1;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose = message.monitored_pose;
    const double marker_scale =
      message.active_level == lio_sam::WarningState::LEVEL_EMERGENCY ? 1.0 :
      message.active_level == lio_sam::WarningState::LEVEL_WARNING ? 0.8 : 0.6;
    sphere.scale.x = marker_scale;
    sphere.scale.y = marker_scale;
    sphere.scale.z = marker_scale;
    sphere.color = colorForLevel(message.active_level);
    sphere.color.a = 0.45f;
    sphere.lifetime = ros::Duration(0.4);
    markers.markers.push_back(sphere);

    visualization_msgs::Marker point;
    point.header = message.header;
    point.ns = "warning_judgment_point";
    point.id = 5;
    point.type = visualization_msgs::Marker::SPHERE;
    point.action = visualization_msgs::Marker::ADD;
    point.pose = message.monitored_pose;
    point.scale.x = 0.18;
    point.scale.y = 0.18;
    point.scale.z = 0.18;
    point.color.r = 1.0f;
    point.color.g = 1.0f;
    point.color.b = 1.0f;
    point.color.a = 1.0f;
    point.lifetime = ros::Duration(0.4);
    markers.markers.push_back(point);

    visualization_msgs::Marker text;
    text.header = message.header;
    text.ns = "warning_text";
    text.id = 2;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose = message.monitored_pose;
    text.pose.position.z += 1.0;
    text.scale.z = 0.45;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;
    std::ostringstream text_stream;
    text_stream.setf(std::ios::fixed);
    text_stream.precision(2);
    text_stream << "LEVEL=" << levelToText(message.active_level)
                << " desired=" << levelToText(message.desired_level)
                << " obj=" << message.monitored_class_name;
    if (!message.reason.empty()) {
      text_stream << "\nreason=" << message.reason;
    }
    text_stream << "\nsrc=" << message.source;
    if (message.reason.find("below_static_lift_gate") != std::string::npos) {
      text_stream << " lift_gate=block";
    } else if (message.reason.find("missing_lift_context") != std::string::npos) {
      text_stream << " lift_gate=missing";
    }
    if (std::isfinite(message.static_clearance)) {
      text_stream << " ds=" << message.static_clearance;
    }
    if (static_support.notice_point_count > 0
        || static_support.warning_point_count > 0
        || static_support.emergency_point_count > 0) {
      text_stream << "\nsp(n/w/e)="
                  << static_support.notice_point_count << "/"
                  << static_support.warning_point_count << "/"
                  << static_support.emergency_point_count;
    }
    if (std::isfinite(message.dynamic_clearance)) {
      text_stream << " dc=" << message.dynamic_clearance;
    }
    if (std::isfinite(message.predicted_clearance)) {
      text_stream << " dd=" << message.predicted_clearance;
    }
    if (std::isfinite(message.ttc)) {
      text_stream << " ttc=" << message.ttc;
    }
    if (std::isfinite(message.relative_speed)) {
      text_stream << " rv=" << message.relative_speed;
    }
    if (message.dynamic_track_id >= 0) {
      text_stream << "\ntrack=" << message.dynamic_track_id;
    }
    if (message.head_on) {
      text_stream << " head_on=true";
    }
    text.text = text_stream.str();
    text.lifetime = ros::Duration(0.4);
    markers.markers.push_back(text);

    visualization_msgs::Marker badge;
    badge.header = message.header;
    badge.ns = "warning_badge";
    badge.id = 4;
    badge.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    badge.action = visualization_msgs::Marker::ADD;
    badge.pose = message.monitored_pose;
    badge.pose.position.z += 2.0;
    badge.scale.z = 0.8;
    badge.color = colorForLevel(message.active_level);
    badge.text = levelToText(message.active_level);
    badge.lifetime = ros::Duration(0.4);
    markers.markers.push_back(badge);

    if (dynamic_risk.track_id >= 0) {
      visualization_msgs::Marker line;
      line.header = message.header;
      line.ns = "warning_link";
      line.id = 3;
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.action = visualization_msgs::Marker::ADD;
      line.scale.x = 0.05;
      line.color = colorForLevel(message.active_level);
      line.color.a = 0.95f;
      line.points.push_back(message.monitored_pose.position);
      line.points.push_back(dynamic_risk.track_position);
      line.lifetime = ros::Duration(0.4);
      markers.markers.push_back(line);
    }

    pub_markers_.publish(markers);
  }

  bool isArrayStale(const ros::Time& stamp, double timeout_sec, const ros::Time& now) const
  {
    if (stamp.isZero()) {
      return false;
    }
    return (now - stamp).toSec() > timeout_sec;
  }

  bool isInputStale(
    const ros::Time& header_stamp,
    const ros::Time& receive_time,
    double timeout_sec,
    const ros::Time& now) const
  {
    const ros::Time freshness_stamp = (!use_sim_time_ && !receive_time.isZero())
      ? receive_time
      : header_stamp;
    return isArrayStale(freshness_stamp, timeout_sec, now);
  }

  ros::Time resolveStamp(const ros::Time& stamp) const
  {
    return stamp.isZero() ? ros::Time::now() : stamp;
  }

  void resetLevelState()
  {
    active_level_ = lio_sam::WarningState::LEVEL_NONE;
    pending_lower_level_ = lio_sam::WarningState::LEVEL_NONE;
    pending_lower_since_ = ros::Time();
    visual_motion_state_ = ObjectMotionState();
    cylinder_motion_state_ = ObjectMotionState();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_visual_objects_;
  ros::Subscriber sub_cylinder_objects_;
  ros::Subscriber sub_tracks_;
  ros::Subscriber sub_static_cloud_;

  ros::Publisher pub_warning_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_visual_object_markers_;
  ros::Timer evaluation_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  lio_sam::SegmentedObjectStateArray latest_visual_objects_;
  lio_sam::SegmentedObjectStateArray latest_cylinder_objects_;
  lio_sam::DynamicTrackArray latest_tracks_;
  sensor_msgs::PointCloud2 latest_static_cloud_msg_;
  ros::Time latest_visual_receive_time_;
  ros::Time latest_cylinder_receive_time_;
  ros::Time latest_tracks_receive_time_;
  ros::Time latest_static_cloud_receive_time_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI> static_kdtree_;

  std::string visual_objects_topic_;
  std::string cylinder_objects_topic_;
  std::string dynamic_tracks_topic_;
  std::string static_cloud_topic_;
  std::string warning_topic_;
  std::string marker_topic_;
  std::string visual_object_marker_topic_;
  std::string output_frame_;
  std::string monitored_source_mode_raw_;
  std::string cylinder_class_name_;

  bool has_visual_objects_{false};
  bool has_cylinder_objects_{false};
  bool has_tracks_{false};
  bool has_static_cloud_{false};
  bool allow_tentative_tracks_{false};
  bool publish_markers_{true};
  bool enable_static_warning_{true};
  bool use_sim_time_{false};

  double object_stale_timeout_sec_{0.5};
  double visual_hold_timeout_sec_{1.0};
  double track_stale_timeout_sec_{1.0};
  double static_cloud_stale_timeout_sec_{1.0};
  double evaluation_rate_hz_{10.0};
  double transform_timeout_sec_{0.05};
  double min_object_confidence_{0.35};
  int min_object_points_{25};
  double assumed_dynamic_track_radius_{0.4};
  double prediction_horizon_sec_{2.0};
  double dynamic_prediction_sample_dt_{0.1};
  double min_closing_speed_{0.05};
  double head_on_closing_speed_{0.4};
  double head_on_cosine_threshold_{-0.2};
  double object_velocity_alpha_{0.6};
  double max_object_velocity_dt_sec_{0.5};
  double downgrade_hold_sec_{1.0};
  double visual_object_marker_lifetime_sec_{0.0};
  double visual_object_envelope_fill_alpha_{0.28};
  bool visual_object_envelope_draw_outline_{false};
  double visual_object_envelope_outline_alpha_{0.95};
  double visual_object_envelope_outline_width_m_{0.08};
  double visual_object_radius_margin_m_{0.15};
  double static_warning_min_lift_height_m_{1.0};
  int visual_object_envelope_outline_segments_{72};

  double static_notice_clearance_{2.5};
  double static_warning_clearance_{1.5};
  double static_emergency_clearance_{0.8};
  int static_notice_min_points_{1};
  int static_warning_min_points_{1};
  int static_emergency_min_points_{1};
  double dynamic_notice_clearance_{2.0};
  double dynamic_warning_clearance_{1.2};
  double dynamic_emergency_clearance_{0.6};
  double notice_ttc_sec_{3.0};
  double warning_ttc_sec_{2.0};
  double emergency_ttc_sec_{1.0};
  double default_cylinder_radius_{2.0};
  double default_cylinder_height_{3.0};
  double cylinder_ground_contact_tolerance_{0.15};
  double cylinder_judgment_z_offset_m_{-5.0};
  bool ignore_dynamic_tracks_inside_cylinder_{true};
  double inside_cylinder_ignore_epsilon_{1e-3};

  uint8_t active_level_{lio_sam::WarningState::LEVEL_NONE};
  uint8_t pending_lower_level_{lio_sam::WarningState::LEVEL_NONE};
  ros::Time pending_lower_since_;

  MonitoredSourceMode monitored_source_mode_{MonitoredSourceMode::Visual};
  ObjectMotionState visual_motion_state_;
  ObjectMotionState cylinder_motion_state_;
  std::vector<std::string> monitored_class_names_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "warning_evaluator");
  WarningEvaluatorNode node;
  ros::spin();
  return 0;
}