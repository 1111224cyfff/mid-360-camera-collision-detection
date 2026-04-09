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
    pnh_.param<std::string>("output_frame", output_frame_, std::string("map"));
    pnh_.param<std::string>("monitored_source_mode", monitored_source_mode_raw_, std::string("visual"));
    pnh_.param<std::string>("cylinder_class_name", cylinder_class_name_, std::string("synthetic_cylinder"));

    pnh_.param<double>("object_stale_timeout", object_stale_timeout_sec_, 0.5);
    pnh_.param<double>("track_stale_timeout", track_stale_timeout_sec_, 1.0);
    pnh_.param<double>("static_cloud_stale_timeout", static_cloud_stale_timeout_sec_, 1.0);
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

    pnh_.param<double>("static_notice_clearance", static_notice_clearance_, 2.5);
    pnh_.param<double>("static_warning_clearance", static_warning_clearance_, 1.5);
    pnh_.param<double>("static_emergency_clearance", static_emergency_clearance_, 0.8);

    pnh_.param<double>("dynamic_notice_clearance", dynamic_notice_clearance_, 2.0);
    pnh_.param<double>("dynamic_warning_clearance", dynamic_warning_clearance_, 1.2);
    pnh_.param<double>("dynamic_emergency_clearance", dynamic_emergency_clearance_, 0.6);

    pnh_.param<double>("notice_ttc", notice_ttc_sec_, 3.0);
    pnh_.param<double>("warning_ttc", warning_ttc_sec_, 2.0);
    pnh_.param<double>("emergency_ttc", emergency_ttc_sec_, 1.0);

    pnh_.param<double>("default_cylinder_radius", default_cylinder_radius_, 2.0);
    pnh_.param<double>("default_cylinder_height", default_cylinder_height_, 3.0);
    pnh_.param<double>("cylinder_ground_contact_tolerance", cylinder_ground_contact_tolerance_, 0.15);
    pnh_.param<double>("cylinder_judgment_z_offset_m", cylinder_judgment_z_offset_m_, -5.0);
    pnh_.param<bool>("ignore_dynamic_tracks_inside_cylinder", ignore_dynamic_tracks_inside_cylinder_, true);
    pnh_.param<double>("inside_cylinder_ignore_epsilon", inside_cylinder_ignore_epsilon_, 1e-3);

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

    ROS_INFO_STREAM("[warning_evaluator] mode=" << monitored_source_mode_raw_
                    << " visual_objects_topic=" << visual_objects_topic_
                    << " cylinder_objects_topic=" << cylinder_objects_topic_
                    << " tracks_topic=" << dynamic_tracks_topic_
                    << " static_cloud_topic=" << static_cloud_topic_
                    << " output_frame=" << output_frame_);
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
    latest_visual_objects_ = *msg;
    has_visual_objects_ = true;
    evaluateAndPublish(resolveStamp(msg->header.stamp));
  }

  void cylinderObjectsCallback(const lio_sam::SegmentedObjectStateArrayConstPtr& msg)
  {
    latest_cylinder_objects_ = *msg;
    has_cylinder_objects_ = true;
    evaluateAndPublish(resolveStamp(msg->header.stamp));
  }

  void tracksCallback(const lio_sam::DynamicTrackArrayConstPtr& msg)
  {
    latest_tracks_ = *msg;
    has_tracks_ = true;
    evaluateAndPublish(resolveStamp(msg->header.stamp));
  }

  void staticCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    latest_static_cloud_msg_ = *msg;
    has_static_cloud_ = true;

    pcl::PointCloud<pcl::PointXYZI>::Ptr parsed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *parsed_cloud);
    static_cloud_ = parsed_cloud;
    static_kdtree_.setInputCloud(static_cloud_);

    evaluateAndPublish(resolveStamp(msg->header.stamp));
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

    const bool visual_ready = shouldEvaluateSource(ObjectSourceKind::Visual)
      && has_visual_objects_
      && !isArrayStale(latest_visual_objects_.header.stamp, object_stale_timeout_sec_, evaluation_stamp);
    const bool cylinder_ready = shouldEvaluateSource(ObjectSourceKind::Cylinder)
      && has_cylinder_objects_
      && !isArrayStale(latest_cylinder_objects_.header.stamp, object_stale_timeout_sec_, evaluation_stamp);
    const bool track_ready = has_tracks_
      && !isArrayStale(latest_tracks_.header.stamp, track_stale_timeout_sec_, evaluation_stamp);
    const bool static_ready = has_static_cloud_
      && !isArrayStale(latest_static_cloud_msg_.header.stamp, static_cloud_stale_timeout_sec_, evaluation_stamp)
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
      publishWarningState(out, publish_markers_, default_pose, zeroVector(), empty_dynamic_risk);
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
      publishWarningState(out, publish_markers_, default_pose, zeroVector(), empty_dynamic_risk);
      resetLevelState();
      return;
    }

    out.monitored_pose = best_evaluation->monitored_pose_output;
    out.monitored_velocity = best_evaluation->monitored_velocity;
    out.monitored_class_id = best_evaluation->selected_object.class_id;
    out.monitored_class_name = best_evaluation->selected_object.class_name;
    out.desired_level = best_evaluation->desired_level;
    out.source = best_evaluation->risk_source;
    out.reason = best_evaluation->reason;

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
    out.system_ready = static_warning_ready || track_ready;

    if (!out.system_ready) {
      out.reason = prefixReason(best_evaluation->source_kind, "missing_static_and_dynamic_context");
      publishWarningState(out, publish_markers_, out.monitored_pose, out.monitored_velocity, best_evaluation->dynamic_risk);
      resetLevelState();
      return;
    }

    out.active_level = applyLevelHysteresis(out.desired_level, evaluation_stamp);
    out.level_text = levelToText(out.active_level);
    publishWarningState(out, publish_markers_, out.monitored_pose, out.monitored_velocity, best_evaluation->dynamic_risk);
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
    const double selected_cylinder_height = cylinderHeight(evaluation.selected_object);
    CylinderJudgmentState cylinder_judgment_state;
    if (source_kind == ObjectSourceKind::Cylinder) {
      cylinder_judgment_state = buildCylinderJudgmentState(
        evaluation.monitored_pose_output,
        evaluation.monitored_velocity,
        selected_cylinder_height);
      evaluation.monitored_pose_output = cylinder_judgment_state.pose;
      evaluation.monitored_velocity = cylinder_judgment_state.velocity;
    }

    if (static_ready) {
      if (source_kind == ObjectSourceKind::Cylinder) {
        evaluation.static_clearance = computeCylinderStaticClearance(
          cylinder_judgment_state.pose,
          selected_cylinder_radius,
          selected_cylinder_height,
          evaluation_stamp);
      } else {
        evaluation.static_clearance = computeStaticClearance(
          evaluation.monitored_pose_output,
          evaluation.selected_object.bounding_radius,
          evaluation_stamp);
      }
    }

    if (track_ready) {
      if (source_kind == ObjectSourceKind::Cylinder) {
        evaluation.dynamic_risk = evaluateCylinderDynamicRisk(
          cylinder_judgment_state.pose,
          cylinder_judgment_state.velocity,
          selected_cylinder_radius,
          selected_cylinder_height);
      } else {
        evaluation.dynamic_risk = evaluateSphereDynamicRisk(
          evaluation.monitored_pose_output,
          evaluation.monitored_velocity,
          evaluation.selected_object.bounding_radius);
      }
    }

    evaluation.static_level = classifyStaticLevel(evaluation.static_clearance);
    evaluation.dynamic_level = track_ready ? evaluation.dynamic_risk.level : lio_sam::WarningState::LEVEL_NONE;
    evaluation.desired_level = std::max(evaluation.static_level, evaluation.dynamic_level);
    evaluation.risk_source = classifySource(evaluation.static_level, evaluation.dynamic_level);
    evaluation.reason = prefixReason(
      source_kind,
      buildReason(
        evaluation.static_level,
        evaluation.dynamic_level,
        evaluation.static_clearance,
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
    const ros::Time& evaluation_stamp)
  {
    if (!static_cloud_ || static_cloud_->empty()) {
      return std::numeric_limits<double>::infinity();
    }

    geometry_msgs::Pose object_pose_cloud = monitored_pose_output;
    if (!transformPose(monitored_pose_output, output_frame_, latest_static_cloud_msg_.header.frame_id, evaluation_stamp, &object_pose_cloud)) {
      return std::numeric_limits<double>::infinity();
    }

    pcl::PointXYZI search_point;
    search_point.x = object_pose_cloud.position.x;
    search_point.y = object_pose_cloud.position.y;
    search_point.z = object_pose_cloud.position.z;
    search_point.intensity = 0.0f;

    std::vector<int> nearest_index(1, -1);
    std::vector<float> nearest_distance_sq(1, std::numeric_limits<float>::infinity());
    if (static_kdtree_.nearestKSearch(search_point, 1, nearest_index, nearest_distance_sq) <= 0) {
      return std::numeric_limits<double>::infinity();
    }

    return std::max(0.0, std::sqrt(nearest_distance_sq.front()) - object_radius);
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

  DynamicRiskMetrics evaluateCylinderDynamicRisk(
    const geometry_msgs::Pose& monitored_pose_output,
    const geometry_msgs::Vector3& monitored_velocity,
    double cylinder_radius,
    double cylinder_height) const
  {
    DynamicRiskMetrics best_risk;

    const Eigen::Vector3d top_center(
      monitored_pose_output.position.x,
      monitored_pose_output.position.y,
      monitored_pose_output.position.z);
    const Eigen::Vector3d object_velocity_vector(
      monitored_velocity.x,
      monitored_velocity.y,
      monitored_velocity.z);
    const double object_speed = object_velocity_vector.norm();
    const double bounded_radius = std::max(0.0, cylinder_radius);
    const double bounded_height = std::max(1e-3, cylinder_height);
    const double sample_dt = std::max(0.02, dynamic_prediction_sample_dt_);

    for (const auto& track : latest_tracks_.tracks) {
      if (!allow_tentative_tracks_ && track.state != lio_sam::DynamicTrack::STATE_CONFIRMED) {
        continue;
      }

      const Eigen::Vector3d track_position(
        track.pose.position.x,
        track.pose.position.y,
        track.pose.position.z);
      if (ignore_dynamic_tracks_inside_cylinder_) {
        const double distance_to_cylinder_surface = pointToVerticalCylinderDistance(
          track_position,
          top_center,
          bounded_radius,
          bounded_height);
        if (distance_to_cylinder_surface <= inside_cylinder_ignore_epsilon_) {
          continue;
        }
      }

      const Eigen::Vector3d track_velocity(track.velocity.x, track.velocity.y, track.velocity.z);

      auto clearanceAtTime = [&](double time_sec) {
        const Eigen::Vector3d top_at_time = top_center + object_velocity_vector * time_sec;
        const Eigen::Vector3d track_at_time = track_position + track_velocity * time_sec;
        const double distance_to_cylinder = pointToVerticalCylinderDistance(
          track_at_time,
          top_at_time,
          bounded_radius,
          bounded_height);
        return std::max(0.0, distance_to_cylinder - assumed_dynamic_track_radius_);
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

      const double relative_speed = (current_clearance - clearanceAtTime(std::min(prediction_horizon_sec_, sample_dt))) / sample_dt;

      bool head_on = false;
      const double track_speed = track_velocity.norm();
      if (object_speed > min_closing_speed_ && track_speed > min_closing_speed_) {
        const double cosine = object_velocity_vector.normalized().dot(track_velocity.normalized());
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

  uint8_t classifyStaticLevel(double clearance) const
  {
    if (!std::isfinite(clearance)) {
      return lio_sam::WarningState::LEVEL_NONE;
    }
    if (clearance <= static_emergency_clearance_) {
      return lio_sam::WarningState::LEVEL_EMERGENCY;
    }
    if (clearance <= static_warning_clearance_) {
      return lio_sam::WarningState::LEVEL_WARNING;
    }
    if (clearance <= static_notice_clearance_) {
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
    const DynamicRiskMetrics& dynamic_risk) const
  {
    if (static_level == lio_sam::WarningState::LEVEL_NONE && dynamic_level == lio_sam::WarningState::LEVEL_NONE) {
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

  void publishWarningState(
    lio_sam::WarningState& message,
    bool publish_markers,
    const geometry_msgs::Pose& monitored_pose_output,
    const geometry_msgs::Vector3& monitored_velocity,
    const DynamicRiskMetrics& dynamic_risk)
  {
    message.level_text = levelToText(message.active_level);
    message.monitored_pose = monitored_pose_output;
    message.monitored_velocity = monitored_velocity;
    pub_warning_.publish(message);

    if (!publish_markers) {
      return;
    }
    publishMarkers(message, dynamic_risk);
  }

  void publishMarkers(const lio_sam::WarningState& message, const DynamicRiskMetrics& dynamic_risk)
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
    if (std::isfinite(message.static_clearance)) {
      text_stream << " ds=" << message.static_clearance;
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

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  lio_sam::SegmentedObjectStateArray latest_visual_objects_;
  lio_sam::SegmentedObjectStateArray latest_cylinder_objects_;
  lio_sam::DynamicTrackArray latest_tracks_;
  sensor_msgs::PointCloud2 latest_static_cloud_msg_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI> static_kdtree_;

  std::string visual_objects_topic_;
  std::string cylinder_objects_topic_;
  std::string dynamic_tracks_topic_;
  std::string static_cloud_topic_;
  std::string warning_topic_;
  std::string marker_topic_;
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

  double object_stale_timeout_sec_{0.5};
  double track_stale_timeout_sec_{1.0};
  double static_cloud_stale_timeout_sec_{1.0};
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

  double static_notice_clearance_{2.5};
  double static_warning_clearance_{1.5};
  double static_emergency_clearance_{0.8};
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