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

double vectorNorm(const geometry_msgs::Vector3& value)
{
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

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

}  // namespace

class WarningEvaluatorNode
{
public:
  WarningEvaluatorNode()
  : pnh_("~")
  , tf_listener_(tf_buffer_)
  , static_cloud_(new pcl::PointCloud<pcl::PointXYZI>())
  {
    pnh_.param<std::string>("segment_objects_topic", segment_objects_topic_, std::string("/segment/object_states"));
    pnh_.param<std::string>("dynamic_tracks_topic", dynamic_tracks_topic_, std::string("/dynamic_tracker/track_states"));
    pnh_.param<std::string>("static_cloud_topic", static_cloud_topic_, std::string("/lio_sam/mapping/map_local"));
    pnh_.param<std::string>("warning_topic", warning_topic_, std::string("/warning/state"));
    pnh_.param<std::string>("marker_topic", marker_topic_, std::string("/warning/markers"));
    pnh_.param<std::string>("output_frame", output_frame_, std::string("map"));

    pnh_.param<double>("object_stale_timeout", object_stale_timeout_sec_, 0.5);
    pnh_.param<double>("track_stale_timeout", track_stale_timeout_sec_, 1.0);
    pnh_.param<double>("static_cloud_stale_timeout", static_cloud_stale_timeout_sec_, 1.0);
    pnh_.param<double>("transform_timeout_sec", transform_timeout_sec_, 0.05);
    pnh_.param<double>("min_object_confidence", min_object_confidence_, 0.35);
    pnh_.param<int>("min_object_points", min_object_points_, 25);
    pnh_.param<bool>("allow_tentative_tracks", allow_tentative_tracks_, false);
    pnh_.param<double>("assumed_dynamic_track_radius", assumed_dynamic_track_radius_, 0.4);
    pnh_.param<double>("prediction_horizon", prediction_horizon_sec_, 2.0);
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

    loadMonitoredClassNames();

    sub_objects_ = nh_.subscribe(segment_objects_topic_, 3, &WarningEvaluatorNode::objectsCallback, this);
    sub_tracks_ = nh_.subscribe(dynamic_tracks_topic_, 3, &WarningEvaluatorNode::tracksCallback, this);
    sub_static_cloud_ = nh_.subscribe(static_cloud_topic_, 1, &WarningEvaluatorNode::staticCloudCallback, this);

    pub_warning_ = nh_.advertise<lio_sam::WarningState>(warning_topic_, 1, true);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

    ROS_INFO_STREAM("[warning_evaluator] objects_topic=" << segment_objects_topic_
                    << " tracks_topic=" << dynamic_tracks_topic_
                    << " static_cloud_topic=" << static_cloud_topic_
                    << " output_frame=" << output_frame_);
  }

private:
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

  void objectsCallback(const lio_sam::SegmentedObjectStateArrayConstPtr& msg)
  {
    latest_objects_ = *msg;
    has_objects_ = true;
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

    bool should_publish_markers = publish_markers_;
    geometry_msgs::Pose monitored_pose_output;
    monitored_pose_output.orientation.w = 1.0;
    geometry_msgs::Vector3 monitored_velocity;
    monitored_velocity.x = 0.0;
    monitored_velocity.y = 0.0;
    monitored_velocity.z = 0.0;

    const bool object_ready = has_objects_ && !isArrayStale(latest_objects_.header.stamp, object_stale_timeout_sec_, evaluation_stamp);
    const bool track_ready = has_tracks_ && !isArrayStale(latest_tracks_.header.stamp, track_stale_timeout_sec_, evaluation_stamp);
    const bool static_ready = has_static_cloud_
      && !isArrayStale(latest_static_cloud_msg_.header.stamp, static_cloud_stale_timeout_sec_, evaluation_stamp)
      && static_cloud_
      && !static_cloud_->empty();

    if (!object_ready) {
      publishWarningState(out, should_publish_markers, monitored_pose_output, monitored_velocity, DynamicRiskMetrics{});
      resetLevelState();
      return;
    }

    lio_sam::SegmentedObjectState selected_object;
    if (!selectMonitoredObject(&selected_object)) {
      out.reason = "no_monitored_object";
      publishWarningState(out, should_publish_markers, monitored_pose_output, monitored_velocity, DynamicRiskMetrics{});
      resetLevelState();
      return;
    }

    const std::string object_frame = latest_objects_.header.frame_id;
    if (!transformPose(selected_object.pose, object_frame, output_frame_, resolveStamp(latest_objects_.header.stamp), &monitored_pose_output)) {
      out.reason = "object_tf_failed";
      publishWarningState(out, should_publish_markers, monitored_pose_output, monitored_velocity, DynamicRiskMetrics{});
      resetLevelState();
      return;
    }

    monitored_velocity = estimateObjectVelocity(monitored_pose_output.position, resolveStamp(latest_objects_.header.stamp));
    out.monitored_pose = monitored_pose_output;
    out.monitored_velocity = monitored_velocity;
    out.monitored_class_id = selected_object.class_id;
    out.monitored_class_name = selected_object.class_name;

    double static_clearance = std::numeric_limits<double>::infinity();
    if (static_ready) {
      static_clearance = computeStaticClearance(monitored_pose_output, selected_object.bounding_radius, evaluation_stamp);
      if (std::isfinite(static_clearance)) {
        out.static_clearance = static_clearance;
      }
    }

    DynamicRiskMetrics dynamic_risk;
    if (track_ready) {
      dynamic_risk = evaluateDynamicRisk(monitored_pose_output, monitored_velocity, selected_object.bounding_radius);
      if (std::isfinite(dynamic_risk.current_clearance)) {
        out.dynamic_clearance = dynamic_risk.current_clearance;
      }
      if (std::isfinite(dynamic_risk.relative_speed)) {
        out.relative_speed = dynamic_risk.relative_speed;
      }
      if (std::isfinite(dynamic_risk.ttc)) {
        out.ttc = dynamic_risk.ttc;
      }
      if (std::isfinite(dynamic_risk.predicted_clearance)) {
        out.predicted_clearance = dynamic_risk.predicted_clearance;
      }
      out.dynamic_track_id = dynamic_risk.track_id;
      out.head_on = dynamic_risk.head_on;
    }

    const uint8_t static_level = classifyStaticLevel(static_clearance);
    const uint8_t dynamic_level = track_ready ? dynamic_risk.level : lio_sam::WarningState::LEVEL_NONE;
    const uint8_t desired_level = std::max(static_level, dynamic_level);

    out.desired_level = desired_level;
    out.source = classifySource(static_level, dynamic_level);
    out.reason = buildReason(static_level, dynamic_level, static_clearance, dynamic_risk);
    out.system_ready = static_ready || track_ready;

    if (!out.system_ready) {
      out.reason = "missing_static_and_dynamic_context";
      publishWarningState(out, should_publish_markers, monitored_pose_output, monitored_velocity, dynamic_risk);
      resetLevelState();
      return;
    }

    const uint8_t active_level = applyLevelHysteresis(desired_level, evaluation_stamp);
    out.active_level = active_level;
    out.level_text = levelToText(active_level);

    publishWarningState(out, should_publish_markers, monitored_pose_output, monitored_velocity, dynamic_risk);
  }

  bool selectMonitoredObject(lio_sam::SegmentedObjectState* selected_object) const
  {
    if (!selected_object) {
      return false;
    }

    const lio_sam::SegmentedObjectState* best = nullptr;
    for (const auto& object : latest_objects_.objects) {
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

  geometry_msgs::Vector3 estimateObjectVelocity(const geometry_msgs::Point& current_position, const ros::Time& stamp)
  {
    geometry_msgs::Vector3 velocity = object_velocity_;
    if (has_previous_object_ && !last_object_stamp_.isZero()) {
      const double dt = (stamp - last_object_stamp_).toSec();
      if (dt > 1e-3 && dt <= max_object_velocity_dt_sec_) {
        geometry_msgs::Vector3 raw_velocity;
        raw_velocity.x = (current_position.x - last_object_position_.x) / dt;
        raw_velocity.y = (current_position.y - last_object_position_.y) / dt;
        raw_velocity.z = (current_position.z - last_object_position_.z) / dt;

        velocity.x = object_velocity_alpha_ * object_velocity_.x + (1.0 - object_velocity_alpha_) * raw_velocity.x;
        velocity.y = object_velocity_alpha_ * object_velocity_.y + (1.0 - object_velocity_alpha_) * raw_velocity.y;
        velocity.z = object_velocity_alpha_ * object_velocity_.z + (1.0 - object_velocity_alpha_) * raw_velocity.z;
      }
    }

    last_object_position_ = current_position;
    last_object_stamp_ = stamp;
    object_velocity_ = velocity;
    has_previous_object_ = true;
    return velocity;
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

  DynamicRiskMetrics evaluateDynamicRisk(
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

    if (message.reason == "waiting_for_inputs" || message.reason == "no_monitored_object" || message.reason == "object_tf_failed") {
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
    sphere.lifetime = ros::Duration(0.4);
    markers.markers.push_back(sphere);

    visualization_msgs::Marker text;
    text.header = message.header;
    text.ns = "warning_text";
    text.id = 2;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose = message.monitored_pose;
    text.pose.position.z += 0.8;
    text.scale.z = 0.35;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;
    std::ostringstream text_stream;
    text_stream.setf(std::ios::fixed);
    text_stream.precision(2);
    text_stream << levelToText(message.active_level)
                << " src=" << message.source;
    if (std::isfinite(message.static_clearance)) {
      text_stream << " ds=" << message.static_clearance;
    }
    if (std::isfinite(message.predicted_clearance)) {
      text_stream << " dd=" << message.predicted_clearance;
    }
    if (std::isfinite(message.ttc)) {
      text_stream << " ttc=" << message.ttc;
    }
    text.text = text_stream.str();
    text.lifetime = ros::Duration(0.4);
    markers.markers.push_back(text);

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
    has_previous_object_ = false;
    object_velocity_.x = 0.0;
    object_velocity_.y = 0.0;
    object_velocity_.z = 0.0;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_objects_;
  ros::Subscriber sub_tracks_;
  ros::Subscriber sub_static_cloud_;

  ros::Publisher pub_warning_;
  ros::Publisher pub_markers_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  lio_sam::SegmentedObjectStateArray latest_objects_;
  lio_sam::DynamicTrackArray latest_tracks_;
  sensor_msgs::PointCloud2 latest_static_cloud_msg_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI> static_kdtree_;

  std::string segment_objects_topic_;
  std::string dynamic_tracks_topic_;
  std::string static_cloud_topic_;
  std::string warning_topic_;
  std::string marker_topic_;
  std::string output_frame_;

  bool has_objects_{false};
  bool has_tracks_{false};
  bool has_static_cloud_{false};
  bool allow_tentative_tracks_{false};
  bool publish_markers_{true};
  bool has_previous_object_{false};

  double object_stale_timeout_sec_{0.5};
  double track_stale_timeout_sec_{1.0};
  double static_cloud_stale_timeout_sec_{1.0};
  double transform_timeout_sec_{0.05};
  double min_object_confidence_{0.35};
  int min_object_points_{25};
  double assumed_dynamic_track_radius_{0.4};
  double prediction_horizon_sec_{2.0};
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

  uint8_t active_level_{lio_sam::WarningState::LEVEL_NONE};
  uint8_t pending_lower_level_{lio_sam::WarningState::LEVEL_NONE};
  ros::Time pending_lower_since_;

  geometry_msgs::Point last_object_position_;
  ros::Time last_object_stamp_;
  geometry_msgs::Vector3 object_velocity_;
  std::vector<std::string> monitored_class_names_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "warning_evaluator");
  WarningEvaluatorNode node;
  ros::spin();
  return 0;
}