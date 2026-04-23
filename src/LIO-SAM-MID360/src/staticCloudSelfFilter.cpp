#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <lio_sam/SegmentedObjectState.h>
#include <lio_sam/SegmentedObjectStateArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>

class StaticCloudSelfFilterNode
{
public:
  StaticCloudSelfFilterNode()
  : pnh_("~")
  , tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, std::string("/lio_sam/mapping/map_local"));
    pnh_.param<std::string>("visual_objects_topic", visual_objects_topic_, std::string("/warning/selected_visual_object_states"));
    pnh_.param<std::string>("output_topic", output_topic_, std::string("/warning/static_cloud_filtered"));
    pnh_.param<double>("object_stale_timeout_sec", object_stale_timeout_sec_, 0.5);
    pnh_.param<double>("history_duration_sec", history_duration_sec_, 1.5);
    pnh_.param<double>("radius_margin_m", radius_margin_m_, 0.15);
    pnh_.param<double>("min_object_confidence", min_object_confidence_, 0.35);
    pnh_.param<int>("min_object_points", min_object_points_, 25);
    pnh_.param<double>("min_object_radius_m", min_object_radius_m_, 0.05);
    nh_.param<bool>("/use_sim_time", use_sim_time_, false);

    sub_visual_objects_ = nh_.subscribe(visual_objects_topic_, 3, &StaticCloudSelfFilterNode::visualObjectsCallback, this);
    sub_cloud_ = nh_.subscribe(input_cloud_topic_, 1, &StaticCloudSelfFilterNode::cloudCallback, this);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

    ROS_INFO_STREAM("[static_cloud_self_filter] input_cloud_topic=" << input_cloud_topic_
                    << " visual_objects_topic=" << visual_objects_topic_
                    << " output_topic=" << output_topic_
                    << " history_duration_sec=" << history_duration_sec_
                    << " radius_margin_m=" << radius_margin_m_);
  }

private:
  struct HistorySphere
  {
    geometry_msgs::Point center;
    double radius{0.0};
    ros::Time stamp;
  };

  void visualObjectsCallback(const lio_sam::SegmentedObjectStateArrayConstPtr& msg)
  {
    if (!msg) {
      return;
    }
    latest_visual_objects_ = *msg;
    latest_visual_receive_time_ = ros::Time::now();
    has_visual_objects_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (!msg) {
      return;
    }

    if (history_frame_id_.empty() || history_frame_id_ != msg->header.frame_id) {
      history_frame_id_ = msg->header.frame_id;
      history_spheres_.clear();
    }

    HistorySphere current_sphere;
    if (buildCurrentHistorySphere(*msg, &current_sphere)) {
      history_spheres_.push_back(current_sphere);
    }
    pruneHistory(resolveStamp(msg->header.stamp));

    if (history_spheres_.empty()) {
      pub_cloud_.publish(*msg);
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
    filtered_cloud.points.reserve(input_cloud.points.size());
    for (const auto& point : input_cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (pointInsideHistory(point)) {
        continue;
      }
      filtered_cloud.points.push_back(point);
    }

    filtered_cloud.width = static_cast<uint32_t>(filtered_cloud.points.size());
    filtered_cloud.height = 1;
    filtered_cloud.is_dense = false;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(filtered_cloud, output_msg);
    output_msg.header = msg->header;
    pub_cloud_.publish(output_msg);
  }

  bool buildCurrentHistorySphere(
    const sensor_msgs::PointCloud2& cloud_msg,
    HistorySphere* sphere)
  {
    if (!sphere || isVisualObjectsStale(resolveStamp(cloud_msg.header.stamp))) {
      return false;
    }

    lio_sam::SegmentedObjectState selected_object;
    if (!selectVisualObject(latest_visual_objects_, &selected_object)) {
      return false;
    }

    geometry_msgs::Pose object_pose_cloud;
    const ros::Time object_stamp = resolveStamp(latest_visual_objects_.header.stamp.isZero()
      ? cloud_msg.header.stamp
      : latest_visual_objects_.header.stamp);
    if (!transformPose(
          selected_object.pose,
          latest_visual_objects_.header.frame_id,
          cloud_msg.header.frame_id,
          object_stamp,
          &object_pose_cloud)) {
      return false;
    }

    sphere->center = object_pose_cloud.position;
    sphere->radius = objectRadius(selected_object) + std::max(0.0, radius_margin_m_);
    sphere->stamp = object_stamp;
    return true;
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

      if (!best) {
        best = &object;
        continue;
      }

      if (object.nearest_range < best->nearest_range - 1e-3f) {
        best = &object;
        continue;
      }

      if (std::fabs(object.nearest_range - best->nearest_range) <= 1e-3f
          && object.confidence > best->confidence) {
        best = &object;
      }
    }

    if (!best) {
      return false;
    }

    *selected_object = *best;
    return true;
  }

  double objectRadius(const lio_sam::SegmentedObjectState& object) const
  {
    if (object.bounding_radius > 1e-3f) {
      return std::max(min_object_radius_m_, static_cast<double>(object.bounding_radius));
    }
    if (object.footprint_radius > 1e-3f) {
      return std::max(min_object_radius_m_, static_cast<double>(object.footprint_radius));
    }

    const double from_size = 0.5 * std::sqrt(
      static_cast<double>(object.size.x) * static_cast<double>(object.size.x)
      + static_cast<double>(object.size.y) * static_cast<double>(object.size.y)
      + static_cast<double>(object.size.z) * static_cast<double>(object.size.z));
    return std::max(min_object_radius_m_, from_size);
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
        ros::Duration(0.05));
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
        ros::Duration(0.05));
      tf2::doTransform(pose_stamped, pose_stamped, transform);
      *transformed_pose = pose_stamped.pose;
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(
        2.0,
        "[static_cloud_self_filter] Failed to transform pose from %s to %s: %s",
        source_frame.c_str(),
        target_frame.c_str(),
        ex.what());
      return false;
    }
  }

  bool isVisualObjectsStale(const ros::Time& reference_stamp) const
  {
    if (!has_visual_objects_) {
      return true;
    }

    const ros::Time freshness_stamp = (!use_sim_time_ && !latest_visual_receive_time_.isZero())
      ? latest_visual_receive_time_
      : latest_visual_objects_.header.stamp;
    if (freshness_stamp.isZero()) {
      return true;
    }

    const ros::Time effective_reference = use_sim_time_ ? resolveStamp(reference_stamp) : ros::Time::now();
    if (effective_reference.isZero()) {
      return false;
    }

    return (effective_reference - freshness_stamp).toSec() > object_stale_timeout_sec_;
  }

  ros::Time resolveStamp(const ros::Time& stamp) const
  {
    return stamp.isZero() ? ros::Time::now() : stamp;
  }

  void pruneHistory(const ros::Time& reference_stamp)
  {
    if (history_duration_sec_ <= 1e-6) {
      if (history_spheres_.size() > 1) {
        const HistorySphere latest = history_spheres_.back();
        history_spheres_.clear();
        history_spheres_.push_back(latest);
      }
      return;
    }

    const ros::Time effective_reference = resolveStamp(reference_stamp);
    while (!history_spheres_.empty()) {
      const double age_sec = (effective_reference - history_spheres_.front().stamp).toSec();
      if (age_sec <= history_duration_sec_) {
        break;
      }
      history_spheres_.pop_front();
    }
  }

  bool pointInsideHistory(const pcl::PointXYZI& point) const
  {
    for (const auto& sphere : history_spheres_) {
      if (pointInsideSphere(point, sphere)) {
        return true;
      }
    }

    if (history_spheres_.size() < 2) {
      return false;
    }

    for (size_t index = 1; index < history_spheres_.size(); ++index) {
      if (pointInsideSweptSegment(point, history_spheres_[index - 1], history_spheres_[index])) {
        return true;
      }
    }

    return false;
  }

  bool pointInsideSphere(const pcl::PointXYZI& point, const HistorySphere& sphere) const
  {
    const double dx = static_cast<double>(point.x) - sphere.center.x;
    const double dy = static_cast<double>(point.y) - sphere.center.y;
    const double dz = static_cast<double>(point.z) - sphere.center.z;
    return dx * dx + dy * dy + dz * dz <= sphere.radius * sphere.radius;
  }

  bool pointInsideSweptSegment(
    const pcl::PointXYZI& point,
    const HistorySphere& start,
    const HistorySphere& end) const
  {
    const double start_to_end_x = end.center.x - start.center.x;
    const double start_to_end_y = end.center.y - start.center.y;
    const double start_to_end_z = end.center.z - start.center.z;
    const double segment_length_sq = start_to_end_x * start_to_end_x
      + start_to_end_y * start_to_end_y
      + start_to_end_z * start_to_end_z;
    if (segment_length_sq <= 1e-9) {
      return false;
    }

    const double start_to_point_x = static_cast<double>(point.x) - start.center.x;
    const double start_to_point_y = static_cast<double>(point.y) - start.center.y;
    const double start_to_point_z = static_cast<double>(point.z) - start.center.z;
    const double projection = start_to_point_x * start_to_end_x
      + start_to_point_y * start_to_end_y
      + start_to_point_z * start_to_end_z;
    const double t = std::max(0.0, std::min(1.0, projection / segment_length_sq));

    const double closest_x = start.center.x + t * start_to_end_x;
    const double closest_y = start.center.y + t * start_to_end_y;
    const double closest_z = start.center.z + t * start_to_end_z;
    const double radius = std::max(start.radius, end.radius);

    const double dx = static_cast<double>(point.x) - closest_x;
    const double dy = static_cast<double>(point.y) - closest_y;
    const double dz = static_cast<double>(point.z) - closest_z;
    return dx * dx + dy * dy + dz * dz <= radius * radius;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_visual_objects_;
  ros::Subscriber sub_cloud_;
  ros::Publisher pub_cloud_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_cloud_topic_;
  std::string visual_objects_topic_;
  std::string output_topic_;
  std::string history_frame_id_;

  lio_sam::SegmentedObjectStateArray latest_visual_objects_;
  ros::Time latest_visual_receive_time_;
  bool has_visual_objects_{false};
  bool use_sim_time_{false};

  double object_stale_timeout_sec_{0.5};
  double history_duration_sec_{1.5};
  double radius_margin_m_{0.15};
  double min_object_confidence_{0.35};
  int min_object_points_{25};
  double min_object_radius_m_{0.05};

  std::deque<HistorySphere> history_spheres_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_cloud_self_filter");
  StaticCloudSelfFilterNode node;
  ros::spin();
  return 0;
}