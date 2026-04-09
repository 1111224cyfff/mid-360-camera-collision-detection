#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <lio_sam/SegmentedObjectState.h>
#include <lio_sam/SegmentedObjectStateArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{
double clampValue(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

std_msgs::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

}  // namespace

class CylinderObjectSourceNode
{
public:
  CylinderObjectSourceNode()
  : pnh_("~")
  , tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, std::string("/merged_pointcloud_leveled"));
    pnh_.param<std::string>("output_frame", output_frame_, std::string("map"));
    pnh_.param<std::string>("output_topic", output_topic_, std::string("/warning/cylinder_object_states"));
    pnh_.param<std::string>("marker_topic", marker_topic_, std::string("/warning/cylinder_object_markers"));
    pnh_.param<std::string>("object_class_name", object_class_name_, std::string("synthetic_cylinder"));
    pnh_.param<int>("object_class_id", object_class_id_, -100);
    pnh_.param<double>("radius", radius_m_, 2.0);
    pnh_.param<double>("default_height", default_height_m_, 3.0);
    pnh_.param<double>("min_height", min_height_m_, 0.5);
    pnh_.param<double>("contact_percentile", contact_percentile_, 0.1);
    pnh_.param<double>("min_support_drop", min_support_drop_m_, 0.2);
    pnh_.param<int>("min_support_points", min_support_points_, 30);
    pnh_.param<double>("transform_timeout_sec", transform_timeout_sec_, 0.05);
    pnh_.param<double>("object_confidence", object_confidence_, 1.0);
    pnh_.param<double>("fallback_confidence", fallback_confidence_, 0.7);
    pnh_.param<bool>("publish_markers", publish_markers_, true);

    sub_cloud_ = nh_.subscribe(input_cloud_topic_, 1, &CylinderObjectSourceNode::cloudCallback, this);
    pub_objects_ = nh_.advertise<lio_sam::SegmentedObjectStateArray>(output_topic_, 1);
    if (publish_markers_) {
      pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
    }

    ROS_INFO_STREAM("[cylinder_object_source] input_cloud_topic=" << input_cloud_topic_
                    << " output_frame=" << output_frame_
                    << " output_topic=" << output_topic_
                    << " radius=" << radius_m_
                    << " default_height=" << default_height_m_);
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (!msg) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    std::vector<double> support_z_values;
    support_z_values.reserve(cloud.points.size());

    for (const auto& point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      const double radial_distance = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
      if (radial_distance > radius_m_) {
        continue;
      }
      if (point.z > -min_support_drop_m_) {
        continue;
      }

      support_z_values.push_back(static_cast<double>(point.z));
    }

    double contact_z_estimate = std::numeric_limits<double>::quiet_NaN();
    if (!support_z_values.empty()) {
      std::sort(support_z_values.begin(), support_z_values.end());
      const double percentile = clampValue(contact_percentile_, 0.0, 1.0);
      const size_t index = static_cast<size_t>(std::floor(percentile * static_cast<double>(support_z_values.size() - 1)));
      contact_z_estimate = support_z_values[index];
    }

    bool used_fallback = true;
    double height_m = default_height_m_;
    if (static_cast<int>(support_z_values.size()) >= min_support_points_) {
      height_m = std::max(min_height_m_, -contact_z_estimate);
      used_fallback = false;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    geometry_msgs::Pose top_pose_output;
    if (!buildTopPoseOutput(msg->header.frame_id, stamp, &top_pose_output)) {
      return;
    }

    lio_sam::SegmentedObjectStateArray out;
    out.header = msg->header;
    out.header.stamp = stamp;
    out.header.frame_id = output_frame_;

    lio_sam::SegmentedObjectState object_state;
    object_state.class_id = object_class_id_;
    object_state.class_name = object_class_name_;
    object_state.confidence = static_cast<float>(used_fallback ? fallback_confidence_ : object_confidence_);
    object_state.pose = top_pose_output;
    object_state.pose.orientation.x = 0.0;
    object_state.pose.orientation.y = 0.0;
    object_state.pose.orientation.z = 0.0;
    object_state.pose.orientation.w = 1.0;
    object_state.size.x = static_cast<float>(2.0 * radius_m_);
    object_state.size.y = static_cast<float>(2.0 * radius_m_);
    object_state.size.z = static_cast<float>(height_m);
    object_state.bounding_radius = static_cast<float>(std::sqrt(radius_m_ * radius_m_ + 0.25 * height_m * height_m));
    object_state.nearest_range = static_cast<float>(radius_m_);
    object_state.point_count = static_cast<uint32_t>(support_z_values.size());
    out.objects.push_back(object_state);

    pub_objects_.publish(out);

    if (publish_markers_) {
      publishMarkers(out.header, top_pose_output.position, height_m, used_fallback, support_z_values.size());
    }
  }

  bool buildTopPoseOutput(
    const std::string& source_frame,
    const ros::Time& stamp,
    geometry_msgs::Pose* top_pose_output)
  {
    if (!top_pose_output) {
      return false;
    }

    geometry_msgs::Pose source_top_pose;
    source_top_pose.orientation.w = 1.0;
    if (!transformPose(source_top_pose, source_frame, output_frame_, stamp, top_pose_output)) {
      return false;
    }

    // Keep the cylinder aligned to the output frame so it always extends along -Z of that frame.
    top_pose_output->orientation.x = 0.0;
    top_pose_output->orientation.y = 0.0;
    top_pose_output->orientation.z = 0.0;
    top_pose_output->orientation.w = 1.0;
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
      ROS_WARN_THROTTLE(2.0,
                        "[cylinder_object_source] Failed to transform pose from %s to %s: %s",
                        source_frame.c_str(),
                        target_frame.c_str(),
                        ex.what());
      return false;
    }
  }

  void publishMarkers(
    const std_msgs::Header& header,
    const geometry_msgs::Point& top_center,
    double height_m,
    bool used_fallback,
    size_t support_points)
  {
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    visualization_msgs::Marker cylinder_marker;
    cylinder_marker.header = header;
    cylinder_marker.ns = "synthetic_cylinder";
    cylinder_marker.id = 1;
    cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
    cylinder_marker.action = visualization_msgs::Marker::ADD;
    cylinder_marker.pose.orientation.w = 1.0;
    cylinder_marker.pose.position = top_center;
    cylinder_marker.pose.position.z -= 0.5 * height_m;
    cylinder_marker.scale.x = 2.0 * radius_m_;
    cylinder_marker.scale.y = 2.0 * radius_m_;
    cylinder_marker.scale.z = height_m;
    cylinder_marker.color = used_fallback ? makeColor(0.72f, 0.30f, 0.04f, 0.55f) : makeColor(0.04f, 0.36f, 0.58f, 0.55f);
    cylinder_marker.lifetime = ros::Duration(0.25);
    markers.markers.push_back(cylinder_marker);

    visualization_msgs::Marker top_marker;
    top_marker.header = header;
    top_marker.ns = "synthetic_cylinder";
    top_marker.id = 2;
    top_marker.type = visualization_msgs::Marker::SPHERE;
    top_marker.action = visualization_msgs::Marker::ADD;
    top_marker.pose.orientation.w = 1.0;
    top_marker.pose.position = top_center;
    top_marker.scale.x = 0.3;
    top_marker.scale.y = 0.3;
    top_marker.scale.z = 0.3;
    top_marker.color = makeColor(1.0f, 1.0f, 1.0f, 0.9f);
    top_marker.lifetime = ros::Duration(0.25);
    markers.markers.push_back(top_marker);

    visualization_msgs::Marker text_marker;
    text_marker.header = header;
    text_marker.ns = "synthetic_cylinder";
    text_marker.id = 3;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.orientation.w = 1.0;
    text_marker.pose.position = top_center;
    text_marker.pose.position.z += 0.3;
    text_marker.scale.z = 0.25;
    text_marker.color = makeColor(1.0f, 1.0f, 1.0f, 1.0f);
    text_marker.text = used_fallback
      ? "synthetic_cylinder fallback height=" + std::to_string(height_m)
      : "synthetic_cylinder height=" + std::to_string(height_m) + " pts=" + std::to_string(support_points);
    text_marker.lifetime = ros::Duration(0.25);
    markers.markers.push_back(text_marker);

    pub_markers_.publish(markers);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_cloud_;
  ros::Publisher pub_objects_;
  ros::Publisher pub_markers_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_cloud_topic_;
  std::string output_frame_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string object_class_name_;

  int object_class_id_{-100};
  double radius_m_{2.0};
  double default_height_m_{3.0};
  double min_height_m_{0.5};
  double contact_percentile_{0.1};
  double min_support_drop_m_{0.2};
  int min_support_points_{30};
  double transform_timeout_sec_{0.05};
  double object_confidence_{1.0};
  double fallback_confidence_{0.7};
  bool publish_markers_{true};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cylinder_object_source");
  CylinderObjectSourceNode node;
  ros::spin();
  return 0;
}