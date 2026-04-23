#include <ros/ros.h>

#include <XmlRpcValue.h>

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
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace
{
double clampValue(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double pointDistance(const geometry_msgs::Point& point)
{
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
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
    pnh_.param<std::string>("visual_objects_topic", visual_objects_topic_, std::string("/segment/object_states"));
    pnh_.param<std::string>("output_frame", output_frame_, std::string("map"));
    pnh_.param<std::string>("output_topic", output_topic_, std::string("/warning/cylinder_object_states"));
    pnh_.param<std::string>("marker_topic", marker_topic_, std::string("/warning/cylinder_object_markers"));
    pnh_.param<std::string>("object_class_name", object_class_name_, std::string("synthetic_cylinder"));
    pnh_.param<bool>("use_fixed_anchor", use_fixed_anchor_, false);
    pnh_.param<std::string>("fixed_anchor_frame", fixed_anchor_frame_, std::string("base_link"));
    pnh_.param<double>("fixed_anchor_x", fixed_anchor_x_, 0.0);
    pnh_.param<double>("fixed_anchor_y", fixed_anchor_y_, 0.0);
    pnh_.param<double>("fixed_anchor_z", fixed_anchor_z_, -3.0);
    pnh_.param<int>("object_class_id", object_class_id_, -100);
    pnh_.param<double>("radius", radius_m_, 2.0);
    pnh_.param<double>("min_height", min_height_m_, 0.5);
    pnh_.param<double>("visual_object_timeout_sec", visual_object_timeout_sec_, 1.0);
    pnh_.param<double>("contact_percentile", contact_percentile_, 0.1);
    pnh_.param<double>("min_support_drop", min_support_drop_m_, 0.2);
    pnh_.param<double>("transform_timeout_sec", transform_timeout_sec_, 0.05);
    pnh_.param<double>("min_object_confidence", min_object_confidence_, 0.35);
    pnh_.param<int>("min_object_points", min_object_points_, 25);
    pnh_.param<double>("object_confidence", object_confidence_, 1.0);
    pnh_.param<bool>("publish_markers", publish_markers_, true);
    pnh_.param<double>("marker_lifetime_sec", marker_lifetime_sec_, 0.0);
    nh_.param<bool>("/use_sim_time", use_sim_time_, false);

    loadAnchorClassNames();

    if (!use_fixed_anchor_ && !visual_objects_topic_.empty()) {
      sub_visual_objects_ = nh_.subscribe(visual_objects_topic_, 3, &CylinderObjectSourceNode::visualObjectsCallback, this);
    }
    sub_cloud_ = nh_.subscribe(input_cloud_topic_, 1, &CylinderObjectSourceNode::cloudCallback, this);
    pub_objects_ = nh_.advertise<lio_sam::SegmentedObjectStateArray>(output_topic_, 1);
    if (publish_markers_) {
      pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
    }

    ROS_INFO_STREAM("[cylinder_object_source] input_cloud_topic=" << input_cloud_topic_
                    << " visual_objects_topic=" << visual_objects_topic_
                << " anchor_mode=" << (use_fixed_anchor_ ? "fixed" : "visual")
                << (use_fixed_anchor_
                    ? (std::string(" fixed_anchor=") + fixed_anchor_frame_ + "(" +
                      std::to_string(fixed_anchor_x_) + "," +
                      std::to_string(fixed_anchor_y_) + "," +
                      std::to_string(fixed_anchor_z_) + ")")
                    : std::string())
                    << " output_frame=" << output_frame_
                    << " output_topic=" << output_topic_
                    << " cylinder_radius=" << radius_m_
                    << " min_height=" << min_height_m_);
  }

private:
  void loadAnchorClassNames()
  {
    XmlRpc::XmlRpcValue class_names_param;
    if (!pnh_.getParam("anchor_class_names", class_names_param)) {
      return;
    }
    if (class_names_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("[cylinder_object_source] anchor_class_names must be a YAML list.");
      return;
    }

    anchor_class_names_.clear();
    for (int index = 0; index < class_names_param.size(); ++index) {
      if (class_names_param[index].getType() != XmlRpc::XmlRpcValue::TypeString) {
        continue;
      }
      anchor_class_names_.push_back(static_cast<std::string>(class_names_param[index]));
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

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (!msg) {
      return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    lio_sam::SegmentedObjectState anchor_object;
    geometry_msgs::Pose top_pose_cloud;
    geometry_msgs::Pose top_pose_output;
    double cylinder_radius_m = radius_m_;
    const bool anchor_ready = use_fixed_anchor_
      ? resolveFixedAnchor(*msg, stamp, &anchor_object, &top_pose_cloud, &top_pose_output, &cylinder_radius_m)
      : resolveVisualAnchor(*msg, stamp, &anchor_object, &top_pose_cloud, &top_pose_output, &cylinder_radius_m);
    if (!anchor_ready) {
      publishEmptyOutputs(stamp);
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

      const double dx = static_cast<double>(point.x) - top_pose_cloud.position.x;
      const double dy = static_cast<double>(point.y) - top_pose_cloud.position.y;
      const double radial_distance = std::hypot(dx, dy);
      if (radial_distance > cylinder_radius_m) {
        continue;
      }
      if (point.z > top_pose_cloud.position.z - min_support_drop_m_) {
        continue;
      }

      support_z_values.push_back(static_cast<double>(point.z));
    }

    double contact_z_estimate = std::numeric_limits<double>::quiet_NaN();
    if (!estimateSupportContactZ(support_z_values, &contact_z_estimate)) {
      publishEmptyOutputs(stamp);
      return;
    }

    const double height_m = std::max(min_height_m_, top_pose_cloud.position.z - contact_z_estimate);

    lio_sam::SegmentedObjectStateArray out;
    out.header = msg->header;
    out.header.stamp = stamp;
    out.header.frame_id = output_frame_;

    lio_sam::SegmentedObjectState object_state;
    object_state.class_id = object_class_id_;
    object_state.class_name = object_class_name_;
    object_state.confidence = static_cast<float>(object_confidence_);
    object_state.pose = top_pose_output;
    object_state.pose.orientation.x = 0.0;
    object_state.pose.orientation.y = 0.0;
    object_state.pose.orientation.z = 0.0;
    object_state.pose.orientation.w = 1.0;
    object_state.size.x = static_cast<float>(2.0 * cylinder_radius_m);
    object_state.size.y = static_cast<float>(2.0 * cylinder_radius_m);
    object_state.size.z = static_cast<float>(height_m);
    object_state.bounding_radius = static_cast<float>(cylinder_radius_m);
    object_state.footprint_radius = static_cast<float>(cylinder_radius_m);
    object_state.nearest_range = static_cast<float>(pointDistance(top_pose_output.position));
    object_state.point_count = static_cast<uint32_t>(support_z_values.size());
    out.objects.push_back(object_state);

    pub_objects_.publish(out);

    if (publish_markers_) {
      publishMarkers(
        out.header,
        top_pose_output.position,
        cylinder_radius_m,
        height_m,
        support_z_values.size(),
        anchor_object.class_name,
        anchor_object.class_id);
    }
  }

  bool estimateSupportContactZ(
    const std::vector<double>& support_z_values,
    double* contact_z_estimate) const
  {
    if (!contact_z_estimate) {
      return false;
    }

    *contact_z_estimate = std::numeric_limits<double>::quiet_NaN();
    if (support_z_values.empty()) {
      return false;
    }

    std::vector<double> sorted_z = support_z_values;
    std::sort(sorted_z.begin(), sorted_z.end());
    const double percentile = clampValue(contact_percentile_, 0.0, 1.0);
    const size_t index = static_cast<size_t>(
      std::floor(percentile * static_cast<double>(sorted_z.size() - 1)));
    *contact_z_estimate = sorted_z[index];
    return true;
  }

  bool selectVisualAnchorObject(
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
      if (!anchor_class_names_.empty()) {
        const bool matches = std::find(anchor_class_names_.begin(), anchor_class_names_.end(), object.class_name)
          != anchor_class_names_.end();
        if (!matches) {
          continue;
        }
      }
      if (!best
          || object.nearest_range < best->nearest_range - 1e-3f
          || (std::fabs(object.nearest_range - best->nearest_range) <= 1e-3f && object.confidence > best->confidence)) {
        best = &object;
      }
    }

    if (!best) {
      return false;
    }

    *selected_object = *best;
    return true;
  }

  bool resolveFixedAnchor(
    const sensor_msgs::PointCloud2& cloud_msg,
    const ros::Time& stamp,
    lio_sam::SegmentedObjectState* anchor_object,
    geometry_msgs::Pose* top_pose_cloud,
    geometry_msgs::Pose* top_pose_output,
    double* cylinder_radius_m)
  {
    if (!anchor_object || !top_pose_cloud || !top_pose_output || !cylinder_radius_m) {
      return false;
    }

    geometry_msgs::Pose fixed_anchor_origin_pose;
    fixed_anchor_origin_pose.orientation.w = 1.0;

    geometry_msgs::Pose fixed_anchor_pose_output;
    if (!transformPose(fixed_anchor_origin_pose, fixed_anchor_frame_, output_frame_, stamp, &fixed_anchor_pose_output)) {
      return false;
    }

    fixed_anchor_pose_output.position.x += fixed_anchor_x_;
    fixed_anchor_pose_output.position.y += fixed_anchor_y_;
    fixed_anchor_pose_output.position.z += fixed_anchor_z_;
    fixed_anchor_pose_output.orientation.x = 0.0;
    fixed_anchor_pose_output.orientation.y = 0.0;
    fixed_anchor_pose_output.orientation.z = 0.0;
    fixed_anchor_pose_output.orientation.w = 1.0;

    if (!transformPose(fixed_anchor_pose_output, output_frame_, cloud_msg.header.frame_id, stamp, top_pose_cloud)) {
      return false;
    }

    *top_pose_output = fixed_anchor_pose_output;

    anchor_object->class_id = -1;
    anchor_object->class_name = std::string("fixed_anchor@") + fixed_anchor_frame_;
    anchor_object->confidence = 1.0f;
    anchor_object->pose = fixed_anchor_pose_output;
    anchor_object->nearest_range = static_cast<float>(pointDistance(fixed_anchor_pose_output.position));
    anchor_object->bounding_radius = static_cast<float>(std::max(0.05, radius_m_));
    anchor_object->footprint_radius = anchor_object->bounding_radius;
    anchor_object->point_count = 1U;

    *cylinder_radius_m = std::max(0.05, radius_m_);

    top_pose_cloud->orientation.x = 0.0;
    top_pose_cloud->orientation.y = 0.0;
    top_pose_cloud->orientation.z = 0.0;
    top_pose_cloud->orientation.w = 1.0;
    top_pose_output->orientation.x = 0.0;
    top_pose_output->orientation.y = 0.0;
    top_pose_output->orientation.z = 0.0;
    top_pose_output->orientation.w = 1.0;
    return true;
  }

  double visualEnvelopeRadius(const lio_sam::SegmentedObjectState& object) const
  {
    if (object.footprint_radius > 1e-3f) {
      return static_cast<double>(object.footprint_radius);
    }
    if (object.bounding_radius > 1e-3f) {
      return static_cast<double>(object.bounding_radius);
    }

    const double xy_extent = 0.5 * std::hypot(
      static_cast<double>(object.size.x),
      static_cast<double>(object.size.y));
    if (xy_extent > 1e-3) {
      return xy_extent;
    }

    return radius_m_;
  }

  bool isVisualObjectStale(const ros::Time& reference_stamp) const
  {
    if (!has_visual_objects_) {
      return true;
    }

    const ros::Time freshness_stamp = use_sim_time_ ? latest_visual_objects_.header.stamp : latest_visual_receive_time_;
    if (freshness_stamp.isZero()) {
      return true;
    }

    ros::Time effective_reference;
    if (use_sim_time_) {
      effective_reference = reference_stamp;
      if (effective_reference.isZero()) {
        effective_reference = latest_visual_objects_.header.stamp;
      }
    } else {
      effective_reference = ros::Time::now();
    }
    if (effective_reference.isZero()) {
      return false;
    }

    return (effective_reference - freshness_stamp).toSec() > visual_object_timeout_sec_;
  }

  bool resolveVisualAnchor(
    const sensor_msgs::PointCloud2& cloud_msg,
    const ros::Time& stamp,
    lio_sam::SegmentedObjectState* anchor_object,
    geometry_msgs::Pose* top_pose_cloud,
    geometry_msgs::Pose* top_pose_output,
    double* cylinder_radius_m)
  {
    if (!anchor_object || !top_pose_cloud || !top_pose_output || !cylinder_radius_m) {
      return false;
    }

    if (isVisualObjectStale(stamp)) {
      return false;
    }

    if (!selectVisualAnchorObject(latest_visual_objects_, anchor_object)) {
      return false;
    }

    if (!transformPose(anchor_object->pose, latest_visual_objects_.header.frame_id, cloud_msg.header.frame_id, stamp, top_pose_cloud)) {
      return false;
    }
    if (!transformPose(anchor_object->pose, latest_visual_objects_.header.frame_id, output_frame_, stamp, top_pose_output)) {
      return false;
    }

    *cylinder_radius_m = std::max(0.05, visualEnvelopeRadius(*anchor_object));

    top_pose_cloud->orientation.x = 0.0;
    top_pose_cloud->orientation.y = 0.0;
    top_pose_cloud->orientation.z = 0.0;
    top_pose_cloud->orientation.w = 1.0;
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

  void publishEmptyOutputs(const ros::Time& stamp)
  {
    lio_sam::SegmentedObjectStateArray out;
    out.header.stamp = stamp;
    out.header.frame_id = output_frame_;
    pub_objects_.publish(out);
  }

  void publishMarkers(
    const std_msgs::Header& header,
    const geometry_msgs::Point& top_center,
    double radius_m,
    double height_m,
    size_t support_points,
    const std::string& anchor_class_name,
    int anchor_class_id)
  {
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker cylinder_marker;
    cylinder_marker.header = header;
    cylinder_marker.ns = "synthetic_cylinder";
    cylinder_marker.id = 1;
    cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
    cylinder_marker.action = visualization_msgs::Marker::ADD;
    cylinder_marker.pose.orientation.w = 1.0;
    cylinder_marker.pose.position = top_center;
    cylinder_marker.pose.position.z -= 0.5 * height_m;
    cylinder_marker.scale.x = 2.0 * radius_m;
    cylinder_marker.scale.y = 2.0 * radius_m;
    cylinder_marker.scale.z = height_m;
    cylinder_marker.color = makeColor(0.02f, 0.42f, 0.70f, 0.78f);
    cylinder_marker.lifetime = ros::Duration(marker_lifetime_sec_);
    markers.markers.push_back(cylinder_marker);

    visualization_msgs::Marker center_marker;
    center_marker.header = header;
    center_marker.ns = "synthetic_cylinder";
    center_marker.id = 2;
    center_marker.type = visualization_msgs::Marker::SPHERE;
    center_marker.action = visualization_msgs::Marker::ADD;
    center_marker.pose.orientation.w = 1.0;
    center_marker.pose.position = top_center;
    center_marker.scale.x = 0.3;
    center_marker.scale.y = 0.3;
    center_marker.scale.z = 0.3;
    center_marker.color = makeColor(1.0f, 1.0f, 1.0f, 0.9f);
    center_marker.lifetime = ros::Duration(marker_lifetime_sec_);
    markers.markers.push_back(center_marker);

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
    std::ostringstream text_stream;
    text_stream << std::fixed << std::setprecision(2);
    text_stream << "anchor=" << anchor_class_name
                << " cls=" << anchor_class_id
                << " top=(" << top_center.x << "," << top_center.y << "," << top_center.z << ")"
                << " r=" << radius_m
                << " h=" << height_m
                << " pts=" << support_points;
    text_marker.text = text_stream.str();
    text_marker.lifetime = ros::Duration(marker_lifetime_sec_);
    markers.markers.push_back(text_marker);

    pub_markers_.publish(markers);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_visual_objects_;
  ros::Subscriber sub_cloud_;
  ros::Publisher pub_objects_;
  ros::Publisher pub_markers_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_cloud_topic_;
  std::string visual_objects_topic_;
  std::string output_frame_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string object_class_name_;
  std::string fixed_anchor_frame_;

  lio_sam::SegmentedObjectStateArray latest_visual_objects_;
  ros::Time latest_visual_receive_time_;
  std::vector<std::string> anchor_class_names_;
  bool has_visual_objects_{false};
  bool use_fixed_anchor_{false};

  int object_class_id_{-100};
  double fixed_anchor_x_{0.0};
  double fixed_anchor_y_{0.0};
  double fixed_anchor_z_{-3.0};
  double radius_m_{2.0};
  double min_height_m_{0.5};
  double visual_object_timeout_sec_{1.0};
  double contact_percentile_{0.1};
  double min_support_drop_m_{0.2};
  double transform_timeout_sec_{0.05};
  double min_object_confidence_{0.35};
  int min_object_points_{25};
  double object_confidence_{1.0};
  bool publish_markers_{true};
  double marker_lifetime_sec_{0.0};
  bool use_sim_time_{false};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cylinder_object_source");
  CylinderObjectSourceNode node;
  ros::spin();
  return 0;
}