/**
 * @file dyn_cluster_node.cpp
 * @brief Clusters dynamic point cloud from M-detector and publishes cluster centers
 *        as geometry_msgs::PoseArray for dynamic_tracker consumption.
 *
 * Subscribes: /m_detector/point_out_map (sensor_msgs/PointCloud2)
 * Publishes:  /m_detector/dynamic_clusters (geometry_msgs::PoseArray)
 *             /m_detector/dynamic_cluster_markers (visualization_msgs::MarkerArray) [optional]
 *             /m_detector/clustered_cloud (sensor_msgs::PointCloud2) - filtered cloud with only clustered points
 *
 * Optional: visual object envelope mask — subscribes to selected visual object states
 *           and removes points that fall inside the object envelope before clustering.
 *           This prevents the suspended payload (吊物) itself from being classified as
 *           dynamic obstacles.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <lio_sam/SegmentedObjectState.h>
#include <lio_sam/SegmentedObjectStateArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>
#include <limits>
#include <algorithm>
#include <deque>
#include <cmath>

class DynClusterNode
{
public:
  DynClusterNode()
    : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
  {
    // Point cloud / clustering parameters
    pnh_.param<std::string>("input_topic", input_topic_, "/m_detector/point_out_map");
    pnh_.param<std::string>("output_topic", output_topic_, "/m_detector/dynamic_clusters");
    pnh_.param<std::string>("marker_topic", marker_topic_, "/m_detector/dynamic_cluster_markers");
    pnh_.param<std::string>("clustered_cloud_topic", clustered_cloud_topic_, "/m_detector/clustered_cloud");
    pnh_.param<std::string>("output_frame", output_frame_, "");  // empty = use input frame

    pnh_.param<bool>("publish_clustered_cloud", publish_clustered_cloud_, true);

    pnh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.6);
    pnh_.param<int>("cluster_min_size", cluster_min_size_, 20);
    pnh_.param<int>("cluster_max_size", cluster_max_size_, 50000);

    pnh_.param<bool>("publish_markers", publish_markers_, true);
    pnh_.param<double>("marker_scale", marker_scale_, 0.5);
    pnh_.param<double>("marker_lifetime", marker_lifetime_, 0.2);

    // Visual object envelope mask parameters
    pnh_.param<bool>("enable_visual_object_mask", enable_visual_object_mask_, false);
    pnh_.param<std::string>("visual_objects_topic", visual_objects_topic_, "/warning/selected_visual_object_states");
    pnh_.param<double>("history_duration_sec", history_duration_sec_, 1.5);
    pnh_.param<double>("radius_margin_m", radius_margin_m_, 0.15);
    pnh_.param<double>("min_object_confidence", min_object_confidence_, 0.35);
    pnh_.param<int>("min_object_points", min_object_points_, 25);
    pnh_.param<double>("object_stale_timeout_sec", object_stale_timeout_sec_, 0.5);
    pnh_.param<double>("min_object_radius_m", min_object_radius_m_, 0.05);
    nh_.param<bool>("/use_sim_time", use_sim_time_, false);

    // Publishers
    pub_centers_ = nh_.advertise<geometry_msgs::PoseArray>(output_topic_, 5);
    if (publish_markers_)
    {
      pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 5);
    }
    if (publish_clustered_cloud_)
    {
      pub_clustered_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(clustered_cloud_topic_, 5);
    }

    // Subscribers
    sub_cloud_ = nh_.subscribe(input_topic_, 5, &DynClusterNode::cloudCallback, this);
    if (enable_visual_object_mask_)
    {
      sub_visual_objects_ = nh_.subscribe(visual_objects_topic_, 3, &DynClusterNode::visualObjectsCallback, this);
    }

    ROS_INFO_STREAM("[dyn_cluster_node] input=" << input_topic_
                    << ", output=" << output_topic_
                    << ", cluster_tolerance=" << cluster_tolerance_
                    << ", min_size=" << cluster_min_size_
                    << ", max_size=" << cluster_max_size_
                    << ", clustered_cloud=" << clustered_cloud_topic_
                    << ", enable_visual_object_mask=" << (enable_visual_object_mask_ ? "true" : "false"));
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
    if (!msg)
    {
      return;
    }
    latest_visual_objects_ = *msg;
    latest_visual_receive_time_ = ros::Time::now();
    has_visual_objects_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (!msg || msg->data.empty())
    {
      return;
    }

    // Maintain visual-object envelope history (same frame as cloud)
    if (enable_visual_object_mask_)
    {
      if (history_frame_id_.empty() || history_frame_id_ != msg->header.frame_id)
      {
        history_frame_id_ = msg->header.frame_id;
        history_spheres_.clear();
      }

      HistorySphere current;
      if (buildCurrentHistorySphere(*msg, &current))
      {
        history_spheres_.push_back(current);
      }
      pruneHistory(resolveStamp(msg->header.stamp));
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty())
    {
      return;
    }

    // Remove points inside the visual-object envelope before clustering
    pcl::PointCloud<pcl::PointXYZI>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (enable_visual_object_mask_ && !history_spheres_.empty())
    {
      working_cloud->reserve(cloud->size());
      for (const auto& pt : cloud->points)
      {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        {
          continue;
        }
        if (pointInsideHistory(pt))
        {
          continue;
        }
        working_cloud->push_back(pt);
      }
    }
    else
    {
      *working_cloud = *cloud;
    }

    if (working_cloud->empty())
    {
      geometry_msgs::PoseArray empty_centers;
      empty_centers.header = msg->header;
      if (!output_frame_.empty())
      {
        empty_centers.header.frame_id = output_frame_;
      }
      pub_centers_.publish(empty_centers);
      return;
    }

    // Euclidean clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(working_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(cluster_min_size_);
    ec.setMaxClusterSize(cluster_max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(working_cloud);
    ec.extract(cluster_indices);

    // Prepare output
    geometry_msgs::PoseArray centers_msg;
    centers_msg.header = msg->header;
    if (!output_frame_.empty())
    {
      centers_msg.header.frame_id = output_frame_;
    }

    visualization_msgs::MarkerArray markers_msg;
    int marker_id = 0;

    // Filtered point cloud (only clustered points)
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (const auto& indices : cluster_indices)
    {
      if (indices.indices.empty())
        continue;

      // Add points to filtered cloud
      for (int idx : indices.indices)
      {
        clustered_cloud->push_back(working_cloud->points[idx]);
      }

      // Compute bounding box center
      float minX = std::numeric_limits<float>::infinity();
      float minY = std::numeric_limits<float>::infinity();
      float minZ = std::numeric_limits<float>::infinity();
      float maxX = -std::numeric_limits<float>::infinity();
      float maxY = -std::numeric_limits<float>::infinity();
      float maxZ = -std::numeric_limits<float>::infinity();

      for (int idx : indices.indices)
      {
        const auto& p = working_cloud->points[idx];
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
        minZ = std::min(minZ, p.z);
        maxX = std::max(maxX, p.x);
        maxY = std::max(maxY, p.y);
        maxZ = std::max(maxZ, p.z);
      }

      float cx = 0.5f * (minX + maxX);
      float cy = 0.5f * (minY + maxY);
      float cz = 0.5f * (minZ + maxZ);

      // Add to PoseArray
      geometry_msgs::Pose pose;
      pose.position.x = cx;
      pose.position.y = cy;
      pose.position.z = cz;
      pose.orientation.w = 1.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      centers_msg.poses.push_back(pose);

      // Add visualization marker (bounding box)
      if (publish_markers_)
      {
        visualization_msgs::Marker marker;
        marker.header = centers_msg.header;
        marker.ns = "dyn_clusters";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = std::max(0.01f, maxX - minX);
        marker.scale.y = std::max(0.01f, maxY - minY);
        marker.scale.z = std::max(0.01f, maxZ - minZ);
        marker.color.r = 0.2f;
        marker.color.g = 1.0f;
        marker.color.b = 0.4f;
        marker.color.a = 0.75f;
        marker.lifetime = ros::Duration(marker_lifetime_);
        markers_msg.markers.push_back(marker);
      }
    }

    // Publish
    pub_centers_.publish(centers_msg);

    if (publish_markers_ && !markers_msg.markers.empty())
    {
      pub_markers_.publish(markers_msg);
    }

    // Publish filtered clustered point cloud
    if (publish_clustered_cloud_ && !clustered_cloud->empty())
    {
      sensor_msgs::PointCloud2 clustered_msg;
      pcl::toROSMsg(*clustered_cloud, clustered_msg);
      clustered_msg.header = msg->header;
      if (!output_frame_.empty())
      {
        clustered_msg.header.frame_id = output_frame_;
      }
      pub_clustered_cloud_.publish(clustered_msg);
    }

    ROS_DEBUG_THROTTLE(1.0, "[dyn_cluster_node] Published %zu clusters, %zu clustered points",
                       centers_msg.poses.size(), clustered_cloud->size());
  }

  // ---------------------------------------------------------------------------
  // Visual-object envelope helpers
  // ---------------------------------------------------------------------------

  bool buildCurrentHistorySphere(
    const sensor_msgs::PointCloud2& cloud_msg,
    HistorySphere* sphere)
  {
    if (!sphere || isVisualObjectsStale(resolveStamp(cloud_msg.header.stamp)))
    {
      return false;
    }

    lio_sam::SegmentedObjectState selected_object;
    if (!selectVisualObject(latest_visual_objects_, &selected_object))
    {
      return false;
    }

    geometry_msgs::Pose object_pose_cloud;
    const ros::Time object_stamp = resolveStamp(
      latest_visual_objects_.header.stamp.isZero()
        ? cloud_msg.header.stamp
        : latest_visual_objects_.header.stamp);

    if (!transformPose(
          selected_object.pose,
          latest_visual_objects_.header.frame_id,
          cloud_msg.header.frame_id,
          object_stamp,
          &object_pose_cloud))
    {
      return false;
    }

    sphere->center = object_pose_cloud.position;
    sphere->radius = computeObjectRadius(selected_object) + std::max(0.0, radius_margin_m_);
    sphere->stamp = object_stamp;
    return true;
  }

  bool selectVisualObject(
    const lio_sam::SegmentedObjectStateArray& objects_msg,
    lio_sam::SegmentedObjectState* selected_object) const
  {
    if (!selected_object)
    {
      return false;
    }

    const lio_sam::SegmentedObjectState* best = nullptr;
    for (const auto& object : objects_msg.objects)
    {
      if (object.confidence < min_object_confidence_)
      {
        continue;
      }
      if (static_cast<int>(object.point_count) < min_object_points_)
      {
        continue;
      }

      if (!best)
      {
        best = &object;
        continue;
      }

      if (object.nearest_range < best->nearest_range - 1e-3f)
      {
        best = &object;
        continue;
      }

      if (std::fabs(object.nearest_range - best->nearest_range) <= 1e-3f
          && object.confidence > best->confidence)
      {
        best = &object;
      }
    }

    if (!best)
    {
      return false;
    }

    *selected_object = *best;
    return true;
  }

  double computeObjectRadius(const lio_sam::SegmentedObjectState& object) const
  {
    double radius = 0.0;
    if (object.bounding_radius > 1e-3f)
    {
      radius = std::max(radius, static_cast<double>(object.bounding_radius));
    }
    if (object.footprint_radius > 1e-3f)
    {
      radius = std::max(radius, static_cast<double>(object.footprint_radius));
    }

    const double size_x = static_cast<double>(object.size.x);
    const double size_y = static_cast<double>(object.size.y);
    const double size_z = static_cast<double>(object.size.z);
    const double from_size = 0.5 * std::sqrt(
      size_x * size_x + size_y * size_y + size_z * size_z);
    radius = std::max(radius, from_size);

    return std::max(min_object_radius_m_, radius);
  }

  bool transformPose(
    const geometry_msgs::Pose& source_pose,
    const std::string& source_frame,
    const std::string& target_frame,
    const ros::Time& stamp,
    geometry_msgs::Pose* transformed_pose)
  {
    if (!transformed_pose)
    {
      return false;
    }
    if (source_frame.empty() || target_frame.empty() || source_frame == target_frame)
    {
      *transformed_pose = source_pose;
      return true;
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = source_frame;
    pose_stamped.pose = source_pose;

    try
    {
      const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        target_frame, source_frame, stamp, ros::Duration(0.05));
      tf2::doTransform(pose_stamped, pose_stamped, transform);
      *transformed_pose = pose_stamped.pose;
      return true;
    }
    catch (const tf2::TransformException&)
    {
    }

    try
    {
      const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        target_frame, source_frame, ros::Time(0), ros::Duration(0.05));
      tf2::doTransform(pose_stamped, pose_stamped, transform);
      *transformed_pose = pose_stamped.pose;
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(
        2.0,
        "[dyn_cluster_node] Failed to transform pose from %s to %s: %s",
        source_frame.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  bool isVisualObjectsStale(const ros::Time& reference_stamp) const
  {
    if (!has_visual_objects_)
    {
      return true;
    }

    const ros::Time freshness_stamp = (!use_sim_time_ && !latest_visual_receive_time_.isZero())
      ? latest_visual_receive_time_
      : latest_visual_objects_.header.stamp;
    if (freshness_stamp.isZero())
    {
      return true;
    }

    const ros::Time effective_reference = use_sim_time_ ? resolveStamp(reference_stamp) : ros::Time::now();
    if (effective_reference.isZero())
    {
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
    if (history_duration_sec_ <= 1e-6)
    {
      if (history_spheres_.size() > 1)
      {
        const HistorySphere latest = history_spheres_.back();
        history_spheres_.clear();
        history_spheres_.push_back(latest);
      }
      return;
    }

    const ros::Time effective_reference = resolveStamp(reference_stamp);
    while (!history_spheres_.empty())
    {
      const double age_sec = (effective_reference - history_spheres_.front().stamp).toSec();
      if (age_sec <= history_duration_sec_)
      {
        break;
      }
      history_spheres_.pop_front();
    }
  }

  bool pointInsideHistory(const pcl::PointXYZI& point) const
  {
    for (const auto& sphere : history_spheres_)
    {
      if (pointInsideSphere(point, sphere))
      {
        return true;
      }
    }

    if (history_spheres_.size() < 2)
    {
      return false;
    }

    for (size_t index = 1; index < history_spheres_.size(); ++index)
    {
      if (pointInsideSweptSegment(point, history_spheres_[index - 1], history_spheres_[index]))
      {
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
    if (segment_length_sq <= 1e-9)
    {
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

  // ---------------------------------------------------------------------------
  // Member variables
  // ---------------------------------------------------------------------------

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_visual_objects_;
  ros::Publisher pub_centers_;
  ros::Publisher pub_markers_;
  ros::Publisher pub_clustered_cloud_;

  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string clustered_cloud_topic_;
  std::string output_frame_;

  bool publish_clustered_cloud_;

  double cluster_tolerance_;
  int cluster_min_size_;
  int cluster_max_size_;

  bool publish_markers_;
  double marker_scale_;
  double marker_lifetime_;

  // Visual object mask
  bool enable_visual_object_mask_;
  std::string visual_objects_topic_;
  double history_duration_sec_;
  double radius_margin_m_;
  double min_object_confidence_;
  int min_object_points_;
  double object_stale_timeout_sec_;
  double min_object_radius_m_;
  bool use_sim_time_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  lio_sam::SegmentedObjectStateArray latest_visual_objects_;
  ros::Time latest_visual_receive_time_;
  bool has_visual_objects_{false};

  std::string history_frame_id_;
  std::deque<HistorySphere> history_spheres_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dyn_cluster_node");
  DynClusterNode node;
  ros::spin();
  return 0;
}
