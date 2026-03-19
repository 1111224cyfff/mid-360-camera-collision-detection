/**
 * @file dyn_cluster_node.cpp
 * @brief Clusters dynamic point cloud from M-detector and publishes cluster centers
 *        as geometry_msgs::PoseArray for dynamic_tracker consumption.
 *
 * Subscribes: /m_detector/point_out_map (sensor_msgs/PointCloud2)
 * Publishes:  /m_detector/dynamic_clusters (geometry_msgs/PoseArray)
 *             /m_detector/dynamic_cluster_markers (visualization_msgs/MarkerArray) [optional]
 *             /m_detector/clustered_cloud (sensor_msgs/PointCloud2) - filtered cloud with only clustered points
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>
#include <limits>

class DynClusterNode
{
public:
  DynClusterNode()
    : nh_(), pnh_("~")
  {
    // Parameters
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

    // Subscriber
    sub_cloud_ = nh_.subscribe(input_topic_, 5, &DynClusterNode::cloudCallback, this);

    ROS_INFO_STREAM("[dyn_cluster_node] input=" << input_topic_
                    << ", output=" << output_topic_
                    << ", cluster_tolerance=" << cluster_tolerance_
                    << ", min_size=" << cluster_min_size_
                    << ", max_size=" << cluster_max_size_
                    << ", clustered_cloud=" << clustered_cloud_topic_);
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (!msg || msg->data.empty())
    {
      return;
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty())
    {
      return;
    }

    // Euclidean clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(cluster_min_size_);
    ec.setMaxClusterSize(cluster_max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
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
        clustered_cloud->push_back(cloud->points[idx]);
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
        const auto& p = cloud->points[idx];
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

      // Add visualization marker
      if (publish_markers_)
      {
        visualization_msgs::Marker marker;
        marker.header = centers_msg.header;
        marker.ns = "dyn_clusters";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = marker_scale_;
        marker.scale.y = marker_scale_;
        marker.scale.z = marker_scale_;
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;
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

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_cloud_;
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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dyn_cluster_node");
  DynClusterNode node;
  ros::spin();
  return 0;
}
