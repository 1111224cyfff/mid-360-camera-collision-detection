#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

class PointCloudRoiFilter {
public:
  PointCloudRoiFilter()
      : nh_(), pnh_("~"), tf_listener_(ros::Duration(20.0)) {
    pnh_.param<std::string>("input_topic", input_topic_, "/lio_sam/mapping/cloud_registered");
    pnh_.param<std::string>("output_topic", output_topic_, "/perception/near_field_cloud_registered");
    pnh_.param<std::string>("crop_frame", crop_frame_, "body_leveled");
    pnh_.param<double>("xy_radius", xy_radius_, 10.0);
    pnh_.param<double>("tf_wait_timeout", tf_wait_timeout_sec_, 0.05);
    pnh_.param<bool>("fallback_to_latest_tf", fallback_to_latest_tf_, true);
    pnh_.param<bool>("preserve_input_stamp", preserve_input_stamp_, true);
    pnh_.param<int>("queue_size", queue_size_, 5);

    xy_radius_sq_ = xy_radius_ * xy_radius_;

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 5);
    cloud_sub_ = nh_.subscribe(input_topic_, queue_size_, &PointCloudRoiFilter::cloudCallback, this);

    ROS_INFO_STREAM("[pointcloud_roi_filter] input=" << input_topic_
                    << ", output=" << output_topic_
                    << ", crop_frame=" << crop_frame_
                    << ", xy_radius=" << xy_radius_
                    << "m, tf_wait_timeout=" << tf_wait_timeout_sec_
                    << "s, fallback_to_latest_tf=" << (fallback_to_latest_tf_ ? "true" : "false")
                    << ", preserve_input_stamp=" << (preserve_input_stamp_ ? "true" : "false"));
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (!msg) {
      return;
    }

    sensor_msgs::PointCloud2 crop_cloud;
    bool used_latest_tf = false;
    if (!transformToCropFrame(*msg, &crop_cloud, &used_latest_tf)) {
      return;
    }

    sensor_msgs::PointCloud2 filtered_cloud;
    if (!buildFilteredCloud(*msg, crop_cloud, &filtered_cloud)) {
      return;
    }

    if (preserve_input_stamp_ || !used_latest_tf) {
      filtered_cloud.header.stamp = msg->header.stamp;
    } else {
      filtered_cloud.header.stamp = ros::Time::now();
    }
    filtered_cloud.header.frame_id = msg->header.frame_id;
    cloud_pub_.publish(filtered_cloud);
  }

  bool transformToCropFrame(const sensor_msgs::PointCloud2 &in_cloud,
                            sensor_msgs::PointCloud2 *out_cloud,
                            bool *used_latest_tf) {
    if (!out_cloud || !used_latest_tf) {
      return false;
    }

    *used_latest_tf = false;
    if (in_cloud.header.frame_id.empty()) {
      ROS_WARN_THROTTLE(2.0, "[pointcloud_roi_filter] drop cloud: empty frame_id");
      return false;
    }

    if (in_cloud.header.frame_id == crop_frame_) {
      *out_cloud = in_cloud;
      return true;
    }

    const ros::Duration timeout(tf_wait_timeout_sec_);
    try {
      if (tf_listener_.waitForTransform(crop_frame_, in_cloud.header.frame_id,
                                        in_cloud.header.stamp, timeout)) {
        pcl_ros::transformPointCloud(crop_frame_, in_cloud, *out_cloud, tf_listener_);
        return true;
      }
    } catch (const tf::TransformException &ex) {
      ROS_WARN_THROTTLE(1.0,
                        "[pointcloud_roi_filter] exact-stamp transform exception: %s",
                        ex.what());
    }

    if (!fallback_to_latest_tf_) {
      ROS_WARN_THROTTLE(1.0,
                        "[pointcloud_roi_filter] drop cloud: TF unavailable %s -> %s (stamp=%.3f)",
                        in_cloud.header.frame_id.c_str(), crop_frame_.c_str(),
                        in_cloud.header.stamp.toSec());
      return false;
    }

    sensor_msgs::PointCloud2 latest_cloud = in_cloud;
    latest_cloud.header.stamp = ros::Time(0);
    try {
      if (tf_listener_.waitForTransform(crop_frame_, latest_cloud.header.frame_id,
                                        latest_cloud.header.stamp, timeout)) {
        pcl_ros::transformPointCloud(crop_frame_, latest_cloud, *out_cloud, tf_listener_);
        *used_latest_tf = true;
        ROS_WARN_THROTTLE(1.0,
                          "[pointcloud_roi_filter] exact-stamp TF missing (%s -> %s, stamp=%.3f), fallback to latest TF",
                          in_cloud.header.frame_id.c_str(), crop_frame_.c_str(),
                          in_cloud.header.stamp.toSec());
        return true;
      }
    } catch (const tf::TransformException &ex) {
      ROS_WARN_THROTTLE(1.0,
                        "[pointcloud_roi_filter] latest-TF transform exception: %s",
                        ex.what());
    }

    ROS_WARN_THROTTLE(1.0,
                      "[pointcloud_roi_filter] drop cloud: TF unavailable %s -> %s (stamp=%.3f)",
                      in_cloud.header.frame_id.c_str(), crop_frame_.c_str(),
                      in_cloud.header.stamp.toSec());
    return false;
  }

  bool buildFilteredCloud(const sensor_msgs::PointCloud2 &original_cloud,
                          const sensor_msgs::PointCloud2 &crop_cloud,
                          sensor_msgs::PointCloud2 *filtered_cloud) {
    if (!filtered_cloud) {
      return false;
    }

    const std::size_t point_count = static_cast<std::size_t>(original_cloud.width) *
                                    static_cast<std::size_t>(original_cloud.height);
    const std::size_t crop_point_count = static_cast<std::size_t>(crop_cloud.width) *
                                         static_cast<std::size_t>(crop_cloud.height);
    if (point_count != crop_point_count ||
        original_cloud.point_step != crop_cloud.point_step ||
        original_cloud.data.size() < point_count * original_cloud.point_step ||
        crop_cloud.data.size() < crop_point_count * crop_cloud.point_step) {
      ROS_WARN_THROTTLE(1.0,
                        "[pointcloud_roi_filter] drop cloud: transformed cloud layout mismatch");
      return false;
    }

    int x_offset = -1;
    int y_offset = -1;
    if (!findFloat32FieldOffset(crop_cloud, "x", &x_offset) ||
        !findFloat32FieldOffset(crop_cloud, "y", &y_offset)) {
      ROS_WARN_THROTTLE(1.0,
                        "[pointcloud_roi_filter] drop cloud: missing float32 x/y fields");
      return false;
    }

    *filtered_cloud = original_cloud;
    filtered_cloud->data.clear();
    filtered_cloud->data.reserve(point_count * original_cloud.point_step);

    std::size_t kept_points = 0;
    for (std::size_t index = 0; index < point_count; ++index) {
      const std::size_t point_offset = index * crop_cloud.point_step;
      float x = 0.0f;
      float y = 0.0f;
      std::memcpy(&x, &crop_cloud.data[point_offset + static_cast<std::size_t>(x_offset)], sizeof(float));
      std::memcpy(&y, &crop_cloud.data[point_offset + static_cast<std::size_t>(y_offset)], sizeof(float));

      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      if (static_cast<double>(x) * static_cast<double>(x) +
              static_cast<double>(y) * static_cast<double>(y) >
          xy_radius_sq_) {
        continue;
      }

      const std::size_t original_offset = index * original_cloud.point_step;
      filtered_cloud->data.insert(
          filtered_cloud->data.end(),
          original_cloud.data.begin() + static_cast<std::ptrdiff_t>(original_offset),
          original_cloud.data.begin() +
              static_cast<std::ptrdiff_t>(original_offset + original_cloud.point_step));
      ++kept_points;
    }

    filtered_cloud->height = 1;
    filtered_cloud->width = static_cast<std::uint32_t>(kept_points);
    filtered_cloud->row_step = filtered_cloud->point_step * filtered_cloud->width;
    filtered_cloud->is_dense = original_cloud.is_dense;
    return true;
  }

  bool findFloat32FieldOffset(const sensor_msgs::PointCloud2 &cloud,
                              const std::string &field_name,
                              int *offset) const {
    if (!offset) {
      return false;
    }

    for (const auto &field : cloud.fields) {
      if (field.name != field_name) {
        continue;
      }
      if (field.datatype != sensor_msgs::PointField::FLOAT32) {
        return false;
      }
      *offset = static_cast<int>(field.offset);
      return true;
    }

    return false;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string crop_frame_;
  double xy_radius_{10.0};
  double xy_radius_sq_{100.0};
  double tf_wait_timeout_sec_{0.05};
  bool fallback_to_latest_tf_{true};
  bool preserve_input_stamp_{true};
  int queue_size_{5};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_roi_filter");
  PointCloudRoiFilter filter;
  ros::spin();
  return 0;
}