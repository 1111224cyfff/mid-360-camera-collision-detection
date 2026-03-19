#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

class PointCloudTfBridge {
public:
  PointCloudTfBridge()
      : nh_(), pnh_("~"), tf_listener_(ros::Duration(20.0)) {
    pnh_.param<std::string>("input_topic", input_topic_, "/m_detector/point_out");
    pnh_.param<std::string>("output_topic", output_topic_, "/m_detector/point_out_map");
    pnh_.param<std::string>("target_frame", target_frame_, "map");
    pnh_.param<std::string>("source_frame_override", source_frame_override_, "");
    pnh_.param<double>("tf_wait_timeout", tf_wait_timeout_sec_, 0.05);
    pnh_.param<bool>("fallback_to_latest_tf", fallback_to_latest_tf_, true);
    pnh_.param<bool>("preserve_input_stamp", preserve_input_stamp_, true);
    pnh_.param<int>("queue_size", queue_size_, 5);

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 5);
    cloud_sub_ = nh_.subscribe(input_topic_, queue_size_, &PointCloudTfBridge::cloudCallback, this);

    ROS_INFO_STREAM("[pointcloud_tf_bridge] input=" << input_topic_
                    << ", output=" << output_topic_
                    << ", target_frame=" << target_frame_
                    << ", source_frame_override="
                    << (source_frame_override_.empty() ? "<empty>" : source_frame_override_)
                    << ", tf_wait_timeout=" << tf_wait_timeout_sec_ << "s"
                    << ", fallback_to_latest_tf=" << (fallback_to_latest_tf_ ? "true" : "false")
                    << ", preserve_input_stamp=" << (preserve_input_stamp_ ? "true" : "false"));
  }

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (!msg) {
      return;
    }

    sensor_msgs::PointCloud2 in_cloud = *msg;
    if (in_cloud.header.frame_id.empty() && !source_frame_override_.empty()) {
      in_cloud.header.frame_id = source_frame_override_;
    }

    if (in_cloud.header.frame_id.empty()) {
      ROS_WARN_THROTTLE(2.0, "[pointcloud_tf_bridge] drop cloud: empty frame_id and no source_frame_override");
      return;
    }

    if (in_cloud.header.frame_id == target_frame_) {
      cloud_pub_.publish(in_cloud);
      return;
    }

    sensor_msgs::PointCloud2 out_cloud;
    const ros::Duration timeout(tf_wait_timeout_sec_);
    bool transformed = false;
    bool used_latest_tf = false;

    try {
      if (tf_listener_.waitForTransform(target_frame_, in_cloud.header.frame_id,
                                        in_cloud.header.stamp, timeout)) {
        pcl_ros::transformPointCloud(target_frame_, in_cloud, out_cloud, tf_listener_);
        transformed = true;
      }
    } catch (const tf::TransformException &ex) {
      ROS_WARN_THROTTLE(1.0,
                        "[pointcloud_tf_bridge] exact-stamp transform exception: %s",
                        ex.what());
    }

    if (!transformed && fallback_to_latest_tf_) {
      sensor_msgs::PointCloud2 latest_cloud = in_cloud;
      latest_cloud.header.stamp = ros::Time(0);
      try {
        if (tf_listener_.waitForTransform(target_frame_, latest_cloud.header.frame_id,
                                          latest_cloud.header.stamp, timeout)) {
          pcl_ros::transformPointCloud(target_frame_, latest_cloud, out_cloud, tf_listener_);
          transformed = true;
          used_latest_tf = true;
          ROS_WARN_THROTTLE(1.0,
                            "[pointcloud_tf_bridge] exact-stamp TF missing (%s -> %s, stamp=%.3f), fallback to latest TF",
                            in_cloud.header.frame_id.c_str(), target_frame_.c_str(),
                            in_cloud.header.stamp.toSec());
        }
      } catch (const tf::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0,
                          "[pointcloud_tf_bridge] latest-TF transform exception: %s",
                          ex.what());
      }
    }

    if (!transformed) {
      ROS_WARN_THROTTLE(1.0,
                        "[pointcloud_tf_bridge] drop cloud: TF unavailable %s -> %s (stamp=%.3f)",
                        in_cloud.header.frame_id.c_str(), target_frame_.c_str(),
                        in_cloud.header.stamp.toSec());
      return;
    }

    if (preserve_input_stamp_ || !used_latest_tf) {
      out_cloud.header.stamp = in_cloud.header.stamp;
    } else {
      out_cloud.header.stamp = ros::Time::now();
    }

    cloud_pub_.publish(out_cloud);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string source_frame_override_;
  double tf_wait_timeout_sec_;
  bool fallback_to_latest_tf_;
  bool preserve_input_stamp_;
  int queue_size_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_tf_bridge");
  PointCloudTfBridge bridge;
  ros::spin();
  return 0;
}
