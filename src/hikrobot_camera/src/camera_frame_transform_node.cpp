#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <memory>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

class CameraFrameTransformNode {
public:
    CameraFrameTransformNode()
        : nh_(), pnh_("~") {
        pnh_.param<std::string>("input_topic", input_topic_, std::string("/hikrobot_camera/rgb/compressed"));
        pnh_.param<std::string>("output_topic", output_topic_, std::string("/hikrobot_camera/rgb/leveled/compressed"));
        pnh_.param<std::string>("source_frame", source_frame_, std::string("hikrobot_camera"));
        pnh_.param<std::string>("parent_frame", parent_frame_, std::string("body_leveled"));
        pnh_.param<std::string>("extrinsic_parent_frame", extrinsic_parent_frame_, std::string("body"));
        pnh_.param<std::string>("output_frame", output_frame_, std::string("hikrobot_camera_leveled"));
        pnh_.param<std::string>("extrinsic_direction", extrinsic_direction_, std::string("camera_to_lidar"));
        pnh_.param<bool>("publish_static_tf", publish_static_tf_, true);
        pnh_.param<double>("tf_lookup_timeout_sec", tf_lookup_timeout_sec_, 0.2);
        if (!pnh_.getParam("extrinsic_matrix", extrinsic_matrix_)) {
            throw std::runtime_error("missing required param ~extrinsic_matrix");
        }

        loadExtrinsicMatrix(extrinsic_matrix_, extrinsic_direction_, source_to_extrinsic_parent_);

        if (publish_static_tf_ && needsParentComposition()) {
            tf_buffer_.reset(new tf2_ros::Buffer());
            tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
        }

        image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(output_topic_, 1);
        image_sub_ = nh_.subscribe(input_topic_, 1, &CameraFrameTransformNode::imageCallback, this);

        if (publish_static_tf_) {
            tryPublishStaticTransform();
        }

        ROS_INFO_STREAM("camera_frame_transform ready: input_topic=" << input_topic_
                        << " output_topic=" << output_topic_
                        << " source_frame=" << source_frame_
                        << " parent_frame=" << parent_frame_
                        << " extrinsic_parent_frame=" << extrinsic_parent_frame_
                        << " output_frame=" << output_frame_
                        << " extrinsic_direction=" << extrinsic_direction_);
        ROS_INFO_STREAM("T_extrinsic_parent_from_source=\n" << source_to_extrinsic_parent_);
    }

private:
    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        if (publish_static_tf_ && !static_tf_published_) {
            tryPublishStaticTransform();
        }

        sensor_msgs::CompressedImage out = *msg;
        out.header.frame_id = output_frame_;
        image_pub_.publish(out);
    }

    bool needsParentComposition() const {
        return !extrinsic_parent_frame_.empty() && extrinsic_parent_frame_ != parent_frame_;
    }

    void tryPublishStaticTransform() {
        if (static_tf_published_) {
            return;
        }

        cv::Matx44d source_to_parent = source_to_extrinsic_parent_;
        if (needsParentComposition()) {
            if (!tf_buffer_) {
                ROS_WARN_THROTTLE(2.0, "camera_frame_transform missing TF buffer while parent composition is required");
                return;
            }

            try {
                const geometry_msgs::TransformStamped parent_from_extrinsic_parent_msg =
                    tf_buffer_->lookupTransform(parent_frame_, extrinsic_parent_frame_, ros::Time(0),
                                                ros::Duration(tf_lookup_timeout_sec_));
                const cv::Matx44d extrinsic_parent_to_parent = transformToMatrix(parent_from_extrinsic_parent_msg);
                source_to_parent = extrinsic_parent_to_parent * source_to_extrinsic_parent_;
            } catch (const tf2::TransformException& ex) {
                ROS_WARN_THROTTLE(2.0,
                                  "camera_frame_transform waiting for TF %s <- %s before publishing %s: %s",
                                  parent_frame_.c_str(),
                                  extrinsic_parent_frame_.c_str(),
                                  output_frame_.c_str(),
                                  ex.what());
                return;
            }
        }

        publishStaticTransform(source_to_parent);
        static_tf_published_ = true;
        ROS_INFO_STREAM("camera_frame_transform published TF " << parent_frame_ << " -> " << output_frame_
                        << " with T_parent_from_source=\n" << source_to_parent);
    }

    void publishStaticTransform(const cv::Matx44d& source_to_parent) {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = parent_frame_;
        tf_msg.child_frame_id = output_frame_;
        tf_msg.transform.translation.x = source_to_parent(0, 3);
        tf_msg.transform.translation.y = source_to_parent(1, 3);
        tf_msg.transform.translation.z = source_to_parent(2, 3);

        tf2::Matrix3x3 rotation(source_to_parent(0, 0), source_to_parent(0, 1), source_to_parent(0, 2),
                                source_to_parent(1, 0), source_to_parent(1, 1), source_to_parent(1, 2),
                                source_to_parent(2, 0), source_to_parent(2, 1), source_to_parent(2, 2));
        tf2::Quaternion quaternion;
        rotation.getRotation(quaternion);

        tf_msg.transform.rotation.x = quaternion.x();
        tf_msg.transform.rotation.y = quaternion.y();
        tf_msg.transform.rotation.z = quaternion.z();
        tf_msg.transform.rotation.w = quaternion.w();
        static_tf_broadcaster_.sendTransform(tf_msg);
    }

    static cv::Matx44d transformToMatrix(const geometry_msgs::TransformStamped& transform) {
        tf2::Quaternion quaternion(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        tf2::Matrix3x3 rotation(quaternion);

        return cv::Matx44d(
            rotation[0][0], rotation[0][1], rotation[0][2], transform.transform.translation.x,
            rotation[1][0], rotation[1][1], rotation[1][2], transform.transform.translation.y,
            rotation[2][0], rotation[2][1], rotation[2][2], transform.transform.translation.z,
            0.0, 0.0, 0.0, 1.0);
    }

    static void loadExtrinsicMatrix(const std::vector<double>& values,
                                    const std::string& direction,
                                    cv::Matx44d& source_to_target) {
        if (values.size() != 16U) {
            throw std::runtime_error("extrinsic_matrix must contain exactly 16 values");
        }

        cv::Matx44d matrix(values[0], values[1], values[2], values[3],
                           values[4], values[5], values[6], values[7],
                           values[8], values[9], values[10], values[11],
                           values[12], values[13], values[14], values[15]);

        if (direction == "camera_to_lidar") {
            source_to_target = matrix;
            return;
        }
        if (direction == "lidar_to_camera") {
            source_to_target = matrix.inv();
            return;
        }
        throw std::runtime_error("extrinsic_direction must be 'camera_to_lidar' or 'lidar_to_camera'");
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string input_topic_;
    std::string output_topic_;
    std::string source_frame_;
    std::string parent_frame_;
    std::string extrinsic_parent_frame_;
    std::string output_frame_;
    std::string extrinsic_direction_;
    bool publish_static_tf_ = true;
    bool static_tf_published_ = false;
    double tf_lookup_timeout_sec_ = 0.2;
    std::vector<double> extrinsic_matrix_;
    cv::Matx44d source_to_extrinsic_parent_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_frame_transform");
    try {
        CameraFrameTransformNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("camera_frame_transform init failed: %s", e.what());
        return 1;
    }
    return 0;
}
