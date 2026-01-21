#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <livox_ros_driver2/CustomMsg.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace
{
int findFieldIndex(const sensor_msgs::PointCloud2& msg, const std::string& name)
{
  for (size_t i = 0; i < msg.fields.size(); ++i)
  {
    if (msg.fields[i].name == name)
      return static_cast<int>(i);
  }
  return -1;
}

std::string fieldDatatypeName(uint8_t datatype)
{
  using PF = sensor_msgs::PointField;
  switch (datatype)
  {
    case PF::INT8:
      return "INT8";
    case PF::UINT8:
      return "UINT8";
    case PF::INT16:
      return "INT16";
    case PF::UINT16:
      return "UINT16";
    case PF::INT32:
      return "INT32";
    case PF::UINT32:
      return "UINT32";
    case PF::FLOAT32:
      return "FLOAT32";
    case PF::FLOAT64:
      return "FLOAT64";
    default:
      return "UNKNOWN";
  }
}

Eigen::Matrix3d rotationFromTwoVectors(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
  // Eigen::Quaterniond::FromTwoVectors gives the minimal rotation mapping "from" onto "to".
  Eigen::Quaterniond q;
  q.setFromTwoVectors(from, to);
  return q.normalized().toRotationMatrix();
}
}  // namespace

class MergedPointcloudLeveler
{
public:
  explicit MergedPointcloudLeveler(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<std::string>("imu_topic", imu_topic_, std::string("/merged_imu"));
    pnh.param<std::string>("imu_output_topic", imu_output_topic_, std::string("/merged_imu_leveled"));
    pnh.param<std::string>("cloud_topic", cloud_topic_, std::string("/merged_pointcloud"));
    pnh.param<std::string>("output_topic", output_topic_, std::string("/merged_pointcloud_leveled"));

    pnh.param<std::string>("livox_custom_topic", livox_custom_topic_, std::string("/merged_livox"));
    pnh.param<std::string>("livox_custom_output_topic", livox_custom_output_topic_, std::string("/merged_livox_leveled"));

    pnh.param<double>("calib_duration_sec", calib_duration_sec_, 2.0);
    pnh.param<int>("min_imu_samples", min_imu_samples_, 800);  // 2s @ 400Hz

    // For your use-case (one-time calibration), default is to publish nothing before calibration is done.
    pnh.param<bool>("publish_before_calibrated", publish_before_calibrated_, false);

    // Target "up" direction in the leveled output frame. In ROS REP-103, Z is up.
    target_up_ = Eigen::Vector3d(0.0, 0.0, 1.0);

    imu_sub_ = nh.subscribe(imu_topic_, 4000, &MergedPointcloudLeveler::imuCb, this);
    cloud_sub_ = nh.subscribe(cloud_topic_, 50, &MergedPointcloudLeveler::cloudCb, this);
    livox_custom_sub_ = nh.subscribe(livox_custom_topic_, 200, &MergedPointcloudLeveler::livoxCustomCb, this);

    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 50);
    livox_custom_pub_ = nh.advertise<livox_ros_driver2::CustomMsg>(livox_custom_output_topic_, 200);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>(imu_output_topic_, 4000);

    ROS_INFO_STREAM("[merged_pointcloud_leveler] Subscribed imu_topic=" << imu_topic_ << ", cloud_topic=" << cloud_topic_
                                   << ", livox_custom_topic=" << livox_custom_topic_);
    ROS_INFO_STREAM("[merged_pointcloud_leveler] Publishing PointCloud2: " << output_topic_);
    ROS_INFO_STREAM("[merged_pointcloud_leveler] Publishing CustomMsg:   " << livox_custom_output_topic_);
    ROS_INFO_STREAM("[merged_pointcloud_leveler] Publishing IMU:         " << imu_output_topic_);
    ROS_INFO_STREAM("[merged_pointcloud_leveler] One-time calibration: duration=" << calib_duration_sec_
                                                                                 << " sec, min_samples=" << min_imu_samples_);
  }

private:
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg)
  {
    // During calibration phase, accumulate accel samples.
    if (!calibrated_)
    {
      const Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
      if (!acc.allFinite())
        return;

      if (imu_samples_ == 0)
        first_imu_stamp_ = msg->header.stamp;

      acc_sum_ += acc;
      ++imu_samples_;

      const double elapsed = (msg->header.stamp - first_imu_stamp_).toSec();
      if (imu_samples_ >= min_imu_samples_ && elapsed >= calib_duration_sec_)
      {
        Eigen::Vector3d avg = acc_sum_ / static_cast<double>(imu_samples_);
        const double norm = avg.norm();
        if (norm < 1e-6)
        {
          ROS_WARN("[merged_pointcloud_leveler] IMU accel average norm too small; cannot calibrate.");
          return;
        }

        // When static, accelerometer measures "up" (proper acceleration) ~= +g.
        Eigen::Vector3d up = avg / norm;

        R_level_ = rotationFromTwoVectors(up, target_up_);
        q_level_ = Eigen::Quaterniond(R_level_).normalized();
        calibrated_ = true;

        ROS_INFO_STREAM("[merged_pointcloud_leveler] Calibrated using " << imu_samples_ << " IMU samples over " << elapsed
                                                                         << " sec");
        ROS_INFO_STREAM("[merged_pointcloud_leveler] avg_accel=" << avg.transpose() << ", up=" << up.transpose());
        ROS_INFO_STREAM("[merged_pointcloud_leveler] R_level=\n" << R_level_);
        ROS_INFO_STREAM("[merged_pointcloud_leveler] q_level (x y z w)= " << q_level_.x() << " " << q_level_.y() << " "
                                                                            << q_level_.z() << " " << q_level_.w());
      }

      if (!calibrated_ && !publish_before_calibrated_)
        return;
    }

    // After calibrated, publish a leveled IMU (vector quantities rotated into the leveled frame).
    sensor_msgs::Imu out = *msg;
    out.header.frame_id = msg->header.frame_id;  // keep original frame_id unless you want to remap it

    const Eigen::Vector3d acc_in(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    const Eigen::Vector3d gyr_in(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    const Eigen::Vector3d acc_out = R_level_ * acc_in;
    const Eigen::Vector3d gyr_out = R_level_ * gyr_in;

    out.linear_acceleration.x = acc_out.x();
    out.linear_acceleration.y = acc_out.y();
    out.linear_acceleration.z = acc_out.z();
    out.angular_velocity.x = gyr_out.x();
    out.angular_velocity.y = gyr_out.y();
    out.angular_velocity.z = gyr_out.z();

    // If the sensor provides a valid orientation quaternion, rotate it into the leveled frame as well.
    const Eigen::Quaterniond q_in(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    if (std::abs(q_in.norm() - 1.0) < 0.5)
    {
      const Eigen::Quaterniond q_out = (q_level_ * q_in).normalized();
      out.orientation.w = q_out.w();
      out.orientation.x = q_out.x();
      out.orientation.y = q_out.y();
      out.orientation.z = q_out.z();
    }

    imu_pub_.publish(out);
  }

  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    if (!calibrated_)
    {
      ROS_WARN_THROTTLE(2.0, "[merged_pointcloud_leveler] Waiting for IMU calibration; dropping PointCloud2 until calibrated");
      if (publish_before_calibrated_)
        cloud_pub_.publish(*msg);
      return;
    }

    const int ix = findFieldIndex(*msg, "x");
    const int iy = findFieldIndex(*msg, "y");
    const int iz = findFieldIndex(*msg, "z");
    if (ix < 0 || iy < 0 || iz < 0)
    {
      ROS_ERROR_THROTTLE(2.0, "[merged_pointcloud_leveler] Input cloud missing x/y/z fields");
      return;
    }

    const uint8_t dx = msg->fields[ix].datatype;
    const uint8_t dy = msg->fields[iy].datatype;
    const uint8_t dz = msg->fields[iz].datatype;
    if (dx != dy || dx != dz)
    {
      ROS_ERROR_THROTTLE(2.0, "[merged_pointcloud_leveler] x/y/z datatypes differ: x=%s y=%s z=%s",
                         fieldDatatypeName(dx).c_str(), fieldDatatypeName(dy).c_str(), fieldDatatypeName(dz).c_str());
      return;
    }

    sensor_msgs::PointCloud2 out = *msg;

    if (dx == sensor_msgs::PointField::FLOAT32)
    {
      sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
      sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
      sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");

      for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
      {
        const Eigen::Vector3d p(*it_x, *it_y, *it_z);
        const Eigen::Vector3d pr = R_level_ * p;
        *it_x = static_cast<float>(pr.x());
        *it_y = static_cast<float>(pr.y());
        *it_z = static_cast<float>(pr.z());
      }
    }
    else if (dx == sensor_msgs::PointField::FLOAT64)
    {
      sensor_msgs::PointCloud2Iterator<double> it_x(out, "x");
      sensor_msgs::PointCloud2Iterator<double> it_y(out, "y");
      sensor_msgs::PointCloud2Iterator<double> it_z(out, "z");

      for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
      {
        const Eigen::Vector3d p(*it_x, *it_y, *it_z);
        const Eigen::Vector3d pr = R_level_ * p;
        *it_x = pr.x();
        *it_y = pr.y();
        *it_z = pr.z();
      }
    }
    else
    {
      ROS_ERROR_THROTTLE(2.0, "[merged_pointcloud_leveler] Unsupported x/y/z datatype: %s",
                         fieldDatatypeName(dx).c_str());
      return;
    }

    cloud_pub_.publish(out);
  }

  void livoxCustomCb(const livox_ros_driver2::CustomMsg::ConstPtr& msg)
  {
    if (!calibrated_)
    {
      ROS_WARN_THROTTLE(2.0, "[merged_pointcloud_leveler] Waiting for IMU calibration; dropping CustomMsg until calibrated");
      if (publish_before_calibrated_)
        livox_custom_pub_.publish(*msg);
      return;
    }

    livox_ros_driver2::CustomMsg out = *msg;
    for (auto& pt : out.points)
    {
      const Eigen::Vector3d p(pt.x, pt.y, pt.z);
      const Eigen::Vector3d pr = R_level_ * p;
      pt.x = static_cast<float>(pr.x());
      pt.y = static_cast<float>(pr.y());
      pt.z = static_cast<float>(pr.z());
    }
    livox_custom_pub_.publish(out);
  }

  ros::Subscriber imu_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber livox_custom_sub_;
  ros::Publisher cloud_pub_;
  ros::Publisher livox_custom_pub_;
  ros::Publisher imu_pub_;

  std::string imu_topic_;
  std::string imu_output_topic_;
  std::string cloud_topic_;
  std::string output_topic_;

  std::string livox_custom_topic_;
  std::string livox_custom_output_topic_;

  double calib_duration_sec_{2.0};
  int min_imu_samples_{800};
  bool publish_before_calibrated_{false};

  bool calibrated_{false};
  ros::Time first_imu_stamp_;
  int imu_samples_{0};
  Eigen::Vector3d acc_sum_{0.0, 0.0, 0.0};

  Eigen::Vector3d target_up_{0.0, 0.0, 1.0};
  Eigen::Matrix3d R_level_{Eigen::Matrix3d::Identity()};
  Eigen::Quaterniond q_level_{Eigen::Quaterniond::Identity()};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "merged_pointcloud_leveler");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  MergedPointcloudLeveler node(nh, pnh);

  ros::spin();
  return 0;
}
