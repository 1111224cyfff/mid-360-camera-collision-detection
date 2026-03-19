#include <iostream>
#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgcodecs.hpp>
#include "hikrobot_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif 

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    //string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);
    //********** rosnode init **********/
    bool publish_rgb = true;
    hikrobot_camera.param("publish_rgb", publish_rgb, true);

    bool publish_rgb_compressed = false;
    hikrobot_camera.param("publish_rgb_compressed", publish_rgb_compressed, false);

    std::string compressed_format = "jpeg";
    hikrobot_camera.param<std::string>("compressed_format", compressed_format, std::string("jpeg"));
    int compressed_jpeg_quality = 80;
    hikrobot_camera.param("compressed_jpeg_quality", compressed_jpeg_quality, 80);
    int compressed_png_level = 3;
    hikrobot_camera.param("compressed_png_level", compressed_png_level, 3);
    double compressed_max_fps = 0.0;
    hikrobot_camera.param("compressed_max_fps", compressed_max_fps, 0.0);
    std::string compressed_output_topic = "/hikrobot_camera/rgb/compressed";
    hikrobot_camera.param<std::string>("compressed_output_topic", compressed_output_topic,
                                       std::string("/hikrobot_camera/rgb/compressed"));

    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    image_transport::CameraPublisher image_pub;
    if (publish_rgb)
    {
        image_pub = main_cam_image.advertiseCamera("/hikrobot_camera/rgb", 1000);
    }

    ros::Publisher camera_info_pub;
    if (!publish_rgb)
    {
        camera_info_pub = hikrobot_camera.advertise<sensor_msgs::CameraInfo>("/hikrobot_camera/camera_info", 10);
    }

    ros::Publisher compressed_pub;
    if (publish_rgb_compressed)
    {
        compressed_pub = hikrobot_camera.advertise<sensor_msgs::CompressedImage>(compressed_output_topic, 1);
    }

    ros::Time last_compressed_stamp;

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 
    
    //********** 10 Hz        **********/
    ros::Rate loop_rate(20);

    while (ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }
#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x,FIT_min_y,FIT_max_x-FIT_min_x,FIT_max_y-FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr->image = src;
#endif
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
        image_msg.header.frame_id = "hikrobot_camera";

        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp = image_msg.header.stamp;

        if (publish_rgb)
        {
            image_pub.publish(image_msg, camera_info_msg);
        }
        else
        {
            if (camera_info_pub)
            {
                camera_info_pub.publish(camera_info_msg);
            }
        }

        if (publish_rgb_compressed && compressed_pub)
        {
            if (compressed_max_fps > 0.0 && !last_compressed_stamp.isZero())
            {
                const double min_dt = 1.0 / compressed_max_fps;
                if ((image_msg.header.stamp - last_compressed_stamp).toSec() < min_dt)
                {
                    continue;
                }
            }

            std::string ext;
            std::vector<int> params;
            if (compressed_format == "jpeg" || compressed_format == "jpg")
            {
                ext = ".jpg";
                params.push_back(cv::IMWRITE_JPEG_QUALITY);
                params.push_back(std::max(0, std::min(100, compressed_jpeg_quality)));
            }
            else if (compressed_format == "png")
            {
                ext = ".png";
                params.push_back(cv::IMWRITE_PNG_COMPRESSION);
                params.push_back(std::max(0, std::min(9, compressed_png_level)));
            }
            else
            {
                ROS_WARN_THROTTLE(2.0, "Unsupported compressed_format '%s' (use jpeg/png).", compressed_format.c_str());
                continue;
            }

            std::vector<unsigned char> buffer;
            try
            {
                if (!cv::imencode(ext, cv_ptr->image, buffer, params))
                {
                    ROS_WARN_THROTTLE(2.0, "cv::imencode failed.");
                    continue;
                }
            }
            catch (const cv::Exception& e)
            {
                ROS_WARN_THROTTLE(2.0, "OpenCV imencode exception: %s", e.what());
                continue;
            }

            sensor_msgs::CompressedImage out;
            out.header = image_msg.header;
            out.format = (compressed_format == "jpg") ? "jpeg" : compressed_format;
            out.data = std::move(buffer);
            compressed_pub.publish(out);
            last_compressed_stamp = image_msg.header.stamp;
        }

        //*******************************************************************************************************************/
    }
    return 0;
}
