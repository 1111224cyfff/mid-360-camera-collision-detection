//
// Created by ubuntu on 4/7/23.
//
#include "chrono"
#include "opencv2/opencv.hpp"
#include "infer.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>

class RosNode
{
public:
    RosNode();
    ~RosNode(){};
    void callback(const sensor_msgs::ImageConstPtr &msg);

private:
    std::string pkg_path_, engine_file_path_;
    std::shared_ptr<YoloDetector> detector_;
    
    cv::Mat  img_res_;

    ros::NodeHandle n_;
    ros::Subscriber sub_img_;
    ros::Publisher pub_img_;
    std::string topic_img_;
    std::string topic_res_img_;
    std::string weight_name_;
};

RosNode::RosNode()
{
    cudaSetDevice(0);
    pkg_path_ = ros::package::getPath("yolo11_ros");
    
    n_.param<std::string>("topic_img", topic_img_, "/camera/color/image_raw");
    n_.param<std::string>("topic_res_img", topic_res_img_, "/pose/image_raw");
    n_.param<std::string>("weight_name",  weight_name_, "yolo11s-pose.engine");
        
    engine_file_path_ = pkg_path_ + "/weights/" + weight_name_;


    std::cout << "\n\033[1;32m--engine_file_path: " << engine_file_path_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m" << "--topic_img       : " << topic_img_  << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--topic_res_img   : " << topic_res_img_    << "\n\033[0m" << std::endl;

    detector_.reset(new YoloDetector(engine_file_path_));

    pub_img_ = n_.advertise<sensor_msgs::Image>(topic_res_img_, 10);
    sub_img_ = n_.subscribe(topic_img_, 10, &RosNode::callback, this);
}

void RosNode::callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    auto start = std::chrono::system_clock::now();
    auto detections = detector_->inference(image);
    auto end = std::chrono::system_clock::now();
    img_res_ = image.clone();
    YoloDetector::draw_image(img_res_, detections, true, true);
    
    auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;
    cv::putText(img_res_, "fps: " + std::to_string(int(1000/tc)) , cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1, 8);
    ROS_INFO("pose cost %2.4lf ms", tc);



    sensor_msgs::ImagePtr msg_img_new;
    msg_img_new = cv_bridge::CvImage(std_msgs::Header(),"bgr8",img_res_).toImageMsg();
	pub_img_.publish(msg_img_new);


}
int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "pose_node");
    ros::NodeHandle n;
    auto pose_node = std::make_shared<RosNode>();
    ros::spin();
    return 0;
}
