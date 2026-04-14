#include "chrono"
#include <cmath>
#include "opencv2/opencv.hpp"
#include <lio_sam/SegmentedObjectStateArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/package.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <limits>
#include <mutex>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <algorithm>

// pcl headers
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "infer.h"

namespace {
struct SegmentObject {
    cv::Rect_<float> rect;
    int label = 0;
    float prob = 0.0f;
    cv::Mat boxMask;
};

struct ProjectionModel {
    cv::Mat intrinsic;
    cv::Mat final_rotation_matrix;
    cv::Mat t_vec;
    bool flip_lidar_y = false;

    ProjectionModel()
    {
        intrinsic = cv::Mat::eye(3, 3, CV_64F);
        final_rotation_matrix = cv::Mat::eye(3, 3, CV_64F);
        t_vec = cv::Mat::zeros(3, 1, CV_64F);
    }

    void loadFromFile(const std::string& config_path)
    {
        std::ifstream config_stream(config_path);
        if (!config_stream.is_open()) {
            throw std::runtime_error("Unable to open projection config: " + config_path);
        }

        std::ostringstream sanitized_config;
        std::string line;
        bool is_first_line = true;
        while (std::getline(config_stream, line)) {
            std::string trimmed = line;
            trimmed.erase(0, trimmed.find_first_not_of(" \t"));

            if (is_first_line) {
                if (trimmed == "# %YAML:1.0" || trimmed == "%YAML:1.0") {
                    sanitized_config << "%YAML:1.0\n";
                    is_first_line = false;
                    continue;
                }
            }

            if (!is_first_line && !trimmed.empty() && trimmed[0] == '#') {
                continue;
            }

            sanitized_config << line << '\n';
            is_first_line = false;
        }

        cv::FileStorage fs(sanitized_config.str(), cv::FileStorage::READ | cv::FileStorage::MEMORY);
        if (!fs.isOpened()) {
            throw std::runtime_error("Unable to open projection config: " + config_path);
        }

        fs["intrinsic"] >> intrinsic;
        fs["final_rotation_matrix"] >> final_rotation_matrix;
        fs["t_vec"] >> t_vec;

        if (intrinsic.size() != cv::Size(3, 3) || intrinsic.type() != CV_64F) {
            throw std::runtime_error("Invalid intrinsic matrix in projection config: " + config_path);
        }
        if (final_rotation_matrix.size() != cv::Size(3, 3) || final_rotation_matrix.type() != CV_64F) {
            throw std::runtime_error("Invalid final_rotation_matrix in projection config: " + config_path);
        }
        if (t_vec.rows != 3 || t_vec.cols != 1 || t_vec.type() != CV_64F) {
            throw std::runtime_error("Invalid t_vec in projection config: " + config_path);
        }
    }
};

struct StageStats {
    double min = std::numeric_limits<double>::infinity();
    double max = 0.0;
    double sum = 0.0;
    std::size_t count = 0;

    void add(double value)
    {
        min = std::min(min, value);
        max = std::max(max, value);
        sum += value;
        ++count;
    }

    double avg() const
    {
        return count == 0 ? 0.0 : sum / static_cast<double>(count);
    }

    void reset()
    {
        min = std::numeric_limits<double>::infinity();
        max = 0.0;
        sum = 0.0;
        count = 0;
    }
};

cv::Point2d projectToImagePlane(const pcl::PointXYZI& point, const ProjectionModel& model)
{
    const double lidar_y = model.flip_lidar_y ? -point.y : point.y;
    cv::Mat point3d = (cv::Mat_<double>(4, 1) << point.x, lidar_y, point.z, 1);
    cv::Mat point_cam = model.final_rotation_matrix * point3d.rowRange(0, 3) + model.t_vec;
    if (point_cam.at<double>(2) <= 0.0) {
        return cv::Point2d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    }
    cv::Mat image_point = model.intrinsic * point_cam;
    return cv::Point2d(
        image_point.at<double>(0) / image_point.at<double>(2),
        image_point.at<double>(1) / image_point.at<double>(2));
}

bool isPointInsideInstance(const cv::Mat& instanceMask, int x, int y)
{
    if (x < 0 || x >= instanceMask.cols || y < 0 || y >= instanceMask.rows) {
        return false;
    }
    return instanceMask.at<uchar>(y, x) != 0;
}

std::vector<SegmentObject> convertDetectionsToObjects(const std::vector<Detection>& detections, const cv::Size& imageSize)
{
    std::vector<SegmentObject> objects;
    objects.reserve(detections.size());
    for (const auto& det : detections) {
        int x1 = std::max(0, std::min((int)std::floor(det.bbox[0]), imageSize.width - 1));
        int y1 = std::max(0, std::min((int)std::floor(det.bbox[1]), imageSize.height - 1));
        int x2 = std::max(0, std::min((int)std::ceil(det.bbox[2]), imageSize.width));
        int y2 = std::max(0, std::min((int)std::ceil(det.bbox[3]), imageSize.height));
        cv::Rect rect(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
        if (rect.width <= 0 || rect.height <= 0 || det.maskMatrix.empty()) {
            continue;
        }

        cv::Mat fullMask(imageSize.height, imageSize.width, CV_32F, const_cast<float*>(det.maskMatrix.data()));
        cv::Mat roiMask = fullMask(rect).clone();
        cv::Mat binaryMask;
        cv::threshold(roiMask, binaryMask, 0.5, 255.0, cv::THRESH_BINARY);
        binaryMask.convertTo(binaryMask, CV_8U);

        SegmentObject obj;
        obj.rect = rect;
        obj.label = det.classId;
        obj.prob = det.conf;
        obj.boxMask = binaryMask;
        objects.push_back(std::move(obj));
    }
    return objects;
}

lio_sam::SegmentedObjectStateArray buildSegmentedObjectStateArray(
    const std_msgs::Header& header,
    const std::vector<SegmentObject>& objects,
    const std::vector<std::vector<pcl::PointXYZI>>& object_clouds,
    const std::vector<std::string>& class_names)
{
    lio_sam::SegmentedObjectStateArray out;
    out.header = header;
    const size_t object_count = std::min(objects.size(), object_clouds.size());
    out.objects.reserve(object_count);

    for (size_t index = 0; index < object_count; ++index) {
        const auto& object = objects[index];
        const auto& cloud = object_clouds[index];
        if (cloud.empty()) {
            continue;
        }

        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double min_z = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        double max_z = -std::numeric_limits<double>::infinity();
        double nearest_range = std::numeric_limits<double>::infinity();

        for (const auto& point : cloud) {
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            min_x = std::min(min_x, static_cast<double>(point.x));
            min_y = std::min(min_y, static_cast<double>(point.y));
            min_z = std::min(min_z, static_cast<double>(point.z));
            max_x = std::max(max_x, static_cast<double>(point.x));
            max_y = std::max(max_y, static_cast<double>(point.y));
            max_z = std::max(max_z, static_cast<double>(point.z));
            nearest_range = std::min(
                nearest_range,
                std::sqrt(
                    static_cast<double>(point.x) * static_cast<double>(point.x)
                    + static_cast<double>(point.y) * static_cast<double>(point.y)
                    + static_cast<double>(point.z) * static_cast<double>(point.z)));
        }

        const double point_count = static_cast<double>(cloud.size());
        const double size_x = std::max(0.0, max_x - min_x);
        const double size_y = std::max(0.0, max_y - min_y);
        const double size_z = std::max(0.0, max_z - min_z);

        lio_sam::SegmentedObjectState state;
        state.class_id = object.label;
        if (object.label >= 0 && static_cast<size_t>(object.label) < class_names.size()) {
            state.class_name = class_names[object.label];
        } else {
            state.class_name = "unknown";
        }
        state.confidence = object.prob;
        state.pose.position.x = sum_x / point_count;
        state.pose.position.y = sum_y / point_count;
        state.pose.position.z = sum_z / point_count;
        state.pose.orientation.w = 1.0;
        state.size.x = size_x;
        state.size.y = size_y;
        state.size.z = size_z;
        state.bounding_radius = 0.5 * std::sqrt(size_x * size_x + size_y * size_y + size_z * size_z);
        state.nearest_range = nearest_range;
        state.point_count = static_cast<uint32_t>(cloud.size());
        out.objects.push_back(state);
    }

    return out;
}

std::vector<std::vector<pcl::PointXYZI>> drawSegmentObjects(
    const cv::Mat& image,
    cv::Mat& res,
    const std::vector<SegmentObject>& objs,
    const std::vector<std::string>& classNames,
    const std::vector<std::vector<unsigned int>>& colors,
    const std::vector<std::vector<unsigned int>>& maskColors,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
    const ProjectionModel& projection_model,
    cv::Mat* projected_overlay_image)
{
    res = image.clone();
    cv::Mat mask = image.clone();
    std::vector<std::vector<pcl::PointXYZI>> objects_pointcloud(objs.size());
    std::vector<cv::Point> projected_points;
    projected_points.reserve(input_cloud_ptr ? input_cloud_ptr->points.size() : 0);

    colored_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& obj : objs) {
        int idx = obj.label;
        cv::Scalar color(colors[idx][0], colors[idx][1], colors[idx][2]);
        cv::Scalar mask_color(maskColors[idx % 20][0], maskColors[idx % 20][1], maskColors[idx % 20][2]);
        cv::rectangle(res, obj.rect, color, 5);
        char text[256];
        sprintf(text, "%s %.1f%%", classNames[idx].c_str(), obj.prob * 100);
        mask(obj.rect).setTo(mask_color, obj.boxMask);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1, 2, &baseLine);
        int x = (int)obj.rect.x;
        int y = (int)obj.rect.y + 1;
        if (y > res.rows) {
            y = res.rows;
        }
        cv::rectangle(res, cv::Rect(x, y, label_size.width, label_size.height + baseLine), {0, 0, 255}, -1);
        cv::putText(res, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
    }

    for (const auto& point : input_cloud_ptr->points) {
        cv::Point2d img_pt = projectToImagePlane(point, projection_model);
        if (!std::isfinite(img_pt.x) || !std::isfinite(img_pt.y)) {
            continue;
        }
        int x = static_cast<int>(img_pt.x);
        int y = static_cast<int>(img_pt.y);
        if (x < 0 || x >= image.cols || y < 0 || y >= image.rows) {
            continue;
        }
        projected_points.emplace_back(x, y);

        bool assigned = false;
        for (size_t i = 0; i < objs.size(); ++i) {
            const auto& obj = objs[i];
            int relative_x = x - static_cast<int>(obj.rect.x);
            int relative_y = y - static_cast<int>(obj.rect.y);
            if (relative_x < 0 || relative_x >= obj.rect.width || relative_y < 0 || relative_y >= obj.rect.height) {
                continue;
            }
            if (isPointInsideInstance(obj.boxMask, relative_x, relative_y)) {
                objects_pointcloud[i].push_back(point);
                assigned = true;
                break;
            }
        }

        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;
        uint32_t rgb = assigned ? ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0)
                                : ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
        colored_point.rgb = *reinterpret_cast<float*>(&rgb);
        colored_cloud->points.push_back(colored_point);
    }

    std::vector<cv::Point3d> object_centroids(objs.size(), cv::Point3d(0, 0, 0));
    for (size_t i = 0; i < objects_pointcloud.size(); ++i) {
        const auto& pc = objects_pointcloud[i];
        if (pc.empty()) {
            continue;
        }
        double sum_x = 0, sum_y = 0, sum_z = 0;
        cv::Point2f rect_show_point2d;
        for (const auto& p : pc) {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
            cv::Point2d img_pt_rec = projectToImagePlane(p, projection_model);
            if (!std::isfinite(img_pt_rec.x) || !std::isfinite(img_pt_rec.y)) {
                continue;
            }
            rect_show_point2d.x = img_pt_rec.x;
            rect_show_point2d.y = img_pt_rec.y;
            cv::circle(res, rect_show_point2d, 3, cv::Scalar(255, 255, 125), -1);
        }
        double cnt = static_cast<double>(pc.size());
        object_centroids[i] = cv::Point3d(sum_x / cnt, sum_y / cnt, sum_z / cnt);
    }

    cv::addWeighted(res, 0.5, mask, 0.8, 1, res);
    if (projected_overlay_image) {
        *projected_overlay_image = res.clone();
        for (const auto& pt : projected_points) {
            cv::circle(*projected_overlay_image, pt, 3, cv::Scalar(255, 255, 125), -1);
        }
    }
    return objects_pointcloud;
}
}  // namespace

// 常量定义
const std::vector<std::string> CLASS_NAMES = {
    "exca-arm", "exca-body", "person", "crane", "truck", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
    "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie",
    "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush"
};

const std::vector<std::vector<unsigned int>> COLORS = {
    {0,114,189}, {217,83,25}, {237,177,32}, {126,47,142}, {119,172,48}, {77,190,238},
    {162,20,47}, {76,76,76}, {153,153,153}, {255,0,0}, {255,128,0}, {191,191,0},
    {0,255,0}, {0,0,255}, {170,0,255}, {85,85,0}, {85,170,0}, {85,255,0},
    {170,85,0}, {170,170,0}, {170,255,0}, {255,85,0}, {255,170,0}, {255,255,0},
    {0,85,128}, {0,170,128}, {0,255,128}, {85,0,128}, {85,85,128}, {85,170,128},
    {85,255,128}, {170,0,128}, {170,85,128}, {170,170,128}, {170,255,128}, {255,0,128},
    {255,85,128}, {255,170,128}, {255,255,128}, {0,85,255}, {0,170,255}, {0,255,255},
    {85,0,255}, {85,85,255}, {85,170,255}, {85,255,255}, {170,0,255}, {170,85,255},
    {170,170,255}, {170,255,255}, {255,0,255}, {255,85,255}, {255,170,255}, {85,0,0},
    {128,0,0}, {170,0,0}, {212,0,0}, {255,0,0}, {0,43,0}, {0,85,0},
    {0,128,0}, {0,170,0}, {0,212,0}, {0,255,0}, {0,0,43}, {0,0,85},
    {0,0,128}, {0,0,170}, {0,0,212}, {0,0,255}, {0,0,0}, {36,36,36},
    {73,73,73}, {109,109,109}, {146,146,146}, {182,182,182}, {219,219,219}, {0,114,189},
    {80,183,189}, {128,128,0}
};

const std::vector<std::vector<unsigned int>> MASK_COLORS = {
    {255,56,56}, {255,157,151}, {255,112,31}, {255,178,29}, {207,210,49}, {72,249,10}, {146,204,23},
    {61,219,134}, {26,147,52}, {0,212,187}, {44,153,168}, {0,194,255}, {52,69,147}, {100,115,255},
    {0,24,236}, {132,56,255}, {82,0,133}, {203,56,255}, {255,149,200}, {255,55,199}
};

class RosNode
{
public:
    RosNode();
    ~RosNode(){};
    void callback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    std::vector<std::vector<pcl::PointXYZI>> pointcloud_vec;
private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>;

    std::string pkg_path_, engine_file_path_, projection_config_path_;
    std::shared_ptr<YoloDetector> detector_;
    ProjectionModel projection_model_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Mat img_res_, image_;
    cv::Size size_ = cv::Size{640, 640};
    std::vector<SegmentObject> objs_;

    ros::NodeHandle n_;
    message_filters::Subscriber<sensor_msgs::Image> sub_img_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    ros::Publisher pub_img_;
    ros::Publisher pub_img_pc_;
    ros::Publisher pub_color_cloud_;
    ros::Publisher pub_object_states_;
    std::string topic_img_, topic_res_img_, weight_name_, topic_pointcloud_, topic_res_img_pc_, topic_object_states_;
    bool flip_lidar_y_for_projection_ = false;
    bool use_tf_for_projection_ = true;
    double tf_lookup_timeout_sec_ = 0.02;
    double sync_max_interval_sec_ = 0.12;
    bool tf_projection_logged_ = false;
    bool yaml_fallback_logged_ = false;
    bool sync_skew_logged_ = false;

    bool enable_perf_stats_ = true;
    double perf_log_period_sec_ = 2.0;
    ros::Time perf_last_log_time_;
    StageStats stat_total_ms_;
    StageStats stat_infer_ms_;
    StageStats stat_convert_ms_;
    StageStats stat_downsample_ms_;
    StageStats stat_tf_ms_;
    StageStats stat_draw_ms_;
    StageStats stat_overlay_ms_;
    StageStats stat_publish_ms_;
    StageStats stat_sync_skew_s_;

    bool enable_pointcloud_downsample_ = true;
    double pointcloud_leaf_size_m_ = 0.03;

    bool tryBuildProjectionModelFromTf(
        const std::string& target_frame,
        const std::string& source_frame,
        const ros::Time& transform_stamp,
        ProjectionModel* projection_model);

    void recordAndMaybeLogPerf(
        double total_ms,
        double infer_ms,
        double convert_ms,
        double downsample_ms,
        double tf_ms,
        double draw_ms,
        double overlay_ms,
        double publish_ms,
        double sync_skew_s);
};

void RosNode::recordAndMaybeLogPerf(
    double total_ms,
    double infer_ms,
    double convert_ms,
    double downsample_ms,
    double tf_ms,
    double draw_ms,
    double overlay_ms,
    double publish_ms,
    double sync_skew_s)
{
    if (!enable_perf_stats_) {
        return;
    }

    stat_total_ms_.add(total_ms);
    stat_infer_ms_.add(infer_ms);
    stat_convert_ms_.add(convert_ms);
    stat_downsample_ms_.add(downsample_ms);
    stat_tf_ms_.add(tf_ms);
    stat_draw_ms_.add(draw_ms);
    stat_overlay_ms_.add(overlay_ms);
    stat_publish_ms_.add(publish_ms);
    stat_sync_skew_s_.add(sync_skew_s);

    const ros::Time now = ros::Time::now();
    if (perf_last_log_time_.isZero()) {
        perf_last_log_time_ = now;
        return;
    }

    if ((now - perf_last_log_time_).toSec() < perf_log_period_sec_) {
        return;
    }

    ROS_INFO(
        "[segment perf] samples=%zu total(ms) avg=%.2f min=%.2f max=%.2f | infer avg=%.2f | convert avg=%.2f | downsample avg=%.2f | tf avg=%.2f | draw avg=%.2f | overlay avg=%.2f | publish avg=%.2f | sync_skew(s) avg=%.4f max=%.4f",
        stat_total_ms_.count,
        stat_total_ms_.avg(), stat_total_ms_.min, stat_total_ms_.max,
        stat_infer_ms_.avg(),
        stat_convert_ms_.avg(),
        stat_downsample_ms_.avg(),
        stat_tf_ms_.avg(),
        stat_draw_ms_.avg(),
        stat_overlay_ms_.avg(),
        stat_publish_ms_.avg(),
        stat_sync_skew_s_.avg(),
        stat_sync_skew_s_.max);

    stat_total_ms_.reset();
    stat_infer_ms_.reset();
    stat_convert_ms_.reset();
    stat_downsample_ms_.reset();
    stat_tf_ms_.reset();
    stat_draw_ms_.reset();
    stat_overlay_ms_.reset();
    stat_publish_ms_.reset();
    stat_sync_skew_s_.reset();
    perf_last_log_time_ = now;
}

bool RosNode::tryBuildProjectionModelFromTf(
    const std::string& target_frame,
    const std::string& source_frame,
    const ros::Time& transform_stamp,
    ProjectionModel* projection_model)
{
    if (!projection_model || !tf_buffer_ || target_frame.empty() || source_frame.empty()) {
        return false;
    }

    try {
        const geometry_msgs::TransformStamped transform =
            tf_buffer_->lookupTransform(target_frame, source_frame, transform_stamp, ros::Duration(tf_lookup_timeout_sec_));
        tf2::Quaternion rotation(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        tf2::Matrix3x3 rotation_matrix(rotation);

        projection_model->final_rotation_matrix = cv::Mat::eye(3, 3, CV_64F);
        projection_model->t_vec = cv::Mat::zeros(3, 1, CV_64F);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                projection_model->final_rotation_matrix.at<double>(row, col) = rotation_matrix[row][col];
            }
        }
        projection_model->t_vec.at<double>(0, 0) = transform.transform.translation.x;
        projection_model->t_vec.at<double>(1, 0) = transform.transform.translation.y;
        projection_model->t_vec.at<double>(2, 0) = transform.transform.translation.z;
        projection_model->flip_lidar_y = flip_lidar_y_for_projection_;
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(
            2.0,
            "TF projection lookup failed from %s to %s: %s. Falling back to YAML extrinsics.",
            source_frame.c_str(),
            target_frame.c_str(),
            ex.what());
        return false;
    }
}

void RosNode::callback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    const auto callback_start = std::chrono::steady_clock::now();

    auto current_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *current_cloud);
    const auto& current_cloud_header = cloud_msg->header;

    // 图像处理与检测
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    const auto infer_start = std::chrono::steady_clock::now();
    auto detections = detector_->inference(image);
    const auto infer_end = std::chrono::steady_clock::now();
    objs_ = convertDetectionsToObjects(detections, image.size());
    const auto convert_end = std::chrono::steady_clock::now();

    const auto downsample_start = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr processing_cloud = current_cloud;
    if (enable_pointcloud_downsample_ && current_cloud && !current_cloud->points.empty()) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(current_cloud);
        voxel_filter.setLeafSize(
            static_cast<float>(pointcloud_leaf_size_m_),
            static_cast<float>(pointcloud_leaf_size_m_),
            static_cast<float>(pointcloud_leaf_size_m_));
        voxel_filter.filter(*filtered_cloud);
        if (!filtered_cloud->points.empty()) {
            processing_cloud = filtered_cloud;
        }
    }
    const auto downsample_end = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

    const ros::Duration stamp_delta = msg->header.stamp - cloud_msg->header.stamp;
    const double stamp_delta_sec = std::fabs(stamp_delta.toSec());
    if (stamp_delta_sec > sync_max_interval_sec_) {
        ROS_WARN_THROTTLE(
            2.0,
            "Image/pointcloud sync skew is %.3f s, larger than sync_max_interval_sec=%.3f s.",
            stamp_delta_sec,
            sync_max_interval_sec_);
    } else if (!sync_skew_logged_) {
        ROS_INFO(
            "Image/pointcloud sync skew is %.3f s (sync_max_interval_sec=%.3f s).",
            stamp_delta_sec,
            sync_max_interval_sec_);
        sync_skew_logged_ = true;
    }

    ProjectionModel active_projection_model = projection_model_;
    const ros::Time transform_stamp = cloud_msg->header.stamp.isZero() ? msg->header.stamp : cloud_msg->header.stamp;
    const auto tf_start = std::chrono::steady_clock::now();
    const bool using_tf_projection =
        use_tf_for_projection_ &&
        tryBuildProjectionModelFromTf(msg->header.frame_id, cloud_msg->header.frame_id, transform_stamp, &active_projection_model);
    const auto tf_end = std::chrono::steady_clock::now();

    if (using_tf_projection) {
        if (!tf_projection_logged_) {
            ROS_INFO(
                "Using TF for projection from point cloud frame '%s' to image frame '%s'.",
                cloud_msg->header.frame_id.c_str(),
                msg->header.frame_id.c_str());
            tf_projection_logged_ = true;
        }
    } else if (!yaml_fallback_logged_) {
        ROS_INFO("Using projection extrinsics from YAML: %s", projection_config_path_.c_str());
        yaml_fallback_logged_ = true;
    }

    const auto draw_start = std::chrono::steady_clock::now();
    cv::Mat image_to_show;
    pointcloud_vec = drawSegmentObjects(
        image,
        img_res_,
        objs_,
        CLASS_NAMES,
        COLORS,
        MASK_COLORS,
        processing_cloud,
        colored_cloud,
        active_projection_model,
        &image_to_show);
    const auto draw_end = std::chrono::steady_clock::now();

    auto tc = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(infer_end - infer_start).count()) / 1000.;
    cv::putText(img_res_, "fps: " + std::to_string(int(1000 / tc)), cv::Point(20, 40),
                cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 0, 255), 2, 8);
    ROS_INFO("segment cost %2.4f ms", tc);

    if (!current_cloud || current_cloud->points.empty()) {
        ROS_WARN_THROTTLE(2.0, "No point cloud data available for projection.");
    }

    // Legacy grid-based risk area and alarm path has been removed.
    // A new danger-area and early-warning pipeline will be added here.

    // 后续图像和点云的处理与发布
    const auto overlay_start = std::chrono::steady_clock::now();
    if (image_to_show.empty()) {
        image_to_show = img_res_.clone();
    }
    const auto overlay_end = std::chrono::steady_clock::now();

    const auto publish_start = std::chrono::steady_clock::now();
    sensor_msgs::ImagePtr msg_img_new = cv_bridge::CvImage(msg->header, "bgr8", img_res_).toImageMsg();
    sensor_msgs::ImagePtr msg_img_new_pc = cv_bridge::CvImage(msg->header, "bgr8", image_to_show).toImageMsg();
    ROS_DEBUG_THROTTLE(2.0, "Output Image Size: Width = %d, Height = %d", img_res_.cols, img_res_.rows);
    pub_img_pc_.publish(msg_img_new_pc);
    pub_img_.publish(msg_img_new);
    pub_object_states_.publish(buildSegmentedObjectStateArray(current_cloud_header, objs_, pointcloud_vec, CLASS_NAMES));

    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*colored_cloud, output_cloud_msg);
    output_cloud_msg.header = current_cloud_header;
    pub_color_cloud_.publish(output_cloud_msg);
    const auto publish_end = std::chrono::steady_clock::now();

    const double infer_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(infer_end - infer_start).count()) / 1000.0;
    const double convert_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(convert_end - infer_end).count()) / 1000.0;
    const double downsample_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(downsample_end - downsample_start).count()) / 1000.0;
    const double tf_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(tf_end - tf_start).count()) / 1000.0;
    const double draw_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(draw_end - draw_start).count()) / 1000.0;
    const double overlay_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(overlay_end - overlay_start).count()) / 1000.0;
    const double publish_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(publish_end - publish_start).count()) / 1000.0;
    const double total_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(publish_end - callback_start).count()) / 1000.0;
    recordAndMaybeLogPerf(total_ms, infer_ms, convert_ms, downsample_ms, tf_ms, draw_ms, overlay_ms, publish_ms, stamp_delta_sec);
}

RosNode::RosNode()
{
    cudaSetDevice(0);
    pkg_path_ = ros::package::getPath("yolo26_ros");
    projection_config_path_ = pkg_path_ + "/config/segment_projection.yaml";
    n_.param<std::string>("topic_img", topic_img_, "/camera/color/image_raw");
    n_.param<std::string>("topic_pointcloud", topic_pointcloud_, "/livox/lidar");
    n_.param<std::string>("topic_res_img", topic_res_img_, "/detect/image_raw");
    n_.param<std::string>("topic_res_img_pc_", topic_res_img_pc_, "/detect/topic_res_img_pc_");
    n_.param<std::string>("topic_object_states", topic_object_states_, "/segment/object_states");
    n_.param<std::string>("weight_name", weight_name_, "yolo26s-seg.engine");
    n_.param<std::string>("projection_config_path", projection_config_path_, projection_config_path_);
    n_.param<bool>("flip_lidar_y_for_projection", flip_lidar_y_for_projection_, false);
    n_.param<bool>("use_tf_for_projection", use_tf_for_projection_, true);
    n_.param<double>("tf_lookup_timeout_sec", tf_lookup_timeout_sec_, tf_lookup_timeout_sec_);
    n_.param<double>("sync_max_interval_sec", sync_max_interval_sec_, sync_max_interval_sec_);
    n_.param<bool>("enable_perf_stats", enable_perf_stats_, enable_perf_stats_);
    n_.param<double>("perf_log_period_sec", perf_log_period_sec_, perf_log_period_sec_);
    n_.param<bool>("enable_pointcloud_downsample", enable_pointcloud_downsample_, enable_pointcloud_downsample_);
    n_.param<double>("pointcloud_leaf_size_m", pointcloud_leaf_size_m_, pointcloud_leaf_size_m_);
    engine_file_path_ = pkg_path_ + "/weights/" + weight_name_;
    projection_model_.loadFromFile(projection_config_path_);
    projection_model_.flip_lidar_y = flip_lidar_y_for_projection_;
    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

    std::cout << "\n\033[1;32m--engine_file_path: " << engine_file_path_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m" << "--topic_img       : " << topic_img_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--topic_res_img   : " << topic_res_img_ << "\n\033[0m" << std::endl;
    std::cout << "\033[1;32m--projection_cfg  : " << projection_config_path_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--flip_lidar_y    : " << (flip_lidar_y_for_projection_ ? "true" : "false") << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--use_tf_projection: " << (use_tf_for_projection_ ? "true" : "false") << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--tf_lookup_timeout: " << tf_lookup_timeout_sec_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--sync_max_interval: " << sync_max_interval_sec_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--enable_perf_stats: " << (enable_perf_stats_ ? "true" : "false") << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--perf_log_period  : " << perf_log_period_sec_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--pc_downsample    : " << (enable_pointcloud_downsample_ ? "true" : "false") << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--pc_leaf_size(m)  : " << pointcloud_leaf_size_m_ << "\033[0m" << std::endl;

    detector_.reset(new YoloDetector(engine_file_path_));

    pub_img_pc_ = n_.advertise<sensor_msgs::Image>(topic_res_img_pc_, 10);
    pub_img_ = n_.advertise<sensor_msgs::Image>(topic_res_img_, 10);
    pub_color_cloud_ = n_.advertise<sensor_msgs::PointCloud2>("/colored_point_cloud", 10);
    pub_object_states_ = n_.advertise<lio_sam::SegmentedObjectStateArray>(topic_object_states_, 10);

    sub_img_.subscribe(n_, topic_img_, 10);
    sub_pc_.subscribe(n_, topic_pointcloud_, 10);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), sub_img_, sub_pc_));
    sync_->setMaxIntervalDuration(ros::Duration(sync_max_interval_sec_));
    sync_->registerCallback(boost::bind(&RosNode::callback, this, _1, _2));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "seg_node");
    ros::NodeHandle n;
    auto seg_node = std::make_shared<RosNode>();
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}    