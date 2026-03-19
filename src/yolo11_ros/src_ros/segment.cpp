#include "chrono"
#include "opencv2/opencv.hpp"
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
#include <ctime>
#include <sstream>
#include <fstream>
#include <stdexcept>

// pcl headers
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

std::vector<std::vector<pcl::PointXYZI>> drawSegmentObjects(
    const cv::Mat& image,
    cv::Mat& res,
    const std::vector<SegmentObject>& objs,
    const std::vector<std::string>& classNames,
    const std::vector<std::vector<unsigned int>>& colors,
    const std::vector<std::vector<unsigned int>>& maskColors,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
    const ProjectionModel& projection_model)
{
    res = image.clone();
    cv::Mat mask = image.clone();
    std::vector<std::vector<pcl::PointXYZI>> objects_pointcloud(objs.size());

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

    static const int PERSIST_GRID_ROWS = 50;  // 固定区域网格行数
    static const int PERSIST_GRID_COLS = 50;  // 固定区域网格列数
    std::vector<std::vector<int>> danger_grid;      // 存储非人危险区域（1表示危险区域）
    std::vector<std::vector<ros::Time>> danger_time;  // 记录对应格子最近一次更新的时间
    ros::Duration danger_duration = ros::Duration(180.0); // 持续时长，180秒
    ros::Time last_alarm_time_; // 上次报警的时间

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
    std::string topic_img_, topic_res_img_, weight_name_, topic_pointcloud_, topic_res_img_pc_;
    bool flip_lidar_y_for_projection_ = false;
    bool use_tf_for_projection_ = true;
    double tf_lookup_timeout_sec_ = 0.02;
    double sync_max_interval_sec_ = 0.12;
    bool tf_projection_logged_ = false;
    bool yaml_fallback_logged_ = false;
    bool sync_skew_logged_ = false;

    bool tryBuildProjectionModelFromTf(
        const std::string& target_frame,
        const std::string& source_frame,
        const ros::Time& transform_stamp,
        ProjectionModel* projection_model);
};

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
    auto current_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *current_cloud);
    const auto& current_cloud_header = cloud_msg->header;

    // 图像处理与检测
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    auto start = std::chrono::system_clock::now();
    auto detections = detector_->inference(image);
    auto end = std::chrono::system_clock::now();
    objs_ = convertDetectionsToObjects(detections, image.size());
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
    const bool using_tf_projection =
        use_tf_for_projection_ &&
        tryBuildProjectionModelFromTf(msg->header.frame_id, cloud_msg->header.frame_id, transform_stamp, &active_projection_model);

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

    pointcloud_vec = drawSegmentObjects(
        image,
        img_res_,
        objs_,
        CLASS_NAMES,
        COLORS,
        MASK_COLORS,
        current_cloud,
        colored_cloud,
        active_projection_model);

    auto tc = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()) / 1000.;
    cv::putText(img_res_, "fps: " + std::to_string(int(1000 / tc)), cv::Point(20, 40),
                cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 0, 255), 2, 8);
    ROS_INFO("segment cost %2.4f ms", tc);

    if (!current_cloud || current_cloud->points.empty()) {
        ROS_WARN_THROTTLE(2.0, "No point cloud data available for projection.");
    }

    if (current_cloud && !current_cloud->points.empty()) {
        // 固定关注区域参数（根据实际情况调整）
        float range_min_x = 0.0f, range_max_x = 100.0f;
        float range_min_y = -50.0f, range_max_y = 50.0f;
        ROS_INFO("Using fixed region for grid: x in [%.2f, %.2f], y in [%.2f, %.2f]",
                 range_min_x, range_max_x, range_min_y, range_max_y);

        // STEP 1: 清理超过 danger_duration 未更新的危险区域
        ros::Time now = ros::Time::now();
        for (int r = 0; r < PERSIST_GRID_ROWS; r++) {
            for (int c = 0; c < PERSIST_GRID_COLS; c++) {
                if (danger_grid[r][c] == 2 || danger_grid[r][c] == 4) {
                    if ((now - danger_time[r][c]) > danger_duration) {
                        danger_grid[r][c] = 0;
                        danger_time[r][c] = ros::Time(0);
                    }
                }
            }
        }

        // STEP 2: 构造本帧实时网格
        std::vector<std::vector<int>> person_grid(PERSIST_GRID_ROWS, std::vector<int>(PERSIST_GRID_COLS, 0));
        std::vector<std::vector<int>> excavator_grid(PERSIST_GRID_ROWS, std::vector<int>(PERSIST_GRID_COLS, 0));
        std::vector<std::vector<int>> crane_grid(PERSIST_GRID_ROWS, std::vector<int>(PERSIST_GRID_COLS, 0));

        // STEP 3: 遍历每个检测对象，更新对应格子的值
        for (size_t i = 0; i < objs_.size(); i++) {
            if (i >= pointcloud_vec.size() || pointcloud_vec[i].empty()) {
                continue;
            }
            std::string cls_name = CLASS_NAMES[objs_[i].label];
            for (const auto &point : pointcloud_vec[i]) {
                if (point.x < range_min_x || point.x > range_max_x ||
                    point.y < range_min_y || point.y > range_max_y)
                {
                    continue;
                }
                int col_idx = int((point.x - range_min_x) / (range_max_x - range_min_x) * PERSIST_GRID_COLS);
                int row_idx = int((point.y - range_min_y) / (range_max_y - range_min_y) * PERSIST_GRID_ROWS);
                col_idx = std::max(0, std::min(col_idx, PERSIST_GRID_COLS - 1));
                row_idx = std::max(0, std::min(row_idx, PERSIST_GRID_ROWS - 1));

                if (cls_name == "person" || point.intensity > 150) {
                    person_grid[row_idx][col_idx] = 1;
                } else if (cls_name == "exca-arm" || cls_name == "exca-body") {
                    excavator_grid[row_idx][col_idx] = 2;
                    danger_grid[row_idx][col_idx] = 2;
                    danger_time[row_idx][col_idx] = now;
                } else if (cls_name == "crane") {
                    crane_grid[row_idx][col_idx] = 4;
                    danger_grid[row_idx][col_idx] = 4;
                    danger_time[row_idx][col_idx] = now;
                }
            }
        }

        // STEP 4: 合并得到最终输出 grid (final_grid)
        std::vector<std::vector<int>> final_grid(PERSIST_GRID_ROWS, std::vector<int>(PERSIST_GRID_COLS, 0));
        for (int r = 0; r < PERSIST_GRID_ROWS; r++) {
            for (int c = 0; c < PERSIST_GRID_COLS; c++) {
                int code = 0;
                if (person_grid[r][c])    code |= 0b001;  // 人
                if (danger_grid[r][c] == 2) code |= 0b010;  // 挖机
                if (danger_grid[r][c] == 4) code |= 0b100;  // 吊机
                final_grid[r][c] = code;
            }
        }

        // STEP 5: 输出最终 grid 并记录报警单元
        std::stringstream grid_ss;
        std::vector<std::pair<int,int>> alarmCells;
        for (int r = 0; r < PERSIST_GRID_ROWS; r++) {
            for (int c = 0; c < PERSIST_GRID_COLS; c++) {
                grid_ss << final_grid[r][c] << " ";
                // 报警条件：只处理包含“人”和“设备”的组合
                if (final_grid[r][c] == 3 || final_grid[r][c] == 5 || final_grid[r][c] == 7) {
                    alarmCells.push_back(std::make_pair(r, c));
                }
            }
            grid_ss << "\n";
        }

        // STEP 6: 创建旋转后的矩阵，尺寸交换行列
        std::vector<std::vector<int>> rotated_grid(PERSIST_GRID_COLS, std::vector<int>(PERSIST_GRID_ROWS, 0));

        // 执行逆时针90°旋转
        for (int r = 0; r < PERSIST_GRID_ROWS; ++r) {
            for (int c = 0; c < PERSIST_GRID_COLS; ++c) {
                rotated_grid[PERSIST_GRID_COLS - c - 1][r] = final_grid[r][c];
            }
        }

        // 旋转后矩阵用于输出显示（替换原来的输出）
        std::stringstream rotated_grid_ss;
        for (int r = 0; r < PERSIST_GRID_COLS; ++r) {
            for (int c = 0; c < PERSIST_GRID_ROWS; ++c) {
                rotated_grid_ss << rotated_grid[r][c] << " ";
            }
            rotated_grid_ss << "\n";
        }
        ROS_INFO_STREAM("Final rotated grid (90 degrees CCW):\n" << rotated_grid_ss.str());

        // STEP 7: 检查报警条件，触发报警和记录日志
        if (!alarmCells.empty()) {
            if ((now - last_alarm_time_) < ros::Duration(10.0)) {
                ROS_INFO("Alarm already triggered within last 10 seconds. Skipping new alarm.");
            } else {
                last_alarm_time_ = now;
                std::string curlCommand = "curl --noproxy \"*\" -v -X POST http://192.168.1.41:8888 -H \"Content-Type: application/json\" -d '{\"sn\": \"ls20://0201B7B1F13D\", \"type\": \"req\", \"name\": \"songs_queue_append\", \"params\": {\"tid\": \"1\", \"vol\": 70, \"urls\": [{\"name\": \"warning.mp3\", \"uri\": \"/mnt/mmcblk0p1/ls20/warning.mp3\"}]}}'";
                int ret = system(curlCommand.c_str());
                if (ret == 0)
                    ROS_INFO("Alarm command executed successfully.");
                else
                    ROS_WARN("Alarm command execution failed.");

                // 构造日志文件路径：/home/nvidia/yolo_ws_invix/log/YYYY-MM-DD_alarm.log
                std::time_t t = std::time(nullptr);
                std::tm* now_tm = std::localtime(&t);
                char dateBuffer[64];
                std::strftime(dateBuffer, sizeof(dateBuffer), "%Y-%m-%d", now_tm);
                std::string logFilePath = "/home/nvidia/yolo_ws_invix/log/" + std::string(dateBuffer) + "_alarm.log";

                std::ofstream logFile;
                logFile.open(logFilePath, std::ios::out | std::ios::app);
                if (logFile.is_open()) {
                    logFile << "Alarm triggered at: " << ros::Time::now() << "\n";
                    logFile << "Alarm grid cells (row, col): ";
                    for (const auto &cell : alarmCells) {
                        logFile << "(" << cell.first << ", " << cell.second << ") ";
                    }
                    logFile << "\n-------------------------------------------\n";
                    logFile.close();
                } else {
                    ROS_WARN("Unable to open alarm log file at: %s", logFilePath.c_str());
                }
            }
        }
    } else {
        ROS_WARN_THROTTLE(2.0, "No point cloud data available for extent calculation.");
    }

    // 后续图像和点云的处理与发布
    std::vector<cv::Point3d> lidar_points;
    std::vector<cv::Point2d> imagePoints;
    for (const auto& point : current_cloud->points) {
        lidar_points.push_back(cv::Point3d(point.x, point.y, point.z));
    }
    cv::Mat image_to_show = img_res_.clone();
    if (lidar_points.empty()) {
        ROS_WARN_THROTTLE(2.0, "No point cloud data available for projection.");
        return;
    }
    for (const auto& point : current_cloud->points) {
        cv::Point2f projected_point = projectToImagePlane(point, active_projection_model);
        if (!std::isfinite(projected_point.x) || !std::isfinite(projected_point.y)) {
            continue;
        }
        cv::circle(image_to_show, projected_point, 3, cv::Scalar(255,255,125), -1);
    }

    sensor_msgs::ImagePtr msg_img_new = cv_bridge::CvImage(msg->header, "bgr8", img_res_).toImageMsg();
    sensor_msgs::ImagePtr msg_img_new_pc = cv_bridge::CvImage(msg->header, "bgr8", image_to_show).toImageMsg();
    ROS_INFO("Output Image Size: Width = %d, Height = %d", img_res_.cols, img_res_.rows);
    pub_img_pc_.publish(msg_img_new_pc);
    pub_img_.publish(msg_img_new);

    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*colored_cloud, output_cloud_msg);
    output_cloud_msg.header = current_cloud_header;
    pub_color_cloud_.publish(output_cloud_msg);
}

RosNode::RosNode()
{
    cudaSetDevice(0);
    pkg_path_ = ros::package::getPath("yolo11_ros");
    projection_config_path_ = pkg_path_ + "/config/segment_projection.yaml";
    n_.param<std::string>("topic_img", topic_img_, "/camera/color/image_raw");
    n_.param<std::string>("topic_pointcloud", topic_pointcloud_, "/livox/lidar");
    n_.param<std::string>("topic_res_img", topic_res_img_, "/detect/image_raw");
    n_.param<std::string>("topic_res_img_pc_", topic_res_img_pc_, "/detect/topic_res_img_pc_");
    n_.param<std::string>("weight_name", weight_name_, "yolo11s-seg.engine");
    n_.param<std::string>("projection_config_path", projection_config_path_, projection_config_path_);
    n_.param<bool>("flip_lidar_y_for_projection", flip_lidar_y_for_projection_, false);
    n_.param<bool>("use_tf_for_projection", use_tf_for_projection_, true);
    n_.param<double>("tf_lookup_timeout_sec", tf_lookup_timeout_sec_, tf_lookup_timeout_sec_);
    n_.param<double>("sync_max_interval_sec", sync_max_interval_sec_, sync_max_interval_sec_);
    engine_file_path_ = pkg_path_ + "/weights/" + weight_name_;
    projection_model_.loadFromFile(projection_config_path_);
    projection_model_.flip_lidar_y = flip_lidar_y_for_projection_;
    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

    // 初始化持久化危险区域网格和更新时间数组
    danger_grid.resize(PERSIST_GRID_ROWS, std::vector<int>(PERSIST_GRID_COLS, 0));
    danger_time.resize(PERSIST_GRID_ROWS, std::vector<ros::Time>(PERSIST_GRID_COLS, ros::Time(0)));
    last_alarm_time_ = ros::Time(0);

    std::cout << "\n\033[1;32m--engine_file_path: " << engine_file_path_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m" << "--topic_img       : " << topic_img_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--topic_res_img   : " << topic_res_img_ << "\n\033[0m" << std::endl;
    std::cout << "\033[1;32m--projection_cfg  : " << projection_config_path_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--flip_lidar_y    : " << (flip_lidar_y_for_projection_ ? "true" : "false") << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--use_tf_projection: " << (use_tf_for_projection_ ? "true" : "false") << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--tf_lookup_timeout: " << tf_lookup_timeout_sec_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--sync_max_interval: " << sync_max_interval_sec_ << "\033[0m" << std::endl;

    detector_.reset(new YoloDetector(engine_file_path_));

    pub_img_pc_ = n_.advertise<sensor_msgs::Image>(topic_res_img_pc_, 10);
    pub_img_ = n_.advertise<sensor_msgs::Image>(topic_res_img_, 10);
    pub_color_cloud_ = n_.advertise<sensor_msgs::PointCloud2>("/colored_point_cloud", 10);

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