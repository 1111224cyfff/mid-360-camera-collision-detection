#include "chrono"
#include <cmath>
#include "opencv2/opencv.hpp"
#include <lio_sam/SegmentedObjectStateArray.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
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
#include <cstdio>
#include <xmlrpcpp/XmlRpcValue.h>
#include <iomanip>

// pcl headers
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include "infer.h"

namespace {
struct SegmentObject {
    cv::Rect_<float> rect;
    int label = 0;
    float prob = 0.0f;
    cv::Mat boxMask;
    cv::Mat coreMask;
};

struct ObjectCloudRefineSettings {
    bool enable = true;
    int mask_erode_pixels = 2;
    int min_core_points = 8;
    double range_gate_min_m = 0.8;
    double range_gate_scale = 0.08;
    double cluster_tolerance_m = 0.6;
    int cluster_min_points = 20;
    double radius_percentile = 0.95;
    double radius_outlier_tolerance_m = 0.25;
    int radius_outlier_min_support_points = 3;
    double radius_padding_m = 0.12;
    double max_accepted_radius_m = 0.0;
};

struct PlanarRadiusStats {
    double robust_radius = 0.0;
    double max_radius = 0.0;
    int outer_support_points = 0;
};

double computeMaxSpatialRadius(
    const std::vector<pcl::PointXYZI>& cloud,
    const geometry_msgs::Point& centroid)
{
    double max_radius = 0.0;
    for (const auto& point : cloud) {
        const double dx = static_cast<double>(point.x) - centroid.x;
        const double dy = static_cast<double>(point.y) - centroid.y;
        const double dz = static_cast<double>(point.z) - centroid.z;
        max_radius = std::max(max_radius, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    return max_radius;
}

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
    double sum_sq = 0.0;
    std::size_t count = 0;

    static constexpr std::size_t MAX_SAMPLES = 1000;
    double samples[MAX_SAMPLES];
    std::size_t sample_head = 0;
    std::size_t sample_count = 0;

    void add(double value)
    {
        if (value < 0.0) value = 0.0;
        min = std::min(min, value);
        max = std::max(max, value);
        sum += value;
        sum_sq += value * value;
        ++count;

        samples[sample_head] = value;
        sample_head = (sample_head + 1) % MAX_SAMPLES;
        if (sample_count < MAX_SAMPLES) {
            ++sample_count;
        }
    }

    double avg() const
    {
        return count == 0 ? 0.0 : sum / static_cast<double>(count);
    }

    double stdDev() const
    {
        if (count < 2) return 0.0;
        double mean = avg();
        double variance = (sum_sq / static_cast<double>(count)) - (mean * mean);
        return std::max(0.0, std::sqrt(variance));
    }

    double p95() const
    {
        if (sample_count == 0) return 0.0;
        std::vector<double> sorted;
        sorted.reserve(sample_count);
        if (sample_count < MAX_SAMPLES) {
            for (std::size_t i = 0; i < sample_count; ++i) {
                sorted.push_back(samples[i]);
            }
        } else {
            for (std::size_t i = 0; i < MAX_SAMPLES; ++i) {
                sorted.push_back(samples[(sample_head + i) % MAX_SAMPLES]);
            }
        }
        std::sort(sorted.begin(), sorted.end());
        std::size_t idx = static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(sorted.size() - 1)));
        idx = std::min(idx, sorted.size() - 1);
        return sorted[idx];
    }

    void reset()
    {
        min = std::numeric_limits<double>::infinity();
        max = 0.0;
        sum = 0.0;
        sum_sq = 0.0;
        count = 0;
        sample_head = 0;
        sample_count = 0;
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

cv::Mat createFullSizeMask(const cv::Size& image_size, const SegmentObject& obj)
{
    cv::Mat mask = cv::Mat::zeros(image_size, CV_8U);
    if (obj.boxMask.empty() || obj.rect.width <= 0 || obj.rect.height <= 0) {
        return mask;
    }
    const int x = static_cast<int>(obj.rect.x);
    const int y = static_cast<int>(obj.rect.y);
    const int w = obj.boxMask.cols;
    const int h = obj.boxMask.rows;
    if (x >= image_size.width || y >= image_size.height || x + w <= 0 || y + h <= 0) {
        return mask;
    }
    int src_x = 0, src_y = 0;
    int dst_x = x, dst_y = y;
    int copy_w = w, copy_h = h;
    if (dst_x < 0) { src_x = -dst_x; copy_w -= src_x; dst_x = 0; }
    if (dst_y < 0) { src_y = -dst_y; copy_h -= src_y; dst_y = 0; }
    if (dst_x + copy_w > image_size.width) { copy_w = image_size.width - dst_x; }
    if (dst_y + copy_h > image_size.height) { copy_h = image_size.height - dst_y; }
    if (copy_w > 0 && copy_h > 0) {
        cv::Mat roi = mask(cv::Rect(dst_x, dst_y, copy_w, copy_h));
        obj.boxMask(cv::Rect(src_x, src_y, copy_w, copy_h)).copyTo(roi);
    }
    return mask;
}

double computeMaskIoU(const cv::Mat& mask1, const cv::Mat& mask2)
{
    if (mask1.empty() || mask2.empty() || mask1.size() != mask2.size()) {
        return 0.0;
    }
    cv::Mat intersection, union_mask;
    cv::bitwise_and(mask1, mask2, intersection);
    cv::bitwise_or(mask1, mask2, union_mask);
    const double inter_area = static_cast<double>(cv::countNonZero(intersection));
    const double union_area = static_cast<double>(cv::countNonZero(union_mask));
    if (union_area < 1.0) {
        return 0.0;
    }
    return inter_area / union_area;
}

template<typename T>
double computeBboxIoU(const cv::Rect_<T>& a, const cv::Rect_<T>& b)
{
    const cv::Rect_<T> inter = a & b;
    if (inter.area() <= 0) {
        return 0.0;
    }
    const double inter_area = static_cast<double>(inter.area());
    const double union_area = static_cast<double>(a.area()) + static_cast<double>(b.area()) - inter_area;
    if (union_area < 1.0) {
        return 0.0;
    }
    return inter_area / union_area;
}

cv::Point2f computeBboxCenter(const cv::Rect_<float>& rect)
{
    return cv::Point2f(rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f);
}

struct TemporalMetricState {
    bool has_prev = false;
    cv::Mat prev_full_mask;
    cv::Rect_<float> prev_rect;
    cv::Point2f prev_center;
    float prev_conf = 0.0f;
    int prev_label = -1;
    size_t frame_counter = 0;
    bool first_detected = false;
    size_t first_detect_frame_idx = 0;
    size_t miss_count = 0;
    size_t max_consecutive_miss = 0;
    size_t current_consecutive_miss = 0;
    double tiou_sum = 0.0;
    size_t tiou_count = 0;
    double drift_sum = 0.0;
    size_t drift_count = 0;
};

const SegmentObject* selectRepresentativeTarget(
    const std::vector<SegmentObject>& objs,
    const TemporalMetricState* prev_state)
{
    if (objs.empty()) {
        return nullptr;
    }
    size_t best_idx = 0;
    float best_conf = objs[0].prob;
    for (size_t i = 1; i < objs.size(); ++i) {
        if (objs[i].prob > best_conf) {
            best_conf = objs[i].prob;
            best_idx = i;
        }
    }
    if (prev_state && prev_state->has_prev) {
        std::vector<size_t> same_label_indices;
        for (size_t i = 0; i < objs.size(); ++i) {
            if (objs[i].label == prev_state->prev_label) {
                same_label_indices.push_back(i);
            }
        }
        if (same_label_indices.size() > 1) {
            double best_iou = -1.0;
            for (size_t idx : same_label_indices) {
                const double iou = computeBboxIoU(objs[idx].rect, prev_state->prev_rect);
                if (iou > best_iou) {
                    best_iou = iou;
                    best_idx = idx;
                }
            }
        }
    }
    return &objs[best_idx];
}

std::string resolveClassName(const std::vector<std::string>& class_names, int class_id)
{
    if (class_id >= 0 && static_cast<size_t>(class_id) < class_names.size()) {
        return class_names[static_cast<size_t>(class_id)];
    }
    return "class" + std::to_string(class_id);
}

std::vector<std::string> readClassNamesParam(ros::NodeHandle& node)
{
    std::vector<std::string> class_names;
    XmlRpc::XmlRpcValue class_names_param;
    if (!node.getParam("class_names", class_names_param)) {
        return class_names;
    }

    if (class_names_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN("Parameter 'class_names' exists but is not a list. Falling back to generated class labels.");
        return class_names;
    }

    class_names.reserve(class_names_param.size());
    for (int i = 0; i < class_names_param.size(); ++i) {
        if (class_names_param[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_WARN("Parameter 'class_names[%d]' is not a string. Ignoring this item.", i);
            continue;
        }
        class_names.push_back(static_cast<std::string>(class_names_param[i]));
    }

    return class_names;
}

std::vector<SegmentObject> convertDetectionsToObjects(
    const std::vector<Detection>& detections,
    const cv::Size& imageSize,
    const ObjectCloudRefineSettings& refine_settings)
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

        cv::Mat coreMask = binaryMask.clone();
        const int min_mask_side = std::min(binaryMask.cols, binaryMask.rows);
        const int erode_pixels = std::max(
            0,
            std::min(refine_settings.mask_erode_pixels, std::max(0, (min_mask_side - 1) / 8)));
        if (erode_pixels > 0) {
            const int kernel_size = 2 * erode_pixels + 1;
            const cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(kernel_size, kernel_size));
            cv::erode(binaryMask, coreMask, kernel);
            if (cv::countNonZero(coreMask) == 0) {
                coreMask = binaryMask.clone();
            }
        }

        SegmentObject obj;
        obj.rect = rect;
        obj.label = det.classId;
        obj.prob = det.conf;
        obj.boxMask = binaryMask;
        obj.coreMask = coreMask;
        objects.push_back(std::move(obj));
    }
    return objects;
}

double pointRange(const pcl::PointXYZI& point)
{
    return std::sqrt(
        static_cast<double>(point.x) * static_cast<double>(point.x)
        + static_cast<double>(point.y) * static_cast<double>(point.y)
        + static_cast<double>(point.z) * static_cast<double>(point.z));
}

double computePercentile(std::vector<double> values, double percentile)
{
    if (values.empty()) {
        return 0.0;
    }

    std::sort(values.begin(), values.end());
    const double clamped_percentile = std::max(0.0, std::min(1.0, percentile));
    const double position = clamped_percentile * static_cast<double>(values.size() - 1);
    const size_t lower_index = static_cast<size_t>(std::floor(position));
    const size_t upper_index = static_cast<size_t>(std::ceil(position));
    if (lower_index == upper_index) {
        return values[lower_index];
    }

    const double blend = position - static_cast<double>(lower_index);
    return values[lower_index] * (1.0 - blend) + values[upper_index] * blend;
}

cv::Point3d computeCentroid(
    const std::vector<pcl::PointXYZI>& points,
    const std::vector<int>* indices = nullptr)
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    size_t count = 0;

    if (indices) {
        for (const int index : *indices) {
            if (index < 0 || static_cast<size_t>(index) >= points.size()) {
                continue;
            }
            const auto& point = points[static_cast<size_t>(index)];
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            ++count;
        }
    } else {
        for (const auto& point : points) {
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
            ++count;
        }
    }

    if (count == 0) {
        return cv::Point3d(0.0, 0.0, 0.0);
    }
    const double inv_count = 1.0 / static_cast<double>(count);
    return cv::Point3d(sum_x * inv_count, sum_y * inv_count, sum_z * inv_count);
}

double distanceSquared(const cv::Point3d& lhs, const cv::Point3d& rhs)
{
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    const double dz = lhs.z - rhs.z;
    return dx * dx + dy * dy + dz * dz;
}

std::vector<pcl::PointXYZI> refineObjectPointCloud(
    const std::vector<pcl::PointXYZI>& points,
    const std::vector<uint8_t>& core_flags,
    const ObjectCloudRefineSettings& settings)
{
    if (!settings.enable || points.size() < static_cast<size_t>(std::max(1, settings.cluster_min_points))) {
        return points;
    }

    std::vector<int> reference_indices;
    reference_indices.reserve(points.size());
    for (size_t index = 0; index < points.size() && index < core_flags.size(); ++index) {
        if (core_flags[index] != 0) {
            reference_indices.push_back(static_cast<int>(index));
        }
    }

    const bool has_core_reference = reference_indices.size() >= static_cast<size_t>(std::max(1, settings.min_core_points));
    if (!has_core_reference) {
        reference_indices.clear();
        reference_indices.reserve(points.size());
        for (size_t index = 0; index < points.size(); ++index) {
            reference_indices.push_back(static_cast<int>(index));
        }
    }

    std::vector<double> reference_ranges;
    reference_ranges.reserve(reference_indices.size());
    for (const int index : reference_indices) {
        reference_ranges.push_back(pointRange(points[static_cast<size_t>(index)]));
    }
    const double reference_range = computePercentile(reference_ranges, 0.5);
    const cv::Point3d reference_centroid = computeCentroid(points, &reference_indices);
    const double range_gate = std::max(
        settings.range_gate_min_m,
        settings.range_gate_scale * std::max(1.0, reference_range));

    std::vector<pcl::PointXYZI> gated_points;
    std::vector<uint8_t> gated_core_flags;
    gated_points.reserve(points.size());
    gated_core_flags.reserve(points.size());
    for (size_t index = 0; index < points.size(); ++index) {
        const bool is_core = index < core_flags.size() && core_flags[index] != 0;
        const double range = pointRange(points[index]);
        if (!is_core && std::fabs(range - reference_range) > range_gate) {
            continue;
        }
        gated_points.push_back(points[index]);
        gated_core_flags.push_back(is_core ? 1 : 0);
    }

    if (gated_points.size() < static_cast<size_t>(std::max(1, settings.cluster_min_points))) {
        gated_points = points;
        gated_core_flags = core_flags;
    }

    if (gated_points.size() < static_cast<size_t>(std::max(1, settings.cluster_min_points))
        || settings.cluster_tolerance_m <= 0.0) {
        return gated_points;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cluster_cloud->points.assign(gated_points.begin(), gated_points.end());
    cluster_cloud->width = static_cast<uint32_t>(gated_points.size());
    cluster_cloud->height = 1;
    cluster_cloud->is_dense = false;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cluster_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> cluster_extractor;
    cluster_extractor.setClusterTolerance(static_cast<float>(settings.cluster_tolerance_m));
    cluster_extractor.setMinClusterSize(std::max(1, settings.cluster_min_points));
    cluster_extractor.setMaxClusterSize(static_cast<int>(gated_points.size()));
    cluster_extractor.setSearchMethod(tree);
    cluster_extractor.setInputCloud(cluster_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    cluster_extractor.extract(cluster_indices);
    if (cluster_indices.empty()) {
        return gated_points;
    }

    size_t best_cluster_index = 0;
    int best_core_hits = -1;
    size_t best_size = 0;
    double best_centroid_distance_sq = std::numeric_limits<double>::infinity();
    double best_range_delta = std::numeric_limits<double>::infinity();

    for (size_t cluster_index = 0; cluster_index < cluster_indices.size(); ++cluster_index) {
        const auto& indices = cluster_indices[cluster_index].indices;
        int core_hits = 0;
        std::vector<double> cluster_ranges;
        cluster_ranges.reserve(indices.size());
        for (const int index : indices) {
            if (index >= 0 && static_cast<size_t>(index) < gated_core_flags.size() && gated_core_flags[static_cast<size_t>(index)] != 0) {
                ++core_hits;
            }
            if (index >= 0 && static_cast<size_t>(index) < gated_points.size()) {
                cluster_ranges.push_back(pointRange(gated_points[static_cast<size_t>(index)]));
            }
        }

        const cv::Point3d cluster_centroid = computeCentroid(gated_points, &indices);
        const double centroid_distance_sq = distanceSquared(cluster_centroid, reference_centroid);
        const double range_delta = std::fabs(computePercentile(cluster_ranges, 0.5) - reference_range);

        bool choose_cluster = false;
        if (has_core_reference) {
            choose_cluster = core_hits > best_core_hits
                || (core_hits == best_core_hits && centroid_distance_sq < best_centroid_distance_sq - 1e-6)
                || (core_hits == best_core_hits && std::fabs(centroid_distance_sq - best_centroid_distance_sq) <= 1e-6 && indices.size() > best_size);
        } else {
            choose_cluster = indices.size() > best_size
                || (indices.size() == best_size && range_delta < best_range_delta - 1e-6)
                || (indices.size() == best_size && std::fabs(range_delta - best_range_delta) <= 1e-6
                    && centroid_distance_sq < best_centroid_distance_sq - 1e-6);
        }

        if (!choose_cluster) {
            continue;
        }

        best_cluster_index = cluster_index;
        best_core_hits = core_hits;
        best_size = indices.size();
        best_centroid_distance_sq = centroid_distance_sq;
        best_range_delta = range_delta;
    }

    std::vector<pcl::PointXYZI> refined_points;
    refined_points.reserve(cluster_indices[best_cluster_index].indices.size());
    for (const int index : cluster_indices[best_cluster_index].indices) {
        if (index < 0 || static_cast<size_t>(index) >= gated_points.size()) {
            continue;
        }
        refined_points.push_back(gated_points[static_cast<size_t>(index)]);
    }
    return refined_points.empty() ? gated_points : refined_points;
}

PlanarRadiusStats computePlanarRadiusStats(
    const std::vector<pcl::PointXYZI>& cloud,
    const geometry_msgs::Point& centroid,
    double percentile,
    double outlier_tolerance_m)
{
    PlanarRadiusStats stats;
    if (cloud.empty()) {
        return stats;
    }

    std::vector<double> distances;
    distances.reserve(cloud.size());
    for (const auto& point : cloud) {
        const double dx = static_cast<double>(point.x) - centroid.x;
        const double dy = static_cast<double>(point.y) - centroid.y;
        distances.push_back(std::hypot(dx, dy));
    }

    stats.robust_radius = computePercentile(distances, percentile);
    const auto max_it = std::max_element(distances.begin(), distances.end());
    if (max_it != distances.end()) {
        stats.max_radius = *max_it;
    }

    const double support_threshold = stats.robust_radius + std::max(0.0, outlier_tolerance_m);
    stats.outer_support_points = static_cast<int>(std::count_if(
        distances.begin(),
        distances.end(),
        [support_threshold](double distance) {
            return distance > support_threshold;
        }));

    return stats;
}

lio_sam::SegmentedObjectStateArray buildSegmentedObjectStateArray(
    const std_msgs::Header& header,
    const std::vector<SegmentObject>& objects,
    const std::vector<std::vector<pcl::PointXYZI>>& object_clouds,
    const std::vector<std::string>& class_names,
    const ObjectCloudRefineSettings& refine_settings)
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
        state.class_name = resolveClassName(class_names, object.label);
        state.confidence = object.prob;
        state.pose.position.x = sum_x / point_count;
        state.pose.position.y = sum_y / point_count;
        state.pose.position.z = sum_z / point_count;
        state.pose.orientation.w = 1.0;
        state.size.x = size_x;
        state.size.y = size_y;
        state.size.z = size_z;
        const PlanarRadiusStats planar_radius_stats = computePlanarRadiusStats(
            cloud,
            state.pose.position,
            refine_settings.radius_percentile,
            refine_settings.radius_outlier_tolerance_m);
        const double fallback_planar_radius = 0.5 * std::hypot(size_x, size_y);
        const double robust_planar_radius = planar_radius_stats.robust_radius > 1e-3
            ? planar_radius_stats.robust_radius
            : fallback_planar_radius;
        const double max_planar_radius = planar_radius_stats.max_radius > 1e-3
            ? planar_radius_stats.max_radius
            : fallback_planar_radius;
        const bool accept_max_radius = max_planar_radius <= robust_planar_radius + refine_settings.radius_outlier_tolerance_m
            || planar_radius_stats.outer_support_points >= refine_settings.radius_outlier_min_support_points;

        double authoritative_radius = (accept_max_radius ? max_planar_radius : robust_planar_radius)
            + std::max(0.0, refine_settings.radius_padding_m);
        if (refine_settings.max_accepted_radius_m > 1e-3) {
            authoritative_radius = std::min(authoritative_radius, refine_settings.max_accepted_radius_m);
        }
        authoritative_radius = std::max(0.05, authoritative_radius);

        double bounding_radius = std::max(
            authoritative_radius,
            computeMaxSpatialRadius(cloud, state.pose.position) + std::max(0.0, refine_settings.radius_padding_m));
        if (refine_settings.max_accepted_radius_m > 1e-3) {
            bounding_radius = std::min(bounding_radius, refine_settings.max_accepted_radius_m);
        }
        bounding_radius = std::max(authoritative_radius, bounding_radius);

        state.bounding_radius = bounding_radius;
        state.footprint_radius = authoritative_radius;
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
    const ObjectCloudRefineSettings& refine_settings,
    cv::Mat* projected_overlay_image,
    cv::Mat* pure_segmentation_image,
    double* out_projection_ms = nullptr)
{
    res = image.clone();
    cv::Mat mask = image.clone();
    std::vector<std::vector<pcl::PointXYZI>> objects_pointcloud(objs.size());
    std::vector<std::vector<uint8_t>> objects_core_flags(objs.size());
    std::vector<cv::Point> projected_points;
    projected_points.reserve(input_cloud_ptr ? input_cloud_ptr->points.size() : 0);

    colored_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& obj : objs) {
        int idx = obj.label;
        const size_t idx_safe = idx >= 0 ? static_cast<size_t>(idx) : 0u;
        const size_t color_idx = colors.empty() ? 0u : (idx_safe % colors.size());
        const size_t mask_color_idx = maskColors.empty() ? 0u : (idx_safe % maskColors.size());

        const cv::Scalar color = colors.empty()
            ? cv::Scalar(0, 255, 0)
            : cv::Scalar(colors[color_idx][0], colors[color_idx][1], colors[color_idx][2]);
        const cv::Scalar mask_color = maskColors.empty()
            ? cv::Scalar(255, 56, 56)
            : cv::Scalar(maskColors[mask_color_idx][0], maskColors[mask_color_idx][1], maskColors[mask_color_idx][2]);
        cv::rectangle(res, obj.rect, color, 5);
        char text[256];
        const std::string class_name = resolveClassName(classNames, idx);
        std::snprintf(text, sizeof(text), "%s %.1f%%", class_name.c_str(), obj.prob * 100.0f);
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

    const auto projection_start = std::chrono::steady_clock::now();
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
                const bool is_core = !obj.coreMask.empty() && isPointInsideInstance(obj.coreMask, relative_x, relative_y);
                objects_core_flags[i].push_back(is_core ? 1 : 0);
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

    for (size_t i = 0; i < objects_pointcloud.size(); ++i) {
        objects_pointcloud[i] = refineObjectPointCloud(objects_pointcloud[i], objects_core_flags[i], refine_settings);
    }
    const auto projection_end = std::chrono::steady_clock::now();
    if (out_projection_ms) {
        *out_projection_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(projection_end - projection_start).count()) / 1000.0;
    }

    std::vector<cv::Point3d> object_centroids(objs.size(), cv::Point3d(0, 0, 0));
    for (size_t i = 0; i < objects_pointcloud.size(); ++i) {
        const auto& pc = objects_pointcloud[i];
        if (pc.empty()) {
            continue;
        }
        double sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto& p : pc) {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }
        double cnt = static_cast<double>(pc.size());
        object_centroids[i] = cv::Point3d(sum_x / cnt, sum_y / cnt, sum_z / cnt);
    }

    cv::addWeighted(res, 0.5, mask, 0.8, 1, res);
    if (pure_segmentation_image) {
        *pure_segmentation_image = res.clone();
    }
    for (size_t i = 0; i < objects_pointcloud.size(); ++i) {
        const auto& pc = objects_pointcloud[i];
        if (pc.empty()) {
            continue;
        }
        cv::Point2f rect_show_point2d;
        for (const auto& p : pc) {
            cv::Point2d img_pt_rec = projectToImagePlane(p, projection_model);
            if (!std::isfinite(img_pt_rec.x) || !std::isfinite(img_pt_rec.y)) {
                continue;
            }
            rect_show_point2d.x = img_pt_rec.x;
            rect_show_point2d.y = img_pt_rec.y;
            cv::circle(res, rect_show_point2d, 3, cv::Scalar(255, 255, 125), -1);
        }
    }
    if (projected_overlay_image) {
        *projected_overlay_image = res.clone();
        for (const auto& pt : projected_points) {
            cv::circle(*projected_overlay_image, pt, 3, cv::Scalar(255, 255, 125), -1);
        }
    }
    return objects_pointcloud;
}
}  // namespace

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
    ~RosNode() { finalizeTemporalMetrics(); }
    void rawCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void compressedCallback(const sensor_msgs::CompressedImageConstPtr &msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void processFrame(const std_msgs::Header& image_header, const cv::Mat& image, const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    std::vector<std::vector<pcl::PointXYZI>> pointcloud_vec;
private:
    using RawSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>;
    using CompressedSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::PointCloud2>;

    std::string pkg_path_, engine_file_path_, projection_config_path_;
    std::shared_ptr<YoloDetector> detector_;
    ProjectionModel projection_model_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Mat img_res_, image_;
    cv::Size size_ = cv::Size{640, 640};
    std::vector<SegmentObject> objs_;

    ros::NodeHandle n_;
    message_filters::Subscriber<sensor_msgs::Image> sub_img_raw_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> sub_img_compressed_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc_;
    std::shared_ptr<message_filters::Synchronizer<RawSyncPolicy>> raw_sync_;
    std::shared_ptr<message_filters::Synchronizer<CompressedSyncPolicy>> compressed_sync_;
    ros::Publisher pub_img_;
    ros::Publisher pub_img_pc_;
    ros::Publisher pub_pure_img_;
    ros::Publisher pub_color_cloud_;
    ros::Publisher pub_object_states_;
    std::string topic_img_, topic_res_img_, weight_name_, topic_pointcloud_, topic_res_img_pc_, topic_pure_img_, topic_object_states_;
    std::vector<std::string> class_names_;
    bool use_compressed_image_ = false;
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

    // Latency stats aligned with the document (Section 5.2)
    bool enable_latency_stats_ = true;
    double latency_log_period_sec_ = 5.0;
    ros::Time latency_last_log_time_;
    // 一、视觉推理模块
    StageStats stat_vision_infer_ms_;
    // 二、点云投影与图像关联模块
    StageStats stat_cloud_preprocess_ms_;
    StageStats stat_tf_lookup_ms_;
    StageStats stat_projection_assoc_ms_;
    StageStats stat_organize_publish_ms_;
    // 四、端到端总时延
    StageStats stat_e2e_total_ms_;
    // 传感器时延（输入时间戳到回调开始）
    StageStats stat_sensor_to_callback_ms_;
    std::string latency_csv_path_;
    std::ofstream latency_csv_;

    bool enable_pointcloud_downsample_ = true;
    double pointcloud_leaf_size_m_ = 0.03;
    ObjectCloudRefineSettings object_cloud_refine_settings_;

    bool enable_temporal_metrics_ = false;
    double temporal_metrics_log_period_sec_ = 5.0;
    std::string temporal_metrics_csv_path_;
    ros::Time temporal_metrics_last_log_time_;
    TemporalMetricState temporal_metric_state_;
    std::ofstream temporal_metrics_csv_;

    void updateTemporalMetrics(const cv::Size& image_size, const std::vector<SegmentObject>& objs);
    void maybeLogTemporalMetrics();
    void finalizeTemporalMetrics();

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

    void recordAndMaybeLogLatency(
        double vision_infer_ms,
        double cloud_preprocess_ms,
        double tf_lookup_ms,
        double projection_assoc_ms,
        double organize_publish_ms,
        double e2e_total_ms,
        double sensor_to_callback_ms);
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

void RosNode::recordAndMaybeLogLatency(
    double vision_infer_ms,
    double cloud_preprocess_ms,
    double tf_lookup_ms,
    double projection_assoc_ms,
    double organize_publish_ms,
    double e2e_total_ms,
    double sensor_to_callback_ms)
{
    if (!enable_latency_stats_) {
        return;
    }

    stat_vision_infer_ms_.add(vision_infer_ms);
    stat_cloud_preprocess_ms_.add(cloud_preprocess_ms);
    stat_tf_lookup_ms_.add(tf_lookup_ms);
    stat_projection_assoc_ms_.add(projection_assoc_ms);
    stat_organize_publish_ms_.add(organize_publish_ms);
    stat_e2e_total_ms_.add(e2e_total_ms);
    stat_sensor_to_callback_ms_.add(sensor_to_callback_ms);

    const ros::Time now = ros::Time::now();
    if (latency_last_log_time_.isZero()) {
        latency_last_log_time_ = now;
        return;
    }

    if ((now - latency_last_log_time_).toSec() < latency_log_period_sec_) {
        return;
    }

    auto dump = [](const char* label, const StageStats& s) {
        ROS_INFO(
            "[segment latency] %-22s samples=%4zu avg=%6.2f std=%6.2f p95=%6.2f max=%6.2f ms",
            label, s.count, s.avg(), s.stdDev(), s.p95(), s.max);
    };

    ROS_INFO("[segment latency] === Module Latency Report ===");
    dump("vision_infer", stat_vision_infer_ms_);
    dump("cloud_preprocess", stat_cloud_preprocess_ms_);
    dump("tf_lookup", stat_tf_lookup_ms_);
    dump("projection_assoc", stat_projection_assoc_ms_);
    dump("organize_publish", stat_organize_publish_ms_);
    dump("e2e_total", stat_e2e_total_ms_);
    dump("sensor_to_callback", stat_sensor_to_callback_ms_);

    if (latency_csv_.is_open()) {
        auto write_row = [&](const char* stage, const StageStats& s) {
            latency_csv_ << std::fixed << std::setprecision(3)
                         << now.toSec() << ","
                         << stage << ","
                         << s.count << ","
                         << s.avg() << ","
                         << s.stdDev() << ","
                         << s.p95() << ","
                         << s.max << "\n";
        };
        write_row("vision_infer", stat_vision_infer_ms_);
        write_row("cloud_preprocess", stat_cloud_preprocess_ms_);
        write_row("tf_lookup", stat_tf_lookup_ms_);
        write_row("projection_assoc", stat_projection_assoc_ms_);
        write_row("organize_publish", stat_organize_publish_ms_);
        write_row("e2e_total", stat_e2e_total_ms_);
        write_row("sensor_to_callback", stat_sensor_to_callback_ms_);
        latency_csv_.flush();
    }

    stat_vision_infer_ms_.reset();
    stat_cloud_preprocess_ms_.reset();
    stat_tf_lookup_ms_.reset();
    stat_projection_assoc_ms_.reset();
    stat_organize_publish_ms_.reset();
    stat_e2e_total_ms_.reset();
    stat_sensor_to_callback_ms_.reset();
    latency_last_log_time_ = now;
}

void RosNode::updateTemporalMetrics(const cv::Size& image_size, const std::vector<SegmentObject>& objs)
{
    if (!enable_temporal_metrics_) {
        return;
    }

    ++temporal_metric_state_.frame_counter;

    const SegmentObject* rep = selectRepresentativeTarget(
        objs, temporal_metric_state_.has_prev ? &temporal_metric_state_ : nullptr);

    const bool detected = (rep != nullptr);

    if (detected) {
        if (!temporal_metric_state_.first_detected) {
            temporal_metric_state_.first_detected = true;
            temporal_metric_state_.first_detect_frame_idx = temporal_metric_state_.frame_counter;
        }
        temporal_metric_state_.current_consecutive_miss = 0;

        cv::Mat full_mask = createFullSizeMask(image_size, *rep);
        cv::Point2f center = computeBboxCenter(rep->rect);

        double tiou = 0.0;
        double drift = 0.0;
        bool has_metrics = false;

        if (temporal_metric_state_.has_prev) {
            tiou = computeMaskIoU(temporal_metric_state_.prev_full_mask, full_mask);
            const double dx = center.x - temporal_metric_state_.prev_center.x;
            const double dy = center.y - temporal_metric_state_.prev_center.y;
            drift = std::sqrt(dx * dx + dy * dy);

            temporal_metric_state_.tiou_sum += tiou;
            ++temporal_metric_state_.tiou_count;
            temporal_metric_state_.drift_sum += drift;
            ++temporal_metric_state_.drift_count;
            has_metrics = true;
        }

        if (temporal_metrics_csv_.is_open()) {
            temporal_metrics_csv_ << temporal_metric_state_.frame_counter << ",1,";
            if (has_metrics) {
                temporal_metrics_csv_ << std::fixed << std::setprecision(4) << tiou << "," << drift << "\n";
            } else {
                temporal_metrics_csv_ << ",\n";
            }
            temporal_metrics_csv_.flush();
        }

        temporal_metric_state_.prev_full_mask = full_mask.clone();
        temporal_metric_state_.prev_rect = rep->rect;
        temporal_metric_state_.prev_center = center;
        temporal_metric_state_.prev_conf = rep->prob;
        temporal_metric_state_.prev_label = rep->label;
        temporal_metric_state_.has_prev = true;
    } else {
        if (temporal_metric_state_.first_detected) {
            ++temporal_metric_state_.miss_count;
            ++temporal_metric_state_.current_consecutive_miss;
            temporal_metric_state_.max_consecutive_miss = std::max(
                temporal_metric_state_.max_consecutive_miss,
                temporal_metric_state_.current_consecutive_miss);
        }

        if (temporal_metrics_csv_.is_open()) {
            temporal_metrics_csv_ << temporal_metric_state_.frame_counter << ",0,,\n";
            temporal_metrics_csv_.flush();
        }
    }
}

void RosNode::maybeLogTemporalMetrics()
{
    if (!enable_temporal_metrics_) {
        return;
    }

    const ros::Time now = ros::Time::now();
    if (temporal_metrics_last_log_time_.isZero()) {
        temporal_metrics_last_log_time_ = now;
        return;
    }

    if ((now - temporal_metrics_last_log_time_).toSec() < temporal_metrics_log_period_sec_) {
        return;
    }

    const double avg_tiou = temporal_metric_state_.tiou_count > 0
        ? temporal_metric_state_.tiou_sum / static_cast<double>(temporal_metric_state_.tiou_count)
        : 0.0;
    const double avg_drift = temporal_metric_state_.drift_count > 0
        ? temporal_metric_state_.drift_sum / static_cast<double>(temporal_metric_state_.drift_count)
        : 0.0;

    size_t eff_frames = 0;
    double miss_rate = 0.0;
    if (temporal_metric_state_.first_detected) {
        eff_frames = temporal_metric_state_.frame_counter - temporal_metric_state_.first_detect_frame_idx + 1;
        if (eff_frames > 0) {
            miss_rate = static_cast<double>(temporal_metric_state_.miss_count) / static_cast<double>(eff_frames);
        }
    }

    ROS_INFO(
        "[temporal metrics] frames=%zu first_detect=%zu eff=%zu miss=%zu miss_rate=%.4f max_consecutive_miss=%zu avg_tiou=%.4f avg_drift=%.2fpx",
        temporal_metric_state_.frame_counter,
        temporal_metric_state_.first_detect_frame_idx,
        eff_frames,
        temporal_metric_state_.miss_count,
        miss_rate,
        temporal_metric_state_.max_consecutive_miss,
        avg_tiou,
        avg_drift);

    temporal_metrics_last_log_time_ = now;
}

void RosNode::finalizeTemporalMetrics()
{
    if (!enable_temporal_metrics_) {
        return;
    }

    if (temporal_metrics_csv_.is_open()) {
        temporal_metrics_csv_.close();
    }

    const double avg_tiou = temporal_metric_state_.tiou_count > 0
        ? temporal_metric_state_.tiou_sum / static_cast<double>(temporal_metric_state_.tiou_count)
        : 0.0;
    const double avg_drift = temporal_metric_state_.drift_count > 0
        ? temporal_metric_state_.drift_sum / static_cast<double>(temporal_metric_state_.drift_count)
        : 0.0;

    size_t eff_frames = 0;
    double miss_rate = 0.0;
    if (temporal_metric_state_.first_detected) {
        eff_frames = temporal_metric_state_.frame_counter - temporal_metric_state_.first_detect_frame_idx + 1;
        if (eff_frames > 0) {
            miss_rate = static_cast<double>(temporal_metric_state_.miss_count) / static_cast<double>(eff_frames);
        }
    }

    ROS_INFO(
        "[temporal metrics final] frames=%zu eff=%zu miss=%zu miss_rate=%.4f max_consecutive_miss=%zu avg_tiou=%.4f avg_drift=%.2fpx",
        temporal_metric_state_.frame_counter,
        eff_frames,
        temporal_metric_state_.miss_count,
        miss_rate,
        temporal_metric_state_.max_consecutive_miss,
        avg_tiou,
        avg_drift);
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

void RosNode::rawCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    if (!msg) {
        return;
    }

    const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    processFrame(msg->header, image, cloud_msg);
}

void RosNode::compressedCallback(const sensor_msgs::CompressedImageConstPtr &msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    if (!msg) {
        return;
    }

    const cv::Mat image = cv::imdecode(msg->data, cv::IMREAD_COLOR);
    if (image.empty()) {
        ROS_WARN_THROTTLE(2.0, "Failed to decode compressed image from topic %s.", topic_img_.c_str());
        return;
    }

    processFrame(msg->header, image, cloud_msg);
}

void RosNode::processFrame(const std_msgs::Header& image_header, const cv::Mat& image_input, const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    if (!cloud_msg) {
        return;
    }

    const auto callback_start = std::chrono::steady_clock::now();
    const ros::Time callback_ros_time = ros::Time::now();
    const double sensor_to_callback_ms = image_header.stamp.isZero() ? 0.0 :
        std::max(0.0, (callback_ros_time - image_header.stamp).toSec() * 1000.0);

    // 点云预处理（格式转换 + 降采样）
    const auto cloud_preprocess_start = std::chrono::steady_clock::now();
    auto current_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *current_cloud);
    const auto& current_cloud_header = cloud_msg->header;

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
    const auto cloud_preprocess_end = std::chrono::steady_clock::now();

    // 图像预处理与推理
    cv::Mat image = image_input;
    const auto infer_start = std::chrono::steady_clock::now();
    auto detections = detector_->inference(image);
    const auto infer_end = std::chrono::steady_clock::now();
    objs_ = convertDetectionsToObjects(detections, image.size(), object_cloud_refine_settings_);
    const auto convert_end = std::chrono::steady_clock::now();

    const ros::Duration stamp_delta = image_header.stamp - cloud_msg->header.stamp;
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

    // 坐标查询与变换
    ProjectionModel active_projection_model = projection_model_;
    const ros::Time transform_stamp = cloud_msg->header.stamp.isZero() ? image_header.stamp : cloud_msg->header.stamp;
    const auto tf_start = std::chrono::steady_clock::now();
    const bool using_tf_projection =
        use_tf_for_projection_ &&
        tryBuildProjectionModelFromTf(image_header.frame_id, cloud_msg->header.frame_id, transform_stamp, &active_projection_model);
    const auto tf_end = std::chrono::steady_clock::now();

    if (using_tf_projection) {
        if (!tf_projection_logged_) {
            ROS_INFO(
                "Using TF for projection from point cloud frame '%s' to image frame '%s'.",
                cloud_msg->header.frame_id.c_str(),
                image_header.frame_id.c_str());
            tf_projection_logged_ = true;
        }
    } else if (!yaml_fallback_logged_) {
        ROS_INFO("Using projection extrinsics from YAML: %s", projection_config_path_.c_str());
        yaml_fallback_logged_ = true;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

    // 投影与像素关联 + 可视化绘制
    const auto draw_start = std::chrono::steady_clock::now();
    cv::Mat image_to_show;
    cv::Mat pure_segmentation;
    double projection_assoc_ms = 0.0;
    pointcloud_vec = drawSegmentObjects(
        image,
        img_res_,
        objs_,
        class_names_,
        COLORS,
        MASK_COLORS,
        processing_cloud,
        colored_cloud,
        active_projection_model,
        object_cloud_refine_settings_,
        &image_to_show,
        &pure_segmentation,
        &projection_assoc_ms);
    const auto draw_end = std::chrono::steady_clock::now();

    auto tc = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(infer_end - infer_start).count()) / 1000.;
    cv::putText(img_res_, "fps: " + std::to_string(int(1000 / tc)), cv::Point(20, 40),
                cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 0, 255), 2, 8);
    ROS_INFO("segment cost %2.4f ms", tc);

    if (!current_cloud || current_cloud->points.empty()) {
        ROS_WARN_THROTTLE(2.0, "No point cloud data available for projection.");
    }

    // 结果组织与发布
    const auto organize_publish_start = std::chrono::steady_clock::now();
    if (image_to_show.empty()) {
        image_to_show = img_res_.clone();
    }
    sensor_msgs::ImagePtr msg_img_new = cv_bridge::CvImage(image_header, "bgr8", img_res_).toImageMsg();
    sensor_msgs::ImagePtr msg_img_new_pc = cv_bridge::CvImage(image_header, "bgr8", image_to_show).toImageMsg();
    sensor_msgs::ImagePtr msg_pure_img = cv_bridge::CvImage(image_header, "bgr8", pure_segmentation).toImageMsg();
    ROS_DEBUG_THROTTLE(2.0, "Output Image Size: Width = %d, Height = %d", img_res_.cols, img_res_.rows);
    pub_img_pc_.publish(msg_img_new_pc);
    pub_img_.publish(msg_img_new);
    pub_pure_img_.publish(msg_pure_img);
    pub_object_states_.publish(buildSegmentedObjectStateArray(
        current_cloud_header,
        objs_,
        pointcloud_vec,
        class_names_,
        object_cloud_refine_settings_));

    sensor_msgs::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*colored_cloud, output_cloud_msg);
    output_cloud_msg.header = current_cloud_header;
    pub_color_cloud_.publish(output_cloud_msg);
    const auto publish_end = std::chrono::steady_clock::now();

    // Legacy perf stats (keep for backward compatibility)
    const double infer_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(infer_end - infer_start).count()) / 1000.0;
    const double convert_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(convert_end - infer_end).count()) / 1000.0;
    const double downsample_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(cloud_preprocess_end - cloud_preprocess_start).count()) / 1000.0;
    const double tf_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(tf_end - tf_start).count()) / 1000.0;
    const double draw_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(draw_end - draw_start).count()) / 1000.0;
    const double overlay_ms = 0.0;
    const double publish_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(publish_end - organize_publish_start).count()) / 1000.0;
    const double total_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(publish_end - callback_start).count()) / 1000.0;
    updateTemporalMetrics(image.size(), objs_);
    maybeLogTemporalMetrics();
    recordAndMaybeLogPerf(total_ms, infer_ms, convert_ms, downsample_ms, tf_ms, draw_ms, overlay_ms, publish_ms, stamp_delta_sec);

    // New latency stats aligned with the document
    const double vision_infer_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(convert_end - infer_start).count()) / 1000.0;
    const double cloud_preprocess_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(cloud_preprocess_end - cloud_preprocess_start).count()) / 1000.0;
    const double tf_lookup_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(tf_end - tf_start).count()) / 1000.0;
    const double organize_publish_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(publish_end - organize_publish_start).count()) / 1000.0;
    const double e2e_total_ms = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(publish_end - callback_start).count()) / 1000.0;
    recordAndMaybeLogLatency(vision_infer_ms, cloud_preprocess_ms, tf_lookup_ms, projection_assoc_ms, organize_publish_ms, e2e_total_ms, sensor_to_callback_ms);
}

RosNode::RosNode()
{
    cudaSetDevice(0);
    pkg_path_ = ros::package::getPath("yolo26_ros");
    projection_config_path_ = pkg_path_ + "/config/segment_projection.yaml";
    n_.param<std::string>("topic_img", topic_img_, "/camera/color/image_raw");
    n_.param<bool>("use_compressed_image", use_compressed_image_, use_compressed_image_);
    n_.param<std::string>("topic_pointcloud", topic_pointcloud_, "/livox/lidar");
    n_.param<std::string>("topic_res_img", topic_res_img_, "/detect/image_raw");
    n_.param<std::string>("topic_res_img_pc_", topic_res_img_pc_, "/detect/topic_res_img_pc_");
    n_.param<std::string>("topic_pure_img", topic_pure_img_, "/segment/pure_image_raw");
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
    n_.param<bool>("enable_object_cloud_refine", object_cloud_refine_settings_.enable, object_cloud_refine_settings_.enable);
    n_.param<int>("object_mask_erode_pixels", object_cloud_refine_settings_.mask_erode_pixels, object_cloud_refine_settings_.mask_erode_pixels);
    n_.param<int>("object_min_core_points", object_cloud_refine_settings_.min_core_points, object_cloud_refine_settings_.min_core_points);
    n_.param<double>("object_range_gate_min_m", object_cloud_refine_settings_.range_gate_min_m, object_cloud_refine_settings_.range_gate_min_m);
    n_.param<double>("object_range_gate_scale", object_cloud_refine_settings_.range_gate_scale, object_cloud_refine_settings_.range_gate_scale);
    n_.param<double>("object_cluster_tolerance_m", object_cloud_refine_settings_.cluster_tolerance_m, object_cloud_refine_settings_.cluster_tolerance_m);
    n_.param<int>("object_cluster_min_points", object_cloud_refine_settings_.cluster_min_points, object_cloud_refine_settings_.cluster_min_points);
    n_.param<double>("object_radius_percentile", object_cloud_refine_settings_.radius_percentile, object_cloud_refine_settings_.radius_percentile);
    n_.param<double>("object_radius_outlier_tolerance_m", object_cloud_refine_settings_.radius_outlier_tolerance_m, object_cloud_refine_settings_.radius_outlier_tolerance_m);
    n_.param<int>("object_radius_outlier_min_support_points", object_cloud_refine_settings_.radius_outlier_min_support_points, object_cloud_refine_settings_.radius_outlier_min_support_points);
    n_.param<double>("object_radius_padding_m", object_cloud_refine_settings_.radius_padding_m, object_cloud_refine_settings_.radius_padding_m);
    n_.param<double>("object_max_accepted_radius_m", object_cloud_refine_settings_.max_accepted_radius_m, object_cloud_refine_settings_.max_accepted_radius_m);
    n_.param<bool>("enable_temporal_metrics", enable_temporal_metrics_, enable_temporal_metrics_);
    n_.param<double>("temporal_metrics_log_period_sec", temporal_metrics_log_period_sec_, temporal_metrics_log_period_sec_);
    n_.param<std::string>("temporal_metrics_csv_path", temporal_metrics_csv_path_, temporal_metrics_csv_path_);
    n_.param<bool>("enable_latency_stats", enable_latency_stats_, enable_latency_stats_);
    n_.param<double>("latency_log_period_sec", latency_log_period_sec_, latency_log_period_sec_);
    n_.param<std::string>("latency_csv_path", latency_csv_path_, latency_csv_path_);
    class_names_ = readClassNamesParam(n_);
    engine_file_path_ = pkg_path_ + "/weights/" + weight_name_;
    projection_model_.loadFromFile(projection_config_path_);
    projection_model_.flip_lidar_y = flip_lidar_y_for_projection_;
    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

    std::cout << "\n\033[1;32m--engine_file_path: " << engine_file_path_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m" << "--topic_img       : " << topic_img_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--img_compressed  : " << (use_compressed_image_ ? "true" : "false") << "\033[0m" << std::endl;
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
    std::cout << "\033[1;32m--obj_refine       : " << (object_cloud_refine_settings_.enable ? "true" : "false") << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--obj_mask_erode(px): " << object_cloud_refine_settings_.mask_erode_pixels << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--obj_range_gate(m): min=" << object_cloud_refine_settings_.range_gate_min_m
              << " scale=" << object_cloud_refine_settings_.range_gate_scale << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--obj_cluster_tol  : " << object_cloud_refine_settings_.cluster_tolerance_m << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--obj_cluster_min  : " << object_cloud_refine_settings_.cluster_min_points << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--obj_radius_pct   : " << object_cloud_refine_settings_.radius_percentile << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--obj_radius_outlier_tol: " << object_cloud_refine_settings_.radius_outlier_tolerance_m << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--obj_radius_outlier_support: " << object_cloud_refine_settings_.radius_outlier_min_support_points << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--obj_radius_padding: " << object_cloud_refine_settings_.radius_padding_m << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--obj_radius_abs_max: " << object_cloud_refine_settings_.max_accepted_radius_m << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--temporal_metrics   : " << (enable_temporal_metrics_ ? "true" : "false") << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--temporal_log_period: " << temporal_metrics_log_period_sec_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--temporal_csv_path  : " << temporal_metrics_csv_path_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--enable_latency_stats: " << (enable_latency_stats_ ? "true" : "false") << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--latency_log_period : " << latency_log_period_sec_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32m--latency_csv_path   : " << latency_csv_path_ << "\033[0m" << std::endl;
    if (class_names_.empty()) {
        ROS_WARN("No 'class_names' parameter provided. Falling back to generated labels like class0/class1.");
    } else {
        ROS_INFO("Loaded %zu class names from ROS parameter 'class_names'.", class_names_.size());
    }

    if (enable_temporal_metrics_ && !temporal_metrics_csv_path_.empty()) {
        temporal_metrics_csv_.open(temporal_metrics_csv_path_, std::ios::out | std::ios::trunc);
        if (temporal_metrics_csv_.is_open()) {
            temporal_metrics_csv_ << "frame_idx,has_detection,tiou,center_drift_px\n";
            temporal_metrics_csv_.flush();
            ROS_INFO("Temporal metrics CSV: %s", temporal_metrics_csv_path_.c_str());
        } else {
            ROS_WARN("Failed to open temporal metrics CSV: %s", temporal_metrics_csv_path_.c_str());
        }
    }

    if (enable_latency_stats_ && !latency_csv_path_.empty()) {
        latency_csv_.open(latency_csv_path_, std::ios::out | std::ios::trunc);
        if (latency_csv_.is_open()) {
            latency_csv_ << "timestamp,stage,samples,avg_ms,std_ms,p95_ms,max_ms\n";
            latency_csv_.flush();
            ROS_INFO("Latency CSV: %s", latency_csv_path_.c_str());
        } else {
            ROS_WARN("Failed to open latency CSV: %s", latency_csv_path_.c_str());
        }
    }

    detector_.reset(new YoloDetector(engine_file_path_));

    pub_img_pc_ = n_.advertise<sensor_msgs::Image>(topic_res_img_pc_, 10);
    pub_img_ = n_.advertise<sensor_msgs::Image>(topic_res_img_, 10);
    pub_pure_img_ = n_.advertise<sensor_msgs::Image>(topic_pure_img_, 10);
    pub_color_cloud_ = n_.advertise<sensor_msgs::PointCloud2>("/colored_point_cloud", 10);
    pub_object_states_ = n_.advertise<lio_sam::SegmentedObjectStateArray>(topic_object_states_, 10);

    sub_pc_.subscribe(n_, topic_pointcloud_, 10);

    if (use_compressed_image_) {
        sub_img_compressed_.subscribe(n_, topic_img_, 10);
        compressed_sync_.reset(new message_filters::Synchronizer<CompressedSyncPolicy>(CompressedSyncPolicy(20), sub_img_compressed_, sub_pc_));
        compressed_sync_->setMaxIntervalDuration(ros::Duration(sync_max_interval_sec_));
        compressed_sync_->registerCallback(boost::bind(&RosNode::compressedCallback, this, _1, _2));
    } else {
        sub_img_raw_.subscribe(n_, topic_img_, 10);
        raw_sync_.reset(new message_filters::Synchronizer<RawSyncPolicy>(RawSyncPolicy(20), sub_img_raw_, sub_pc_));
        raw_sync_->setMaxIntervalDuration(ros::Duration(sync_max_interval_sec_));
        raw_sync_->registerCallback(boost::bind(&RosNode::rawCallback, this, _1, _2));
    }
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