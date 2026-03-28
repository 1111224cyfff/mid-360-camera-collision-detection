#ifndef YOLO26_ROS_INFER_H
#define YOLO26_ROS_INFER_H

#include <algorithm>
#include <array>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "tensorrt/yolo_tensorrt.h"

#if !defined(YOLO26_TASK_DETECT) && !defined(YOLO26_TASK_SEGMENT) && !defined(YOLO26_TASK_POSE)
#error "One of YOLO26_TASK_DETECT, YOLO26_TASK_SEGMENT, or YOLO26_TASK_POSE must be defined."
#endif

struct Detection
{
    float bbox[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float conf = 0.0f;
    int classId = 0;
    float mask[32] = {0.0f};
    std::vector<float> maskMatrix;
    std::vector<float> keypoint;
};

class YoloDetector
{
public:
    explicit YoloDetector(const std::string& trtFile)
    {
        detector_.init(YOLO26, GPU, FP16, trtFile);
    }

    ~YoloDetector()
    {
        static_cast<YOLO&>(detector_).release();
    }

    std::vector<Detection> inference(cv::Mat& img)
    {
        static_cast<YOLO&>(detector_).infer(img, false);
        return convertOutputs(img.size());
    }

    static void draw_image(cv::Mat& img, std::vector<Detection>& inferResult, bool drawBbox = true, bool drawPose = false)
    {
        static const std::array<std::pair<int, int>, 19> skeletons = {{
            {16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13},
            {6, 12}, {7, 13}, {6, 7}, {6, 8}, {7, 9},
            {8, 10}, {9, 11}, {2, 3}, {1, 2}, {1, 3},
            {2, 4}, {3, 5}, {4, 6}, {5, 7}
        }};

        for (const auto& det : inferResult)
        {
            cv::Rect box(
                static_cast<int>(det.bbox[0]),
                static_cast<int>(det.bbox[1]),
                std::max(0, static_cast<int>(det.bbox[2] - det.bbox[0])),
                std::max(0, static_cast<int>(det.bbox[3] - det.bbox[1])));

            if (drawBbox)
            {
                cv::rectangle(img, box, cv::Scalar(0, 255, 0), 2);
                std::string label = "class" + std::to_string(det.classId) + ":" + cv::format("%.2f", det.conf);
                cv::putText(img, label, cv::Point(box.x, std::max(0, box.y - 4)), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }

            if (!drawPose || det.keypoint.empty())
            {
                continue;
            }

            for (size_t i = 0; i < det.keypoint.size() / 3; ++i)
            {
                float x = det.keypoint[3 * i];
                float y = det.keypoint[3 * i + 1];
                float conf = det.keypoint[3 * i + 2];
                if (conf < 0.5f)
                {
                    continue;
                }
                cv::circle(img, cv::Point(static_cast<int>(x), static_cast<int>(y)), 4, cv::Scalar(255, 0, 0), -1);
            }

            for (const auto& skeleton : skeletons)
            {
                const int first = skeleton.first - 1;
                const int second = skeleton.second - 1;
                if (det.keypoint.size() < static_cast<size_t>((std::max(first, second) + 1) * 3))
                {
                    continue;
                }

                float conf1 = det.keypoint[3 * first + 2];
                float conf2 = det.keypoint[3 * second + 2];
                if (conf1 < 0.5f || conf2 < 0.5f)
                {
                    continue;
                }

                cv::Point p1(static_cast<int>(det.keypoint[3 * first]), static_cast<int>(det.keypoint[3 * first + 1]));
                cv::Point p2(static_cast<int>(det.keypoint[3 * second]), static_cast<int>(det.keypoint[3 * second + 1]));
                cv::line(img, p1, p2, cv::Scalar(255, 0, 0), 2);
            }
        }
    }

private:
#if defined(YOLO26_TASK_DETECT)
    using BackendDetector = YOLO_TensorRT_Detect;
#elif defined(YOLO26_TASK_SEGMENT)
    using BackendDetector = YOLO_TensorRT_Segment;
#else
    using BackendDetector = YOLO_TensorRT_Pose;
#endif

    std::vector<Detection> convertOutputs(const cv::Size& imageSize) const
    {
        std::vector<Detection> detections;

#if defined(YOLO26_TASK_DETECT)
        const auto& outputs = detector_.detections();
        detections.reserve(outputs.size());
        for (const auto& output : outputs)
        {
            Detection det;
            det.bbox[0] = output.box.x;
            det.bbox[1] = output.box.y;
            det.bbox[2] = output.box.x + output.box.width;
            det.bbox[3] = output.box.y + output.box.height;
            det.conf = output.score;
            det.classId = output.id;
            detections.push_back(std::move(det));
        }
#elif defined(YOLO26_TASK_SEGMENT)
        const auto& outputs = detector_.segments();
        detections.reserve(outputs.size());
        for (const auto& output : outputs)
        {
            Detection det;
            const cv::Rect rect = output.box & cv::Rect(0, 0, imageSize.width, imageSize.height);
            det.bbox[0] = rect.x;
            det.bbox[1] = rect.y;
            det.bbox[2] = rect.x + rect.width;
            det.bbox[3] = rect.y + rect.height;
            det.conf = output.score;
            det.classId = output.id;

            cv::Mat fullMask = cv::Mat::zeros(imageSize, CV_32F);
            if (!output.mask.empty() && rect.width > 0 && rect.height > 0)
            {
                cv::Mat maskFloat;
                output.mask.convertTo(maskFloat, CV_32F, 1.0 / 255.0);
                if (maskFloat.size() != rect.size())
                {
                    cv::resize(maskFloat, maskFloat, rect.size(), 0.0, 0.0, cv::INTER_NEAREST);
                }
                maskFloat.copyTo(fullMask(rect));
            }

            det.maskMatrix.assign(
                reinterpret_cast<const float*>(fullMask.datastart),
                reinterpret_cast<const float*>(fullMask.dataend));
            detections.push_back(std::move(det));
        }
#else
        const auto& outputs = detector_.poses();
        detections.reserve(outputs.size());
        for (const auto& output : outputs)
        {
            Detection det;
            det.bbox[0] = output.box.x;
            det.bbox[1] = output.box.y;
            det.bbox[2] = output.box.x + output.box.width;
            det.bbox[3] = output.box.y + output.box.height;
            det.conf = output.score;
            det.classId = output.id;
            det.keypoint = output.keypoint;
            detections.push_back(std::move(det));
        }
#endif

        return detections;
    }

    BackendDetector detector_;
};

#endif