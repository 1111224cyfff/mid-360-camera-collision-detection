#include <ros/ros.h>

#include <XmlRpcValue.h>

#include <lio_sam/SegmentedObjectState.h>
#include <lio_sam/SegmentedObjectStateArray.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{

double clampValue(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double pointDistance(
  const geometry_msgs::Point& lhs,
  const geometry_msgs::Point& rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

class VisualObjectSelectorNode
{
public:
  VisualObjectSelectorNode()
  : pnh_("~")
  {
    pnh_.param<std::string>("input_objects_topic", input_objects_topic_, std::string("/segment/object_states"));
    pnh_.param<std::string>("output_topic", output_topic_, std::string("/warning/selected_visual_object_states"));
    pnh_.param<double>("min_object_confidence", min_object_confidence_, 0.35);
    pnh_.param<int>("min_object_points", min_object_points_, 25);
    pnh_.param<double>("radius_smoothing_alpha", radius_smoothing_alpha_, 0.6);
    pnh_.param<double>("max_radius_growth_per_frame_m", max_radius_growth_per_frame_m_, 0.15);
    pnh_.param<double>("large_expansion_confirm_delta_m", large_expansion_confirm_delta_m_, 0.3);
    pnh_.param<int>("large_expansion_confirm_frames", large_expansion_confirm_frames_, 3);
    pnh_.param<double>("large_expansion_match_tolerance_m", large_expansion_match_tolerance_m_, 0.15);
    pnh_.param<double>("absolute_max_radius_m", absolute_max_radius_m_, 0.0);
    pnh_.param<double>("target_match_position_tolerance_m", target_match_position_tolerance_m_, 1.5);
    pnh_.param<double>("target_match_range_tolerance_m", target_match_range_tolerance_m_, 1.0);

    loadSelectedClassNames();

    sub_objects_ = nh_.subscribe(input_objects_topic_, 3, &VisualObjectSelectorNode::objectsCallback, this);
    pub_selected_objects_ = nh_.advertise<lio_sam::SegmentedObjectStateArray>(output_topic_, 1);

    ROS_INFO_STREAM("[visual_object_selector] input_objects_topic=" << input_objects_topic_
                    << " output_topic=" << output_topic_
                    << " min_object_confidence=" << min_object_confidence_
                    << " min_object_points=" << min_object_points_
                    << " max_radius_growth_per_frame_m=" << max_radius_growth_per_frame_m_
                    << " large_expansion_confirm_delta_m=" << large_expansion_confirm_delta_m_
                    << " large_expansion_confirm_frames=" << large_expansion_confirm_frames_
                    << " absolute_max_radius_m=" << absolute_max_radius_m_);
  }

private:
  struct RadiusFilterState
  {
    bool has_active_target{false};
    lio_sam::SegmentedObjectState reference_object;
    double filtered_radius{0.0};
    double pending_large_radius{0.0};
    int pending_large_expansion_frames{0};
  };

  void loadSelectedClassNames()
  {
    XmlRpc::XmlRpcValue class_names_param;
    if (!pnh_.getParam("selected_class_names", class_names_param)) {
      return;
    }
    if (class_names_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("[visual_object_selector] selected_class_names must be a YAML list.");
      return;
    }

    selected_class_names_.clear();
    for (int index = 0; index < class_names_param.size(); ++index) {
      if (class_names_param[index].getType() != XmlRpc::XmlRpcValue::TypeString) {
        continue;
      }
      selected_class_names_.push_back(static_cast<std::string>(class_names_param[index]));
    }
  }

  bool selectObject(
    const lio_sam::SegmentedObjectStateArray& objects_msg,
    lio_sam::SegmentedObjectState* selected_object) const
  {
    if (!selected_object) {
      return false;
    }

    const lio_sam::SegmentedObjectState* best = nullptr;
    for (const auto& object : objects_msg.objects) {
      if (object.confidence < min_object_confidence_) {
        continue;
      }
      if (static_cast<int>(object.point_count) < min_object_points_) {
        continue;
      }
      if (!selected_class_names_.empty()) {
        const bool matches = std::find(
          selected_class_names_.begin(),
          selected_class_names_.end(),
          object.class_name) != selected_class_names_.end();
        if (!matches) {
          continue;
        }
      }

      if (!best) {
        best = &object;
        continue;
      }

      if (object.nearest_range < best->nearest_range - 1e-3f) {
        best = &object;
        continue;
      }

      if (std::fabs(object.nearest_range - best->nearest_range) <= 1e-3f && object.confidence > best->confidence) {
        best = &object;
      }
    }

    if (!best) {
      return false;
    }

    *selected_object = *best;
    return true;
  }

  double objectRadius(const lio_sam::SegmentedObjectState& object) const
  {
    if (object.footprint_radius > 1e-3f) {
      return static_cast<double>(object.footprint_radius);
    }

    const double size_x = static_cast<double>(object.size.x);
    const double size_y = static_cast<double>(object.size.y);
    const double from_size = 0.5 * std::hypot(size_x, size_y);
    if (from_size > 1e-3) {
      return from_size;
    }

    if (object.bounding_radius > 1e-3f) {
      return static_cast<double>(object.bounding_radius);
    }

    return 0.0;
  }

  bool isSameTarget(const lio_sam::SegmentedObjectState& object) const
  {
    if (!radius_filter_state_.has_active_target) {
      return false;
    }

    if (object.class_id != radius_filter_state_.reference_object.class_id
        || object.class_name != radius_filter_state_.reference_object.class_name) {
      return false;
    }

    const double position_delta = pointDistance(object.pose.position, radius_filter_state_.reference_object.pose.position);
    const double range_delta = std::fabs(
      static_cast<double>(object.nearest_range) - static_cast<double>(radius_filter_state_.reference_object.nearest_range));

    return position_delta <= target_match_position_tolerance_m_
      && range_delta <= target_match_range_tolerance_m_;
  }

  double applyRadiusLimit(double radius) const
  {
    const double min_radius = 0.05;
    double bounded_radius = std::max(min_radius, radius);
    if (absolute_max_radius_m_ > min_radius) {
      bounded_radius = std::min(bounded_radius, absolute_max_radius_m_);
    }
    return bounded_radius;
  }

  void clearPendingLargeExpansion()
  {
    radius_filter_state_.pending_large_radius = 0.0;
    radius_filter_state_.pending_large_expansion_frames = 0;
  }

  void resetRadiusFilterState(
    const lio_sam::SegmentedObjectState& object,
    double observed_radius)
  {
    radius_filter_state_.has_active_target = true;
    radius_filter_state_.reference_object = object;
    radius_filter_state_.filtered_radius = observed_radius;
    clearPendingLargeExpansion();
  }

  double updateFilteredRadius(const lio_sam::SegmentedObjectState& object)
  {
    const double observed_radius = applyRadiusLimit(objectRadius(object));
    if (!isSameTarget(object)) {
      resetRadiusFilterState(object, observed_radius);
      return observed_radius;
    }

    radius_filter_state_.reference_object = object;
    const double previous_radius = radius_filter_state_.filtered_radius > 1e-3
      ? radius_filter_state_.filtered_radius
      : observed_radius;

    if (observed_radius <= previous_radius) {
      clearPendingLargeExpansion();
      const double alpha = clampValue(radius_smoothing_alpha_, 0.0, 1.0);
      const double filtered_radius = alpha * previous_radius + (1.0 - alpha) * observed_radius;
      radius_filter_state_.filtered_radius = applyRadiusLimit(filtered_radius);
      return radius_filter_state_.filtered_radius;
    }

    const double growth = observed_radius - previous_radius;
    if (growth > large_expansion_confirm_delta_m_) {
      if (radius_filter_state_.pending_large_expansion_frames <= 0
          || std::fabs(observed_radius - radius_filter_state_.pending_large_radius) > large_expansion_match_tolerance_m_) {
        radius_filter_state_.pending_large_radius = observed_radius;
        radius_filter_state_.pending_large_expansion_frames = 1;
      } else {
        radius_filter_state_.pending_large_radius = std::max(radius_filter_state_.pending_large_radius, observed_radius);
        ++radius_filter_state_.pending_large_expansion_frames;
      }

      if (radius_filter_state_.pending_large_expansion_frames < std::max(1, large_expansion_confirm_frames_)) {
        return previous_radius;
      }

      clearPendingLargeExpansion();
    } else {
      clearPendingLargeExpansion();
    }

    const double max_growth = std::max(0.0, max_radius_growth_per_frame_m_);
    const double filtered_radius = max_growth > 1e-6
      ? std::min(observed_radius, previous_radius + max_growth)
      : observed_radius;
    radius_filter_state_.filtered_radius = applyRadiusLimit(filtered_radius);
    return radius_filter_state_.filtered_radius;
  }

  void objectsCallback(const lio_sam::SegmentedObjectStateArrayConstPtr& msg)
  {
    if (!msg) {
      return;
    }

    lio_sam::SegmentedObjectStateArray out;
    out.header = msg->header;

    lio_sam::SegmentedObjectState selected_object;
    if (selectObject(*msg, &selected_object)) {
      const double filtered_radius = updateFilteredRadius(selected_object);
      selected_object.footprint_radius = static_cast<float>(filtered_radius);
      if (selected_object.bounding_radius < selected_object.footprint_radius) {
        selected_object.bounding_radius = selected_object.footprint_radius;
      }
      out.objects.push_back(selected_object);
    }

    pub_selected_objects_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_objects_;
  ros::Publisher pub_selected_objects_;

  std::string input_objects_topic_;
  std::string output_topic_;
  std::vector<std::string> selected_class_names_;
  double min_object_confidence_{0.35};
  int min_object_points_{25};
  double radius_smoothing_alpha_{0.6};
  double max_radius_growth_per_frame_m_{0.15};
  double large_expansion_confirm_delta_m_{0.3};
  int large_expansion_confirm_frames_{3};
  double large_expansion_match_tolerance_m_{0.15};
  double absolute_max_radius_m_{0.0};
  double target_match_position_tolerance_m_{1.5};
  double target_match_range_tolerance_m_{1.0};
  RadiusFilterState radius_filter_state_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_object_selector");
  VisualObjectSelectorNode node;
  ros::spin();
  return 0;
}