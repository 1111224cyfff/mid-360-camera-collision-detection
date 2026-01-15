#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
struct KalmanCV2D
{
  // State: [px, py, vx, vy]
  Eigen::Vector4d x{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d P{Eigen::Matrix4d::Identity()};

  double process_noise_acc{2.0};
  double meas_noise_pos{0.15};

  void init(double px, double py)
  {
    x.setZero();
    x(0) = px;
    x(1) = py;

    P.setIdentity();
    P(0, 0) = 0.2 * 0.2;
    P(1, 1) = 0.2 * 0.2;
    P(2, 2) = 1.0 * 1.0;
    P(3, 3) = 1.0 * 1.0;
  }

  void predict(double dt)
  {
    if (dt <= 0.0)
      return;

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    const double q = process_noise_acc * process_noise_acc;
    Q(0, 0) = q * dt4 / 4.0;
    Q(1, 1) = q * dt4 / 4.0;
    Q(0, 2) = q * dt3 / 2.0;
    Q(1, 3) = q * dt3 / 2.0;
    Q(2, 0) = q * dt3 / 2.0;
    Q(3, 1) = q * dt3 / 2.0;
    Q(2, 2) = q * dt2;
    Q(3, 3) = q * dt2;

    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  void update(double meas_px, double meas_py)
  {
    Eigen::Matrix<double, 2, 4> H;
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Vector2d z;
    z << meas_px, meas_py;

    Eigen::Vector2d y = z - H * x;

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (meas_noise_pos * meas_noise_pos);
    Eigen::Matrix2d S = H * P * H.transpose() + R;

    Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();

    x = x + K * y;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P = (I - K * H) * P;
  }
};

enum class TrackState
{
  Tentative,
  Confirmed
};

struct Track
{
  int id{-1};
  TrackState state{TrackState::Tentative};

  ros::Time created;
  ros::Time last_update;
  ros::Time last_predict;

  int hits_total{0};
  int hits_in_window{0};
  int misses{0};

  double z{0.0};

  KalmanCV2D kf;

  Eigen::Vector2d position() const { return kf.x.head<2>(); }
  Eigen::Vector2d velocity() const { return kf.x.tail<2>(); }
};

struct Detection
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

static double dist2(const Eigen::Vector2d& a, const Detection& b)
{
  const double dx = a.x() - b.x;
  const double dy = a.y() - b.y;
  return dx * dx + dy * dy;
}

static std::string stateToString(TrackState s)
{
  return (s == TrackState::Confirmed) ? std::string("CONF") : std::string("TENT");
}

}  // namespace

class DynamicTrackerNode
{
public:
  DynamicTrackerNode()
  : pnh_("~")
  {
    pnh_.param<std::string>("input_topic", input_topic_, std::string("/lio_sam/mapping/dynamic_cluster_centers"));
    pnh_.param<std::string>("output_frame", output_frame_, std::string(""));

    // Preference=2 (avoid misses): defaults are permissive.
    pnh_.param<double>("gate_distance_base", gate_base_, 1.0);              // meters
    pnh_.param<double>("gate_distance_per_second", gate_per_sec_, 2.0);     // meters/sec

    pnh_.param<int>("confirm_hits", confirm_hits_, 1);                      // 1 = confirm immediately
    pnh_.param<int>("confirm_window", confirm_window_, 2);                  // not heavily used in this minimal version
    pnh_.param<int>("max_missed_frames", max_missed_frames_, 12);
    pnh_.param<double>("max_missed_time", max_missed_time_, 1.2);           // seconds

    pnh_.param<double>("process_noise_acc", process_noise_acc_, 3.0);
    pnh_.param<double>("meas_noise_pos", meas_noise_pos_, 0.12);

    pnh_.param<double>("z_lowpass_alpha", z_alpha_, 0.8);

    pnh_.param<double>("prediction_horizon", pred_horizon_, 2.0);
    pnh_.param<double>("prediction_dt", pred_dt_, 0.2);

    pnh_.param<bool>("publish_markers", publish_markers_, true);
    pnh_.param<bool>("publish_tracked_centers", publish_centers_, true);

    sub_centers_ = nh_.subscribe(input_topic_, 5, &DynamicTrackerNode::centersCallback, this);

    pub_tracks_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("dynamic_tracker/tracks", 1);
    pub_pred_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("dynamic_tracker/predictions", 1);
    pub_tracked_centers_ = nh_.advertise<geometry_msgs::PoseArray>("dynamic_tracker/tracked_centers", 1);

    ROS_INFO_STREAM("[dynamic_tracker] input_topic=" << input_topic_);
  }

private:
  void centersCallback(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    const std::string frame = (!output_frame_.empty()) ? output_frame_ : msg->header.frame_id;

    std::vector<Detection> dets;
    dets.reserve(msg->poses.size());
    for (const auto& p : msg->poses)
    {
      Detection d;
      d.x = p.position.x;
      d.y = p.position.y;
      d.z = p.position.z;
      dets.push_back(d);
    }

    predictAll(stamp);

    // Build candidate pairs (trackIdx, detIdx, dist)
    struct Pair
    {
      int ti;
      int di;
      double d2;
    };

    std::vector<Pair> pairs;
    pairs.reserve(tracks_.size() * std::max<size_t>(1, dets.size()));

    for (int ti = 0; ti < static_cast<int>(tracks_.size()); ++ti)
    {
      // NOTE: last_predict is set to `stamp` in predictAll(), so using it here would
      // always yield dt=0 and effectively disable the time-scaled gating.
      // We want the gate to expand with the time since last successful update.
      const double dt = std::max(0.0, (stamp - tracks_[ti].last_update).toSec());
      const double gate = gate_base_ + gate_per_sec_ * dt;
      const double gate2 = gate * gate;

      const Eigen::Vector2d pred_pos = tracks_[ti].position();
      for (int di = 0; di < static_cast<int>(dets.size()); ++di)
      {
        const double d2 = dist2(pred_pos, dets[di]);
        if (d2 <= gate2)
          pairs.push_back({ti, di, d2});
      }
    }

    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d2 < b.d2; });

    std::vector<int> det_to_track(dets.size(), -1);
    std::vector<int> track_to_det(tracks_.size(), -1);

    for (const auto& pr : pairs)
    {
      if (track_to_det[pr.ti] != -1)
        continue;
      if (det_to_track[pr.di] != -1)
        continue;
      track_to_det[pr.ti] = pr.di;
      det_to_track[pr.di] = pr.ti;
    }

    // Update matched tracks
    for (int ti = 0; ti < static_cast<int>(tracks_.size()); ++ti)
    {
      const int di = track_to_det[ti];
      if (di < 0)
        continue;

      tracks_[ti].kf.update(dets[di].x, dets[di].y);
      tracks_[ti].z = z_alpha_ * tracks_[ti].z + (1.0 - z_alpha_) * dets[di].z;

      tracks_[ti].last_update = stamp;
      tracks_[ti].hits_total++;
      tracks_[ti].hits_in_window++;
      tracks_[ti].misses = 0;

      if (tracks_[ti].state == TrackState::Tentative && tracks_[ti].hits_in_window >= confirm_hits_)
        tracks_[ti].state = TrackState::Confirmed;
    }

    // Missed tracks
    for (auto& tr : tracks_)
    {
      if (tr.last_update != stamp)
        tr.misses++;
    }

    // New tracks for unmatched detections
    for (int di = 0; di < static_cast<int>(dets.size()); ++di)
    {
      if (det_to_track[di] != -1)
        continue;

      Track tr;
      tr.id = next_id_++;
      tr.state = TrackState::Tentative;
      tr.created = stamp;
      tr.last_update = stamp;
      tr.last_predict = stamp;
      tr.hits_total = 1;
      tr.hits_in_window = 1;
      tr.misses = 0;
      tr.z = dets[di].z;
      tr.kf.process_noise_acc = process_noise_acc_;
      tr.kf.meas_noise_pos = meas_noise_pos_;
      tr.kf.init(dets[di].x, dets[di].y);

      if (confirm_hits_ <= 1)
        tr.state = TrackState::Confirmed;

      tracks_.push_back(tr);
    }

    pruneTracks(stamp);

    if (publish_markers_)
      publishMarkers(frame, stamp);
    if (publish_centers_)
      publishCenters(frame, stamp);
  }

  void predictAll(const ros::Time& stamp)
  {
    for (auto& tr : tracks_)
    {
      const double dt = std::max(0.0, (stamp - tr.last_predict).toSec());
      tr.kf.predict(dt);
      tr.last_predict = stamp;

      // decay hit window lightly (very simple)
      if (dt > 0.5)
        tr.hits_in_window = 0;
    }
  }

  void pruneTracks(const ros::Time& stamp)
  {
    std::vector<Track> kept;
    kept.reserve(tracks_.size());

    for (auto& tr : tracks_)
    {
      const double miss_time = (stamp - tr.last_update).toSec();
      const bool too_many_misses = (max_missed_frames_ > 0) ? (tr.misses > max_missed_frames_) : false;
      const bool too_long = (max_missed_time_ > 0.0) ? (miss_time > max_missed_time_) : false;

      // For tentative tracks, be slightly more aggressive if it never gets confirmed.
      if (tr.state == TrackState::Tentative)
      {
        if (tr.misses > 2)
          continue;
      }

      if (too_many_misses || too_long)
        continue;

      kept.push_back(tr);
    }

    tracks_.swap(kept);
  }

  void publishCenters(const std::string& frame, const ros::Time& stamp)
  {
    geometry_msgs::PoseArray out;
    out.header.stamp = stamp;
    out.header.frame_id = frame;

    for (const auto& tr : tracks_)
    {
      if (tr.state != TrackState::Confirmed)
        continue;

      geometry_msgs::Pose p;
      p.position.x = tr.position().x();
      p.position.y = tr.position().y();
      p.position.z = tr.z;
      p.orientation.w = 1.0;
      out.poses.push_back(p);
    }

    pub_tracked_centers_.publish(out);
  }

  void publishMarkers(const std::string& frame, const ros::Time& stamp)
  {
    visualization_msgs::MarkerArray tracks;
    visualization_msgs::MarkerArray preds;

    std::unordered_set<int> alive_ids;

    for (const auto& tr : tracks_)
    {
      if (tr.state != TrackState::Confirmed)
        continue;

      alive_ids.insert(tr.id);

      // Track marker (sphere)
      visualization_msgs::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame;
      m.ns = "dyn_track";
      m.id = tr.id;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x = tr.position().x();
      m.pose.position.y = tr.position().y();
      m.pose.position.z = tr.z;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.3;
      m.scale.y = 0.3;
      m.scale.z = 0.3;
      m.color.r = 0.2f;
      m.color.g = 0.8f;
      m.color.b = 1.0f;
      m.color.a = 0.9f;
      m.lifetime = ros::Duration(0.5);
      tracks.markers.push_back(m);

      // Text marker
      const Eigen::Vector2d v = tr.velocity();
      const double speed = std::sqrt(v.x() * v.x() + v.y() * v.y());

      visualization_msgs::Marker t;
      t.header = m.header;
      t.ns = "dyn_track_text";
      t.id = tr.id;
      t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::Marker::ADD;
      t.pose.position.x = m.pose.position.x;
      t.pose.position.y = m.pose.position.y;
      t.pose.position.z = m.pose.position.z + 0.4;
      t.pose.orientation.w = 1.0;
      t.scale.z = 0.25;
      t.color.r = 1.0f;
      t.color.g = 1.0f;
      t.color.b = 1.0f;
      t.color.a = 0.9f;
      t.text = "ID=" + std::to_string(tr.id) + " " + stateToString(tr.state) + " v=" + std::to_string(speed);
      t.lifetime = ros::Duration(0.5);
      tracks.markers.push_back(t);

      // Prediction line strip
      visualization_msgs::Marker line;
      line.header = m.header;
      line.ns = "dyn_pred";
      line.id = tr.id;
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.action = visualization_msgs::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = 0.05;
      line.color.r = 1.0f;
      line.color.g = 0.7f;
      line.color.b = 0.2f;
      line.color.a = 0.9f;
      line.lifetime = ros::Duration(0.5);

      const Eigen::Vector2d p0 = tr.position();
      const Eigen::Vector2d v0 = tr.velocity();
      for (double tau = 0.0; tau <= pred_horizon_ + 1e-6; tau += std::max(0.05, pred_dt_))
      {
        geometry_msgs::Point pt;
        pt.x = p0.x() + v0.x() * tau;
        pt.y = p0.y() + v0.y() * tau;
        pt.z = tr.z;
        line.points.push_back(pt);
      }
      preds.markers.push_back(line);
    }

    // Delete markers for tracks that disappeared
    for (int last_id : last_published_ids_)
    {
      if (alive_ids.count(last_id) != 0)
        continue;

      visualization_msgs::Marker del;
      del.header.stamp = stamp;
      del.header.frame_id = frame;

      del.action = visualization_msgs::Marker::DELETE;

      del.ns = "dyn_track";
      del.id = last_id;
      tracks.markers.push_back(del);

      del.ns = "dyn_track_text";
      del.id = last_id;
      tracks.markers.push_back(del);

      del.ns = "dyn_pred";
      del.id = last_id;
      preds.markers.push_back(del);
    }

    last_published_ids_.clear();
    last_published_ids_.reserve(alive_ids.size());
    for (int id : alive_ids)
      last_published_ids_.push_back(id);

    pub_tracks_markers_.publish(tracks);
    pub_pred_markers_.publish(preds);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_centers_;

  ros::Publisher pub_tracks_markers_;
  ros::Publisher pub_pred_markers_;
  ros::Publisher pub_tracked_centers_;

  std::string input_topic_;
  std::string output_frame_;

  double gate_base_{1.0};
  double gate_per_sec_{2.0};

  int confirm_hits_{1};
  int confirm_window_{2};
  int max_missed_frames_{12};
  double max_missed_time_{1.2};

  double process_noise_acc_{3.0};
  double meas_noise_pos_{0.12};

  double z_alpha_{0.8};

  double pred_horizon_{2.0};
  double pred_dt_{0.2};

  bool publish_markers_{true};
  bool publish_centers_{true};

  int next_id_{1};
  std::vector<Track> tracks_;
  std::vector<int> last_published_ids_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_tracker");
  DynamicTrackerNode node;
  ros::spin();
  return 0;
}
