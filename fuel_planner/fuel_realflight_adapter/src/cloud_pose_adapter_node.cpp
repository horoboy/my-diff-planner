#include <fuel_realflight_adapter/pose_math.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>

namespace fuel_realflight_adapter {

class CloudPoseAdapter {
 public:
  CloudPoseAdapter() : pnh_("~") {
    loadParameters();

    pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("sensor_pose", 20);
    odom_sub_ = pnh_.subscribe("odom", odom_queue_size_, &CloudPoseAdapter::odomCallback,
                               this, ros::TransportHints().tcpNoDelay());
    cloud_sub_ = pnh_.subscribe("cloud", cloud_queue_size_, &CloudPoseAdapter::cloudCallback,
                                this, ros::TransportHints().tcpNoDelay());
    timeout_timer_ = pnh_.createWallTimer(ros::WallDuration(0.02),
                                          &CloudPoseAdapter::timeoutCallback, this);

    ROS_INFO("[Cloud pose adapter]: ready, output frame '%s'.", output_frame_.c_str());
    ROS_INFO("[Cloud pose adapter]: body->sensor translation [%.6f, %.6f, %.6f].",
             body_sensor_.position.x(), body_sensor_.position.y(), body_sensor_.position.z());
    if (!require_matching_frame_ids_) {
      ROS_WARN("[Cloud pose adapter]: frame alias mode enabled; no TF is applied between labels.");
    }
  }

 private:
  struct OdomSample {
    ros::Time stamp;
    std::string frame_id;
    Pose3d pose;
  };

  struct PendingCloud {
    std_msgs::Header header;
    ros::WallTime receipt_time;
  };

  enum class LookupResult { kSuccess, kWaitForNewerOdom, kTooOld, kGapTooLarge };

  void loadParameters() {
    pnh_.param("output_frame", output_frame_, std::string("world"));
    pnh_.param("expected_odom_frame", expected_odom_frame_, std::string());
    pnh_.param("expected_cloud_frame", expected_cloud_frame_, std::string());
    pnh_.param("require_matching_frame_ids", require_matching_frame_ids_, false);

    pnh_.param("odom_buffer_duration", odom_buffer_duration_, 2.0);
    pnh_.param("max_interpolation_gap", max_interpolation_gap_, 0.05);
    pnh_.param("max_wait_time", max_wait_time_, 0.15);
    pnh_.param("max_cloud_age", max_cloud_age_, 0.5);
    pnh_.param("max_future_offset", max_future_offset_, 0.1);
    pnh_.param("odom_queue_size", odom_queue_size_, 400);
    pnh_.param("cloud_queue_size", cloud_queue_size_, 30);
    pnh_.param("pending_cloud_limit", pending_cloud_limit_, 30);

    pnh_.param("body_to_sensor/tx", body_sensor_.position.x(), 0.0);
    pnh_.param("body_to_sensor/ty", body_sensor_.position.y(), 0.0);
    pnh_.param("body_to_sensor/tz", body_sensor_.position.z(), 0.0);

    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    pnh_.param("body_to_sensor/qx", qx, qx);
    pnh_.param("body_to_sensor/qy", qy, qy);
    pnh_.param("body_to_sensor/qz", qz, qz);
    pnh_.param("body_to_sensor/qw", qw, qw);
    body_sensor_.orientation = Eigen::Quaterniond(qw, qx, qy, qz);

    if (output_frame_.empty()) {
      throw std::runtime_error("output_frame must not be empty");
    }
    if (!isFinite(body_sensor_) || body_sensor_.orientation.norm() < 1e-6) {
      throw std::runtime_error("body_to_sensor contains an invalid transform");
    }
    body_sensor_.orientation.normalize();

    if (odom_buffer_duration_ <= 0.0 || max_interpolation_gap_ <= 0.0 ||
        max_wait_time_ <= 0.0 || max_future_offset_ < 0.0 || odom_queue_size_ < 2 ||
        cloud_queue_size_ < 1 || pending_cloud_limit_ < 1) {
      throw std::runtime_error("adapter queue and timing parameters must be positive");
    }
  }

  static bool validQuaternion(const geometry_msgs::Quaternion& orientation) {
    const double norm_squared = orientation.x * orientation.x + orientation.y * orientation.y +
                                orientation.z * orientation.z + orientation.w * orientation.w;
    return std::isfinite(norm_squared) && norm_squared > 1e-12;
  }

  bool frameAccepted(const std::string& actual, const std::string& expected,
                     const char* source) const {
    if (actual.empty()) {
      ROS_ERROR_THROTTLE(1.0, "[Cloud pose adapter]: %s frame_id is empty.", source);
      return false;
    }
    if (!expected.empty() && actual != expected) {
      ROS_ERROR_THROTTLE(1.0,
                         "[Cloud pose adapter]: %s frame '%s' does not match expected '%s'.",
                         source, actual.c_str(), expected.c_str());
      return false;
    }
    return true;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    if (msg->header.stamp.isZero() ||
        !frameAccepted(msg->header.frame_id, expected_odom_frame_, "odom")) {
      ++dropped_odom_;
      return;
    }

    const auto& position = msg->pose.pose.position;
    const auto& orientation = msg->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !validQuaternion(orientation)) {
      ROS_ERROR_THROTTLE(1.0, "[Cloud pose adapter]: rejecting non-finite odometry.");
      ++dropped_odom_;
      return;
    }

    OdomSample sample;
    sample.stamp = msg->header.stamp;
    sample.frame_id = msg->header.frame_id;
    sample.pose.position = Eigen::Vector3d(position.x, position.y, position.z);
    sample.pose.orientation = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y,
                                                 orientation.z)
                                  .normalized();

    const auto compare_stamp = [](const OdomSample& item, const ros::Time& stamp) {
      return item.stamp < stamp;
    };
    auto insertion =
        std::lower_bound(odom_buffer_.begin(), odom_buffer_.end(), sample.stamp, compare_stamp);
    if (insertion != odom_buffer_.end() && insertion->stamp == sample.stamp) {
      *insertion = sample;
    } else {
      odom_buffer_.insert(insertion, sample);
    }

    pruneOdomBuffer();
    processPendingClouds();
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (msg->header.stamp.isZero() ||
        !frameAccepted(msg->header.frame_id, expected_cloud_frame_, "cloud")) {
      ++dropped_clouds_;
      return;
    }

    const ros::Time now = ros::Time::now();
    const double age = (now - msg->header.stamp).toSec();
    if ((max_cloud_age_ > 0.0 && age > max_cloud_age_) || age < -max_future_offset_) {
      ROS_ERROR_THROTTLE(1.0, "[Cloud pose adapter]: rejecting cloud with age %.3f s.", age);
      ++dropped_clouds_;
      return;
    }

    if (!last_published_stamp_.isZero() && msg->header.stamp <= last_published_stamp_) {
      ROS_WARN_THROTTLE(1.0, "[Cloud pose adapter]: dropping an old or duplicate cloud stamp.");
      ++dropped_clouds_;
      return;
    }
    if (!pending_clouds_.empty() && msg->header.stamp <= pending_clouds_.back().header.stamp) {
      ROS_WARN_THROTTLE(1.0, "[Cloud pose adapter]: dropping an out-of-order cloud stamp.");
      ++dropped_clouds_;
      return;
    }

    if (pending_clouds_.size() >= static_cast<size_t>(pending_cloud_limit_)) {
      ROS_ERROR("[Cloud pose adapter]: pending cloud queue overflow; dropping oldest cloud.");
      pending_clouds_.pop_front();
      ++dropped_clouds_;
    }

    pending_clouds_.push_back(PendingCloud{msg->header, ros::WallTime::now()});
    processPendingClouds();
  }

  void timeoutCallback(const ros::WallTimerEvent&) { processPendingClouds(); }

  void pruneOdomBuffer() {
    if (odom_buffer_.empty()) return;

    const ros::Time newest_stamp = odom_buffer_.back().stamp;
    while (odom_buffer_.size() > 2 &&
           (newest_stamp - odom_buffer_.front().stamp).toSec() > odom_buffer_duration_) {
      odom_buffer_.pop_front();
    }
    while (odom_buffer_.size() > static_cast<size_t>(odom_queue_size_)) {
      odom_buffer_.pop_front();
    }
  }

  LookupResult lookupPose(const ros::Time& stamp, OdomSample* output) const {
    if (odom_buffer_.empty()) return LookupResult::kWaitForNewerOdom;

    const auto compare_stamp = [](const OdomSample& item, const ros::Time& target) {
      return item.stamp < target;
    };
    auto upper = std::lower_bound(odom_buffer_.begin(), odom_buffer_.end(), stamp, compare_stamp);

    if (upper != odom_buffer_.end() && upper->stamp == stamp) {
      *output = *upper;
      return LookupResult::kSuccess;
    }
    if (upper == odom_buffer_.begin()) return LookupResult::kTooOld;
    if (upper == odom_buffer_.end()) return LookupResult::kWaitForNewerOdom;

    const OdomSample& second = *upper;
    const OdomSample& first = *(upper - 1);
    const double interval = (second.stamp - first.stamp).toSec();
    if (!std::isfinite(interval) || interval <= 0.0 || interval > max_interpolation_gap_) {
      return LookupResult::kGapTooLarge;
    }

    const double ratio = (stamp - first.stamp).toSec() / interval;
    output->stamp = stamp;
    output->frame_id = first.frame_id;
    output->pose = interpolate(first.pose, second.pose, ratio);
    return LookupResult::kSuccess;
  }

  void processPendingClouds() {
    while (!pending_clouds_.empty()) {
      const PendingCloud& cloud = pending_clouds_.front();
      OdomSample body_sample;
      const LookupResult result = lookupPose(cloud.header.stamp, &body_sample);

      if (result == LookupResult::kWaitForNewerOdom) {
        const double wait = (ros::WallTime::now() - cloud.receipt_time).toSec();
        if (wait <= max_wait_time_) return;
        ROS_ERROR_THROTTLE(1.0,
                           "[Cloud pose adapter]: no newer odometry within %.3f s; dropping cloud.",
                           max_wait_time_);
        pending_clouds_.pop_front();
        ++dropped_clouds_;
        continue;
      }

      if (result == LookupResult::kTooOld) {
        ROS_ERROR_THROTTLE(1.0, "[Cloud pose adapter]: cloud is older than odometry buffer.");
        pending_clouds_.pop_front();
        ++dropped_clouds_;
        continue;
      }

      if (result == LookupResult::kGapTooLarge) {
        ROS_ERROR_THROTTLE(1.0,
                           "[Cloud pose adapter]: odometry interpolation gap exceeds %.3f s.",
                           max_interpolation_gap_);
        pending_clouds_.pop_front();
        ++dropped_clouds_;
        continue;
      }

      if (require_matching_frame_ids_ && body_sample.frame_id != cloud.header.frame_id) {
        ROS_ERROR_THROTTLE(1.0,
                           "[Cloud pose adapter]: odom frame '%s' and cloud frame '%s' differ.",
                           body_sample.frame_id.c_str(), cloud.header.frame_id.c_str());
        pending_clouds_.pop_front();
        ++dropped_clouds_;
        continue;
      }
      if (!require_matching_frame_ids_ && body_sample.frame_id != cloud.header.frame_id) {
        ROS_WARN_THROTTLE(5.0,
                          "[Cloud pose adapter]: treating odom frame '%s' and cloud frame '%s' "
                          "as numeric aliases.",
                          body_sample.frame_id.c_str(), cloud.header.frame_id.c_str());
      }

      const Pose3d world_sensor = compose(body_sample.pose, body_sensor_);
      if (!isFinite(world_sensor)) {
        ROS_ERROR_THROTTLE(1.0, "[Cloud pose adapter]: computed sensor pose is non-finite.");
        pending_clouds_.pop_front();
        ++dropped_clouds_;
        continue;
      }

      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.seq = cloud.header.seq;
      pose_msg.header.stamp = cloud.header.stamp;
      pose_msg.header.frame_id = output_frame_;
      pose_msg.pose.position.x = world_sensor.position.x();
      pose_msg.pose.position.y = world_sensor.position.y();
      pose_msg.pose.position.z = world_sensor.position.z();
      pose_msg.pose.orientation.x = world_sensor.orientation.x();
      pose_msg.pose.orientation.y = world_sensor.orientation.y();
      pose_msg.pose.orientation.z = world_sensor.orientation.z();
      pose_msg.pose.orientation.w = world_sensor.orientation.w();
      pose_pub_.publish(pose_msg);

      last_published_stamp_ = cloud.header.stamp;
      pending_clouds_.pop_front();
      ++published_poses_;
      ROS_INFO_THROTTLE(5.0,
                        "[Cloud pose adapter]: published=%llu dropped_cloud=%llu "
                        "dropped_odom=%llu pending=%zu.",
                        static_cast<unsigned long long>(published_poses_),
                        static_cast<unsigned long long>(dropped_clouds_),
                        static_cast<unsigned long long>(dropped_odom_), pending_clouds_.size());
    }
  }

  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher pose_pub_;
  ros::WallTimer timeout_timer_;

  std::deque<OdomSample> odom_buffer_;
  std::deque<PendingCloud> pending_clouds_;
  Pose3d body_sensor_;
  ros::Time last_published_stamp_;

  std::string output_frame_;
  std::string expected_odom_frame_;
  std::string expected_cloud_frame_;
  bool require_matching_frame_ids_ = false;
  double odom_buffer_duration_ = 2.0;
  double max_interpolation_gap_ = 0.05;
  double max_wait_time_ = 0.15;
  double max_cloud_age_ = 0.5;
  double max_future_offset_ = 0.1;
  int odom_queue_size_ = 400;
  int cloud_queue_size_ = 30;
  int pending_cloud_limit_ = 30;

  uint64_t published_poses_ = 0;
  uint64_t dropped_clouds_ = 0;
  uint64_t dropped_odom_ = 0;
};

}  // namespace fuel_realflight_adapter

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_pose_adapter");
  try {
    fuel_realflight_adapter::CloudPoseAdapter adapter;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("[Cloud pose adapter]: initialization failed: %s", error.what());
    return 1;
  }
  return 0;
}
