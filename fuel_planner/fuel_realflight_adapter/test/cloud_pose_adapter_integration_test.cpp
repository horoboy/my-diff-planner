#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <cstddef>
#include <functional>
#include <string>

namespace {

geometry_msgs::PoseStamped latest_pose;
size_t pose_count = 0;

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  latest_pose = *msg;
  ++pose_count;
}

bool waitFor(const std::function<bool()>& condition, const double timeout) {
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout);
  while (ros::ok() && ros::WallTime::now() < deadline) {
    ros::spinOnce();
    if (condition()) return true;
    ros::WallDuration(0.01).sleep();
  }
  return condition();
}

nav_msgs::Odometry makeOdom(const double stamp, const double x) {
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time(stamp);
  msg.header.frame_id = "world";
  msg.child_frame_id = "imu";
  msg.pose.pose.position.x = x;
  msg.pose.pose.orientation.w = 1.0;
  return msg;
}

sensor_msgs::PointCloud2 makeCloud(const double stamp, const std::string& frame) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ros::Time(stamp);
  msg.header.frame_id = frame;
  msg.height = 1;
  msg.width = 0;
  msg.is_dense = true;
  return msg;
}

TEST(CloudPoseAdapterIntegration, WaitsInterpolatesAndRejectsWrongFrame) {
  ros::NodeHandle nh;
  ros::Publisher odom_pub =
      nh.advertise<nav_msgs::Odometry>("/cloud_pose_adapter_test/odom", 10);
  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_pose_adapter_test/cloud", 10);
  ros::Subscriber pose_sub = nh.subscribe("/cloud_pose_adapter_test/sensor_pose", 10, poseCallback);

  ASSERT_TRUE(waitFor(
      [&]() { return odom_pub.getNumSubscribers() > 0 && cloud_pub.getNumSubscribers() > 0; },
      3.0));

  odom_pub.publish(makeOdom(300.0, 0.0));
  ros::WallDuration(0.05).sleep();
  cloud_pub.publish(makeCloud(301.0, "world"));
  ros::WallDuration(0.05).sleep();
  EXPECT_EQ(pose_count, 0u);

  odom_pub.publish(makeOdom(302.0, 2.0));
  ASSERT_TRUE(waitFor([]() { return pose_count == 1; }, 2.0));

  EXPECT_EQ(latest_pose.header.stamp, ros::Time(301.0));
  EXPECT_EQ(latest_pose.header.frame_id, "world");
  EXPECT_NEAR(latest_pose.pose.position.x, 1.094614, 1e-9);
  EXPECT_NEAR(latest_pose.pose.position.y, -0.012512, 1e-9);
  EXPECT_NEAR(latest_pose.pose.position.z, 0.0437, 1e-9);
  EXPECT_NEAR(latest_pose.pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(latest_pose.pose.orientation.y, 0.1305261922, 1e-9);
  EXPECT_NEAR(latest_pose.pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(latest_pose.pose.orientation.w, 0.9914448614, 1e-9);

  odom_pub.publish(makeOdom(304.0, 4.0));
  cloud_pub.publish(makeCloud(303.0, "camera_init"));
  ros::WallDuration(0.3).sleep();
  ros::spinOnce();
  EXPECT_EQ(pose_count, 1u);

  (void)pose_sub;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_pose_adapter_integration_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
