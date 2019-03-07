/* Copyright 2019 HuaTsai */
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory>
#include "esr_ndt/pose_extrapolator.h"
#include <ros/ros.h>

namespace esr_ndt {
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithPoseStamped(
    const geometry_msgs::PoseStamped &initialposet) {
  auto extrapolator = std::make_unique<PoseExtrapolator>();
  extrapolator->AddPoseStamped(initialposet);
  return extrapolator;
}

PoseExtrapolator::PoseExtrapolator() {}

void PoseExtrapolator::AddImuMsg(const sensor_msgs::Imu &msg) {}

void PoseExtrapolator::AddOdometryMsg(const nav_msgs::Odometry &msg) {
  odometry_msg_queue_.push_back(msg);
  while (odometry_msg_queue_.size() > 2) {
    odometry_msg_queue_.pop_front();
  }
  // should update linear velocity
}

void PoseExtrapolator::AddPoseStamped(const geometry_msgs::PoseStamped &msg) {
  poset_msg_queue_.push_back(msg);
  while (poset_msg_queue_.size() > 2) {
    poset_msg_queue_.pop_front();
  }
}

geometry_msgs::PoseStamped PoseExtrapolator::ExtrapolatePoseStamped(
    const ros::Time &time) {
  auto new_poset_msg = poset_msg_queue_.back();
  ROS_ASSERT_MSG(time > new_poset_msg.header.stamp,
      "extrapolate time should be later than newest pose time");
  if (odometry_msg_queue_.empty()) {
    geometry_msgs::PoseStamped predict_poset_msg = poset_msg_queue_.back();
    predict_poset_msg.header.stamp = time;
    return predict_poset_msg;
  }
  geometry_msgs::PoseStamped predict_poset_msg;
  auto old_odom_msg = odometry_msg_queue_.front();
  auto new_odom_msg = odometry_msg_queue_.back();
  tf2::Transform new_pose;
  tf2::Transform old_odom;
  tf2::Transform new_odom;
  tf2::convert(new_poset_msg.pose, new_pose);
  tf2::convert(old_odom_msg.pose.pose, old_odom);
  tf2::convert(new_odom_msg.pose.pose, new_odom);
  auto predict_pose = new_pose * old_odom.inverse() * new_odom;
  tf2::toMsg(predict_pose, predict_poset_msg.pose);
  predict_poset_msg.header.stamp = time;
  predict_poset_msg.header.frame_id = "map";
  return predict_poset_msg;
}
}  // namespace esr_ndt
