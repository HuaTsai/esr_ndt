/* Copyright 2019 HuaTsai */
#ifndef ESR_NDT_POSE_EXTRAPOLATOR_H_
#define ESR_NDT_POSE_EXTRAPOLATOR_H_
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <deque>

namespace esr_ndt {
class PoseExtrapolator {
 public:
  static std::unique_ptr<PoseExtrapolator> InitializeWithPoseStamped(
      const geometry_msgs::PoseStamped &initialposet);
  PoseExtrapolator();
  void AddImuMsg(const sensor_msgs::Imu &msg);
  void AddOdometryMsg(const nav_msgs::Odometry &msg);
  void AddPoseStamped(const geometry_msgs::PoseStamped &msg);
  geometry_msgs::PoseStamped ExtrapolatePoseStamped(const ros::Time &time);
 private:
  std::deque<geometry_msgs::PoseStamped> poset_msg_queue_;
  std::deque<nav_msgs::Odometry> odometry_msg_queue_;
};
}  // namespace esr_ndt

#endif  // ESR_NDT_POSE_EXTRAPOLATOR_H_
