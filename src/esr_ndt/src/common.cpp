/* Copyright 2019 HuaTsai */
#include "esr_ndt/common.h"

namespace esr_ndt {
geometry_msgs::Pose PoseInterpolate(
      const geometry_msgs::PoseStamped &p1,
      const geometry_msgs::PoseStamped &p2, const ros::Time &time) {
  ROS_ASSERT_MSG(time >= p1.header.stamp, "invalid interpolation");
  ROS_ASSERT_MSG(p2.header.stamp >= time, "invalid interpolation");
  geometry_msgs::Pose ret;
  double num = (time - p1.header.stamp).toSec();
  double den = (p2.header.stamp - p1.header.stamp).toSec();
  double ration = num / den;
  tf2::Vector3 v1, v2;
  tf2::Quaternion q1, q2;
  tf2::convert(p1.pose.position, v1);
  tf2::convert(p2.pose.position, v2);
  tf2::convert(p1.pose.orientation, q1);
  tf2::convert(p2.pose.orientation, q2);
  tf2::toMsg(v1.lerp(v2, ration), ret.position);
  tf2::convert(q1.slerp(q2, ration), ret.orientation);
  return ret;
}
}  // namespace esr_ndt
