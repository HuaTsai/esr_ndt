/* Copyright 2019 HuaTsai */
#ifndef ESR_NDT_NODE_H_
#define ESR_NDT_NODE_H_
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>

#include <deque>
#include <memory>
#include <string>
#include "esr_ndt/pose_extrapolator.h"

namespace esr_ndt {
class Node {
 public:
  explicit Node(const std::shared_ptr<ros::NodeHandle> &nh,
                const std::shared_ptr<ros::NodeHandle> &private_nh);
  Node(const Node &) = delete;
  Node &operator=(const Node &) = delete;

 private:
  void OdometryCallback(const nav_msgs::Odometry &msg);
  void ImuCallback(const sensor_msgs::Imu &msg);
  void PointCloudCallback(const sensor_msgs::PointCloud2 &msg);
  void InitialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped &msg);
  void HandleTopics();
  void LoadMap();
  void SetupTransformBroadcaster();
  void InitializePose();
  void SendTransformBeforeRun();
  sensor_msgs::PointCloud2 AugmentPointCloud(
      const std::deque<sensor_msgs::PointCloud2> &pcs,
      const geometry_msgs::PoseStamped &predict_poset_msg);
  geometry_msgs::Pose PoseInterpolate(
      const geometry_msgs::PoseStamped &p1,
      const geometry_msgs::PoseStamped &p2, const ros::Time &time);
  geometry_msgs::TransformStamped ToTransformStampedMsg(
      const tf2::Transform &tf, const ros::Time &time,
      const std::string &frame, const std::string &child_frame = "");
  Eigen::Matrix4f PoseMsgToMatrix4f(const geometry_msgs::Pose &msg);
  geometry_msgs::Pose Matrix4fToPoseMsg(const Eigen::Matrix4f &mtx);

  // HodeHandle, Publisher and Subscriber
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::NodeHandle> private_nh_;
  ros::Publisher ndt_pose_publisher_;
  ros::Publisher ndt_path_publisher_;
  ros::Publisher map_publisher_;
  ros::Publisher esr_pc_publisher_;
  ros::Subscriber pc_subscriber_;
  ros::Subscriber odometry_subscriber_;

  // tf tools
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  geometry_msgs::TransformStamped localizer_to_base_link_;
  geometry_msgs::TransformStamped imu_to_base_link_;

  // messages
  geometry_msgs::PoseStamped initialpose_;
  geometry_msgs::PoseStamped ndt_poset_msg_;
  nav_msgs::Path ndt_path_;
  sensor_msgs::PointCloud2 map_;

  // ndt
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

  // extrapolator
  std::unique_ptr<PoseExtrapolator> pose_extrapolator_;

  // queue ?
  std::deque<sensor_msgs::PointCloud2> esr_queue_;
  std::deque<geometry_msgs::PoseStamped> pose_queue_;

  // run flag
  bool is_run_;
};
}  // namespace esr_ndt

#endif  // ESR_NDT_NODE_H_
