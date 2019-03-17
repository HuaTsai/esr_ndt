/* Copyright 2019 HuaTsai */
#include <pcl_ros/io/pcd_io.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <boost/bind.hpp>
#include <thread>
#include "esr_ndt/node.h"

namespace esr_ndt {

Node::Node(const std::shared_ptr<ros::NodeHandle> &nh,
           const std::shared_ptr<ros::NodeHandle> &private_nh)
    : nh_(nh), private_nh_(private_nh) {
  HandleTopics();
  LoadMap();
  SetupTransformBroadcaster();
  InitializePose();
  static std::thread th(&Node::SendTransformBeforeRun, this);
  th.detach();
  ROS_INFO("Initialize Done");
}

void Node::OdometryCallback(const nav_msgs::Odometry &msg) {
  is_run_ = true;
  auto time = msg.header.stamp;
  auto tmppp = msg;
  tmppp.pose.pose.position.x -= 1.131;
  tmppp.pose.pose.position.y += 0.101;
  tmppp.pose.pose.position.z -= 2.550;
  pose_extrapolator_->AddOdometryMsg(tmppp);
  // pose_extrapolator_->AddOdometryMsg(msg);

  // predict pc
  auto predict_poset_msg = pose_extrapolator_->ExtrapolatePoseStamped(time);
  auto pcs = esr_queue_;
  auto predict_pc = AugmentPointCloud(pcs, predict_poset_msg);

  // ndt TODO(HuaTsai): align shall not be here, create thread in future
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(predict_pc, *src);
  ndt_.setInputSource(src);
  auto predict_pose_mtx = PoseMsgToMatrix4f(predict_poset_msg.pose);
  ndt_.align(*out, predict_pose_mtx);
  geometry_msgs::PoseStamped ndt_poset_msg;
  ndt_poset_msg.header.frame_id = "map";
  ndt_poset_msg.header.stamp = time;
  ndt_poset_msg.pose = Matrix4fToPoseMsg(ndt_.getFinalTransformation());
  // ndt_poset_msg.pose = predict_poset_msg.pose;

  tf2::Transform base_link_to_map;
  tf2::Transform imu_to_base_link;
  tf2::Transform imu_to_odom;
  tf2::Transform odom_to_map;
  tf2::convert(ndt_poset_msg.pose, base_link_to_map);
  tf2::convert(imu_to_base_link_.transform, imu_to_base_link);
  tf2::convert(msg.pose.pose, imu_to_odom);
  odom_to_map = base_link_to_map * imu_to_base_link * imu_to_odom.inverse();

  geometry_msgs::TransformStamped odom_to_map_msg;
  tf2::convert(odom_to_map, odom_to_map_msg.transform);
  odom_to_map_msg.header.stamp = msg.header.stamp;
  odom_to_map_msg.header.frame_id = "map";
  odom_to_map_msg.child_frame_id = "odom";
  tfb_->sendTransform(odom_to_map_msg);

  // geometry_msgs::TransformStamped base_link_to_map_msg;
  // tf2::convert(base_link_to_map, base_link_to_map_msg.transform);
  // base_link_to_map_msg.header.stamp = msg.header.stamp;
  // base_link_to_map_msg.header.frame_id = "map";
  // base_link_to_map_msg.child_frame_id = "base_link";
  // ROS_INFO_STREAM(base_link_to_map_msg);
  // tfb_->sendTransform(base_link_to_map_msg);

  // auto update_pc = AugmentPointCloud(pcs, time);
  auto update_pc = AugmentPointCloud(pcs, ndt_poset_msg);
  esr_pc_publisher_.publish(update_pc);

  pose_extrapolator_->AddPoseStamped(ndt_poset_msg);
  ndt_pose_publisher_.publish(ndt_poset_msg);
  pose_queue_.push_back(ndt_poset_msg);
  if (pose_queue_.size() > 20) {
    pose_queue_.pop_front();
  }

  ndt_path_.poses.push_back(ndt_poset_msg);
  ndt_path_.header.stamp = time;
  ndt_path_.header.frame_id = "map";
  ndt_path_publisher_.publish(ndt_path_);

  ndt_poset_msg_ = ndt_poset_msg;
}

void Node::ImuCallback(const sensor_msgs::Imu &msg) {
  /* currently not implement
   * TODO(HuaTsai): add in future */
}

void Node::PointCloudCallback(const sensor_msgs::PointCloud2 &msg) {
  sensor_msgs::PointCloud2 msg2;
  localizer_to_base_link_.header.stamp = msg.header.stamp;
  tf2::doTransform(msg, msg2, localizer_to_base_link_);
  esr_queue_.push_back(msg2);
  while (esr_queue_.size() > 40) {
    esr_queue_.pop_front();
  }
}

void Node::HandleTopics() {
  ndt_pose_publisher_ =
      nh_->advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
  ndt_path_publisher_ = nh_->advertise<nav_msgs::Path>("ndt_path", 10);
  map_publisher_ = nh_->advertise<sensor_msgs::PointCloud2>("map", 0, true);
  esr_pc_publisher_ = nh_->advertise<sensor_msgs::PointCloud2>("esr_pc", 10);
  odometry_subscriber_ = nh_->subscribe(
      "vins_estimator/odometry", 10, &Node::OdometryCallback, this);
  pc_subscriber_ = nh_->subscribe(
      "esr_static_obj_pc", 10, &Node::PointCloudCallback, this);
}

void Node::LoadMap() {
  std::string map_path;
  if (!private_nh_->getParam("map_path", map_path)) {
    ROS_ERROR("usage: rosrun esr_ndt node_main _map_path:=[map_path]");
    exit(1);
  }
  pcl::io::loadPCDFile(map_path, map_);
  map_.header.frame_id = "map";
  map_publisher_.publish(map_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(map_, *map);
  ndt_.setInputTarget(map);
}

void Node::SetupTransformBroadcaster() {
  tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  static tf2_ros::TransformListener listener_(tf_buffer_);
  localizer_to_base_link_ =
      tf_buffer_.lookupTransform("base_link", "velodyne",
          ros::Time(0), ros::Duration(2, 0));
  imu_to_base_link_ =
      tf_buffer_.lookupTransform("base_link", "imu",
          ros::Time(0), ros::Duration(2, 0));
}

void Node::InitializePose() {
  // TODO(HuaTsai): changable
  tf2::Stamped<tf2::Transform> base_link_to_map(
      tf2::Transform::getIdentity(), ros::Time::now(), "map");
  tf2::toMsg(base_link_to_map, initialpose_);
  pose_extrapolator_ =
      PoseExtrapolator::InitializeWithPoseStamped(initialpose_);
  ndt_poset_msg_ = initialpose_;
  pose_queue_.push_back(ndt_poset_msg_);
}

void Node::SendTransformBeforeRun() {
  is_run_ = false;
  while (!is_run_) {
    auto time = ros::Time::now();
    tf2::Stamped<tf2::Transform> tmp;
    geometry_msgs::TransformStamped odom_to_map_msg;
    geometry_msgs::TransformStamped base_link_to_odom_msg;
    tf2::convert(initialpose_, tmp);
    tf2::convert(tmp, odom_to_map_msg);
    tf2::convert(tmp, base_link_to_odom_msg);
    odom_to_map_msg.header.stamp = time;
    odom_to_map_msg.header.frame_id = "map";
    odom_to_map_msg.child_frame_id = "odom";
    base_link_to_odom_msg.header.stamp = time;
    base_link_to_odom_msg.header.frame_id = "odom";
    base_link_to_odom_msg.child_frame_id = "base_link";
    tfb_->sendTransform(odom_to_map_msg);
    tfb_->sendTransform(base_link_to_odom_msg);
    ros::Rate(10).sleep();
  }
}

sensor_msgs::PointCloud2 Node::AugmentPointCloud(
      const std::deque<sensor_msgs::PointCloud2> &pcs,
      const geometry_msgs::PoseStamped &predict_poset_msg) {
  sensor_msgs::PointCloud2 ret;
  auto predict_time = predict_poset_msg.header.stamp;

  int i = 0, j = 0, k = 0;
  for (const auto &pc : pcs) {
    sensor_msgs::PointCloud2 tmp;
    ros::Time time = pc.header.stamp;
    ROS_DEBUG("pc time: %f", time.toSec());
    geometry_msgs::Pose inter_pose_msg;
    if (time <= pose_queue_.back().header.stamp) {
      ++i;
      const auto end = std::lower_bound(
        pose_queue_.begin(), pose_queue_.end(), time,
        [](const geometry_msgs::PoseStamped &a, const ros::Time &b) {
          return a.header.stamp < b;
        });
      if (end->header.stamp == time) {
        inter_pose_msg = end->pose;
      } else {
        const auto start = std::prev(end);
        inter_pose_msg = PoseInterpolate(*start, *end, time);
      }
    } else if (time <= predict_time) {
      ++j;
      inter_pose_msg = PoseInterpolate(pose_queue_.back(),
          predict_poset_msg, time);
    } else {
      ++k;
      continue;
    }
    tf2::Transform inter_to_predict;
    tf2::Transform predict_to_map;
    tf2::Transform inter_to_map;
    tf2::convert(predict_poset_msg.pose, predict_to_map);
    tf2::convert(inter_pose_msg, inter_to_map);
    inter_to_predict = predict_to_map.inverse() * inter_to_map;

    geometry_msgs::TransformStamped inter_to_predict_msg;
    tf2::convert(inter_to_predict, inter_to_predict_msg.transform);
    inter_to_predict_msg.header.stamp = predict_time;
    inter_to_predict_msg.header.frame_id = "base_link";

    tf2::doTransform(pc, tmp, inter_to_predict_msg);
    pcl::concatenatePointCloud(ret, tmp, ret);
  }
  ROS_INFO("known: %d, pre: %d, extrapolate: %d", i, j, k);
  ret.header.frame_id = "base_link";
  ret.header.stamp = predict_time;
  return ret;
}

geometry_msgs::Pose Node::PoseInterpolate(
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

geometry_msgs::TransformStamped Node::ToTransformStampedMsg(
      const tf2::Transform &tf, const ros::Time &time,
      const std::string &frame, const std::string &child_frame) {
  geometry_msgs::TransformStamped ret;
  tf2::convert(tf, ret.transform);
  ret.header.stamp = time;
  ret.header.frame_id = frame;
  ret.child_frame_id = child_frame;
  return ret;
}

Eigen::Matrix4f Node::PoseMsgToMatrix4f(const geometry_msgs::Pose &msg) {
  Eigen::Isometry3d mtx;
  tf2::convert(msg, mtx);
  return mtx.matrix().cast<float>();
}

geometry_msgs::Pose Node::Matrix4fToPoseMsg(const Eigen::Matrix4f &mtx) {
  Eigen::Isometry3d mtx2(mtx.cast<double>());
  return tf2::toMsg(mtx2);
}

}  // namespace esr_ndt
