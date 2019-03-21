#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <fstream>
#include <sstream>

#include <thread>

std::vector<geometry_msgs::PoseStamped> poses;
std::vector<sensor_msgs::PointCloud2> clouds;
geometry_msgs::TransformStamped velodyne_to_base_link;

geometry_msgs::Pose PoseInterpolate(const geometry_msgs::PoseStamped &p1,
                                    const geometry_msgs::PoseStamped &p2,
                                    const ros::Time &time) {
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

void cb(const sensor_msgs::PointCloud2 &msg) {
  sensor_msgs::PointCloud2 pc;
  tf2::doTransform(msg, pc, velodyne_to_base_link);
  pc.header.stamp = msg.header.stamp;
  clouds.push_back(pc);
}

void save(const std::shared_ptr<ros::NodeHandle> &nh) {
  sensor_msgs::PointCloud2 augpc;
  bool save;
  while (true) {
    ros::Rate(0.5).sleep();
    if (nh->hasParam("saveaugpc") && nh->getParam("saveaugpc", save) && save) {
      for (auto elem : clouds) {
        auto time = elem.header.stamp;
        const auto end = std::lower_bound(
            poses.begin(), poses.end(), time,
            [](const geometry_msgs::PoseStamped &a, const ros::Time &b) {
              return a.header.stamp < b;
            });
        const auto start = std::prev(end);
        auto cur_msg = PoseInterpolate(*start, *end, time);
        tf2::Transform cur;
        tf2::fromMsg(cur_msg, cur);
        geometry_msgs::TransformStamped trs;
        trs.transform = tf2::toMsg(cur);
        tf2::doTransform(elem, elem, trs);
        pcl::concatenatePointCloud(augpc, elem, augpc);
      }
      pcl::io::savePCDFile("/home/ee904/Desktop/HuaTsai/esr_ndt/data/map.pcd",
                           augpc);
      ROS_INFO("Save world.pcd success!");
    }
    nh->setParam("saveaugpc", false);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_build");
  auto nh = std::make_shared<ros::NodeHandle>();

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  velodyne_to_base_link = buffer.lookupTransform(
      "base_link", "velodyne", ros::Time(0), ros::Duration(2, 0));
  ROS_INFO("get static tf message successfully");

  std::fstream fin;
  fin.open(
      "/home/ee904/Desktop/HuaTsai/esr/varify/20180721-1/base_link_pose.csv",
      std::ios::in);
  std::string str;
  while (!getline(fin, str).eof()) {
    std::istringstream ss(str);
    std::string str2;
    std::vector<double> data;
    while (getline(ss, str2, ',')) {
      data.push_back(std::stod(str2));
    }
    geometry_msgs::PoseStamped p;
    p.header.stamp.sec = data.at(0);
    p.header.stamp.nsec = data.at(1);
    p.pose.position.x = data.at(2);
    p.pose.position.y = data.at(3);
    p.pose.position.z = data.at(4);
    p.pose.orientation.x = data.at(5);
    p.pose.orientation.y = data.at(6);
    p.pose.orientation.z = data.at(7);
    p.pose.orientation.w = data.at(8);
    poses.push_back(p);
  }
  ros::Subscriber sub = nh->subscribe("esr_static_obj_pc", 10, cb);
  std::thread thread(save, nh);
  thread.detach();
  ros::spin();
}
