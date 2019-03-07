/* Copyright 2019 HuaTsai */
#include <memory>
#include "esr_ndt/node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "esr_ndt");
  auto nh = std::make_shared<ros::NodeHandle>();
  auto private_nh = std::make_shared<ros::NodeHandle>("~");
  esr_ndt::Node node(nh, private_nh);
  ros::spin();
}
