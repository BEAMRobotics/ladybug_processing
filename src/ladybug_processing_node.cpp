#include <ros/ros.h>
#include "ladybug_processing/LadybugColorizerNode.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ladybug_processing");
  ros::NodeHandle nodeHandle("~");

  ladybug_processing::LadybugColorizerNode ladybug_colorizer_node(nodeHandle);

  ros::spin();
  return 0;
}
