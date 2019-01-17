#pragma once

#include <string>

#include "ladybug_processing/Colorizer.hpp"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"

namespace ladybug_processing {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class LadybugColorizerNode {
 public:
  explicit LadybugColorizerNode(ros::NodeHandle &node_handle);

  virtual ~LadybugColorizerNode() = default;

 private:
  /**
   * @brief Method for obtaining node parameters from the parameter server
   *
   */
  bool ReadParameters();

  /**
   * @brief Callback for receiving image tiles
   *
   */
  void ImageTopicCallback(const ladybug_msgs::LadybugTilesPtr &tiles_);

  /**
   * @brief Callback for receiving point clouds
   *
   */
  void PointcloudTopicCallback(const sensor_msgs::PointCloud2 &pc_);

  bool publish_colored_pointcloud_;
  bool enable_debug_logging_ = false;
  bool save_data_ = false;
  std::string root_folder_name_ = "/home/";
  std::string ladybug_conf_location_ = "/home/";

  ros::NodeHandle &node_handle_;
  ros::NodeHandle node_handle_priv_;

  //Topic names
  std::string image_topic_name_;
  std::string pointcloud_topic_name_;
  std::string colored_pointcloud_topic_name_;
  std::string colored_pointcloud_frame_;

  //Subscribers
  ros::Subscriber image_subscriber_;
  ros::Subscriber pointcloud_subscriber_;

  // Publishers
  ros::Publisher colored_pointcloud_publisher_;

  // Object for colorizing
  Colorizer colorizer_;
  std::string ladybug_config_location_;

  // Image & point cloud data holders
  sensor_msgs::Image current_image_;
  ladybug_msgs::LadybugTiles current_tiles_;
  sensor_msgs::PointCloud2 current_pointcloud_;

  // Transform objects
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf::TransformListener tf1_listener_;

};

} /* namespace */
