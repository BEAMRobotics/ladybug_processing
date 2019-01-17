#include "ladybug_processing/LadybugColorizerNode.hpp"

namespace ladybug_processing {

LadybugColorizerNode::LadybugColorizerNode(ros::NodeHandle &node_handle)
    : node_handle_(node_handle),
      node_handle_priv_("~"),
      tf_listener_(tf_buffer_),
      colorizer_(([](ros::NodeHandle nh, std::string str) {
        nh.param<std::string>("ladybug_conf_location", str, "/home");
        return str;
      })(node_handle_priv_, ladybug_conf_location_)) {

  ReadParameters();

  // Initialize subscribers & publishers
  image_subscriber_ = node_handle_.subscribe(image_topic_name_, 1,
                                             &LadybugColorizerNode::ImageTopicCallback, this);

  pointcloud_subscriber_ = node_handle_.subscribe(pointcloud_topic_name_, 1,
                                                  &LadybugColorizerNode::PointcloudTopicCallback, this);

  colored_pointcloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(colored_pointcloud_topic_name_, 1, true);

  ROS_INFO("Successfully launched node.");
}

bool LadybugColorizerNode::ReadParameters() {

  //Start by setting console log level
  node_handle_priv_.param<bool>("enable_debug_logging", enable_debug_logging_, false);
  if (enable_debug_logging_) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::Level::Debug)) {
      ROS_INFO("Log level set to Debug");
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  node_handle_priv_.param<bool>("save_data", save_data_, false);
  node_handle_priv_.param<bool>("publish_colored_pointcloud", publish_colored_pointcloud_, true);
  node_handle_priv_.param<std::string>("image_topic_name", image_topic_name_, "/image_tiles");
  node_handle_priv_.param<std::string>("pointcloud_topic_name", pointcloud_topic_name_, "/m3d_test/aggregator/cloud");
  node_handle_priv_.param<std::string>("colored_pointcloud_frame", colored_pointcloud_frame_, "/m3d_link");
  node_handle_priv_.param<std::string>("root_data_folder_name", root_folder_name_, "~/home");
  node_handle_priv_.param<std::string>("colored_pointcloud_topic_name",
                                       colored_pointcloud_topic_name_,
                                       pointcloud_topic_name_ + std::string("/colored"));

  ROS_DEBUG("save_data: '%s'", std::to_string(save_data_).c_str());
  if (save_data_) ROS_DEBUG("root_data_folder_name: '%s'", root_folder_name_.c_str());
  ROS_DEBUG("publish_colored_pointcloud: '%s'", std::to_string(publish_colored_pointcloud_).c_str());
  ROS_DEBUG("image_topic_name (subscriber): '%s'", image_topic_name_.c_str());
  ROS_DEBUG("pointcloud_topic_name (subscriber): '%s'", pointcloud_topic_name_.c_str());
  ROS_DEBUG("colored_pointcloud_topic_name (publisher): '%s'", colored_pointcloud_topic_name_.c_str());
  return true;
}

void LadybugColorizerNode::ImageTopicCallback(const ladybug_msgs::LadybugTilesPtr &tiles_) {
  current_tiles_ = *(tiles_);
}

void LadybugColorizerNode::PointcloudTopicCallback(const sensor_msgs::PointCloud2 &pc2_msg) {

  sensor_msgs::PointCloud2 pc2_transformed;
  const std::string image_frame = current_tiles_.header.frame_id;

  if (pcl_ros::transformPointCloud(image_frame, pc2_msg, pc2_transformed, tf1_listener_))
    ROS_INFO("Transformed point cloud into camera frame (%s)", image_frame.c_str());
  else
    ROS_WARN("Failed to transform point cloud into camera frame (%s)", image_frame.c_str());

  // Convert to pcl point cloud
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(pc2_transformed, pcl_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr colored_cloud2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  auto colored_cloud = boost::make_shared<const pcl::PointCloud<pcl::PointXYZRGB> >();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_col_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


  // Colorize the point cloud
  //pcl_col_cloud = colorizer_.ColorizePointCloud(current_tiles_, pcl_cloud);

  //colorizer_.colorizePointCloud(current_tiles_, pcl_cloud, *pcl_col_cloud);

  // Re-publish the colored cloud
  if (publish_colored_pointcloud_) {
    sensor_msgs::PointCloud2 pc2_colored, pc2_colored_tf;
    pcl::toROSMsg(*pcl_col_cloud, pc2_colored);
    pc2_colored.header = pc2_msg.header;
    pc2_colored.header.frame_id = image_frame;

    if (pcl_ros::transformPointCloud(colored_pointcloud_frame_, pc2_colored, pc2_colored_tf, tf1_listener_))
      ROS_INFO("Transformed colored point cloud to frame %s", image_frame.c_str());
    else
      ROS_WARN_STREAM("Failed to transform colored point cloud to frame : " << image_frame);

    colored_pointcloud_publisher_.publish(pc2_colored_tf);
  }

}

} /* namespace */
