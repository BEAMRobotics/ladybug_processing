#pragma once

#include "ladybug_msgs/LadybugTiles.h"

//Ladybug includes
#include <ladybug/ladybug.h>
#include <ladybug/ladybuggeom.h>
#include <ladybug/ladybugrenderer.h>

#include <iostream>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

// OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>

//DC1394
#include <dc1394/conversions.h>

namespace ladybug_processing {

/*!
 * Class containing the colorizing part of the package.
 */
class Colorizer {
 public:

  /**
   * Constructor
   * @param ladybug_config_file_path
   */
  explicit Colorizer(const std::string &ladybug_config_file_path);

  /**
   * Destructor
   */
  virtual ~Colorizer() = default;

  /**
   * Method for colorizing point clouds based on projection.
   * @param tiles_ Colored images.
   * @param pc_ Point cloud transformed into camera frame.
   * @return Returns colored point cloud.
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorizePointCloud(const ladybug_msgs::LadybugTiles &tiles_,
                                                            const pcl::PointCloud<pcl::PointXYZ> &pc_) const;

 private:

  /**
   * Projects point in camera coordinate system into the natural rectified image.
   * @param cam
   * @param x
   * @param y
   * @param z
   * @return
   */
  std::pair<double, double> XYZToUVRect(int cam,
                                        double x,
                                        double y,
                                        double z) const;

  /**
   * Construct transformation matrix from EulerZYX
   * @param camera_id Camera number to use
   * @param extrinsics Array[6] of extrinsics obtained from Ladybug API
   */
  void ConstructTransform(const unsigned int &camera_id, const double *extrinsics);

  /**
   * Construct Eigen Matrix transform for individual camera based on extrinsics
   * @param rot_x
   * @param rot_y
   * @param rot_z
   * @param trans_x
   * @param trans_y
   * @param trans_z
   * @return
   */
  Eigen::MatrixXf ConstructTransform(double rot_x, double rot_y, double rot_z,
                                     double trans_x, double trans_y, double trans_z);

  /**
   * Method for processing the raw bayer tiles into colored images (colored_imgs_)
   * @param tiles_
   * @return
   */
  ladybug_msgs::LadybugTilesPtr ProcessImageTiles(const ladybug_msgs::LadybugTiles &tiles_) const;

  /**
   * Method for reading the ladybug.conf file and filling out the CameraConf struct
   */
  void ReadCameraCalibration();

  /**
   * Struct for holding relevant calibration information for a given camera.
   */
  struct CameraConf {
    unsigned int FULL_WIDTH,
        FULL_HEIGHT; // The width and height of a ladybug image (after all image processing operations: decompression, rotation, demosaicing)
    double f;
    double cx;
    double cy;
    double rx, ry, rz;
    double tx, ty, tz;
    double center[3];
    double topleft[3];
    double topright[3];
    double bottomleft[3];
    double bottomright[3];
    double distorted_cx;
    double distorted_cy;
    Eigen::MatrixXf ladybug_to_cam;
    Eigen::MatrixXf cam_to_ladybug;
  };

  const unsigned int FULL_WIDTH = 2048;
  const unsigned int FULL_HEIGHT = 2464;
  //std::vector<CameraConf> cameras_;
  CameraConf cameras_[6];

  LadybugContext context_;
  LadybugError error_;

  std::vector<unsigned char> combined_images_[6];

  std::string ladybug_config_file_path_; // Path to ladybug config file

  // TODO: Switch from OpenCV debayering to DC1394 debayering
  dc1394bayer_method_t debayer_alg_; //defaults to DC1394_BAYER_METHOD_HQLINEAR
};

} /* namespace */
