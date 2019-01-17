#include <ladybug_processing/Colorizer.hpp>

namespace ladybug_processing {

Colorizer::Colorizer(const std::string &ladybug_config_file_path)
    : ladybug_config_file_path_(ladybug_config_file_path),
      debayer_alg_(DC1394_BAYER_METHOD_SIMPLE) {
  ReadCameraCalibration();
}

void Colorizer::ReadCameraCalibration() {

  // Create the ladybugContext
  error_ = ladybugCreateContext(&context_);
  if (error_ != LADYBUG_OK)
    ROS_FATAL("ladybugCreateContext failed to initialize context with error: %s,", ladybugErrorToString(error_));

  // Load the Ladybug calibration file
  ROS_DEBUG_STREAM("Loading ladybug calibration file from: " << ladybug_config_file_path_);
  error_ = ladybugLoadConfig(context_, ladybug_config_file_path_.c_str());
  if (error_ != LADYBUG_OK)
    ROS_FATAL("ladybugLoadConfig failed to load calibration file with error: %s,", ladybugErrorToString(error_));

  // These (dummy) calls are required in order to obtain intrinsics
  error_ = ladybugConfigureOutputImages(context_, LADYBUG_RECTIFIED_CAM0);
  error_ = ladybugSetOffScreenImageSize(context_, LADYBUG_RECTIFIED_CAM4, FULL_HEIGHT, FULL_WIDTH);

  // Loop through each of the 6 cameras
  for (unsigned int cam = 0; cam < LADYBUG_NUM_CAMERAS; cam++) {
    error_ = ladybugGetCameraUnitFocalLength(context_, cam, &cameras_[cam].f);
    error_ = ladybugGetCameraUnitImageCenter(context_, cam, &cameras_[cam].cx, &cameras_[cam].cy);

    double camera_extrinsics[6] = {0.0};
    error_ = ladybugGetCameraUnitExtrinsics(context_, cam, camera_extrinsics);

    ConstructTransform(cam, camera_extrinsics);
  }

  ROS_INFO("Finished populating ladybug intrinsics / extrinsics.");
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Colorizer::ColorizePointCloud(const ladybug_msgs::LadybugTiles &tiles_,
                                                                     const pcl::PointCloud<pcl::PointXYZ> &pc_) const {
  ROS_INFO("Starting colorization: Point cloud size: %d", pc_.width);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  // Start by coloring our image tiles to populate colored_imgs
  auto colored_tiles = ProcessImageTiles(tiles_);

  unsigned int byte_per_pixel = 3;
  double u = -1, v = -1, ur_row = -1, ur_col = -1;

  LadybugError error;

  // Loop over each of the 6 cameras
  for (auto cam = 0; cam < LADYBUG_NUM_CAMERAS; ++cam) {
    ROS_INFO("  Colorizing image: %d", cam);

    // TODO: Parameterize which cameras to use in colorization (e.g., [1,1,0,1,1,1])
    if (cam == 2) continue;// Skip camera pointing at velodyne

    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::transformPointCloud(pc_,
                             point_cloud,
                             cameras_[cam].ladybug_to_cam); // Transform point cloud into camera frame

    int index = -1;
    for (auto point : point_cloud) {
      ++index;
      if (point.z < 0) continue; // Skip points behind image plane

      std::tie(u, v) = XYZToUVRect(cam, point.x, point.y, point.z); // TODO: Why don't structured bindings work here?
      if (u == -1 || v == -1) continue;

      error = ladybugUnrectifyPixel(context_, 0, v, u, &ur_row, &ur_col);
      if (error != LADYBUG_OK) continue;

      if (ur_row > 10 && ur_col > 10 && ur_row < 2000 && ur_col < 2000) {
        unsigned char *pixel_offset = &(colored_tiles->images[cam].data[0])
            + (int(floor(ur_col)) + int(floor(FULL_HEIGHT)) * int(floor(ur_row))) * byte_per_pixel;

        pcl::PointXYZRGB p = {pixel_offset[2], pixel_offset[1], pixel_offset[0]};
        p.x = pc_.points[index].x;
        p.y = pc_.points[index].y;
        p.z = pc_.points[index].z;
        //p.r = uint8_t(pixel_offset[2]); // p.r = int(pixel_offset[2]);
        //p.g = uint8_t(pixel_offset[1]); // p.g = int(pixel_offset[1]);
        //p.b = uint8_t(pixel_offset[0]); // p.b = int(pixel_offset[0]);
        colored_cloud->push_back(p);
      }
    }
  }
  ROS_INFO("Finished colorizing point cloud");
  return colored_cloud;
}

ladybug_msgs::LadybugTilesPtr Colorizer::ProcessImageTiles(const ladybug_msgs::LadybugTiles &tiles_) const {

  ladybug_msgs::LadybugTilesPtr colored_tiles = boost::make_shared<ladybug_msgs::LadybugTiles>();
  colored_tiles->header = tiles_.header;
  colored_tiles->images.resize(6);

  // Need to convert our sensor_msgs into OpenCV format
  cv::Size img_dims_cv(FULL_HEIGHT, FULL_WIDTH);
  cv::Mat colored_image_cv(img_dims_cv, CV_8UC3); //Create OpenCV colored containers
  size_t image_size = FULL_WIDTH * FULL_HEIGHT * colored_image_cv.elemSize();

  for (auto cam = 0; cam < LADYBUG_NUM_CAMERAS; ++cam) {
    cv::Mat raw_image_cv(img_dims_cv,
                         CV_8UC1,
                         const_cast<unsigned char *>(&(tiles_.images[cam].data[0]))); //Define OpenCV raw containers
    cv::cvtColor(raw_image_cv, colored_image_cv, cv::COLOR_BayerRG2RGB); // Color images
    colored_tiles->images[cam].data.resize(image_size);
    memcpy(&(colored_tiles->images[cam].data[0]), colored_image_cv.data, image_size);
  }

  ROS_INFO("Finished processing ladybug image tiles");
  return colored_tiles;
}

std::pair<double, double> Colorizer::XYZToUVRect(int cam,
                                                 double x,
                                                 double y,
                                                 double z) const {

  double focal_length_ = cameras_[cam].f;
  double rect_center_u_ = cameras_[cam].cx;
  double rect_center_v_ = cameras_[cam].cy;

  double rect_cols_ = FULL_HEIGHT; //2460;
  double rect_rows_ = FULL_WIDTH; // 2040;

  double u = -1;
  double v = -1;

  u = ((focal_length_) * x / z + rect_center_u_); // This projection is into the natural rectified image.
  v = ((focal_length_) * y / z + rect_center_v_);

  // Make sure point is in rectified image
  if (u < 0 || u > rect_cols_ - 1 || v < 0 || v > rect_rows_ - 1) {
    u = -1;
    v = -1;
  }

  return std::make_pair(u, v);
}

void Colorizer::ConstructTransform(const unsigned int &camera_id, const double *extrinsics) {

  double rot_x = extrinsics[0];
  double rot_y = extrinsics[1];
  double rot_z = extrinsics[2];
  double trans_x = extrinsics[3];
  double trans_y = extrinsics[4];
  double trans_z = extrinsics[5];

  ROS_INFO("Camera %d extrinsics: (%.03f, %.03f, %.03f, %.03f, %.03f, %.03f)",
           camera_id, rot_x, rot_y, rot_z, trans_x, trans_y, trans_z);

  cameras_[camera_id].rx = rot_x;
  cameras_[camera_id].ry = rot_y;
  cameras_[camera_id].rz = rot_z;
  cameras_[camera_id].tx = trans_x;
  cameras_[camera_id].ty = trans_y;
  cameras_[camera_id].tz = trans_z;
  cameras_[camera_id].cam_to_ladybug = ConstructTransform(rot_x, rot_y, rot_z, trans_x, trans_y, trans_z);
  cameras_[camera_id].ladybug_to_cam = cameras_[camera_id].cam_to_ladybug.inverse();
}

Eigen::MatrixXf Colorizer::ConstructTransform(double rot_x, double rot_y, double rot_z,
                                              double trans_x, double trans_y, double trans_z) {
  double crx = cos(rot_x);
  double cry = cos(rot_y);
  double crz = cos(rot_z);
  double srx = sin(rot_x);
  double sry = sin(rot_y);
  double srz = sin(rot_z);

  Eigen::MatrixXf transform = Eigen::MatrixXf::Identity(4, 4);

  transform(0, 0) = crz * cry;
  transform(0, 1) = crz * sry * srx - srz * crx;
  transform(0, 2) = crz * sry * crx + srz * srx;
  transform(0, 3) = trans_x;

  transform(1, 0) = srz * cry;
  transform(1, 1) = srz * sry * srx + crz * crx;
  transform(1, 2) = srz * sry * crx - crz * srx;
  transform(1, 3) = trans_y;

  transform(2, 0) = -sry;
  transform(2, 1) = cry * srx;
  transform(2, 2) = cry * crx;
  transform(2, 3) = trans_z;

  return transform;
}

} /* namespace */
