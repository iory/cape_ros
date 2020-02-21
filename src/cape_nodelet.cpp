#include "cape_ros/cape_nodelet.h"

namespace cape_ros {

CAPENodelet::~CAPENodelet() {
  if (approximate_sync_) {
    async_.reset();
  } else {
    sync_.reset();
  }
}

void CAPENodelet::onInit() {
  DiagnosticNodelet::onInit();

  srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> > (*pnh_);
  dynamic_reconfigure::Server<Config>::CallbackType f =
    boost::bind (&CAPENodelet::configCallback, this, _1, _2);
  srv_->setCallback (f);

  pnh_->param("approximate_sync", approximate_sync_, true);
  pnh_->param("queue_size", queue_size_, 5);
  update_ = true;
  camera_info_updated_ = false;

  pub_img_ = advertiseImage(*pnh_, "output/viz", 1);
  pub_polygon = advertise<jsk_recognition_msgs::PolygonArray>(
      *pnh_, "output/polygons", 1);
  pub_cluster = advertise<jsk_recognition_msgs::ClusterPointIndices>(
      *pnh_, "output/cluster_indices", 1);
  pub_coeffs = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
      *pnh_, "output/coefficients", 1);
  pub_not_plane_indices =
      advertise<pcl_msgs::PointIndices>(*pnh_, "output/non_plane_indices", 1);

  // Populate with random color codes
  for (int i = 0; i < 100; i++) {
    cv::Vec3b color;
    color[0] = rand() % 255;
    color[1] = rand() % 255;
    color[2] = rand() % 255;
    color_code_.push_back(color);
  }
  // Add specific colors for planes
  color_code_[0][0] = 0;
  color_code_[0][1] = 0;
  color_code_[0][2] = 255;
  color_code_[1][0] = 255;
  color_code_[1][1] = 0;
  color_code_[1][2] = 204;
  color_code_[2][0] = 255;
  color_code_[2][1] = 100;
  color_code_[2][2] = 0;
  color_code_[3][0] = 0;
  color_code_[3][1] = 153;
  color_code_[3][2] = 255;
  // Add specific colors for cylinders
  color_code_[50][0] = 178;
  color_code_[50][1] = 255;
  color_code_[50][2] = 0;
  color_code_[51][0] = 255;
  color_code_[51][1] = 0;
  color_code_[51][2] = 51;
  color_code_[52][0] = 0;
  color_code_[52][1] = 255;
  color_code_[52][2] = 51;
  color_code_[53][0] = 153;
  color_code_[53][1] = 0;
  color_code_[53][2] = 255;

  onInitPostProcess();
}

void CAPENodelet::configCallback(Config &config, uint32_t level) {
  boost::mutex::scoped_lock lock(mutex_);
  patch_size_ = config.patch_size;
  cylinder_detection_ = config.cylinder_detection;
  update_ = true;
}


void CAPENodelet::updateDiagnostic(
  diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (vital_checker_->isAlive()) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                 "CAPE running");
  }
  DiagnosticNodelet::updateDiagnostic(stat);
}

void CAPENodelet::subscribe() {
  it_.reset(new image_transport::ImageTransport(*pnh_));
  sub_image_.subscribe(*it_, "input", 1);
  sub_depth_.subscribe(*it_, "input/depth", 1);
  sub_camera_info_.subscribe(*pnh_, "input/info", 1);

  if (approximate_sync_) {
    async_ = boost::make_shared<
        message_filters::Synchronizer<ApproxSyncPolicyWithCameraInfo> >(
        queue_size_);
    async_->connectInput(sub_image_, sub_depth_, sub_camera_info_);
    async_->registerCallback(
        boost::bind(&CAPENodelet::callback, this, _1, _2, _3));
  } else {
    sync_ = boost::make_shared<
        message_filters::Synchronizer<SyncPolicyWithCameraInfo> >(queue_size_);
    sync_->connectInput(sub_image_, sub_depth_, sub_camera_info_);
    sync_->registerCallback(
        boost::bind(&CAPENodelet::callback, this, _1, _2, _3));
  }
}

void CAPENodelet::unsubscribe() {
  sub_image_.unsubscribe();
  sub_depth_.unsubscribe();
  sub_camera_info_.unsubscribe();
}

void CAPENodelet::callback(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::Image::ConstPtr& depth_msg,
    const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
  boost::mutex::scoped_lock lock(mutex_);
  vital_checker_->poke();
  bool ret = check_and_update_intrinsic_parameters(image_msg, depth_msg,
                                                   camera_info_msg);
  if (ret || update_) {
    init_cape();
    update_ = false;
  }
  process(image_msg, depth_msg);
  diagnostic_updater_->update();
}

bool CAPENodelet::check_and_update_intrinsic_parameters(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::Image::ConstPtr& depth_msg,
    const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
  if (image_msg->header.frame_id != depth_msg->header.frame_id) {
    NODELET_ERROR("Current rgb and depth should be same frame_id! %s != %s",
                  image_msg->header.frame_id, depth_msg->header.frame_id);
    return false;
  }

  if (camera_info_updated_) {
    return false;
  }

  camera_model_.fromCameraInfo(*camera_info_msg);
  camera_info_updated_ = true;

  t_stereo_ = (cv::Mat_<float>(3, 1) << 0, 0, 0);
  R_stereo_ = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  K_rgb_ = camera_model_.intrinsicMatrix();
  K_ir_ = camera_model_.intrinsicMatrix();
  dist_coeffs_rgb_ = camera_model_.distortionCoeffs();
  dist_coeffs_ir_ = camera_model_.distortionCoeffs();
  width_ = image_msg->width;
  height_ = image_msg->height;
  return true;
}

void CAPENodelet::init_cape() {
  int nr_horizontal_cells = width_ / patch_size_;
  int nr_vertical_cells = height_ / patch_size_;

  X_pre_ = cv::Mat_<float>(height_, width_);
  Y_pre_ = cv::Mat_<float>(height_, width_);

  double fx_ir = K_ir_(0, 0);
  double fy_ir = K_ir_(1, 1);
  double cx_ir = K_ir_(0, 2);
  double cy_ir = K_ir_(1, 2);
  fx_rgb_ = K_rgb_(0, 0);
  fy_rgb_ = K_rgb_(1, 1);
  cx_rgb_ = K_rgb_(0, 2);
  cy_rgb_ = K_rgb_(1, 2);

  // Pre-computations for backprojection
  for (int r = 0; r < height_; r++) {
    for (int c = 0; c < width_; c++) {
      // Not efficient but at this stage doesn t matter
      X_pre_.at<float>(r, c) = (c - cx_ir) / fx_ir;
      Y_pre_.at<float>(r, c) = (r - cy_ir) / fy_ir;
    }
  }

  // Pre-computations for maping an image point cloud to
  // a cache-friendly array where cell's local point clouds are contiguous
  cell_map_ = cv::Mat_<int>(height_, width_);
  for (int r = 0; r < height_; r++) {
    int cell_r = r / patch_size_;
    int local_r = r % patch_size_;
    for (int c = 0; c < width_; c++) {
      int cell_c = c / patch_size_;
      int local_c = c % patch_size_;
      cell_map_.at<int>(r, c) =
          (cell_r * nr_horizontal_cells + cell_c) * patch_size_ * patch_size_ +
          local_r * patch_size_ + local_c;
    }
  }

  X_ = cv::Mat_<float>(height_, width_);
  Y_ = cv::Mat_<float>(height_, width_);
  X_t_ = cv::Mat_<float>(height_, width_);
  Y_t_ = cv::Mat_<float>(height_, width_);

  cloud_array_ = Eigen::MatrixXf(width_ * height_, 3);
  cloud_array_organized_ = Eigen::MatrixXf(width_ * height_, 3);

  // Initialize CAPE
  float COS_ANGLE_MAX = cos(M_PI / 12.0);
  float MAX_MERGE_DIST = 50.0f;
  plane_detector_ = boost::make_shared<CAPE>(height_, width_, patch_size_,
                                             patch_size_, cylinder_detection_,
                                             COS_ANGLE_MAX, MAX_MERGE_DIST);
}

void CAPENodelet::projectPointCloud(cv::Mat& X, cv::Mat& Y, cv::Mat& Z,
                                    cv::Mat& U, cv::Mat& V, float fx_rgb,
                                    float fy_rgb, float cx_rgb, float cy_rgb,
                                    double z_min,
                                    Eigen::MatrixXf& cloud_array) {
  int width = X.cols;
  int height = X.rows;

  // Project to image coordinates
  cv::divide(X, Z, U, 1);
  cv::divide(Y, Z, V, 1);
  U = U * fx_rgb + cx_rgb;
  V = V * fy_rgb + cy_rgb;
  // Reusing U as cloud index
  // U = V*width + U + 0.5;

  float *sz, *sx, *sy, *u_ptr, *v_ptr, *id_ptr;
  float z, u, v;
  int id;
  for (int r = 0; r < height; r++) {
    sx = X.ptr<float>(r);
    sy = Y.ptr<float>(r);
    sz = Z.ptr<float>(r);
    u_ptr = U.ptr<float>(r);
    v_ptr = V.ptr<float>(r);
    for (int c = 0; c < width; c++) {
      z = sz[c];
      u = u_ptr[c];
      v = v_ptr[c];
      if (z > z_min && u > 0 && v > 0 && u < width && v < height) {
        id = floor(v) * width + u;
        cloud_array(id, 0) = sx[c];
        cloud_array(id, 1) = sy[c];
        cloud_array(id, 2) = z;
      }
    }
  }
}

void CAPENodelet::organizePointCloudByCell(Eigen::MatrixXf& cloud_in,
                                           Eigen::MatrixXf& cloud_out,
                                           cv::Mat& cell_map) {
  int width = cell_map.cols;
  int height = cell_map.rows;
  int mxn = width * height;
  int mxn2 = 2 * mxn;

  int id, it(0);
  int* cell_map_ptr;
  for (int r = 0; r < height; r++) {
    cell_map_ptr = cell_map.ptr<int>(r);
    for (int c = 0; c < width; c++) {
      id = cell_map_ptr[c];
      *(cloud_out.data() + id) = *(cloud_in.data() + it);
      *(cloud_out.data() + mxn + id) = *(cloud_in.data() + mxn + it);
      *(cloud_out.data() + mxn2 + id) = *(cloud_in.data() + mxn2 + it);
      it++;
    }
  }
}

void CAPENodelet::process(const sensor_msgs::Image::ConstPtr& image_msg,
                          const sensor_msgs::Image::ConstPtr& depth_msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImagePtr depth_cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    depth_cv_ptr = cv_bridge::toCvCopy(
        depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Backproject to point cloud
  X_ = X_pre_.mul(depth_cv_ptr->image);
  Y_ = Y_pre_.mul(depth_cv_ptr->image);
  cloud_array_.setZero();

  // The following transformation+projection is only necessary to visualize RGB
  // with overlapped segments Transform point cloud to color reference frame
  X_t_ = ((float)R_stereo_.at<double>(0, 0)) * X_ +
         ((float)R_stereo_.at<double>(0, 1)) * Y_ +
         ((float)R_stereo_.at<double>(0, 2)) * depth_cv_ptr->image +
         (float)t_stereo_.at<double>(0);
  Y_t_ = ((float)R_stereo_.at<double>(1, 0)) * X_ +
         ((float)R_stereo_.at<double>(1, 1)) * Y_ +
         ((float)R_stereo_.at<double>(1, 2)) * depth_cv_ptr->image +
         (float)t_stereo_.at<double>(1);
  depth_cv_ptr->image =
      ((float)R_stereo_.at<double>(2, 0)) * X_ +
      ((float)R_stereo_.at<double>(2, 1)) * Y_ +
      ((float)R_stereo_.at<double>(2, 2)) * depth_cv_ptr->image +
      (float)t_stereo_.at<double>(2);

  projectPointCloud(X_t_, Y_t_, depth_cv_ptr->image, U_, V_, fx_rgb_, fy_rgb_,
                    cx_rgb_, cy_rgb_, t_stereo_.at<double>(2), cloud_array_);

  cv::Mat_<cv::Vec3b> seg_rz =
      cv::Mat_<cv::Vec3b>(height_, width_, cv::Vec3b(0, 0, 0));
  cv::Mat_<uchar> seg_output = cv::Mat_<uchar>(height_, width_, uchar(0));

  // Run CAPE
  int nr_planes, nr_cylinders;
  vector<PlaneSeg> plane_params;
  vector<CylinderSeg> cylinder_params;
  double t1 = cv::getTickCount();
  std::vector<std::vector<cv::Point> > polygons;
  organizePointCloudByCell(cloud_array_, cloud_array_organized_, cell_map_);
  plane_detector_->process(cloud_array_organized_, nr_planes, nr_cylinders,
                           seg_output, plane_params, cylinder_params, polygons);
  double t2 = cv::getTickCount();
  double time_elapsed = (t2 - t1) / (double)cv::getTickFrequency();

  // Map segments with color codes and overlap segmented image w/ RGB
  uchar* sCode;
  uchar* dColor;
  uchar* srgb;
  int code;
  for (int r = 0; r < height_; r++) {
    dColor = seg_rz.ptr<uchar>(r);
    sCode = seg_output.ptr<uchar>(r);
    srgb = cv_ptr->image.ptr<uchar>(r);
    for (int c = 0; c < width_; c++) {
      code = *sCode;
      if (code > 0) {
        dColor[c * 3] = color_code_[code - 1][0] / 2 + srgb[0] / 2;
        dColor[c * 3 + 1] = color_code_[code - 1][1] / 2 + srgb[1] / 2;
        dColor[c * 3 + 2] = color_code_[code - 1][2] / 2 + srgb[2] / 2;
        ;
      } else {
        dColor[c * 3] = srgb[0];
        dColor[c * 3 + 1] = srgb[1];
        dColor[c * 3 + 2] = srgb[2];
      }
      sCode++;
      srgb++;
      srgb++;
      srgb++;
    }
  }

  polygons.clear();
  std::vector<std::vector<cv::Point> > contours;
  cv::Mat mask(height_, width_, CV_8U);
  for (int i = 0; i < nr_planes; ++i) {
    mask = cv::Scalar(0);
    mask.setTo(1, seg_output == i + 1);
    contours.clear();
    cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> points;
    for (size_t j = 0; j < contours.size(); j++) {
      points.insert(points.end(), contours[j].begin(), contours[j].end());
    }
    std::vector<cv::Point> approx;
    cv::convexHull(points, approx);
    polygons.push_back(approx);
  }

  // Show frame rate and labels
  cv::rectangle(seg_rz, cv::Point(0, 0), cv::Point(width_, 20),
                cv::Scalar(0, 0, 0), -1);
  std::stringstream fps;
  fps << (int)(1 / time_elapsed + 0.5) << " fps";
  cv::putText(seg_rz, fps.str(), cv::Point(15, 15), cv::FONT_HERSHEY_SIMPLEX,
              0.5, cv::Scalar(255, 255, 255, 1));
  int cylinder_code_offset = 50;
  // show cylinder labels
  if (nr_cylinders > 0) {
    std::stringstream text;
    text << "Cylinders:";
    cv::putText(seg_rz, text.str(), cv::Point(width_ / 2, 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255, 1));
    for (int j = 0; j < nr_cylinders; j++) {
      cv::rectangle(seg_rz, cv::Point(width_ / 2 + 80 + 15 * j, 6),
                    cv::Point(width_ / 2 + 90 + 15 * j, 16),
                    cv::Scalar(color_code_[cylinder_code_offset + j][0],
                               color_code_[cylinder_code_offset + j][1],
                               color_code_[cylinder_code_offset + j][2]),
                    -1);
    }
  }

  jsk_recognition_msgs::PolygonArray polygon_array_msg;
  polygon_array_msg.header = depth_msg->header;

  for (int i = 0; i < polygons.size(); ++i) {
    geometry_msgs::PolygonStamped ps_msg;
    geometry_msgs::Polygon pol;
    for (int j = 0; j < polygons[i].size(); ++j) {
      geometry_msgs::Point32 p;
      int u = polygons[i][j].x;
      int v = polygons[i][j].y;
      int id = floor(v) * width_ + u;
      double x = cloud_array_(id, 0) / 1000.0;
      double y = cloud_array_(id, 1) / 1000.0;
      double z = cloud_array_(id, 2) / 1000.0;
      if (z <= 0.0) {
        // TODO(Avoid this case)
        continue;
      }
      double tmp =
          std::sqrt(plane_params[i].normal[0] * plane_params[i].normal[0] +
                    plane_params[i].normal[1] * plane_params[i].normal[1] +
                    plane_params[i].normal[2] * plane_params[i].normal[2]);
      plane_params[i].normal[0] /= tmp;
      plane_params[i].normal[1] /= tmp;
      plane_params[i].normal[2] /= tmp;
      double d =
          (plane_params[i].normal[0] * (x - plane_params[i].mean[0] / 1000.0) +
           plane_params[i].normal[1] * (y - plane_params[i].mean[1] / 1000.0) +
           plane_params[i].normal[2] * (z - plane_params[i].mean[2] / 1000.0)) /
          (plane_params[i].normal[0] * plane_params[i].normal[0] +
           plane_params[i].normal[1] * plane_params[i].normal[1] +
           plane_params[i].normal[2] * plane_params[i].normal[2]);
      x = x - d * plane_params[i].normal[0];
      y = y - d * plane_params[i].normal[1];
      z = z - d * plane_params[i].normal[2];
      p.x = x;
      p.y = y;
      p.z = z;
      pol.points.push_back(p);
    }

    ps_msg.header = depth_msg->header;
    ps_msg.polygon = pol;

    polygon_array_msg.polygons.push_back(ps_msg);
  }
  pub_polygon.publish(polygon_array_msg);

  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(depth_msg->header, "bgr8", seg_rz).toImageMsg();
  pub_img_.publish(msg);

  jsk_recognition_msgs::ClusterPointIndices cluster_point_indices_msg;
  jsk_recognition_msgs::ModelCoefficientsArray coeffs_msg;

  for (int i = 0; i < nr_planes; ++i) {
    pcl_msgs::PointIndices pi;
    pi.header = depth_msg->header;
    cluster_point_indices_msg.cluster_indices.push_back(pi);

    pcl_msgs::ModelCoefficients coef;
    coef.header = depth_msg->header;
    coef.values.push_back(plane_params[i].normal[0]);
    coef.values.push_back(plane_params[i].normal[1]);
    coef.values.push_back(plane_params[i].normal[2]);
    coef.values.push_back(plane_params[i].d / 1000.0);
    coeffs_msg.coefficients.push_back(coef);
  }

  for (int r = 0; r < height_; r++) {
    for (int c = 0; c < width_; c++) {
      uchar id = seg_output.at<uchar>(r, c);
      if (id == 0) {
        continue;
      }
      if (id > nr_planes) {
        continue;
      }
      cluster_point_indices_msg.cluster_indices[id - 1].indices.push_back(
          r * width_ + c);
    }
  }
  cluster_point_indices_msg.header = depth_msg->header;
  pub_cluster.publish(cluster_point_indices_msg);

  coeffs_msg.header = depth_msg->header;

  pub_coeffs.publish(coeffs_msg);

  pcl_msgs::PointIndices non_plane_indices_msg;
  non_plane_indices_msg.header = depth_msg->header;

  mask = cv::Scalar(0);
  for (int r = 0; r < height_; r++) {
    for (int c = 0; c < width_; c++) {
      uchar id = seg_output.at<uchar>(r, c);
      if (id == 0 || id > nr_planes) {
        continue;
      }
      mask.at<uchar>(r, c) = 1;
    }
  }

  for (int r = 0; r < height_; r++) {
    for (int c = 0; c < width_; c++) {
      if (mask.at<uchar>(r, c) == 0) {
        non_plane_indices_msg.indices.push_back(r * width_ + c);
      }
    }
  }

  pub_not_plane_indices.publish(non_plane_indices_msg);
};

}  // namespace cape_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cape_ros::CAPENodelet, nodelet::Nodelet);
