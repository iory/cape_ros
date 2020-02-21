// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Iori Yanokura
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef _CAPE_H_
#define _CAPE_H_

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_msgs/PointIndices.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include "cape_ros/CapeConfig.h"

#include "cape_ros/CAPE.h"

namespace cape_ros {
class CAPENodelet : public jsk_topic_tools::DiagnosticNodelet {
 public:
  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
      SyncPolicyWithCameraInfo;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
      ApproxSyncPolicyWithCameraInfo;
  typedef cape_ros::CapeConfig Config;

  CAPENodelet() : DiagnosticNodelet("CAPE") {}

 protected:
  virtual ~CAPENodelet();
  virtual void onInit();
  virtual void callback(const sensor_msgs::Image::ConstPtr& image_msg,
                        const sensor_msgs::Image::ConstPtr& depth_msg,
                        const sensor_msgs::CameraInfoConstPtr& camera_info_msg);
  virtual void subscribe();
  virtual void unsubscribe();
  virtual bool check_and_update_intrinsic_parameters(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::Image::ConstPtr& depth_msg,
      const sensor_msgs::CameraInfoConstPtr& camera_info_msg);
  virtual void init_cape();

  boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
  virtual void configCallback(Config &config, uint32_t level);
  virtual void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);

  ros::NodeHandle nh_;
  boost::shared_ptr<image_transport::ImageTransport> it_;

  image_transport::Publisher pub_img_;
  image_transport::SubscriberFilter sub_image_;
  image_transport::SubscriberFilter sub_depth_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicyWithCameraInfo> >
      sync_;
  boost::shared_ptr<
      message_filters::Synchronizer<ApproxSyncPolicyWithCameraInfo> >
      async_;
  boost::mutex mutex_;

  ros::Publisher pub_polygon;
  ros::Publisher pub_cluster;
  ros::Publisher pub_not_plane_indices;
  ros::Publisher pub_coeffs;

  bool approximate_sync_;
  int queue_size_;
  int patch_size_;
  bool cylinder_detection_;
  bool camera_info_updated_;
  bool update_;

  cv::Matx33d K_rgb_, K_ir_;
  cv::Mat dist_coeffs_rgb_, dist_coeffs_ir_, R_stereo_, t_stereo_;

  sensor_msgs::CameraInfo prev_camera_info_msg_;
  boost::shared_ptr<CAPE> plane_detector_;

  std::vector<cv::Vec3b> color_code_;

  int height_;
  int width_;

  cv::Mat_<float> X_pre_;
  cv::Mat_<float> Y_pre_;
  cv::Mat_<int> cell_map_;
  cv::Mat_<float> U_;
  cv::Mat_<float> V_;
  cv::Mat_<float> X_;
  cv::Mat_<float> Y_;
  cv::Mat_<float> X_t_;
  cv::Mat_<float> Y_t_;
  Eigen::MatrixXf cloud_array_;
  Eigen::MatrixXf cloud_array_organized_;

  double fx_rgb_;
  double fy_rgb_;
  double cx_rgb_;
  double cy_rgb_;

  image_geometry::PinholeCameraModel camera_model_;

  void projectPointCloud(cv::Mat& X, cv::Mat& Y, cv::Mat& Z, cv::Mat& U,
                         cv::Mat& V, float fx_rgb, float fy_rgb, float cx_rgb,
                         float cy_rgb, double z_min,
                         Eigen::MatrixXf& cloud_array);
  void organizePointCloudByCell(Eigen::MatrixXf& cloud_in,
                                Eigen::MatrixXf& cloud_out, cv::Mat& cell_map);
  void process(const sensor_msgs::Image::ConstPtr& image_msg,
               const sensor_msgs::Image::ConstPtr& depth_msg);

 private:
};
}  // namespace cape_ros

#endif
