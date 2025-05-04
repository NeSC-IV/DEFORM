#pragma once
#include <iostream>
#include <vector>

// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>

//include opencv and eigen
#include <Eigen/Eigen>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

namespace detect {

/// equal number of vehicles
const int max_drone_num_ = 12;

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class DroneDetector
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  DroneDetector(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~DroneDetector();

  void test();
 private:
  void readParameters();

  double getDist2(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);
  double getDist2(const Eigen::Vector4d &p1, const Eigen::Vector4d &p2);
  Eigen::Vector4d depth2Pos(int u, int v, float depth);
  Eigen::Vector4d depth2Pos(const Eigen::Vector2i &pixel, float depth) ;
  Eigen::Vector2i pos2Depth(const Eigen::Vector4d &pose_in_camera) ;
  bool isInSensorRange(const Eigen::Vector2i &pixel);

  void countPixel(int drone_id);
  void detect(int drone_id);
  
  void rcvMyOdomCallback(const nav_msgs::Odometry& odom);
  void rcvDepthImgCallback(const sensor_msgs::ImageConstPtr& depth_img);

  void rcvDroneOdomCallbackBase(const nav_msgs::Odometry& odom, const int drone_id);

  //! ROS node handle.
  ros::NodeHandle& nh_;

  //! ROS topic subscriber.
  // other drones subscriber
  ros::Subscriber droneX_odom_sub_;
  ros::Subscriber my_odom_sub_, depth_img_sub_;
  bool has_odom_;
  nav_msgs::Odometry my_odom_;
  // ROS topic publisher
  // new_depth_img: erase the detected drones
  ros::Publisher new_depth_img_pub_;
  ros::Publisher debug_depth_img_pub_;

  // parameters
  //camera param
  int img_width_, img_height_;
  double fx_,fy_,cx_,cy_;

  double max_pose_error_;
  double max_pose_error2_;
  double drone_width_, drone_height_;
  double pixel_ratio_;

  // for debug
  ros::Time debug_start_time_, debug_end_time_;

  ros::Publisher debug_info_pub_;

  int my_id_;
  cv::Mat depth_img_, color_img_;

  Eigen::Matrix4d cam2body_;
  Eigen::Matrix4d cam2world_;
  Eigen::Quaterniond cam2world_quat_;
  Eigen::Vector4d my_pose_world_;
  Eigen::Quaterniond my_attitude_world_;
  Eigen::Vector4d my_last_pose_world_;
  ros::Time my_last_odom_stamp_ = ros::TIME_MAX;
  ros::Time my_last_camera_stamp_ = ros::TIME_MAX;

  Eigen::Matrix4d drone2world_[max_drone_num_];
  Eigen::Vector4d drone_pose_world_[max_drone_num_];
  Eigen::Quaterniond drone_attitude_world_[max_drone_num_];
  Eigen::Vector4d drone_pose_cam_[max_drone_num_];
  Eigen::Vector2i drone_ref_pixel_[max_drone_num_];

  std::vector<Eigen::Vector2i> hit_pixels_[max_drone_num_];
  int valid_pixel_cnt_[max_drone_num_];
  
  bool in_depth_[max_drone_num_] = {false};
  cv::Point searchbox_lu_[max_drone_num_], searchbox_rd_[max_drone_num_];
  cv::Point boundingbox_lu_[max_drone_num_], boundingbox_rd_[max_drone_num_];

  //changed part
  ros::Subscriber agentX_odom_sub_[max_drone_num_];
};

} /* namespace */
