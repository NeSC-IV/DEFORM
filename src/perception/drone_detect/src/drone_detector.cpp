#include "drone_detector/drone_detector.h"

// STD
#include <string>

namespace detect {

DroneDetector::DroneDetector(ros::NodeHandle& nodeHandle)
    : nh_(nodeHandle)
{
  readParameters();

  my_odom_sub_ = nh_.subscribe("odometry", 100, &DroneDetector::rcvMyOdomCallback, this, ros::TransportHints().tcpNoDelay());
  depth_img_sub_ = nh_.subscribe("depth", 50, &DroneDetector::rcvDepthImgCallback, this, ros::TransportHints().tcpNoDelay());

  new_depth_img_pub_ = nh_.advertise<sensor_msgs::Image>("new_depth_image", 50);

  cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.16,
      0.0, 0.0, 0.0, 1.0;

  for(int i = 0; i < max_drone_num_; i++)
        agentX_odom_sub_[i] = nh_.subscribe<nav_msgs::Odometry>(
                std::string("/tb_") + std::to_string(i) + std::string("/odom"), 1, [=](const nav_msgs::OdometryConstPtr &msg){
                    if(i == my_id_)
                        return;

                    rcvDroneOdomCallbackBase(*msg, i);
                });
}

DroneDetector::~DroneDetector() = default;

void DroneDetector::readParameters()
{
  // camera params
  nh_.getParam("cam_width", img_width_);
  nh_.getParam("cam_height", img_height_);
  nh_.getParam("cam_fx", fx_);
  nh_.getParam("cam_fy", fy_);
  nh_.getParam("cam_cx", cx_);
  nh_.getParam("cam_cy", cy_);

  nh_.getParam("pixel_ratio", pixel_ratio_);
  nh_.getParam("my_id", my_id_);
  nh_.getParam("estimate/drone_width", drone_width_);
  nh_.getParam("estimate/drone_height", drone_height_);
  nh_.getParam("estimate/max_pose_error", max_pose_error_);

  max_pose_error2_ = max_pose_error_ * max_pose_error_;
}

// inline functions
inline double DroneDetector::getDist2(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
{
    double delta_x = p1(0) - p2(0);
    double delta_y = p1(1) - p2(1);
    double delta_z = p1(2) - p2(2);
    return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
}

inline double DroneDetector::getDist2(const Eigen::Vector4d &p1, const Eigen::Vector4d &p2)
{
    double delta_x = p1(0) - p2(0);
    double delta_y = p1(1) - p2(1);
    double delta_z = p1(2) - p2(2);
    return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
}

inline Eigen::Vector4d DroneDetector::depth2Pos(int u, int v, float depth)
{
  Eigen::Vector4d pose_in_camera;
  pose_in_camera(0) = (u - cx_) * depth / fx_;
  pose_in_camera(1) = (v - cy_) * depth / fy_;
  pose_in_camera(2) = depth; 
  pose_in_camera(3) = 1.0;
  return pose_in_camera;
}

inline Eigen::Vector4d DroneDetector::depth2Pos(const Eigen::Vector2i &pixel, float depth) 
{
  Eigen::Vector4d pose_in_camera;
  pose_in_camera(0) = (pixel(0) - cx_) * depth / fx_;
  pose_in_camera(1) = (pixel(1) - cy_) * depth / fy_;
  pose_in_camera(2) = depth; 
  pose_in_camera(3) = 1.0;
  return pose_in_camera;
}

inline Eigen::Vector2i DroneDetector::pos2Depth(const Eigen::Vector4d &pose_in_camera) 
{
  float depth = pose_in_camera(2);
  Eigen::Vector2i pixel;
  pixel(0) = pose_in_camera(0) * fx_ / depth + cx_ + 0.5;
  pixel(1) = pose_in_camera(1) * fy_ / depth + cy_ + 0.5;
  return pixel;
}

// determine whether a pixel is within the image
inline bool DroneDetector::isInSensorRange(const Eigen::Vector2i &pixel)
{
	if (pixel(0) >= 0 && pixel(1) >= 0 && pixel(0) <= img_width_ && pixel(1) <= img_height_)
		return true;
	else 
		return false;
}

void DroneDetector::rcvMyOdomCallback(const nav_msgs::Odometry& odom)
{
  my_odom_ = odom;
  Eigen::Matrix4d body2world = Eigen::Matrix4d::Identity();

  my_pose_world_(0) = odom.pose.pose.position.x;
  my_pose_world_(1) = odom.pose.pose.position.y;
  my_pose_world_(2) = odom.pose.pose.position.z;
  my_pose_world_(3) = 1.0;
  my_attitude_world_.x() = odom.pose.pose.orientation.x;
  my_attitude_world_.y() = odom.pose.pose.orientation.y;
  my_attitude_world_.z() = odom.pose.pose.orientation.z;
  my_attitude_world_.w() = odom.pose.pose.orientation.w;
  body2world.block<3,3>(0,0) = my_attitude_world_.toRotationMatrix();
  body2world(0,3) = my_pose_world_(0);
  body2world(1,3) = my_pose_world_(1);
  body2world(2,3) = my_pose_world_(2);

  //convert to cam pose
  cam2world_ = body2world * cam2body_;
  cam2world_quat_ = cam2world_.block<3,3>(0,0);

}

void DroneDetector::rcvDepthImgCallback(const sensor_msgs::ImageConstPtr& depth_img)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(depth_img, depth_img->encoding);
  cv_ptr->image.copyTo(depth_img_);

  debug_start_time_ = ros::Time::now();

  for (int i = 0; i < max_drone_num_; i++) {
    if (in_depth_[i]) {
      detect(i);
    }
  }   

  cv_bridge::CvImage out_msg;
  for (int i = 0; i < max_drone_num_; i++) {
    if (in_depth_[i]) {
      // erase hit pixels in depth
      for(int k = 0; k < int(hit_pixels_[i].size()); k++) {
        // depth_img_.at<float>(hit_pixels_[i][k](1), hit_pixels_[i][k](0)) = 0;
        uint16_t *row_ptr;
        row_ptr = depth_img_.ptr<uint16_t>(hit_pixels_[i][k](1));
        (*(row_ptr+hit_pixels_[i][k](0))) = 0.0;
      } 
    }
  }  
  debug_end_time_ = ros::Time::now();
  // ROS_WARN("cost_total_time = %lf", (debug_end_time_ - debug_start_time_).toSec()*1000.0);
  out_msg.header = depth_img->header;
  out_msg.encoding = depth_img->encoding;
  out_msg.image = depth_img_.clone();
  new_depth_img_pub_.publish(out_msg.toImageMsg());
}

void DroneDetector::rcvDroneOdomCallbackBase(const nav_msgs::Odometry& odom, int drone_id)
{
  if (drone_id == my_id_) {
    return;
  }

  Eigen::Matrix4d drone2world = Eigen::Matrix4d::Identity();
  drone_pose_world_[drone_id](0) = odom.pose.pose.position.x;
  drone_pose_world_[drone_id](1) = odom.pose.pose.position.y;
  drone_pose_world_[drone_id](2) = odom.pose.pose.position.z;
  drone_pose_world_[drone_id](3) = 1.0;

  drone_attitude_world_[drone_id].x() = odom.pose.pose.orientation.x;
  drone_attitude_world_[drone_id].y() = odom.pose.pose.orientation.y;
  drone_attitude_world_[drone_id].z() = odom.pose.pose.orientation.z;
  drone_attitude_world_[drone_id].w() = odom.pose.pose.orientation.w;
  drone2world.block<3,3>(0,0) = drone_attitude_world_[drone_id].toRotationMatrix();
  
  drone2world(0,3) = drone_pose_world_[drone_id](0);
  drone2world(1,3) = drone_pose_world_[drone_id](1);
  drone2world(2,3) = drone_pose_world_[drone_id](2);

  drone_pose_cam_[drone_id] = cam2world_.inverse() * drone_pose_world_[drone_id];
  // if the drone is in sensor range
  drone_ref_pixel_[drone_id] = pos2Depth(drone_pose_cam_[drone_id]);

  if (drone_pose_cam_[drone_id](2) > 0.1) {
    in_depth_[drone_id] = true;
  } else {
    in_depth_[drone_id] = false;
  }
}

void DroneDetector::countPixel(int drone_id)
{
  boundingbox_lu_[drone_id].x = img_width_;
  boundingbox_rd_[drone_id].x = 0;
  boundingbox_lu_[drone_id].y = img_height_;
  boundingbox_rd_[drone_id].y = 0;

  valid_pixel_cnt_[drone_id] = 0;
  hit_pixels_[drone_id].clear();

  Eigen::Vector2i tmp_pixel;
  Eigen::Vector4d tmp_pose_cam;
  int search_radius = 2*max_pose_error_*fx_/drone_pose_cam_[drone_id](2);
  float depth;
  searchbox_lu_[drone_id].x = drone_ref_pixel_[drone_id](0) - search_radius;
  searchbox_lu_[drone_id].y = drone_ref_pixel_[drone_id](1) - search_radius;
  searchbox_rd_[drone_id].x = drone_ref_pixel_[drone_id](0) + search_radius;
  searchbox_rd_[drone_id].y = drone_ref_pixel_[drone_id](1) + search_radius;
  // check the tmp_p around ref_pixel
  for(int i = -search_radius; i <= search_radius; i++)
    for(int j = -search_radius; j <= search_radius; j++)
    {
      tmp_pixel(0) = drone_ref_pixel_[drone_id](0) + j;
      tmp_pixel(1) = drone_ref_pixel_[drone_id](1) + i;
      if(tmp_pixel(0) < 0 || tmp_pixel(0) >= img_width_ || tmp_pixel(1) < 0 || tmp_pixel(1) >= img_height_)
        continue;
      // depth = depth_img_.at<float>(tmp_pixel(1), tmp_pixel(0));
      uint16_t *row_ptr;
      row_ptr = depth_img_.ptr<uint16_t>(tmp_pixel(1));
      depth = (*(row_ptr+tmp_pixel(0))) / 1000.0;

      // get tmp_pose in cam frame
      tmp_pose_cam = depth2Pos(tmp_pixel(0), tmp_pixel(1), depth);
      double dist2 = getDist2(tmp_pose_cam, drone_pose_cam_[drone_id]);
      if (dist2 < max_pose_error2_) {
        valid_pixel_cnt_[drone_id]++;
        hit_pixels_[drone_id].push_back(tmp_pixel);
        boundingbox_lu_[drone_id].x = tmp_pixel(0) < boundingbox_lu_[drone_id].x ? tmp_pixel(0) : boundingbox_lu_[drone_id].x;
        boundingbox_lu_[drone_id].y = tmp_pixel(1) < boundingbox_lu_[drone_id].y ? tmp_pixel(1) : boundingbox_lu_[drone_id].y;
        boundingbox_rd_[drone_id].x = tmp_pixel(0) > boundingbox_rd_[drone_id].x ? tmp_pixel(0) : boundingbox_rd_[drone_id].x;
        boundingbox_rd_[drone_id].y = tmp_pixel(1) > boundingbox_rd_[drone_id].y ? tmp_pixel(1) : boundingbox_rd_[drone_id].y;
      }
    }
}

void DroneDetector::detect(int drone_id)
{
  countPixel(drone_id);
}

void DroneDetector::test() {
  ROS_WARN("my_id = %d", my_id_);
}

} /* namespace */
