#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;


// constant parameters

struct MappingParameters
{

  /* map properties */
  Eigen::Vector2d map_origin_, map_size_;
  Eigen::Vector2d map_min_boundary_, map_max_boundary_; // map range in pos
  Eigen::Vector2i map_voxel_num_;                       // map range in index
  Eigen::Vector2d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  string frame_id_;
  int pose_type_;
  int car_height;

  /* camera parameters */
  double cx_, cy_, fx_, fy_;
  int width,height;

  /* time out */
  double odom_depth_timeout_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_depth_filter_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                  // logit of occupancy probability
  double min_ray_length_, max_ray_length_; // range of doing raycasting

  /* local map update and clear */
  int local_map_margin_;

  /* visualization and computation time display */
  double visualization_truncate_height_, virtual_ceil_height_, ground_height_;
  bool show_occ_time_;

  /* active mapping */
  double unknown_flag_;

  /* esdf */
  double esdf_slice_height_;
  bool show_esdf_time_;
  double local_bound_inflate_;
};

// intermediate mapping data for fusion

struct MappingData
{
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_neg_;
  std::vector<char> occupancy_buffer_inflate_;

  std::vector<double> tmp_buffer1_, tmp_buffer2_;
  std::vector<double> distance_buffer_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_all_;

  // camera position and pose data

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Vector2d camera_pos_two,last_camera_pos_two;
  Eigen::Matrix3d camera_r_m_, last_camera_r_m_;
  Eigen::Matrix4d cam2body_;

  // depth image data

  cv::Mat depth_image_, last_depth_image_;
  int image_cnt_;

  // flags of map state

  bool occ_need_update_, local_updated_;
  bool has_first_depth_;
  bool has_odom_, has_cloud_;

  // odom_depth_timeout_
  ros::Time last_occ_update_time_;
  bool flag_depth_odom_timeout_;
  bool flag_use_depth_fusion;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  vector<Eigen::Vector2d> proj_pts_to_2d;
  int pts_2d_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector2i> cache_voxel_;

  // range of updating grid

  Eigen::Vector2i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  // esdf map
  bool esdf_need_update_;
  double esdf_time_, max_esdf_time_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:

  GridMap() {}
  ~GridMap() {}

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };
  // occupancy map management

  void resetBuffer();
  void resetBuffer(Eigen::Vector2d min, Eigen::Vector2d max);

  inline void posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id);
  inline void indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos);
  inline int toAddress(const Eigen::Vector2i &id);
  inline int toAddress(int &x, int &y);
  inline bool isInMap(const Eigen::Vector2d &pos);
  inline bool isInMap(const Eigen::Vector2i &idx);

  inline void setOccupancy(Eigen::Vector2d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector2d pos);
  inline int getOccupancy(Eigen::Vector2d pos);
  inline int getOccupancy(Eigen::Vector2i id);
  inline int getInflateOccupancy(Eigen::Vector2d pos);
  
  inline void boundIndex(Eigen::Vector2i &id);
  inline bool isUnknown(const Eigen::Vector2i &id);
  inline bool isUnknown(const Eigen::Vector2d &pos);
  inline bool isKnownFree(const Eigen::Vector2i &id);
  inline bool isKnownOccupied(const Eigen::Vector2i &id);

  void initMap(ros::NodeHandle &nh);
	
  void publishMap();
  void publishMapInflate(bool all_info = false);

  void publishDepth();

  bool hasDepthObservation();
  bool odomValid();
  void getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size);
  inline double getResolution();
  Eigen::Vector2d getOrigin();
  int getVoxelNum();
  bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

  // esdf
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
  void updateESDF3d();
  void publishESDF();
  inline double getDistance(const Eigen::Vector2d& pos);
  inline double getDistance(const Eigen::Vector2i& id);
  void getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff);
  void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
  void interpolateBilinearEDT(double values[2][2], const Eigen::Vector2d &diff, double &value);
  void interpolateBilinearFirstGrad(double values[2][2], const Eigen::Vector2d& diff, Eigen::Vector2d& grad);

  void evaluateEDT(const Eigen::Vector2d& pos, double& dist);
  void evaluateFirstGrad(const Eigen::Vector2d& pos, Eigen::Vector2d& grad);

  // the esdf be used to frm_rrt
  void updateESDFMap(Eigen::Vector2d bound_max);

  // get designated area
  void getBoundaryMap(const Eigen::Vector2d& boundary_min,const Eigen::Vector2d& boundary_max, vector<Eigen::Vector2d>& local_map);
  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  // get depth image and camera pose
  void depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                         const geometry_msgs::PoseStampedConstPtr &pose);
  void extrinsicCallback(const nav_msgs::OdometryConstPtr &odom);
  void depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img);
  void odomCallback(const nav_msgs::OdometryConstPtr &odom);

  // update occupancy by raycasting
  void updateOccupancyCallback(const ros::TimerEvent & /*event*/);
  void visCallback(const ros::TimerEvent & /*event*/);
  void updateESDFCallback(const ros::TimerEvent& /*event*/);

  // main update process
  void projectDepthImage();
  void project2D();
  void raycastProcess();
  void clearAndInflateLocalMap();

  inline void inflatePoint(const Eigen::Vector2i &pt, int step, vector<Eigen::Vector2i> &pts);
  int setCacheOccupancy(Eigen::Vector2d pos, int occ);
  Eigen::Vector2d closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &camera_pt);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;

  ros::Subscriber indep_cloud_sub_, indep_odom_sub_, extrinsic_sub_;
  ros::Publisher map_pub_, map_inf_pub_, esdf_pub_;
  ros::Timer occ_timer_, vis_timer_, esdf_timer_;

  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector2i &id)
{
  return id(0) * mp_.map_voxel_num_(1) + id(1);
}

inline int GridMap::toAddress(int &x, int &y)
{
  return x * mp_.map_voxel_num_(1) + y;
}

inline void GridMap::boundIndex(Eigen::Vector2i &id)
{
  Eigen::Vector2i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector2i &id)
{
  Eigen::Vector2i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector2d &pos)
{
  Eigen::Vector2i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector2i &id)
{
  Eigen::Vector2i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector2i &id)
{
  Eigen::Vector2i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector2d pos)
{
  if (!isInMap(pos))
    return;

  Eigen::Vector2i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1)  + id(1) ] = 1;
}

inline void GridMap::setOccupancy(Eigen::Vector2d pos, double occ)
{
  if (occ != 1 && occ != 0)
  {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos))
    return;

  Eigen::Vector2i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ;
}

inline int GridMap::getOccupancy(Eigen::Vector2d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector2i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector2d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector2i id;
  posToIndex(pos, id);

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline int GridMap::getOccupancy(Eigen::Vector2i id)
{
  if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector2d &pos)
{
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4)
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector2i &idx)
{
  if (idx(0) < 0 || idx(1) < 0 )
  {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 )
  {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id)
{
  for (int i = 0; i < 2; ++i)
    id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos)
{
  for (int i = 0; i < 2; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void GridMap::inflatePoint(const Eigen::Vector2i &pt, int step, vector<Eigen::Vector2i> &pts)
{
  int num = 0;

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
    {
        pts[num++] = Eigen::Vector2i(pt(0) + x, pt(1) + y);
    }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

inline double GridMap::getDistance(const Eigen::Vector2d& pos) {
  Eigen::Vector2i id;
  posToIndex(pos, id);
  boundIndex(id);

  return md_.distance_buffer_all_[toAddress(id)];
}

inline double GridMap::getDistance(const Eigen::Vector2i& id) {
  Eigen::Vector2i id1 = id;
  boundIndex(id1);
  return md_.distance_buffer_all_[toAddress(id1)];
}

#endif
