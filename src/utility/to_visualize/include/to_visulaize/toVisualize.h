#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
class toVisualization
{
public:
    toVisualization() = default;
    void init(ros::NodeHandle& nh);
    void showWidthLineMiddle(vector<Eigen::Vector2d> line);
    void showWidthLineOne(vector<Eigen::Vector2d> line);
    void showWidthLineOther(vector<Eigen::Vector2d> line);
    void showWidthMidPoint(Eigen::Vector2d pt);
    void showLocalGoal(Eigen::Vector2d goal);
    void showGlobalGoal(Eigen::Vector2d goal);
    void showAstarLists(vector<Eigen::Vector2d> path);
    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector2d> &list, double scale,
                           Eigen::Vector4d color, int id, bool show_sphere = true  );
    void displayPoint(ros::Publisher &pub, Eigen::Vector2d point,double scale,Eigen::Vector4d color,int id);
    void showWidthLocalMap_Greater(vector<Eigen::Vector2d>& map);
    void showWidthLocalMap_Less(vector<Eigen::Vector2d>& map);
    void showPath(vector<double> path);
    void showOdom(Eigen::Vector2d pos, bool isClear=false);
    void showUpdateArea(vector<Eigen::Vector2d> sides);
    void showTFBefore(vector<Eigen::Vector2d> pts);
    void showTFAfter(vector<Eigen::Vector2d> pts);
    void showFormationLines(vector<Eigen::Vector2d> pts);
    void showOdomAndLocalLine(vector<Eigen::Vector2d> line);
    void showCustomPath(vector<Eigen::Vector2d> path);
    typedef std::shared_ptr<toVisualization> Ptr;
private:
    ros::NodeHandle nh_pri;
    ros::Publisher show_astar, show_path_opt, show_width_localMap_greater, show_width_localMap_less;
    ros::Publisher show_width_line_middle, show_width_line_one, show_width_line_other, show_width_mid_point;
    ros::Publisher show_local_goal_marker;
    ros::Publisher show_nmpc_path;
    ros::Publisher show_odom_traj, show_global_point;
    ros::Publisher show_update_area;
    ros::Publisher show_tf_pts_before, show_tf_pts_after;
    ros::Publisher show_formation_lines;
    ros::Publisher show_odom_and_local_line;
    nav_msgs::Path odom_path;

    ros::Publisher show_width_line_middle_leader, show_width_line_one_leader, show_width_line_other_leader, show_width_mid_point_leader;
    ros::Publisher show_update_area_leader;
    ros::Publisher show_custom_path;
};
