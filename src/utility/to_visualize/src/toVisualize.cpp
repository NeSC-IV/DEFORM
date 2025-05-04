#include "to_visulaize/toVisualize.h"


void toVisualization::init(ros::NodeHandle &nh) {
    nh_pri = nh;

    show_astar = nh_pri.advertise<visualization_msgs::Marker>("astar_result",1);
    show_width_localMap_greater = nh_pri.advertise<sensor_msgs::PointCloud2>("width_local_map_greater",1);
    show_width_localMap_less = nh_pri.advertise<sensor_msgs::PointCloud2>("width_local_map_less",1);
    show_width_line_middle = nh_pri.advertise<visualization_msgs::Marker>("width_line_middle",1);
    show_width_line_one = nh_pri.advertise<visualization_msgs::Marker>("width_line_one",1);
    show_width_line_other = nh_pri.advertise<visualization_msgs::Marker>("width_line_other",1);
    show_width_mid_point = nh_pri.advertise<visualization_msgs::Marker>("width_mid_point",1);
    show_local_goal_marker = nh_pri.advertise<visualization_msgs::Marker>("local_goal",1);
    show_nmpc_path = nh_pri.advertise<nav_msgs::Path>("nmpc_path",1);
    show_odom_traj = nh_pri.advertise<nav_msgs::Path>("odom_traj",1);
    show_global_point = nh_pri.advertise<visualization_msgs::Marker>("global_point",1);
    show_tf_pts_after = nh_pri.advertise<visualization_msgs::Marker>("tf_after",1);
    show_tf_pts_before = nh_pri.advertise<visualization_msgs::Marker>("tf_before",2);
    show_formation_lines = nh_pri.advertise<visualization_msgs::Marker>("formation_line",1);
    show_odom_and_local_line = nh_pri.advertise<visualization_msgs::Marker>("odom_and_local_line",1);
    odom_path.poses.clear();
    odom_path.header.frame_id = "world";

    show_update_area = nh_pri.advertise<visualization_msgs::Marker>("update_area",1);
	
	// not only leader publish
    show_width_line_middle_leader = nh_pri.advertise<visualization_msgs::Marker>("width_line_middle",1);
    show_width_line_one_leader = nh_pri.advertise<visualization_msgs::Marker>("width_line_one",1);
    show_width_line_other_leader = nh_pri.advertise<visualization_msgs::Marker>("width_line_other",1);
    show_width_mid_point_leader = nh_pri.advertise<visualization_msgs::Marker>("width_mid_point",1);
    show_update_area_leader = nh_pri.advertise<visualization_msgs::Marker>("update_area",1);

	show_custom_path = nh_pri.advertise<nav_msgs::Path>("custom_path", 1);
}

void toVisualization::showCustomPath(vector<Eigen::Vector2d> path) {
    nav_msgs::Path path_msgs;
    path_msgs.header.frame_id = "world";
    path_msgs.header.stamp = ros::Time::now();
    path_msgs.poses.clear();
    for(int p_idx = 0; p_idx < path.size(); p_idx++)
    {
        geometry_msgs::PoseStamped tmp_pose;
        tmp_pose.header.frame_id = "world";
        tmp_pose.header.stamp = ros::Time::now();
        tmp_pose.pose.position.x = path[p_idx].x();
        tmp_pose.pose.position.y = path[p_idx].y();
        tmp_pose.pose.position.z = 0.2;
        path_msgs.poses.push_back(tmp_pose);
    }

    show_custom_path.publish(path_msgs);
}

void toVisualization::showFormationLines(vector<Eigen::Vector2d> pts) {
    Eigen::Vector4d color(0.6, 0.0, 0.0, 0.6);
    displayMarkerList(show_formation_lines, pts, 0.2, color, 0, false);
}
void toVisualization::showOdom(Eigen::Vector2d pos, bool isClear) {

    if(isClear)
        odom_path.poses.clear();

    odom_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.pose.position.x = pos.x();
    tmp_pose.pose.position.y = pos.y();
    odom_path.poses.push_back(tmp_pose);
    show_odom_traj.publish(odom_path);
}

void toVisualization::showTFBefore(vector<Eigen::Vector2d> pts) {
   Eigen::Vector4d color(0.5, 0.3, 0.8, 0.5);
    displayMarkerList(show_tf_pts_before, pts, 0.2, color, 0);
}

void toVisualization::showTFAfter(vector<Eigen::Vector2d> pts) {

    Eigen::Vector4d color(0.5, 0.8, 0.8, 0.4);
    displayMarkerList(show_tf_pts_after, pts, 0.2, color, 0);
}

void toVisualization::showPath(vector<double> path) {
    nav_msgs::Path path_msgs;
    path_msgs.header.frame_id = "world";
    path_msgs.header.stamp = ros::Time::now();
    path_msgs.poses.clear();
    for(int p_idx = 0; p_idx < path.size(); p_idx += 3)
    {
        geometry_msgs::PoseStamped tmp_pose;
        tmp_pose.header.frame_id = "world";
        tmp_pose.header.stamp = ros::Time::now();
        tmp_pose.pose.position.x = path[p_idx];
        tmp_pose.pose.position.y = path[p_idx + 1];
        path_msgs.poses.push_back(tmp_pose);
    }

    show_nmpc_path.publish(path_msgs);
}

void toVisualization::showGlobalGoal(Eigen::Vector2d goal) {
    Eigen::Vector4d color(0.0 ,0.3, 0.3, 0.8);
    displayPoint(show_global_point, goal, 0.3, color, 0);
}

void toVisualization::showLocalGoal(Eigen::Vector2d goal) {
    Eigen::Vector4d color(0.0, 0.0, 0.0, 0.4);
    displayPoint(show_local_goal_marker, goal, 0.3, color, 0);
}

void toVisualization::showWidthMidPoint(Eigen::Vector2d pt) {
    std::vector<Eigen::Vector2d> pts;
    pts.resize(2);
    pts[0] = pt;
    pts[1] = pt;
    Eigen::Vector4d color(0.3, 0.2, 0.8, 0.6);
    displayMarkerList(show_width_mid_point, pts, 0.3, color, 0);

}

void toVisualization::showUpdateArea(vector<Eigen::Vector2d> sides) {
    Eigen::Vector4d color(0.6, 0.6, 0.8, 1.0);
    displayMarkerList(show_update_area, sides, 0.1, color, 0);
    displayMarkerList(show_update_area_leader, sides, 0.1, color, 0);
}

void toVisualization::showOdomAndLocalLine(vector<Eigen::Vector2d> line) {
    Eigen::Vector4d color(0.5, 0.8, 0.2, 0.7);
    displayMarkerList(show_odom_and_local_line, line, 0.1, color, 0);
}

void toVisualization::showWidthLineOne(vector<Eigen::Vector2d> line) {
    Eigen::Vector4d color(0.6, 0.0, 0.0, 0.8);
    displayMarkerList(show_width_line_one, line, 0.1, color, 0);
    displayMarkerList(show_width_line_one_leader, line, 0.1, color, 0);
}

void toVisualization::showWidthLineOther(vector<Eigen::Vector2d> line) {
    Eigen::Vector4d color(0.0, 0.0, 0.6, 0.8);
    displayMarkerList(show_width_line_other, line, 0.1, color, 0);
    displayMarkerList(show_width_line_other_leader, line, 0.1, color, 0);
}

void toVisualization::showWidthLineMiddle(vector<Eigen::Vector2d> line) {
    Eigen::Vector4d color(0.2, 0.2, 0.2, 0.5);
    displayMarkerList(show_width_line_middle, line, 0.1, color, 0);
    displayMarkerList(show_width_line_middle_leader, line, 0.1, color, 0);
}

void toVisualization::showWidthLocalMap_Less(vector<Eigen::Vector2d> &map) {

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (auto i : map) {
        pt.x = i.x();
        pt.y = i.y();
        pt.z = 0.0;
        cloud.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud,cloud_msg);
    show_width_localMap_less.publish(cloud_msg);
}

void toVisualization::showWidthLocalMap_Greater(vector<Eigen::Vector2d> &map) {

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (auto i : map) {
        pt.x = i.x();
        pt.y = i.y();
        pt.z = 0.0;
        cloud.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    show_width_localMap_greater.publish(cloud_msg);
}

void toVisualization::showAstarLists(vector<Eigen::Vector2d> path)
{
    Eigen::Vector4d color(0.0, 0.5, 0.0, 0.7);
    displayMarkerList(show_astar, path, 0.1, color, 0);
}

void toVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector2d> &list, double scale,
                       Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
{
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = 0.1;
        if (show_sphere) sphere.points.push_back(pt);
        line_strip.points.push_back(pt);
    }
    if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);
}
void toVisualization::displayPoint(ros::Publisher &pub, Eigen::Vector2d point, double scale, Eigen::Vector4d color,
                                   int id) {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = point(0);
    sphere.pose.position.y = point(1);
    sphere.pose.position.z = 0.2;

    pub.publish(sphere);
}
