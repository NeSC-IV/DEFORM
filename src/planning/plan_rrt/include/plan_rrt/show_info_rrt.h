
#ifndef SHOW_INFO_RRT
#define SHOW_INFO_RRT

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Eigen>
#include <vector>
#include <memory>

struct showTree {
	Eigen::Vector2d pt;
	std::vector<std::unique_ptr<showTree>> children;
	showTree() = default ;
	showTree(Eigen::Vector2d p) : pt(p){}
};

class ShowInfoRRT {
public:
	inline ShowInfoRRT(const ros::NodeHandle& nh) {
		nh_ = nh;
		show_path_pub = nh_.advertise<nav_msgs::Path>("rrt_path", 1);
		show_circle_pub = nh_.advertise<visualization_msgs::MarkerArray>("rrt_circles", 2);
		show_chord_pub = nh_.advertise<visualization_msgs::Marker>("rrt_chord", 2);
		show_random_node = nh_.advertise<visualization_msgs::Marker>("rrt_random_node", 1);
		show_new_node = nh_.advertise<visualization_msgs::Marker>("rrt_new_node", 1);
		show_tree = nh_.advertise<visualization_msgs::MarkerArray>("rrt_tree", 1);
		show_goal = nh_.advertise<visualization_msgs::Marker>("goal", 1);
		show_test_point = nh_.advertise<visualization_msgs::Marker>("test_point", 1);

		nh_.param("r", cr, 255.0);
		nh_.param("g", cg, 186.0);
		nh_.param("b", cb, 165.0);
	}

	inline void showPath(const std::vector<Eigen::Vector2d>& path) {
		nav_msgs::Path path_msgs;
		path_msgs.header.frame_id = "world";
		path_msgs.header.stamp = ros::Time::now();
		for(int idx = 0; idx < path.size(); idx++) {
			geometry_msgs::PoseStamped tmp_pose;
			tmp_pose.header.frame_id = "world";
			tmp_pose.header.stamp = ros::Time::now();
			tmp_pose.pose.position.x = path[idx].x();
			tmp_pose.pose.position.y = path[idx].y();
			path_msgs.poses.push_back(tmp_pose);
		}             

		show_path_pub.publish(path_msgs);
	}

	inline void showCircle(const std::vector<Eigen::Vector3d>& circles) {
		ROS_INFO("mks size:%d", circles.size());
		visualization_msgs::MarkerArray mka;
		for(size_t i = 0; i < circles.size(); i++) {
			auto marker = createCircleMarker(i, circles[i].x(), circles[i].y(), circles[i].z());
			mka.markers.push_back(marker);
		}
		show_circle_pub.publish(mka);
	}

	// two points conbine a line
	inline void showChord(const std::vector<Eigen::Vector2d>& pts) {
		visualization_msgs::Marker line_list;
		line_list.header.frame_id = "world";
		line_list.header.stamp = ros::Time::now();	
		line_list.ns = "disconnected_lines";
		line_list.action = visualization_msgs::Marker::ADD;
		line_list.pose.orientation.w = 1.0;
		line_list.id = 0;
		line_list.type = visualization_msgs::Marker::LINE_LIST;

		line_list.scale.x = 0.07;

		line_list.color.r = 0.8;
		line_list.color.g = 0.2;
		line_list.color.b = 0.1;
		line_list.color.a = 0.9;
		
		for (size_t i = 0; i < pts.size(); i += 2) {
			geometry_msgs::Point tmp_pt1, tmp_pt2;
			tmp_pt1.x = pts[i].x(); tmp_pt1.y = pts[i].y();
			tmp_pt2.x = pts[i + 1].x(); tmp_pt2.y = pts[i + 1].y();
			line_list.points.push_back(tmp_pt1);
			line_list.points.push_back(tmp_pt2);
		}
		show_chord_pub.publish(line_list);
	}
	
	// show random node
	inline void showRandomNode(const Eigen::Vector2d& pt) {
		visualization_msgs::Marker point;
		point.header.frame_id = "world";
		point.header.stamp = ros::Time::now();
		point.ns = "point";
		point.action = visualization_msgs::Marker::ADD;
		point.pose.orientation.w = 1.0;
		point.id = 0;
		point.type = visualization_msgs::Marker::POINTS;
		
		point.scale.x = 0.5;
		point.scale.y = 0.5; 
		
		point.color.r = 0.8f;
		point.color.g = 0.0f;
		point.color.b = 0.0f;
		point.color.a = 0.8f;

		geometry_msgs::Point p;
		p.x = pt.x();
		p.y = pt.y();
		p.z = 0.0;
		
		point.points.push_back(p);
		show_random_node.publish(point);
		
	}
	
	// show goal
	inline void showGoal(const Eigen::Vector2d& pt) {
		visualization_msgs::Marker point;
		point.header.frame_id = "world";
		point.header.stamp = ros::Time::now();
		point.ns = "goal";
		point.action = visualization_msgs::Marker::ADD;
		point.pose.orientation.w = 1.0;
		point.id = 0;
		point.type = visualization_msgs::Marker::POINTS;
		
		point.scale.x = 0.5;
		point.scale.y = 0.5; 
		
		point.color.r = 0.8f;
		point.color.g = 0.8f;
		point.color.b = 0.8f;
		point.color.a = 0.5f;

		geometry_msgs::Point p;
		p.x = pt.x();
		p.y = pt.y();
		p.z = 0.0;
		
		point.points.push_back(p);
		show_goal.publish(point);
		
	}

	// show test point
	inline void showTestPoint(const Eigen::Vector2d& pt) {
		visualization_msgs::Marker point;
		point.header.frame_id = "world";
		point.header.stamp = ros::Time::now();
		point.ns = "test_point";
		point.action = visualization_msgs::Marker::ADD;
		point.pose.orientation.w = 1.0;
		point.id = 0;
		point.type = visualization_msgs::Marker::POINTS;
		
		point.scale.x = 0.5;
		point.scale.y = 0.5; 
		
		point.color.r = 0.8f;
		point.color.g = 0.3f;
		point.color.b = 0.5f;
		point.color.a = 0.7f;

		geometry_msgs::Point p;
		p.x = pt.x();
		p.y = pt.y();
		p.z = 0.0;
		
		point.points.push_back(p);
		show_test_point.publish(point);
		
	}

	inline void showNewNode(const Eigen::Vector2d& pt) {
		visualization_msgs::Marker point;
		point.header.frame_id = "world";
		point.header.stamp = ros::Time::now();
		point.ns = "point";
		point.action = visualization_msgs::Marker::ADD;
		point.pose.orientation.w = 1.0;
		point.id = 0;
		point.type = visualization_msgs::Marker::POINTS;
		
		point.scale.x = 0.5;
		point.scale.y = 0.5; 
		
		point.color.r = 0.0f;
		point.color.g = 0.8f;
		point.color.b = 0.0f;
		point.color.a = 0.8f;

		geometry_msgs::Point p;
		p.x = pt.x();
		p.y = pt.y();
		p.z = 0.0;
		
		point.points.push_back(p);
		show_new_node.publish(point);

	}
	
	inline void toShowTree(const std::unique_ptr<showTree>& st) {
		visualization_msgs::Marker points, lines;
		points.header.frame_id = lines.header.frame_id = "world";
		points.header.stamp = lines.header.stamp = ros::Time::now();
		points.ns = lines.ns = "tree_visualization";
		points.action = lines.action = visualization_msgs::Marker::ADD;
		points.pose.orientation.w = lines.pose.orientation.w = 1.0;
		points.id = 0;
		lines.id = 1;
		points.type = visualization_msgs::Marker::POINTS;
		lines.type = visualization_msgs::Marker::LINE_LIST;
		points.scale.x = points.scale.y = 0.3;
		lines.scale.x = 0.1;
		points.color.r = points.color.g = points.color.b = lines.color.r = lines.color.g = lines.color.b = 0.0;
		points.color.a = lines.color.a = 1.0;

		// traverse Tree
		traverseTree(st, points, lines);
		
		visualization_msgs::MarkerArray marker_array;
		marker_array.markers.push_back(points);
		marker_array.markers.push_back(lines);
		
		show_tree.publish(marker_array);
	}

private:
	inline void traverseTree(const std::unique_ptr<showTree>& node, visualization_msgs::Marker& points, visualization_msgs::Marker& lines) {
		geometry_msgs::Point p;
		p.x = node->pt.x();
		p.y = node->pt.y();
		p.z = 0.0;
		points.points.push_back(p);
		
		for(const auto& child : node->children) {
			geometry_msgs::Point start = p;
			geometry_msgs::Point end;
			end.x = child->pt.x();
			end.y = child->pt.y();
			end.z = 0.0;
			lines.points.push_back(start);
			lines.points.push_back(end);
			
			traverseTree(child, points, lines);
		}
	} 

	inline visualization_msgs::Marker createCircleMarker(int id, double x, double y, double r) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "world";
		marker.header.stamp = ros::Time::now();
		marker.ns = "circle_makers";
		marker.id = id;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = -0.1;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 2 * r;// set color
		marker.scale.y = 2 * r;
		marker.scale.z = 0.01;
		marker.color.r = cr / 255.0;
		marker.color.g = cg / 255.0;
		marker.color.b = cb / 255.0;
		marker.color.a = 0.7;
		return marker;
	}


private:

	ros::NodeHandle nh_;
	ros::Publisher show_path_pub, show_circle_pub, show_chord_pub;
	ros::Publisher show_random_node, show_new_node, show_tree, show_goal, show_test_point;
	
	double cr, cg, cb;
};

#endif
