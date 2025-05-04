#include <plan_rrt/plan_rrt.h>
#include <stack>
#include <algorithm>
#include <limits>

// temp timer
#include <string>
#include <chrono>

class Timer {
public:
	struct TimePoint {
		std::string name;
		std::chrono::high_resolution_clock::time_point time;
	};

	void mark(const std::string& pointName) {
		points.push_back({pointName, std::chrono::high_resolution_clock::now()});
	}
	
	void print() {
		if(points.size() < 2) return;
		
		auto total = points.back().time - points.front().time;
		std::cout <<"\n======== Timing report =======\n";
		for(size_t i=1; i < points.size(); ++i) {
			auto duration = points[i].time - points[i - 1].time;
			std::cout << points[i - 1].name << " --> " << points[i].name << ": "
					  << std::chrono::duration_cast<std::chrono::microseconds>(duration).count()
					  << "us\n";
		}
		
		std::cout << "Total duration: "
				  << std::chrono::duration_cast<std::chrono::milliseconds>(total).count()
				  << "ms\n";
	}
private:
	std::vector<TimePoint> points;
};

FormationRRT::FormationRRT() : nh_("~") {

}

FormationRRT::FormationRRT(ros::NodeHandle& nh, const ros::NodeHandle& show_nh) {
	nh_ = nh;
	nh_.param("frm_rrt/iter_num", iter_num, -1);
	nh_.param("frm_rrt/goal_tolerance", goal_tol, -1.0);
	nh_.param("frm_rrt/min_dis_to_obs", r_min, -1.0);

	nh_.param("frm_rrt/rho_d", rho_d, -1.0);
	nh_.param("frm_rrt/rho_v", rho_v, -1.0);
	nh_.param("frm_rrt/sigma_v", sigma_v, -0.1);
	nh_.param("frm_rrt/var", var, -1.0);
	nh_.param("frm_rrt/desire_width", desire_width, -1.0);
	nh_.param("frm_rrt/rho_w", rho_w, -1.0);
	nh_.param("frm_rrt/steerMaxStep", steerMaxStep, 0.5);
	
	sir = std::make_shared<ShowInfoRRT>(show_nh);
	
	test_min_goal_dis = 20.0;
	
	has_path_to_goal = false;
	cur_min_dist_to_goal = std::numeric_limits<double>::max();

	is_normal_rrt = false;
}

// public method
void FormationRRT::initGridMap(GridMap::Ptr occ_map, Eigen::Vector2d map_size) {

	map_size_ = map_size;
	Eigen::Vector2d pool_size = map_size / occ_map->getResolution();
	
	POOL_SIZE_ = pool_size.cast<int>();
	CENTER_IDX_ = POOL_SIZE_ / 2;
	
	grid_map_ = occ_map;

	step_size_ = grid_map_->getResolution();
	inv_step_size_ = 1 / step_size_;
}

// public method
void FormationRRT::RRTSearchAndGetSimplePath(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt) {

	// clear Tree root
	T_ptr_ = nullptr;
	start_pt_ = start_pt;
	end_pt_ = end_pt;
	has_path_to_goal = false;
	cur_min_dist_to_goal = std::numeric_limits<double>::max();

	sir->showGoal(end_pt);

	std::vector<Eigen::Vector2i> path;
	std::shared_ptr<Tree> success_node = RRTSearch(start_pt, end_pt);
	if(success_node != nullptr) {
		SaveResource(success_node);
	} else {
		ROS_ERROR("[RRT path search]:Not found path");
	}
}


// save path point circle and xianchang
void FormationRRT::SaveResource(TreePtr last_tree) {

	path.clear();
	while(last_tree != nullptr) {
		path.push_back(last_tree->n);
		last_tree = last_tree->parent_node;
	}
	std::reverse(path.begin(), path.end());

	line_pts.resize(path.size() - 1);
	
	for(int i = 0; i < path.size() - 1; i++) 
		line_pts[i] = findCircleIntersections(path[i], path[i + 1]);

	for(int i = 0; i < line_pts.size() - 1; i++) {
		Eigen::Vector2d line_p1 = line_pts[i].p;
		Eigen::Vector2d line_p2 = line_pts[i + 1].p;
		
		double line_A = line_p2.y() - line_p1.y();
		double line_B = line_p1.x() - line_p2.x();
		double line_C = line_p2.x() * line_p1.y() - line_p1.x() * line_p2.y();

		double d1 = line_A * line_pts[i].p1.x() + line_B * line_pts[i].p1.y() + line_C;
		double d2 = line_A * line_pts[i + 1].p1.x() + line_B * line_pts[i + 1].p1.y() + line_C;

		if(d1 * d2 < 0) {
			Eigen::Vector2d swap = line_pts[i + 1].p1;
			line_pts[i + 1].p1 = line_pts[i + 1].p2;
			line_pts[i + 1].p2 = swap;
		}
	}

	std::vector<Eigen::Vector2d> path_2d;
	std::vector<Eigen::Vector3d> circles;
	std::vector<Eigen::Vector2d> lines;
	for(int i = 0; i < path.size(); i++) {
		path_2d.push_back(Index2Coord(path[i].pt));
		Eigen::Vector3d tmp_circle;
		tmp_circle << path_2d.back().x(), path_2d.back().y(), path[i].r;
		circles.push_back(tmp_circle);
	}

	sir->showPath(path_2d);

	sir->showCircle(circles);

	for(int i = 0; i < line_pts.size(); i++) {
		lines.push_back(line_pts[i].p1);
		lines.push_back(line_pts[i].p2);
	}
	
	sir->showChord(lines);
}

// special for experiment
std::vector<Eigen::Vector3d> FormationRRT::getPathResource() {
	std::vector<Eigen::Vector3d> circles;
	for(int i = 0; i < path.size(); i++) {
		Eigen::Vector2d path_2d = Index2Coord(path[i].pt);
		Eigen::Vector3d tmp_circle;
		tmp_circle << path_2d.x(), path_2d.y(), path[i].r;
		circles.push_back(tmp_circle);
	}
	return circles;
}

PointPair FormationRRT::findCircleIntersections(Node n1, Node n2) {
	PointPair pp; 

	double r1 = n1.r, r2 = n2.r;
	Eigen::Vector2d c1 = Index2Coord(n1.pt), c2 = Index2Coord(n2.pt);
	double d = (c1 - c2).norm();

	double a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
	double h = std::sqrt(r1 * r1 - a * a);
	
	Eigen::Vector2d p = c1 + a * (c2 - c1) / d;

	Eigen::Vector2d perp(-(c2.y() - c1.y()), c2.x() - c1.x());
	perp.normalize();

	pp.p << p;
	pp.p1 << p + h * perp;
	pp.p2 << p - h * perp;
	return pp;
}

//the reference width is obtained by interpolation 
double FormationRRT::getReferenceWidth(double dist) {
	// create new path
	std::vector<Eigen::Vector2d> new_path;
	new_path.push_back(Index2Coord(path.front().pt));
	for(int i = 0; i < path.size(); i++) 
		new_path.push_back(line_pts[i].p);
	new_path.push_back(Index2Coord(path.back().pt));

	// traverse new_path, the distance between point(find) and start point equal to dist 
	double dist_sum = 0.0;
	int cur_idx = -1;
	Eigen::Vector2d find_pt;
	for(int i = 0; i < new_path.size() - 1; i++) {
		dist_sum += (new_path[i + 1] - new_path[i]).norm();
		if(dist < dist_sum) {
			Eigen::Vector2d start_pt = new_path[i];
			Eigen::Vector2d end_pt = new_path[i + 1];
			double dist_sum_before = dist_sum - (end_pt - start_pt).norm();
			double dist_delta = dist - dist_sum_before;
			
			Eigen::Vector2d left_pt = start_pt;
			Eigen::Vector2d right_pt = end_pt;

			double mid_dist;
			Eigen::Vector2d mid_pt;

			do {
				mid_pt = (left_pt + right_pt) / 2;
				mid_dist = (mid_pt - start_pt).norm();
				if(mid_dist > dist_delta) {
					right_pt = mid_pt;
				} else {
					left_pt = mid_pt;
				}
			} while(abs(mid_dist - dist_delta) > 0.06);
			find_pt = mid_pt;
			cur_idx = i;
			break;
		}
	}	
	
	if(cur_idx == -1) {
		ROS_WARN("[getWidthInRRT]:the distance is big!");
		return path.back().r;
	}
	
	// In new_path the beginning, the middle, and the end are treated differently.

	double width;
	if(cur_idx == 0) {
		double proportion = (new_path[1] - find_pt).norm() / (new_path[1] - new_path[0]).norm();
		double width_a = (line_pts[0].p1 - line_pts[0].p2).norm();
		double width_b = path[0].r;
		width =  width_a + proportion * (width_b - width_a);

	} else if(cur_idx == (new_path.size() - 2)) { 

		double proportion = (line_pts.back().p - find_pt).norm() / (line_pts.back().p - new_path.back()).norm();
		double width_a = (line_pts.back().p1 - line_pts.back().p2).norm();
		double width_b = path.back().r;
		width = width_a + proportion * (width_b - width_a);

	} else {

		double proportion = (find_pt - new_path[cur_idx]).norm() / (new_path[cur_idx] - new_path[cur_idx + 1]).norm();
		Eigen::Vector2d x1 = line_pts[cur_idx - 1].p1 + proportion * (line_pts[cur_idx].p1 - line_pts[cur_idx - 1].p1);
		Eigen::Vector2d x2 = line_pts[cur_idx - 1].p2 + proportion * (line_pts[cur_idx].p2 - line_pts[cur_idx - 1].p2);
		width = (x1 - x2).norm();

	}
	
	// ROS_WARN_STREAM("[debug] the distance width:" << width);
	return width;
}

Eigen::Vector2d FormationRRT::getPathPoint(double dist) {
	double dist_sum = 0.0;
	for (int i = 0; i < path.size() - 1; i++) {
		dist_sum += (Index2Coord(path[i + 1].pt) - Index2Coord(path[i].pt)).norm();
		if (dist < dist_sum) {
			double dist_before = dist_sum - (Index2Coord(path[i + 1].pt) - Index2Coord(path[i].pt)).norm();
			double delta_dist = dist - dist_before;
			Eigen::Vector2d start_pt = Index2Coord(path[i].pt);
			Eigen::Vector2d end_pt = Index2Coord(path[i + 1].pt);
			
			int check_num = ceil((end_pt - start_pt).norm() / 0.01);
			for(int j = 0; j <= check_num; j++) {
				double alpha = double(1.0 / check_num) * j;
				Eigen::Vector2d check_pt = (1 - alpha) * start_pt + alpha * end_pt;
				
				double curr_dist = (check_pt - start_pt).norm();
				if(abs(curr_dist - delta_dist) < 0.06) {
					return check_pt;
				}
			}
		}
	}
	
	ROS_WARN("rrt path less than preset distance!!! return the last element of rrt path");
	return path.back().pt.cast<double>();
}

// [show]: all tree node
std::unique_ptr<showTree> FormationRRT::copyTree(const TreePtr& root, int& cnt) {
	if(!root) {
		return nullptr;
	}
	
	cnt++;
	auto newNode = std::make_unique<showTree>(Index2Coord(root->n.pt));
	
	for(const auto& child : root->children) {
		newNode->children.push_back(std::move(copyTree(child, cnt)));
	}
	return newNode;
}

// Formation RRT Algorithm
std::shared_ptr<Tree> FormationRRT::RRTSearch(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt) {

	// start_pt and end_pt are converted in grid map
	std::shared_ptr<Tree> success_node; // last node
	std::shared_ptr<Tree> min_dist_node; // the node has min distance to goal 

	center_ = (start_pt + end_pt) / 2;

	Eigen::Vector2i start_idx, end_idx;
	if(!convertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx)) {

		ROS_ERROR("Unable to handle the start or end point, please reset end point");

		return success_node;

	}

	grid_map_->updateESDFMap(map_size_);
	
	// start Algorithm
	// add start_idx to tree
	Node start_node;
	start_node.pt = start_idx;
	start_node.r = getDistance(start_idx);

	T_ptr_ = std::make_shared<Tree>(start_node);
	int i_num = iter_num;
	
	Timer tt;
	tt.mark("start");

	while( i_num-- > 0) {
		ROS_WARN("iter_num:%d", i_num);
	
		Node random_node;
		random_node.pt = randomPoint({0,0}, POOL_SIZE_);
		random_node.r = getDistance(random_node.pt);
			
		if(random_node.r <= 0.2) {
			i_num++;
			continue;
		}

		// [show]:random node
		Eigen::Vector2d show_rdm = Index2Coord(random_node.pt);
		ROS_INFO("random node: (%f, %f), r = %f", show_rdm.x(), show_rdm.y(), random_node.r);
		sir->showRandomNode(show_rdm);

		std::shared_ptr<Tree> nearest_tree;
		nearest_tree = Nearest(T_ptr_, random_node.pt);
		nearest_tree->n.r = getDistance(nearest_tree->n.pt);

		Eigen::Vector2d show_nrt = Index2Coord(nearest_tree->n.pt);
		ROS_INFO("nearest node: (%f, %f), r = %f", show_nrt.x(), show_nrt.y(), nearest_tree->n.r);
		
		sir->showNewNode(show_nrt);

		Node new_node;
		new_node = TubeSteer(random_node, nearest_tree->n);
		if(new_node.r == -max(map_size_.x(), map_size_.y())) {
			i_num++;
			continue;
		}
			
		Eigen::Vector2d show_new = Index2Coord(new_node.pt);;
		ROS_INFO("new node: (%f, %f), r: %f", show_new.x(), show_new.y(), new_node.r);

		std::vector<Eigen::Vector3d> cls;
		Eigen::Vector3d c;
		Eigen::Vector2d pos = Index2Coord(new_node.pt);
		c << pos.x(), pos.y(), new_node.r;
		cls.push_back(c);
		pos = Index2Coord(nearest_tree->n.pt);
		c << pos.x(), pos.y(), nearest_tree->n.r;
		cls.push_back(c);
		
		if( new_node.r > r_min || is_normal_rrt) {
			std::vector<TreePtr> near_trees = NearConnect(T_ptr_, new_node);
			std::cout << "near tree number:" << near_trees.size() << std::endl;
			TreePtr min_tree;
			vector<double> costs;
			
			for (int i = 0; i < near_trees.size(); i++) {
				TreePtr near_tree = near_trees[i];
				double cur_cost = Cost(near_tree) + Score(near_tree->n, new_node);
				costs.push_back(cur_cost);
				// std::cout << "tree node:" << Index2Coord(near_tree->n.pt).transpose() << " cost:" << costs.back() << std::endl;
			}
			
			while(costs.size() > 0) {
				auto minIt = std::min_element(costs.begin(), costs.end());
				int min_idx = static_cast<int>(std::distance(costs.begin(), minIt));
				TreePtr near_tree = near_trees[min_idx];
				//std::cout << "nearest tree pos:" << Index2Coord(near_tree->n.pt).transpose() << " cost:" << costs[min_idx] << std::endl;
				if(checkObstacleInLine(near_tree->n, new_node)) {
					min_tree = near_tree;
					break;
				}

				
				costs[min_idx] = costs.back();
				costs.pop_back();
			}

			if (costs.size() == 0) {
				ROS_WARN("cost is zero");
				i_num++;
				continue;
			}

			TreePtr new_tree = std::make_shared<Tree>(new_node);
			new_tree->parent_node = min_tree;
			min_tree->children.push_back(new_tree);
			ROS_INFO_STREAM("new node.parent_node:" << Index2Coord(min_tree->n.pt).transpose());

			if (has_path_to_goal) {
				double dist1 = Cost(new_tree) + (Index2Coord(new_tree->n.pt) - Index2Coord(end_idx)).norm();
				double dist2 = Cost(success_node) + (Index2Coord(success_node->n.pt) - Index2Coord(end_idx)).norm();

				if (is_normal_rrt) {
					if ( dist1 < dist2 && dist1 < goal_tol ) 
						success_node = new_tree;
				}
				else {
					if ( dist1 < dist2 && (dist1 < goal_tol || dist1 < new_tree->n.r) ) 
						success_node = new_tree;
				}

			} else {
				double to_goal_dis = (Index2Coord(new_tree->n.pt) - Index2Coord(end_idx)).norm();
				
				// this block is show info
				if (to_goal_dis < cur_min_dist_to_goal) {
					cur_min_dist_to_goal =  to_goal_dis;
					min_dist_node = new_tree;	
				}
				ROS_INFO("the min distance to goal is %f", cur_min_dist_to_goal);
				
				if (is_normal_rrt) {
					if (to_goal_dis < goal_tol) {
						has_path_to_goal = true;
						success_node = new_tree;
					}
				} 
				else {
					if (to_goal_dis < goal_tol || to_goal_dis < new_tree->n.r) {
						has_path_to_goal = true;
						success_node = new_tree;
					}
				}

			}

			// rewire
			Rewire(new_tree, near_trees);
		}
		
		// [show]: all tree node
		std::unique_ptr<showTree> st;
		int cnt = 0;
		st = copyTree(T_ptr_, cnt);
		std::cout << "[Debug] tree size:" << cnt << std::endl;
		sir->toShowTree(std::move(st));
	}

	tt.mark("all finish");
	tt.print();
	
	Node end_node;
	end_node.pt = end_idx;
	end_node.r = 0.2;
	TreePtr end_tree = std::make_shared<Tree>(end_node);

	if (has_path_to_goal) {
		ROS_INFO("arrived the preset goal!");
		end_tree->parent_node = success_node;
		success_node->children.push_back(end_tree);
		return end_tree;
	} else {
		ROS_WARN_STREAM("iteration count has been exceeded. the node nearest goal is " << Index2Coord(min_dist_node->n.pt).transpose() << " And the distance to goal is " << cur_min_dist_to_goal);
		if (checkObstacleInLine(min_dist_node->n, end_node)) {
			end_tree->parent_node = min_dist_node;
			min_dist_node->children.push_back(end_tree);
			return end_tree;
		} else {
			return min_dist_node;
		}
	}
}

void FormationRRT::Rewire(TreePtr new_tree, std::vector<TreePtr>& near_trees) {
	for (int i = 0; i <near_trees.size(); i++) {
		TreePtr near_tree = near_trees[i];
		double cur_cost = Cost(new_tree) + Score(near_tree->n, new_tree->n);
		if (cur_cost < Cost(near_tree)) {
			if (checkObstacleInLine(near_tree->n, new_tree->n)) {
				ROS_ERROR("need Rewire!");
				near_tree->parent_node = new_tree;
			}
		}
	}

}

double FormationRRT::Cost(TreePtr t) {

	double c = 0.0;
	if (t->parent_node == nullptr)
		return c;

	TreePtr cur_tree = t;
	TreePtr pre_tree;

	while ( cur_tree->parent_node != nullptr ) {
		
		pre_tree = cur_tree->parent_node;

		c += Score(pre_tree->n, cur_tree->n);

		cur_tree = pre_tree;
	}
	
	return c;

}

void FormationRRT::setRRTDynaParam(double d, double v, double w, int in) {
	rho_d = d;
	rho_v = v;
	rho_w = w;
	iter_num = in;
	std::cout << "rho_d, rho_v, rho_w, iter_num:" << rho_d << " " << rho_v << " " << rho_w << " " << iter_num << std::endl;

	if (rho_v == 0.0 && rho_w == 0.0) {
		is_normal_rrt = true;
		ROS_WARN("enter normal rrt");
	}
	else {
		is_normal_rrt = false;
		ROS_WARN("out of normal rrt");
	}
}

double FormationRRT::Score(Node n1, Node n2) {
	
	double sc1 = rho_d / (end_pt_ - start_pt_).norm() * (Index2Coord(n1.pt) - Index2Coord(n2.pt)).norm();
	
	double item_v = calculateIntersectionArea(n1.r, n2.r, n1.pt, n2.pt) / sigma_v + var;
	double sc2 = rho_v * pow(item_v, -1);

	// consider width
	double sc3 = rho_w * abs( n1.r + n2.r - 2 * desire_width) / 2 * desire_width;
	// sc3 += rho_w * abs(n1.r - n2.r) / max(n1.r, n2.r);
	
	double sum = sc1 + sc2 + sc3;

	return sum;
}

double FormationRRT::calculateIntersectionArea(double r1, double r2, Eigen::Vector2i center1, Eigen::Vector2i center2) {

	double d = (Index2Coord(center1) - Index2Coord(center2)).norm();

	if (d >= r1 + r2) return 0.0;
	
	if (d <= std::abs(r1 - r2)) {
		double smallerRadius = std::min(r1, r2);
		return M_PI * smallerRadius * smallerRadius;
	}

	double part1 = r1 * r1 * std::acos((d * d + r1 * r1 - r2 * r2) / (2 * d * r1));
	double part2 = r2 * r2 * std::acos((d * d + r2 * r2 - r1 * r1) / (2 * d * r2));
	double part3 = 0.5 * std::sqrt((-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2));

	return part1 + part2 + part3;
}

// safe : return true; 
// unsafe : return false;
bool FormationRRT::checkObstacleInLine(Node n1, Node n2) {
	Eigen::Vector2d start_pt = Index2Coord(n1.pt);
	bool is_safe = true;
	Eigen::Vector2d end_pt = Index2Coord(n2.pt);
	int check_num = ceil((end_pt - start_pt).norm() / 0.07);
	for (int i = 0; i <= check_num; i++) {
		double alpha = double(1.0 / check_num) * i;
		Eigen::Vector2d check_pt = (1 - alpha) * start_pt + alpha * end_pt;
		
		if (!checkObstacleFree(check_pt)) {
			is_safe = false;
			break;
		}
	}
	
	return is_safe;
}

std::shared_ptr<Tree> FormationRRT::Nearest(std::shared_ptr<Tree> tr, Eigen::Vector2i p_rand) {
	// find nearest sphere x_nearest
	auto sorted_queue = sortToTree(tr, p_rand);

	// debug
	if(sorted_queue.size() > 1) {
		std::cout << "base point: " << Index2Coord(p_rand).transpose() << std::endl;
		auto test_queue = sorted_queue;
		std::shared_ptr<Tree> t1 = test_queue.top();
		test_queue.pop();
		std::shared_ptr<Tree> t2 = test_queue.top();
		std::cout << "queue 1: " << Index2Coord(t1->n.pt).transpose() << " queue 2: " << Index2Coord(t2->n.pt).transpose() << std::endl;
	}

	return sorted_queue.top();
}

std::vector<TreePtr> FormationRRT::NearConnect(std::shared_ptr<Tree> tr, Node node) {
	
	std::vector<TreePtr> res;

	auto sorted_queue = sortToTree(tr, node.pt);
	TreePtr near_tree;
	while (!sorted_queue.empty()) { 
		near_tree = sorted_queue.top();
		sorted_queue.pop();
		near_tree->n.r = getDistance(near_tree->n.pt);
		Node near_node = near_tree->n;

		// guaranteed two intersection
		double dist = (Index2Coord(near_node.pt) - Index2Coord(node.pt)).norm();
		if (near_node.r + node.r > dist) { // add search radius
			res.push_back(near_tree);
		}	
	}

	return std::move(res);

}

// add member method 
// @param tr a Tree
// @param pt a predefined point
// @return sorted into a priority_queue based on the distance to pt
std::priority_queue<std::shared_ptr<Tree>, std::vector<std::shared_ptr<Tree>>, std::function<bool(std::shared_ptr<Tree>, std::shared_ptr<Tree>)>> FormationRRT::sortToTree(std::shared_ptr<Tree> tr, Eigen::Vector2i pt) {

	std::function<bool(std::shared_ptr<Tree>, std::shared_ptr<Tree>)> cmp = [&pt, this](std::shared_ptr<Tree> t1, std::shared_ptr<Tree> t2) -> bool {
		Eigen::Vector2i p1 = t1->n.pt;
		Eigen::Vector2i p2 = t2->n.pt;
		double dist1 = (this->Index2Coord(p1) - this->Index2Coord(pt)).norm();
		double dist2 = (this->Index2Coord(p2) - this->Index2Coord(pt)).norm();
		return dist1 > dist2;
	};

	std::priority_queue<std::shared_ptr<Tree>, std::vector<std::shared_ptr<Tree>>, decltype(cmp)> q(cmp);

	if( tr->children.size() == 0 ) {
		q.push(tr);
		return q;
	}

	//DFS
	std::stack<std::shared_ptr<Tree>> tree_stk;
	tree_stk.push(tr);

	while (!tree_stk.empty() && q.size() < iter_num / 2) { // 100 is free value

		auto cur_elem = tree_stk.top();
		tree_stk.pop();
		
		q.push(cur_elem);
	
		for(auto elem_ptr : cur_elem->children) {
			tree_stk.push(elem_ptr);
		}
	}
	
	return std::move(q);
}

Node FormationRRT::TubeSteer(Node rand_n, Node nearest_n) {

	Eigen::Vector2d rand_n_d = Index2Coord(rand_n.pt);
	Eigen::Vector2d nearest_n_d = Index2Coord(nearest_n.pt);
	double dist = (rand_n_d - nearest_n_d).norm();
	Eigen::Vector2d unit_v = (Index2Coord(rand_n.pt) - Index2Coord(nearest_n.pt)) / dist;

	if(is_normal_rrt) {
		ROS_WARN("[test]: in normal rrt");
		Node new_n;
		if(dist <= steerMaxStep)
			new_n = rand_n;
		else {
			Eigen::Vector2d temp_pt = Index2Coord(nearest_n.pt) + steerMaxStep * unit_v;
			Coord2Index(temp_pt, new_n.pt);
			new_n.r = getDistance(new_n.pt);
		}
		if(!checkObstacleInLine(nearest_n, new_n)) 
			new_n.r = -max(map_size_.x(), map_size_.y());
		return new_n;

	} else {

		Node new_n = rand_n;
		while ( nearest_n.r + new_n.r < dist ) { 
			dist = nearest_n.r > new_n.r ? nearest_n.r : new_n.r;
			Eigen::Vector2d temp_pt = Index2Coord(nearest_n.pt) + 0.6 * dist * unit_v;

			Coord2Index(temp_pt, new_n.pt);
			new_n.r = getDistance(new_n.pt);
			
		}

		return new_n;

	}
}

bool FormationRRT::convertToIndexAndAdjustStartEndPoints(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt, Eigen::Vector2i &start_idx, Eigen::Vector2i &end_idx) {

	if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
		return false;

	if (!checkObstacleFree(start_pt)) {
		ROS_WARN("start point is within the obstacle !");
		
		int count = 0;
		do {

			Eigen::Vector2d dist_grad;
			grid_map_->evaluateFirstGrad(start_pt, dist_grad);
			start_pt += grid_map_->getResolution() * dist_grad;
			count++;

		} while (count == 2);
		
		Coord2Index(start_pt, start_idx);
	}

	if (!checkObstacleFree(end_pt)) {
		ROS_WARN("end point is within the obstacle !");
		
		int count = 0;
		do {

			Eigen::Vector2d dist_grad;
			grid_map_->evaluateFirstGrad(start_pt, dist_grad);
			end_pt += grid_map_->getResolution() * dist_grad;

		} while (count == 2);

		Coord2Index(end_pt, end_idx);
	}

	if (checkObstacleFree(start_pt) && checkObstacleFree(end_pt)) 
		return true;
	else
		return false;
}

bool FormationRRT::checkObstacleFree(Eigen::Vector2d pt) {

		const double dist_to_obs = 0.1;
		if (grid_map_->getDistance(pt) < dist_to_obs) {
			return false;
		}
		else {
			return true;
		}
}

Eigen::Vector2i FormationRRT::randomPoint(const Eigen::Vector2i &min_idx, const Eigen::Vector2i &max_idx) {
	auto randomInt = [](int min, int max){
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<> dis(min, max);
		return dis(gen);
	};

	Eigen::Vector2i res_pt;
	res_pt << randomInt(min_idx[0], max_idx[0]), 
			 randomInt(min_idx[1], max_idx[1]);
	return res_pt;
}
