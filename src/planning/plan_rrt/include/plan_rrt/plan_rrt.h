
#ifndef _PLAN_RRT_H_
#define _PLAN_RRT_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <plan_env/grid_map.h>

#include "show_info_rrt.h"

#include <Eigen/Eigen>
#include <thread>
#include <vector>
#include <memory>
#include <random>
#include <queue>
#include <functional>
#include <cmath>

struct Node {
    Eigen::Vector2i pt;
    double r;
};

struct PointPair {
    Eigen::Vector2d p;
    Eigen::Vector2d p1;
    Eigen::Vector2d p2;
};

struct Tree {
    Node n;
    std::vector<std::shared_ptr<Tree>> children;
    
    // used for getPath() 
    std::shared_ptr<Tree> parent_node;
    
    // two constructor functions
    Tree(Node node) : n(node), parent_node(nullptr) { children.clear(); }
    Tree() : n(), parent_node(nullptr) { children.clear(); }
    
};

using TreePtr = std::shared_ptr<Tree>;

class FormationRRT {

public:
    FormationRRT();
    FormationRRT(ros::NodeHandle& nh, const ros::NodeHandle& show_nh = ros::NodeHandle("~"));
    void initGridMap(GridMap::Ptr occ_map, Eigen::Vector2d map_size);
    void RRTSearchAndGetSimplePath(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);
    double getReferenceWidth(double dist);
    Eigen::Vector2d getPathPoint(double dist);
    void setRRTDynaParam(double d, double v, double w, int in);
    std::vector<Eigen::Vector3d> getPathResource();

    typedef std::shared_ptr<FormationRRT> Ptr;
private:
    void SaveResource(TreePtr last_tree);
    std::shared_ptr<Tree> RRTSearch(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);
    bool convertToIndexAndAdjustStartEndPoints(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt, Eigen::Vector2i &start_idx, Eigen::Vector2i &end_idx);
    inline bool Coord2Index(const Eigen::Vector2d &pt, Eigen::Vector2i &idx) const;
    inline Eigen::Vector2d Index2Coord(const Eigen::Vector2i &index) const;
    inline bool checkOccupancy(const Eigen::Vector2d &pos) { return (bool) grid_map_->getInflateOccupancy(pos); }
    inline double getDistance(const Eigen::Vector2i &idx);
    Eigen::Vector2i randomPoint(const Eigen::Vector2i &min_idx, const Eigen::Vector2i &max_idx);
    TreePtr Nearest(std::shared_ptr<Tree> tr, Eigen::Vector2i p_rand);
    std::vector<TreePtr> NearConnect(std::shared_ptr<Tree> tr, Node node);
    std::priority_queue<TreePtr, std::vector<TreePtr>, std::function<bool(TreePtr, TreePtr)>> sortToTree(std::shared_ptr<Tree> tr, Eigen::Vector2i pt);
    Node TubeSteer(Node rand_n, Node nearest_n);

    void Rewire(TreePtr new_tree, std::vector<TreePtr>& near_trees);
    double Score(Node n1, Node n2);
    bool checkObstacleInLine(Node n1, Node n2);
    bool checkObstacleFree(Eigen::Vector2d pt);
    double Cost(TreePtr t);
    double calculateIntersectionArea(double r1, double r2, Eigen::Vector2i center1, Eigen::Vector2i center2);
    PointPair findCircleIntersections(Node n1, Node n2);
private:
    // [show]
    std::unique_ptr<showTree> copyTree(const TreePtr& root, int& cnt);

private:
    ros::NodeHandle nh_;
    
    ros::Timer t1;

    Eigen::Vector2d cur_pos_;

    GridMap::Ptr grid_map_;
    Eigen::Vector2i POOL_SIZE_, CENTER_IDX_;
    Eigen::Vector2d map_size_;
    
    Eigen::Vector2d center_;
    double step_size_, inv_step_size_;

    std::shared_ptr<Tree> T_ptr_;

    // RRT Param
    int iter_num; // the number of iterations
    double goal_tol; // goal tolerance
    double r_min;// min distance to obs
    double steerMaxStep;
    bool is_normal_rrt;
    //  double d;
    double rho_d;
    double rho_v;
    double sigma_v;
    double var;
    double rho_w;
    double desire_width;

    bool has_path_to_goal;
    double cur_min_dist_to_goal;

    Eigen::Vector2d start_pt_, end_pt_;

    std::vector<Node> path;
    std::vector<PointPair> line_pts;

    // debug
    std::shared_ptr<ShowInfoRRT> sir;
    double test_min_goal_dis;
    
};


inline double FormationRRT::getDistance(const Eigen::Vector2i &idx) {
    return grid_map_->getDistance(Index2Coord(idx));
}

inline bool FormationRRT::Coord2Index(const Eigen::Vector2d &pt, Eigen::Vector2i &idx) const {
    idx = (pt * inv_step_size_ + Eigen::Vector2d(0.5, 0.5)).cast<int>() + CENTER_IDX_;

    if(idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1)) {
        ROS_ERROR("[FormationRRT]:Ran out of pool, index=( %d, %d )", idx(0), idx(1));
        return false;
    }
    
    return true;
}

inline Eigen::Vector2d FormationRRT::Index2Coord(const Eigen::Vector2i &index) const {
    
    return (index - CENTER_IDX_).cast<double>() * step_size_;

}
#endif
