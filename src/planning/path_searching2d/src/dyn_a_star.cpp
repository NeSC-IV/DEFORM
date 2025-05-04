#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            delete GridNodeMap_[i][j];
}


void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector2d map_size)
{

    Eigen::Vector2d middle_pool_size=map_size/occ_map->getResolution();
    Eigen::Vector2i pool_size(middle_pool_size.x(),middle_pool_size.y());

    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr *[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr [POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNode;
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getDiagHeu2d(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    double h = 0.0;
    int diag = min(dx, dy);
    dx -= diag;
    dy -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(2.0) * diag + dy;
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(2.0) * diag + dx;
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    return dx + dy ;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector2d start_pt, Vector2d end_pt, Vector2i &start_idx, Vector2i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        ROS_WARN("Start point is inside an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    Vector2d temp_end_pt=end_pt;
    if (checkOccupancy(Index2Coord(end_idx)))
    {
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }
    return true;
}

bool AStar::AstarSearch(const double step_size, Vector2d start_pt, Vector2d end_pt, bool use_esdf_check)
{
    ros::WallTime time_1 = ros::WallTime::now();
    ++rounds_;
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector2i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return false;
    }

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1))
        {
            ros::WallTime time_2 = ros::WallTime::now();
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET; 

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                {
                    if (dx == 0 && dy == 0 )
                        continue;

                    Vector2i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 )
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if(use_esdf_check){
                        if (checkOccupancy_esdf(Index2Coord(neighborPtr->index)))
                            continue;
                    } else {
                        if (checkOccupancy(Index2Coord(neighborPtr->index)))
                            continue;
                    }
                    
                    double static_cost = sqrt(dx * dx + dy * dy);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }

        ros::WallTime time_2 = ros::WallTime::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    ros::WallTime time_2 = ros::WallTime::now();
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    //if planning failure, return one point that start point
    gridPath_= retrievePath(startPtr);
    ROS_WARN("[No result]:Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return false;
}

vector<Vector2d> AStar::getPath()
{
    vector<Vector2d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}

vector<Vector2d> AStar::astarSearchAndGetSimplePath(const double step_size, Vector2d start_pt, Vector2d end_pt) {
    // call astar search and get the path

    ros::WallTime nowTime1 = ros::WallTime::now();

    vector<Vector2d> path;
    AstarSearch(step_size, start_pt, end_pt, true);
    path = getPath();

    if ((path[0] - start_pt).norm() > 0.8) {
        ROS_WARN("I don't know why, but only try A* again");
        AstarSearch(step_size, start_pt, end_pt, false);
        path = getPath();
    }

    path.push_back(end_pt);
    return path;
}

vector<Vector2d> AStar::refinePath(vector<Vector2d> path)
{
    bool is_show_debug = false;
    // generate the simple path
    vector<Vector2d> simple_path;
    int size = path.size();
    if (size == 2){
        ROS_WARN("the path only have two points");
        return path;
    }
    else if(size ==1)
    {
        ROS_WARN("planning failed!!!");
        return path;
    }
        
    int end_idx   = 1;
    Vector2d cut_start = path[0];
    simple_path.push_back(cut_start);

    bool finish = false;
    while (!finish) {
        for (int i = end_idx; i < size; i++){
            bool is_safe = true;
            Vector2d check_pt = path[i];
            int check_num = ceil((check_pt - cut_start).norm() / 0.01);
            double pts_dis=(check_pt - cut_start).norm();
            if(pts_dis>0.3)
                is_safe= false;
            // check collision
            for (int j=0; j<=check_num; j++){
                double alpha = double(1.0 / check_num) * j;
                Vector2d check_safe_pt = (1 - alpha) * cut_start + alpha * check_pt;
                if (checkOccupancy_esdf(check_safe_pt)){
                    is_safe = false;
                    break;
                }
            }
            
            if (is_safe && i == (size -1)){
                finish = true;
                simple_path.push_back(check_pt);
            }

            if (is_safe){
                continue;
            }
            else{
                end_idx = i;
                cut_start = path[end_idx-1];
                simple_path.push_back(cut_start);
            }
        }
    }

    // debug
    if (is_show_debug){
        cout << "[simple A* path] : --------- " << endl;
        int n1 = simple_path.size();
        cout << "simple A* path size : " << n1 << endl;
        for (int i=0; i<n1; i++)
            cout << simple_path[i].transpose() << endl;
    }
    

    // check the near points and delete it
    bool near_flag;
    do
    {
        near_flag = false;
        if (simple_path.size() <=2){
            near_flag = false;
            break;
        }

        int num_same_check = simple_path.size();
        for (int i=0; i<num_same_check-1; i++){
            double len = (simple_path[i+1] - simple_path[i]).norm();
            if (len < 0.3){
                simple_path.erase(simple_path.begin()+i+1);
                near_flag = true;
                break;
            }
        }
        
    } while (near_flag);

    // debug
    if (is_show_debug){
        cout << "[delete simple path] : --------- " << endl;
        int n2 = simple_path.size();
        cout << "delete simple path size : " << n2 << endl;
        for (int i=0; i<n2; i++)
            cout << simple_path[i].transpose() << endl;
    }
    
    // check the path and add a point if two of them are too far away
    bool too_long_flag;
    const double length_threshold = 3;
    int debug_num = 0;
    do
    {
        debug_num ++;
        too_long_flag = false;
        int num = simple_path.size();
        for (int i=0; i<num-1; i++){
            double leng = (simple_path[i+1] - simple_path[i]).norm();
            if (leng > length_threshold){
                Vector2d insert_point = (simple_path[i+1] + simple_path[i]) / 2;
                simple_path.insert(simple_path.begin()+i+1 ,insert_point);
                too_long_flag = true;
                break;
            }
        }
    } while (too_long_flag && debug_num < 10);

    // debug
    if (is_show_debug){
        cout << "[final simple path] : --------- " << endl;
        int n3 = simple_path.size();
        cout << "final simple path size : " << n3 << endl;
        for (int i=0; i<n3; i++)
            cout << simple_path[i].transpose() << endl;
    }

    ros::WallTime nowTime2=ros::WallTime::now();
//    ROS_WARN("[Planning total Time]:%f",(nowTime2-nowTime1).toSec());

    return simple_path;
}
