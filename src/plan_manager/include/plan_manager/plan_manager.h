#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <Eigen/Eigen>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>

#include <plan_manager/RotationVec.h>
#include <plan_manager/widthInfo.h>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <path_opti/path_opti.h>
#include <width_check/find_width.h>
#include <traj_opti/traj_opti.h>
#include <to_visulaize/toVisualize.h>
#include <ot_process/ot_client.h>
#include <plan_rrt/plan_rrt.h>

#include <chrono>
#include <utility>
#include <vector>
#include <fstream>
#include <sstream>
#include <queue>

class ComputeTiming {
public:

    ComputeTiming(std::string name) : dur{}, timing_name(std::move(name)) {
        data.clear();
    }

    void setFileName(std::string fn) {
        fn_ = std::move(fn);
    }

    void start_timing() {
        start = std::chrono::high_resolution_clock::now();
    }
    void stop_timing() {
        end = std::chrono::high_resolution_clock::now();
        dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        data.push_back(dur.count());
    }

    void printf_timing_info() {
        std::ostringstream oss;
        oss << "[" << timing_name << "]:" << dur.count() << " ms" << endl;
        cout << oss.str();

    }

    void save_data() {
        data_out.open(fn_, std::ios::out);

        for(auto elem : data)
            data_out << elem << std::endl;

        data_out.close();
    }

private:
    using time_type = decltype(std::chrono::high_resolution_clock::now());
    time_type start;
    time_type end;
    decltype(std::chrono::duration_cast<std::chrono::milliseconds>(end - start)) dur;
    std::string timing_name;

    std::string fn_;
    std::vector<long> data;
    std::ofstream data_out;

};

struct State
{
    Eigen::Vector2d pos;
    double yaw;
    Eigen::Vector2d vel;
    Eigen::Vector2d orient_vec;
};

class PlanManager
{
private:
    /*           flag      */
    enum FORMATION_EXEC_STATE
    {
        WAIT_ODOM_OR_TARGET,
        FRM_INIT_PATH,
        FRM_KEEP,
        FRM_DEFORM,
        AVOID_OBS,
        APPROACH_GOAL,
        EMERGENCY_STOP
    };

    enum FORMATION_TYPE
    {
        LINE,
        RECTANGLE,
        THREE,
        FOUR,
        FIVE,
    };

    enum TASK_TYPE
    {
        NAVIGATION = 1,
        DEFORM,
    };

//method
public:
    PlanManager();
    ~PlanManager();
private:
    void readParameters();
    void setDesiredShape(int type);
    std::vector<Eigen::Vector2d> generateFormationPositions(int robot_num, int one_row_num, double frm_gap);
    void changeFRMExecState(FORMATION_EXEC_STATE new_state);
    void execCallback(const ros::TimerEvent&);
    void subGlobalGoal(const geometry_msgs::PoseStampedConstPtr& msg);
    void trajPlanning();
    void trajPlanningDeform();
    bool trajController(Eigen::Vector2d goal);
    void pathPlanning(const ros::TimerEvent&);
    void setPathPoint(const geometry_msgs::PointConstPtr& msg);
    void findPointInPath(double forward_distance,Eigen::Vector2d& pt);
    bool widthHandle(const Eigen::Vector2d& start_pt,Eigen::Vector2d& end_pt);
    void widthHandleInExtend();
    void isReduceFRM();
    void isExtendFRM();
    void rotateAndShareFRM();
    void rotateAndShareDeform();
    void updateFRMInfo(const plan_manager::RotationVecConstPtr& msg) ;
    void FRMConsensus(Eigen::Vector2d& pos);
    void taskAllocation();
    void FRMBias(double& e);
    void calcDesireAngularVel(double& w,const Eigen::Vector2d& goal);
    void calcDesireLinearVel(double& v,const Eigen::Vector2d& goal);
    void pubVelCmd(const std::vector<double>& c);
    void updateOtherCallback(const ros::TimerEvent&);
    void findClosestPointOnPath(const Eigen::Vector2d& target, const std::vector<Eigen::Vector2d>& path);
    void updateAvoidModeCallback(const ros::TimerEvent&);
    void timeoutCallback(const ros::TimerEvent&);

//variable
private:

    ros::NodeHandle nh_,nh_pri_;
    ros::Timer execTimer;
    ros::Timer pathTimer;
    std::vector<ros::Subscriber> odoms_sub;
    ros::Publisher vel_pub;
    ros::Publisher path_point_pub;
    ros::Subscriber path_point_sub;
    ros::Subscriber goal_sub;
    ros::Publisher frm_info_pub;
    ros::Subscriber frm_info_sub;
    ros::Publisher width_pub;
    ros::Subscriber width_sub;
    ros::Publisher is_avoid_obs_pub_;
    std::vector<ros::Subscriber> is_avoid_obs_sub_;
    FORMATION_EXEC_STATE exec_state_;
    
    GridMap::Ptr grid_map_;
    AStar::Ptr aster_;
    FindMaxFreeRegion::Ptr find_region_;
    PathOptimizer::Ptr path_opti_;
    TrajectoryOptimizer::Ptr traj_opt_;
    OT_Transfer::Ptr ot_cli_;
    toVisualization::Ptr to_vis_;

    int fixed_id;
    int fixed_num;
    double formation_interval;
    double formation_width;
    int formation_type;
    int formation_min_type;
    int formation_max_type;
    int leader_id;
    int tb_id;
    int formation_max_num;
    double formation_yaw;
    int start_frm_type;
    std::vector<State> tb_states_;
    std::vector<int> realloc_;
    std::vector<double> all_width;

    Eigen::Vector2d global_goal;
    Eigen::Vector2d tb_global_goal;
    Eigen::Vector2d local_goal;
    Eigen::Vector2d leader_path_point;
    std::vector<Eigen::Vector2d> path_;
    double fwd_dist;
    double obs_clearance;

    bool has_odom;
    bool has_goal;
    bool first_start;
    bool do_allocation;

    std::vector<Eigen::Vector2d> swarm_des_origin;// original target formation
    std::vector<Eigen::Vector2d> swarm_des;// desired formation considering rotation
    std::vector<Eigen::Vector2d> swarm_alloc;// deisred formation considering task allocation
    std::vector<Eigen::Vector3d> pathInfo; // save global path
    std::vector<int> is_avoid_obs; // count the number of robots in obstacle avoidance mode

    // reocord other robots info  
    std::vector<int> other_idx;
    int other_num_to_opti;
    std::vector<State> first_min_in_other;
    ros::Timer updateOtherTimer;
    ros::Timer avoidModeTimer;

    ros::Timer timeoutTimer;
    bool is_timeout;
    double cur_bias;

    // deform task 
    int task_type;
    int vehicle_number;

};

inline void PlanManager::setDesiredShape(int type) {

    swarm_des_origin = generateFormationPositions(fixed_num, type, formation_interval);
    if(type == 1) {
        formation_width = 0.3;
    } else {
        formation_width = (type - 1) * formation_interval;
    }
}
