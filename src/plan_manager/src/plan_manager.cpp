#include "plan_manager/plan_manager.h"
#include <algorithm>
#include <limits>
#include <numeric>

PlanManager::PlanManager() : nh_(), nh_pri_("~"){

//    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
//        ros::console::notifyLoggerLevelsChanged();
//    }

    readParameters();

    tb_id = fixed_id;
    formation_max_num = fixed_num;
    formation_min_type = 1;

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh_pri_);
    aster_.reset(new AStar);
    aster_->initGridMap(grid_map_,Eigen::Vector2d(80,80));
    find_region_.reset(new FindMaxFreeRegion);
    find_region_->initMapandMaxGap(grid_map_, (formation_max_num - 1) * formation_interval + formation_interval);
    path_opti_.reset(new PathOptimizer); 
    path_opti_->setGridMapandObsClearance(grid_map_,obs_clearance);
    traj_opt_.reset(new TrajectoryOptimizer(nh_pri_));
    ot_cli_.reset(new OT_Transfer);
    ot_cli_->Init(nh_pri_);
    to_vis_.reset(new toVisualization);
    to_vis_->init(nh_pri_);

    swarm_des_origin.resize(fixed_num);
    swarm_alloc.resize(fixed_num);
    tb_states_.resize(fixed_num);
    odoms_sub.resize(fixed_num);
    realloc_.resize(fixed_num);
    all_width.resize(fixed_num,0.2);
	is_avoid_obs.resize(fixed_num, 0);
	is_avoid_obs_sub_.resize(fixed_num);

    for (int i = 0; i < fixed_num; i++) {

        realloc_[i] = i;
		
		if(i != tb_id) {
			other_idx.push_back(i);
		}
	}

    goal_sub = nh_.subscribe("/move_base_simple/goal", 1, &PlanManager::subGlobalGoal, this);
    vel_pub = nh_pri_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	is_avoid_obs_pub_ = nh_pri_.advertise<std_msgs::Int16>("/tb_" + std::to_string(fixed_id) + "/is_avoid", 1);
	for (int id = 0; id < fixed_num; id++) {
		is_avoid_obs_sub_[id] = nh_.subscribe<std_msgs::Int16>("/tb_" + std::to_string(id) + "/is_avoid", 1, [=](const std_msgs::Int16ConstPtr& msg) {
			// ROS_WARN("receive avoid_mode info");
			is_avoid_obs[id] = msg->data;
		});
	}

    //get all turtlebot position info
    for (int id = 0; id < fixed_num; id++)
        odoms_sub[id] = nh_.subscribe<nav_msgs::Odometry>("/tb_" + std::to_string(id) + "/odom", 1, [=](const nav_msgs::OdometryConstPtr& msg){
            tb_states_[realloc_[id]].pos.x() = msg->pose.pose.position.x;
            tb_states_[realloc_[id]].pos.y() = msg->pose.pose.position.y;
            tb_states_[realloc_[id]].yaw = tf2::getYaw(msg->pose.pose.orientation);
            tb_states_[realloc_[id]].vel.x() = msg->twist.twist.linear.x;
            tb_states_[realloc_[id]].vel.y() = msg->twist.twist.angular.z;
            Eigen::Quaterniond q;
            tf2::fromMsg(msg->pose.pose.orientation, q);
            Eigen::Matrix2d r_2d = q.toRotationMatrix().block<2,2>(0, 0);
            Eigen::Vector2d unit_x_axis(1.0, 0.0);
            tb_states_[realloc_[id]].orient_vec = r_2d * unit_x_axis;

            if (realloc_[id] == tb_id) {
                Eigen::Vector3d state(tb_states_[tb_id].pos.x(), tb_states_[tb_id].pos.y(), tb_states_[tb_id].yaw);
                traj_opt_->updateState(state,tb_states_[tb_id].vel);
                has_odom = true;
            }
        });

	path_point_pub = nh_.advertise<geometry_msgs::Point>("/path_point_sharing", 1);
    path_point_sub = nh_.subscribe<geometry_msgs::Point>("/path_point_sharing", 1, &PlanManager::setPathPoint, this); 

    frm_info_pub = nh_.advertise<plan_manager::RotationVec>("/formation_info", 1);
    frm_info_sub = nh_.subscribe("/formation_info", 4, &PlanManager::updateFRMInfo, this);

    width_pub = nh_.advertise<plan_manager::widthInfo>("/current_width", 1);
    width_sub = nh_.subscribe<plan_manager::widthInfo>("/current_width", 1, [=](const plan_manager::widthInfoConstPtr& msg){
        all_width[msg->id] = msg->width;
    });

    execTimer = nh_.createTimer(ros::Duration(0.2), &PlanManager::execCallback, this);
    pathTimer = nh_.createTimer(ros::Duration(0.4),&PlanManager::pathPlanning, this, false, false);
    exec_state_= FORMATION_EXEC_STATE::WAIT_ODOM_OR_TARGET;

	// sorted the distance between other robot and current robot, used to reduce traj constraints
	updateOtherTimer = nh_.createTimer(ros::Duration(0.4), &PlanManager::updateOtherCallback, this, false, false); 

	//If many robots enter obstacle avoidance mode, shrink the formation.
	avoidModeTimer = nh_.createTimer(ros::Duration(2.0), &PlanManager::updateAvoidModeCallback, this, false, false);
	
	timeoutTimer = nh_.createTimer(ros::Duration(2.0), &PlanManager::timeoutCallback, this, false, false);

	if (task_type == TASK_TYPE::NAVIGATION) {
		if (tb_id == leader_id) 
			ROS_WARN("Task type is navigation");
		formation_type = start_frm_type; // fixed_num : the widthest formation, 1 : the narrowest formation
		setDesiredShape(formation_type);
	} else if ( task_type ==  TASK_TYPE::DEFORM) {
		if (tb_id == leader_id) 
			ROS_WARN("Task type is deform");
		for (int i = 1; i <= swarm_des_origin.size(); i++) {
			nh_pri_.param("desired_shape_positions/vehicle_pos_" + to_string(i) + "/x", swarm_des_origin[i - 1][0], -1.0);					
			nh_pri_.param("desired_shape_positions/vehicle_pos_" + to_string(i) + "/y", swarm_des_origin[i - 1][1], -1.0);					
		}
	} else {
		ROS_ERROR("task_type invalid");
		throw std::runtime_error("task_type invalid");
	}

    //init foramtion type
    swarm_des = swarm_des_origin;
    formation_yaw = 0.0; //The initial formation orientation is aligned with the x-axis.

    has_goal=false;
    has_odom= false;
    do_allocation=false;
    first_start= false;//startup to one do_allocation
	is_timeout = false;
} 

PlanManager::~PlanManager() {
}

void PlanManager::timeoutCallback(const ros::TimerEvent&) {

	if(tb_id != leader_id)
		return;

	if (is_timeout) {
		ROS_WARN("Timeout, reassignment!");
		do_allocation= true;
		is_timeout = false;
		return;
	}

	// When the formation gets "stuck" during navigation
	double cur_linear_vel = tb_states_[0].vel[0];
	double cur_angular_vel = tb_states_[0].vel[1];
	
	double linear_vel_th = 0.05;
	double angular_vel_th = 0.07;

	// ROS_WARN("Timeout true test, frm bias: %.2f, cur_lin_vel: %.2f, cur_ang_vel: %.2f ", cur_bias, cur_linear_vel, cur_angular_vel);
	if (cur_bias > 0.6 && 
		cur_linear_vel < linear_vel_th && 
		abs(cur_angular_vel) < angular_vel_th ) {
		is_timeout = true;
	}
}

void PlanManager::updateAvoidModeCallback(const ros::TimerEvent&) {

	if(tb_id != leader_id) 
		return;

	// the weight of number in avoid mode
	double in_avoid_weight = 0.2;
	int in_avoid_num_th = std::floor(in_avoid_weight * fixed_num);

	int avoid_sum = 0;
	for (int i = 0; i < is_avoid_obs.size(); i++) {
		avoid_sum += is_avoid_obs[i];
	}

	if (avoid_sum >= in_avoid_num_th) {
		
		if(formation_type > 1) {
			formation_type -= 1;
			setDesiredShape(formation_type);
			do_allocation=true;
			rotateAndShareFRM();
		} else if (formation_type == 1 && false) {
			formation_type += 1;
			setDesiredShape(formation_type);
			do_allocation=true;
			rotateAndShareFRM();
		}

		is_avoid_obs.assign(fixed_num, 0);
		ROS_WARN("tb in avoid status is much, reduce formation");
	}
}

void PlanManager::execCallback(const ros::TimerEvent &) {

    switch (exec_state_) {

        case FORMATION_EXEC_STATE::WAIT_ODOM_OR_TARGET:
        {
            if(has_goal && has_odom)
                changeFRMExecState(FRM_INIT_PATH);
            break;
        }

        case FORMATION_EXEC_STATE::FRM_INIT_PATH:
        {

			pathPlanning(ros::TimerEvent{});

			// frm_rrt need
			// grid_map_->updateESDFMap(Eigen::Vector2d{60, 30});

			pathTimer.start();
			updateOtherTimer.start();
			avoidModeTimer.start();
			timeoutTimer.start();
			
			changeFRMExecState(FRM_KEEP);
            break;
        }

        case FORMATION_EXEC_STATE::FRM_KEEP:
        {
			
			if (task_type == TASK_TYPE::NAVIGATION) {
				double dist_to_goal=((swarm_des[tb_id]+global_goal)-tb_states_[tb_id].pos).norm();
				if (dist_to_goal < 1.0) {
					changeFRMExecState(APPROACH_GOAL);
				}
				else {
						trajPlanning();
				}
			} else if (task_type == TASK_TYPE::DEFORM) {
				double traj_e;
				FRMBias(traj_e);
				// ROS_INFO("current bias: %.2f", traj_e);
				if (traj_e < 0.4) {
					ROS_WARN("Convergence success!");
					std::vector<double> c;
					c.push_back(0.0);
					c.push_back(0.0);
					pubVelCmd(c);
					execTimer.stop();
					timeoutTimer.stop();
				}
				else
					trajPlanningDeform();
			}

            break;
        }

        case FORMATION_EXEC_STATE::FRM_DEFORM:
        {
            break;
        }

        case FORMATION_EXEC_STATE::AVOID_OBS:
        {
			std_msgs::Int16 temp_avoid_mode;
			temp_avoid_mode.data = 1;
			is_avoid_obs_pub_.publish(temp_avoid_mode);

            double esdf_dist=grid_map_->getDistance(tb_states_[tb_id].pos);
            // std::cout<<"avoid current dist="<<esdf_dist<<std::endl;
            if (esdf_dist < 0.35) 
            {
				Eigen::Vector2d avoid_pt = tb_states_[tb_id].pos;
                path_opti_->setWeightToLocalOpt(1.0,2.0);
                path_opti_->ESDFAndLocalPointOpt(avoid_pt,0.4);
                to_vis_->showGlobalGoal(avoid_pt);
                trajController(avoid_pt);
            } else {
                changeFRMExecState(FRM_KEEP);
				std_msgs::Int16 temp_avoid_mode;
				temp_avoid_mode.data = 0;
				is_avoid_obs_pub_.publish(temp_avoid_mode);
            }
            break;
        }

        // 1m obstacle-free environment around goal
        case FORMATION_EXEC_STATE::APPROACH_GOAL:
        {
            tb_global_goal=global_goal+swarm_des_origin[tb_id];
            traj_opt_->setGlobalGoal(tb_global_goal);

            if(trajController(tb_global_goal))
            {
                pathTimer.stop();
				timeoutTimer.stop();
                has_goal=false;
                changeFRMExecState(WAIT_ODOM_OR_TARGET);
                ROS_WARN("tb %d reach goal!",fixed_id);
            }
            break;
        }

        case FORMATION_EXEC_STATE::EMERGENCY_STOP:
        {
            std::vector<double> v_cmd={0.0,0.0};
            pubVelCmd(v_cmd);
            break;
        }

        default:
            std::cout<<"formation_exec_state error"<<std::endl;
    }
}

void PlanManager::updateOtherCallback(const ros::TimerEvent&) {

	auto cur_pos = tb_states_[tb_id].pos;
	auto cmp = [&cur_pos](Eigen::Vector2d pos1, Eigen::Vector2d pos2) {
		double dist1 = (pos1 - cur_pos).norm();
		double dist2 = (pos2 - cur_pos).norm();
		return dist1 > dist2;
	};
	std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, decltype(cmp)> pq(cmp);

	for (int i = 0; i < fixed_num - 1; i++) {
		pq.push(tb_states_[realloc_[other_idx[i]]].pos);
	}

	for(int i = 0; i < other_num_to_opti; i++) {
		
		traj_opt_->updateOtherOdom(pq.top(), i);
		pq.pop();
	}

}

void PlanManager::trajPlanningDeform() {

        if(tb_id == leader_id) {
            rotateAndShareDeform();
		}

		Eigen::Vector2d temp_pt = global_goal+swarm_des[tb_id];
		to_vis_->showGlobalGoal(temp_pt);

        FRMConsensus(local_goal);
        trajController(local_goal);
}

// main logic
void PlanManager::trajPlanning() {

    double esdf_dist = grid_map_->getDistance(tb_states_[tb_id].pos);

    if(esdf_dist<0.15)
    {
       changeFRMExecState(AVOID_OBS);
      return;
    }

	findPointInPath(fwd_dist, local_goal);

    bool widthHandleResult;
    widthHandleResult=widthHandle(tb_states_[tb_id].pos,local_goal);
	widthHandleResult = true;
    if (widthHandleResult) {

        widthHandleInExtend();

        //test: check current shap in RViz
        Eigen::Vector2d temp_pt=global_goal+swarm_des[tb_id];
        to_vis_->showGlobalGoal(temp_pt);

        if (tb_id == leader_id) {

			// ROS_WARN("leader robot: %d, current width:%f",fixed_id, find_region_->getWidth());

			// DEBUG width result
			to_vis_->showUpdateArea(find_region_->getBox());
			std::vector<Eigen::Vector2d> one_line, other_line;
			find_region_->test(one_line, other_line);
			to_vis_->showWidthLineOne(one_line);
			to_vis_->showWidthLineOther(other_line);

            double traj_e;
            FRMBias(traj_e);
			cur_bias = traj_e;

            isReduceFRM();

			// running this function is not considered
            isExtendFRM();

            rotateAndShareFRM();
        }

        FRMConsensus(local_goal);

        //show formation consensus local point and line
        std::vector<Eigen::Vector2d> odom_and_local_line;
        odom_and_local_line.push_back(tb_states_[tb_id].pos);
        odom_and_local_line.push_back(local_goal);
        to_vis_->showOdomAndLocalLine(odom_and_local_line);

        trajController(local_goal);
    }
    else
	{
		// follow the path
        trajController(local_goal);
	}
}

bool PlanManager::trajController(Eigen::Vector2d goal) {

    //nmpc controller
    double w_des, v_des;
    calcDesireLinearVel(v_des,goal);
    calcDesireAngularVel(w_des,goal);

    Eigen::Vector2d x_ref={goal.x(),goal.y()};
    Eigen::Vector2d u_ref={v_des,w_des};
    traj_opt_->setLocalGoal(x_ref,u_ref);

    bool reach_goal;
    reach_goal=traj_opt_->run();
    pubVelCmd(traj_opt_->getControlVelocity());

	// show predict traj
    to_vis_->showPath(traj_opt_->getState());
	
    return reach_goal;
}

void PlanManager::pathPlanning(const ros::TimerEvent&) {
	if(tb_id != leader_id) {
		path_ = aster_->astarSearchAndGetSimplePath(grid_map_->getResolution(), tb_states_[tb_id].pos, leader_path_point);
	} else {
		path_ = aster_->astarSearchAndGetSimplePath(grid_map_->getResolution(), tb_states_[tb_id].pos, tb_global_goal);
	}

	path_opti_->startOpt(path_);
	path_ = aster_->refinePath(path_);

    to_vis_->showAstarLists(path_);

	if(leader_id == tb_id) {
		double perception_range = 5.5;
		findPointInPath(perception_range, leader_path_point);
		geometry_msgs::Point tmp_pt;
		tmp_pt.x = leader_path_point.x();
		tmp_pt.y = leader_path_point.y();
		tmp_pt.z = 0.0;
		path_point_pub.publish(tmp_pt);
	}
}

void PlanManager::widthHandleInExtend() {

    if(formation_type == formation_max_num)
        return;

    plan_manager::widthInfo temp_width;
    temp_width.id = fixed_id;
    temp_width.width = find_region_->getWidth();
    width_pub.publish(temp_width);
}

bool PlanManager::widthHandle(const Eigen::Vector2d& start_pt, Eigen::Vector2d& end_pt) {

        find_region_->updateFormationGap(formation_width+formation_interval);
        find_region_->setEndpointandupdateBoundary(start_pt, end_pt);
        find_region_->updatePointsRegion();
        find_region_->solve_max_b();

		// [Debug]: show width box and lines
        to_vis_->showUpdateArea(find_region_->getBox());
		std::vector<Eigen::Vector2d> one_line, other_line;
		find_region_->test(one_line, other_line);
		to_vis_->showWidthLineOne(one_line);
		to_vis_->showWidthLineOther(other_line);

        if(find_region_->getWidth() < 0.1) {
			return false;	
		}
		
		// [Debug]:show width local point
		to_vis_->showWidthMidPoint(end_pt);
		return true;
}

// if feasiable width less than formation width, to reset desired formation shape
void PlanManager::isReduceFRM() {

    if (find_region_->getWidth() < formation_width) {
        int cur_type = std::floor(find_region_->getWidth() / formation_interval) + 1;
        std::cout << "[formation reconfiguration] current shape:" << cur_type<<std::endl;

		if (cur_type == formation_type) 
			return;

        formation_type = cur_type;
        setDesiredShape(formation_type);

        do_allocation = true;
    }
}

void PlanManager::isExtendFRM() {

	// consider formation extend when the convergence of formation error less than threshold
    double thresh=0.4;
    double extend_e;
    FRMBias(extend_e);
	// ROS_WARN("current FrmBias:%f", extend_e);
    if (extend_e > thresh)
        return;

	// method 1
	// consensus formation_width of all robots
    double min = formation_interval * (formation_max_num - 1);
    for (int i = 0; i < fixed_num; i++)
		if (all_width[i] < min)
			min = all_width[i];

	// method 2
	// std::vector<double> vec = all_width;
	// double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
	// double min = sum / vec.size();
	
	// method 3
	// double min = all_width[fixed_id];

	// ROS_WARN("min width:%f",min);
    int new_type = std::floor(min / formation_interval) + 1;

	// new_formation greater than current_formation
    if (new_type > formation_type) {

        ROS_WARN("formation expansion. type: %d", new_type);
        formation_type = new_type;
        setDesiredShape(formation_type);
        do_allocation = true;
    }
}

void PlanManager::rotateAndShareDeform() {

    if (first_start) {
        do_allocation= true;
        first_start= false;
    }

    if (do_allocation) {
        do_allocation = false;
        swarm_alloc = swarm_des;
        taskAllocation();

        tb_id = realloc_[fixed_id];
    }

    // update shape info
    plan_manager::RotationVec temp_rotation;
    temp_rotation.shape_type = formation_type;
	temp_rotation.remap_id.resize(fixed_num);
    for (int i = 0; i < fixed_num; i++)
        temp_rotation.remap_id[i] = realloc_[i];
    temp_rotation.yaw = 0.0;
    frm_info_pub.publish(temp_rotation);
}

// rotate desired formation based on orientation, and share Frm infomation to other robots
void PlanManager::rotateAndShareFRM() {

	// compute the diff angle between formation and local goal
	Eigen::Vector2d tb_center_(0.0, 0.0);
	for (const auto& i : tb_states_) 
		tb_center_ += i.pos;
	tb_center_ /= fixed_num;
	auto diff_ = local_goal - tb_center_;
	double yaw = atan2(diff_.y(), diff_.x());
	double yaw_e = yaw - formation_yaw;
	
	//yaw \in [-M_PI, M_PI]
	if (yaw_e > M_PI)
		yaw_e -= 2*M_PI;
	else if (yaw_e < -M_PI)
		yaw_e += 2 * M_PI;


    if (first_start) {
        do_allocation = true;
        first_start = false;
    }

	if (abs(yaw_e) > M_PI / 2.5) {
		ROS_WARN("rotation angle is %.2f, task allocation", yaw_e * 180);
        do_allocation = true;
	}

	formation_yaw = yaw;
	Eigen::Matrix2d r_mat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix().block<2, 2>(0, 0);
	for (int i = 0; i < fixed_num; i++)
		swarm_des[i] = r_mat * swarm_des_origin[i];

    if (do_allocation)
    {
        do_allocation = false;
        swarm_alloc = swarm_des;
        taskAllocation();

        tb_id = realloc_[fixed_id];
    }
    tb_global_goal = global_goal + swarm_des_origin[tb_id];

    // update shape info
    plan_manager::RotationVec temp_rotation;
    temp_rotation.shape_type = formation_type;
	temp_rotation.remap_id.resize(fixed_num);
    for (int i = 0; i < fixed_num; i++)
        temp_rotation.remap_id[i] = realloc_[i];
    temp_rotation.rotation_vec[0] = r_mat(0, 0);
    temp_rotation.rotation_vec[1] = r_mat(0, 1);
    temp_rotation.rotation_vec[2] = r_mat(1, 0);
    temp_rotation.rotation_vec[3] = r_mat(1, 1);
    temp_rotation.yaw = formation_yaw;
    frm_info_pub.publish(temp_rotation);
}

// other robot update FRM info
void PlanManager::updateFRMInfo(const plan_manager::RotationVecConstPtr &msg) {
    if (leader_id == tb_id)
        return;

    for (int i = 0; i < fixed_num; i++)
        realloc_[i] = msg->remap_id[i];
    tb_id = realloc_[fixed_id];

	if (task_type == TASK_TYPE::DEFORM) 
		return;

    Eigen::Matrix2d rotation_2d;
    rotation_2d(0, 0) = msg->rotation_vec[0];
    rotation_2d(0, 1) = msg->rotation_vec[1];
    rotation_2d(1, 0) = msg->rotation_vec[2];
    rotation_2d(1, 1) = msg->rotation_vec[3];
    formation_yaw = msg->yaw;

    if (formation_type != msg->shape_type) {
        formation_type = msg->shape_type;
        setDesiredShape(formation_type);
    }

    for (int i = 0; i < fixed_num; i++)
        swarm_des[i] = rotation_2d * swarm_des_origin[i];

    tb_global_goal = global_goal + swarm_des_origin[tb_id];
}

void PlanManager::FRMConsensus(Eigen::Vector2d& pos) {
    Eigen::Vector2d vec(0.0, 0.0);

    for (int i = 0; i < fixed_num; i++) {
        if (i == tb_id)
            continue;
        vec += tb_states_[i].pos + (swarm_des[tb_id] - swarm_des[i]);
    }

    double consensus_width = 1.0; // The value range is preferably within [0.9, 1].
	vec += consensus_width * pos;
	vec /= (fixed_num - 1 + 1);
    pos = vec;
}

// it is find Point from static path
void PlanManager::findPointInPath(double forward_distance, Eigen::Vector2d &pt) {
    double to_goal_dist = (global_goal - tb_states_[tb_id].pos).norm();
    if (to_goal_dist<forward_distance+0.2) {
        pt = global_goal;
        return;
    }
	
	//find Closest Point On Path 
	std::vector<Eigen::Vector2d> cur_path = path_;
	Eigen::Vector2d cur_pos = tb_states_[tb_id].pos;
	
	auto closestPointOnSegment = [](const Eigen::Vector2d& p, const Eigen::Vector2d& segStart, const Eigen::Vector2d& segEnd) -> Eigen::Vector2d {
		Eigen::Vector2d segVec = segEnd - segStart;
		Eigen::Vector2d vecToPoint = p - segStart;

		double segLengthSq = segVec.squaredNorm();
		double t = 0.0;
	
		if (segLengthSq > 1e-8) {
			t = std::max(0.0, std::min(1.0,vecToPoint.dot(segVec) / segLengthSq)); 
		}
		
		return segStart + t * segVec;
	};

	Eigen::Vector2d closestPoint = cur_path[0];
	int curPointNextIdx = 0;

	double minDistSq = std::numeric_limits<double>::max();
	
	for (size_t i = 0; i < cur_path.size() - 1; ++i) {
		Eigen::Vector2d candidate = closestPointOnSegment(cur_pos, cur_path[i], cur_path[i + 1]);
		double distSq = (cur_pos - candidate).squaredNorm();
		if (distSq < minDistSq) {
			minDistSq = distSq;
			closestPoint = candidate;
			curPointNextIdx = i + 1;
		}
	}

	double distToClosestPoint = (cur_pos - closestPoint).norm();

	double cur_forward_distance;

	cur_forward_distance = forward_distance;
	
	// Bisection method
	auto findDesiredPointOnSeg = [](const double& dist_delta, const Eigen::Vector2d& start, const Eigen::Vector2d& end) -> Eigen::Vector2d {
		Eigen::Vector2d left_pt = start;
		Eigen::Vector2d right_pt = end;
		double mid_dist;
		Eigen::Vector2d mid_pt;

		do {
			mid_pt = (left_pt + right_pt) / 2;
			mid_dist = (mid_pt -start).norm();
			if(mid_dist > dist_delta) 
				right_pt = mid_pt;
			else
				left_pt = mid_pt;

		} while (abs(mid_dist - dist_delta) > 0.1);
		return mid_pt;
	};

	double distToNextPoint = (closestPoint - cur_path[curPointNextIdx]).norm();
	if (distToNextPoint > cur_forward_distance) {
		pt = findDesiredPointOnSeg(cur_forward_distance, closestPoint, cur_path[curPointNextIdx]);
		return ;
	}

	double dist_sum = distToNextPoint;
	for (int i = curPointNextIdx; i < cur_path.size() - 1; i++) {
		dist_sum += (cur_path[i + 1] - cur_path[i]).norm();
		if (cur_forward_distance < dist_sum) {
			double remaining_dist = cur_forward_distance - (dist_sum - (cur_path[i + 1] - cur_path[i]).norm());
			pt = findDesiredPointOnSeg(remaining_dist, cur_path[i], cur_path[i + 1]);
			return;
		}
	}
	
	ROS_WARN("path less than present distance!!! return the last point in path");
	pt = cur_path.back();
}

void PlanManager::setPathPoint(const geometry_msgs::PointConstPtr &msg) {
    if (tb_id == leader_id)
        return;

    leader_path_point << msg->x,msg->y;
}

// task assignment and show assignment result
void PlanManager::taskAllocation() {
    std::vector<geometry_msgs::Point> start_pts,goal_pts;
    start_pts.resize(fixed_num);
    goal_pts.resize(fixed_num);

    for (int i = 0; i < fixed_num; i++) {
        start_pts[i].x = tb_states_[i].pos.x();
        start_pts[i].y = tb_states_[i].pos.y();
        goal_pts[i].x = swarm_alloc[i].x();
        goal_pts[i].y = swarm_alloc[i].y();
    }

    std::vector<int> result;
    ot_cli_->compute_assignment(start_pts, goal_pts, result);

    std::vector<int> cur_result(fixed_num);
    for (int i = 0; i < fixed_num; i++)
        cur_result[i] = result[realloc_[i]];
    realloc_ = cur_result;

    //print task allocation info
    for (int i = 0; i<fixed_num; i++) {
        std::cout << i << "-->" << realloc_[i] << "\t";
    }

    std::cout<<"\n";
}

// compute formation bias
void PlanManager::FRMBias(double& e) {

    std::vector<Eigen::Vector2d> vec_des;
    vec_des.resize(fixed_num, Eigen::Vector2d(0.0, 0.0));

    for (int id = 0; id < fixed_num; id++) {
        for(int i = 0; i < fixed_num; i++) {
            if(i == id)
                continue;
            vec_des[id] += tb_states_[i].pos + (swarm_des[id] - swarm_des[i]);
        }
        vec_des[id] /= (fixed_num - 1);
    }

    e = 0.0;
    for (int i = 0; i < fixed_num; i++)
        e += (vec_des[i] - tb_states_[i].pos).norm();
}

// print change State 
void PlanManager::changeFRMExecState(PlanManager::FORMATION_EXEC_STATE new_state) {

    static string state_str[7] = {"WAIT_ODOM_OR_TARGET", "FRM_INIT_PATH", "FRM_KEEP", "FRM_RECONFIG", "AVOID_OBS", "APPROACH_GOAL", "EMERGENCY_STOP"};
    int last_state = int(exec_state_);
    exec_state_ = new_state;
    std::cout << "tb " + std::to_string(fixed_id) + " from " + state_str[last_state] + " to " + state_str[int(new_state)] << std::endl;
}

// subscribe latest global goal
void PlanManager::subGlobalGoal(const geometry_msgs::PoseStampedConstPtr &msg) {
    global_goal.x() = msg->pose.position.x;
    global_goal.y() = msg->pose.position.y;

	leader_path_point << msg->pose.position.x, msg->pose.position.y;

    tb_global_goal = global_goal + swarm_des_origin[tb_id];
    traj_opt_->setGlobalGoal(tb_global_goal);
    has_goal = true;
    first_start = true;
}

std::vector<Eigen::Vector2d> PlanManager::generateFormationPositions(int robot_num, int one_row_num, double frm_gap) {

	if(robot_num < one_row_num) {
		std::cout << "invaild number" << std::endl;
		return std::vector<Eigen::Vector2d>(1, Eigen::Vector2d(-1, -1));
	}

	Eigen::Vector2d tmp_pt;
	
	std::vector<Eigen::Vector2d> frm_pos;
	Eigen::Vector2d leader_pos;
	int lid;

	int rows = robot_num / one_row_num;
	int remaining = robot_num % one_row_num;

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < one_row_num; j++) {
			frm_pos.emplace_back(i * frm_gap, j * frm_gap);
		}	
	}

	if (remaining > 0) {
		double base_x = rows * frm_gap;
		double center_y = (one_row_num - 1) * frm_gap / 2;

		if (remaining % 2 == 1) {
			leader_pos << base_x, center_y;
			lid = frm_pos.size();
			frm_pos.push_back(leader_pos);
			remaining -= 1;
			for (int i = 1; i <= remaining / 2; i++) {
				frm_pos.emplace_back(base_x, leader_pos.y() + i * frm_gap);
				frm_pos.emplace_back(base_x, leader_pos.y() - i * frm_gap);
			}
		} else {
			leader_pos << base_x, center_y + frm_gap / 2;
			lid = frm_pos.size();
			frm_pos.push_back(leader_pos);
			Eigen::Vector2d right_pos(base_x, center_y - frm_gap / 2);
			frm_pos.push_back(right_pos);
			remaining -= 2;
			for (int i = 1; i <= remaining / 2; i++) {
				frm_pos.emplace_back(leader_pos.x(), leader_pos.y() + i * frm_gap);
				frm_pos.emplace_back(right_pos.x(), right_pos.y() - i * frm_gap);
			}
		}
	} else {
		leader_pos << (rows - 1) * frm_gap, (one_row_num / 2) * frm_gap;
		lid = (rows - 1) * one_row_num + (one_row_num / 2);
	}
	
	// offset frm_pos based on leader_pos
	for (int i = 0; i < frm_pos.size(); i++) {
		frm_pos[i] -= leader_pos;
	}
	std::swap(frm_pos[0], frm_pos[lid]);

	// show formation info 
	bool show_frm_pos = false;
	if (show_frm_pos) {
		std::cout << "robot_num:" << robot_num << " one_row_num:" << one_row_num << std::endl;;
		std::cout << "formation position:\n";
		for (int i = 0; i < frm_pos.size(); i++) {
			std::cout << i << "\t" << frm_pos[i].x() << "," << frm_pos[i].y() << (i == lid ? "* \n" : " \n");
		}
	}
	
	return frm_pos;
}

void PlanManager::calcDesireAngularVel(double &w, const Eigen::Vector2d& goal) {
    Eigen::Vector2d to_goal_orient = (goal - tb_states_[tb_id].pos) / (goal - tb_states_[tb_id].pos).norm();

    //w \in [-1.5,-1.5]
    w = 0.5 * (1 - tb_states_[tb_id].orient_vec.dot(to_goal_orient)) * 1.5;

    Eigen::Vector2d diff_vec = goal - tb_states_[tb_id].pos;
    double angular_diff = atan2(diff_vec.y(), diff_vec.x()) - tb_states_[tb_id].yaw;

    if (angular_diff > M_PI)
        angular_diff -= 2 * M_PI;
    if (angular_diff < -M_PI)
        angular_diff += 2 * M_PI;

    int sign_ = angular_diff > 0 ? 1 : -1;

    w = sign_ * w;
}

void PlanManager::calcDesireLinearVel(double &v, const Eigen::Vector2d& goal) {
    double x_e_threah = 0.5;
    double x_e = (goal-tb_states_[tb_id].pos).norm();

    if(x_e > x_e_threah)
        v = 0.2;
    else
        v = x_e / x_e_threah * 0.2;
}

void PlanManager::pubVelCmd(const std::vector<double> &c) {
    geometry_msgs::Twist vel;
    vel.linear.x = c[0];
    vel.angular.z = c[1];
    vel_pub.publish(vel);
}

void PlanManager::readParameters() {
    nh_pri_.param("plan_manager/tb_id", fixed_id, -1);
    nh_pri_.param("plan_manager/tb_num", fixed_num, -1);
    nh_pri_.param("plan_manager/other_num_in_opti", other_num_to_opti, -1);
    nh_pri_.param("plan_manager/formation_interval", formation_interval, -1.0);
    nh_pri_.param("plan_manager/leader_id", leader_id, -1);
    nh_pri_.param("plan_manager/forward_distance", fwd_dist, -1.0);
    nh_pri_.param("plan_manager/obs_clearance", obs_clearance, -1.0);
	nh_pri_.param("plan_manager/frm_type", start_frm_type, -1);
	nh_pri_.param("task_type", task_type, -1);
	nh_pri_.param("vehicle_number", vehicle_number, -1);
}
