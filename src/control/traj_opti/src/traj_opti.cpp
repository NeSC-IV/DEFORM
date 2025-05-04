#include "traj_opti/traj_opti.h"

/*==========================================*/
/*             TrajectoryOptimizer          */
/*==========================================*/

TrajectoryOptimizer::TrajectoryOptimizer() {

}

TrajectoryOptimizer::TrajectoryOptimizer(ros::NodeHandle &nh_) {
    nh_.param("optimization/goal_distance_thresh", dist_thresh, -1.0);
    nmpc_.reset(new NMPCController(nh_));

//    nmpc_->initialize();
    nmpc_->initializeInHighCollision();

    cur_cost_type = NORMAL_MODE;

}

void TrajectoryOptimizer::setLocalGoal(Eigen::Vector2d local_goal, Eigen::Vector2d desired_wv) {
    goal = local_goal;
    x_ref = {local_goal.x(), local_goal.y(), 0.0};
    u_ref = {desired_wv.x(), desired_wv.y()};
    nmpc_->setReference(x_ref, u_ref);
}

void TrajectoryOptimizer::setGlobalGoal(Eigen::Vector2d glb_goal) {
    global_goal = glb_goal;
}

void TrajectoryOptimizer::updateState(Eigen::Vector3d s, Eigen::Vector2d c) {
   state = s;
   std::vector<double> current_control = {c.x(), c.y()};
   std::vector<double> current_state = {s.x(), s.y(), s.z()};
   nmpc_->setXUInit(current_state, current_control);
}

void TrajectoryOptimizer::updateOtherOdom(Eigen::Vector2d pos, int id) {
    nmpc_->setOthersState(pos,id);
}

void TrajectoryOptimizer::toggleCostFunc(int costFuncType) {

    if (costFuncType == cur_cost_type)
        return;

    if (costFuncType == HIGH_COLLISION_MODE)
        nmpc_->initializeInHighCollision();
    else if (costFuncType == NORMAL_MODE)
        nmpc_->initialize();
    else
        ROS_WARN("valid costFuncType Value");
}

bool TrajectoryOptimizer::run() {

    vel.clear();
    vel.resize(2);

    Eigen::Vector2d tb_pos(state.x(), state.y());
    Eigen::Vector2d goal_pos(global_goal.x(), global_goal.y());
    double dist = (goal_pos - tb_pos).norm();
    if (dist < dist_thresh) {
        //finish goal
        vel = {0.0, 0.0};
        return true;
    }
    else {
        try {
            nmpc_->solve();
            vel = nmpc_->getu();
        }
        catch (casadi::CasadiException& e) {
            vel = {0.0,0.0};
            std::cout << std::endl;
            std::cout << e.what() << std::endl;
            std::cout << std::endl;
        }
        return false;
    }
}

std::vector<double> TrajectoryOptimizer::getControlVelocity() {
    return vel;
}

std::vector<double> TrajectoryOptimizer::getState() {
    return nmpc_->getState();
}

/*==========================================*/
/*             NMPCController          */
/*==========================================*/

NMPCController::NMPCController() : T_(40), nx_(3), nu_(2), dt_(0.1) {

}

NMPCController::NMPCController(ros::NodeHandle &nh_) : nx_(3), nu_(2){
    nh_.param("controller/T", T_, -1);
    nh_.param("controller/dt", dt_, -0.1);
    nh_.param("controller/distance_other_tb_dist", dist_other_th, -0.1);
	nh_.param("controller/other_num_in_opti", other_num, -1);
    other_constraint_horizon = T_;
}

void NMPCController::initialize() {
    x_ = opti_.variable(nx_, T_+1);
    u_ = opti_.variable(nu_, T_);

    _x_ref = opti_.parameter(nx_, T_+1);
    _u_ref = opti_.parameter(nu_, T_);

    p_ = opti_.parameter(nx_, 1);
    Q = opti_.parameter(nx_, nx_);
    R = opti_.parameter(nu_, nu_);

    J_ = casadi::MX::dot(x_ - _x_ref, casadi::MX::mtimes(Q, x_ - _x_ref)) +
         casadi::MX::dot(u_ - _u_ref, casadi::MX::mtimes(R,u_ - _u_ref));

    opti_.minimize(J_);

    // Define the control horizons
    for (int i = 0; i < T_; i++) {
        opti_.subject_to(x_(casadi::Slice(), i + 1) == x_(casadi::Slice(), casadi::Slice(i)) + \
                                                         dynamics_(x_(casadi::Slice(), casadi::Slice(i)), u_(casadi::Slice(), casadi::Slice(i)), dt_));
    }

    // constraint u and it derivative
    for (int i = 0; i < T_ - 1; i++) {
        auto devel = u_(casadi::Slice(), i + 1) - u_(casadi::Slice(), i) / dt_;
        opti_.subject_to(casadi::Opti::bounded(-MAX_DELTA_VELOCITY, devel(0), MAX_DELTA_VELOCITY));
        opti_.subject_to(casadi::Opti::bounded(-MAX_DELTA_OMEGA, devel(1), MAX_DELTA_OMEGA));
    }

    opti_.subject_to(casadi::Opti::bounded(0.0, u_(0), MAX_LINEAR_VELOCITY));
    opti_.subject_to(casadi::Opti::bounded(-MAX_ANGULAR_VELOCITY, u_(1), MAX_ANGULAR_VELOCITY));

    // constraint x
    opti_.subject_to(casadi::Opti::bounded(-M_PI, x_(2), M_PI));
    opti_.subject_to(x_(casadi::Slice(), 0) == p_);

    solver_options_["ipopt.print_level"] = 0;
    solver_options_["ipopt.sb"] = "yes";
    solver_options_["ipopt.max_iter"] = 2000;
    solver_options_["ipopt.tol"] = 1e-8;
    solver_options_["print_time"] = 0;
    solver_options_["ipopt.acceptable_obj_change_tol"] = 1e-6;
    opti_.solver("ipopt", solver_options_);

    // state and control weight
    opti_.set_value(Q, casadi::DM::diag(casadi::DM::vertcat({1.0, 1.0, 0.0})));
    opti_.set_value(R, casadi::DM::diag(casadi::DM::vertcat({1.0, 5.0})));

    // parameter initialize
    opti_.set_value(_x_ref, casadi::DM::zeros(3, T_ + 1));
    opti_.set_value(_u_ref, casadi::DM::zeros(2, T_));

    u_init = casadi::DM::repmat({0,0}, 1, T_);
    x_init = casadi::DM::repmat({0.0, 0.0, 0.0}, 1, T_ + 1);
}

void NMPCController::initializeInHighCollision() {

    x_ = opti_.variable(nx_, T_ + 1);
    u_ = opti_.variable(nu_, T_);
    espsilon_ = opti_.variable(other_num);
    _x_ref = opti_.parameter(nx_, T_ + 1);
    _u_ref = opti_.parameter(nu_, T_);

    p_ = opti_.parameter(nx_, 1);
    Q = opti_.parameter(nx_, nx_);
    R = opti_.parameter(nu_, nu_);

    J_ = casadi::MX::dot(x_ - _x_ref, casadi::MX::mtimes(Q, x_ - _x_ref)) +
       casadi::MX::dot(u_ - _u_ref,casadi::MX::mtimes(R, u_ - _u_ref)) +
       10000*casadi::MX::dot(espsilon_, espsilon_) + // hard --> soft in collision avoidance
       1.5 * (u_(1) - 0.2); // deadlock heuristic method

    opti_.minimize(J_);

    // Define the control horizons
    for (int i = 0; i < T_; i++) {
        opti_.subject_to(x_(casadi::Slice(), i + 1) == x_(casadi::Slice(), casadi::Slice(i)) + \
                                                         dynamics_(x_(casadi::Slice(), casadi::Slice(i)), u_(casadi::Slice(), casadi::Slice(i)), dt_));
    }

    // constraint u and it derivative
    for (int i = 0; i < T_ - 1; i++) {
        auto devel = u_(casadi::Slice(), i + 1) - u_(casadi::Slice(), i) / dt_;
        opti_.subject_to(casadi::Opti::bounded(-MAX_DELTA_VELOCITY, devel(0), MAX_DELTA_VELOCITY));
        opti_.subject_to(casadi::Opti::bounded(-MAX_DELTA_OMEGA,devel(1), MAX_DELTA_OMEGA));
    }

    opti_.subject_to(casadi::Opti::bounded(0.0, u_(0), MAX_LINEAR_VELOCITY));
    opti_.subject_to(casadi::Opti::bounded(-MAX_ANGULAR_VELOCITY, u_(1), MAX_ANGULAR_VELOCITY));

    // constraint x
    opti_.subject_to(casadi::Opti::bounded(-M_PI, x_(2), M_PI));
    opti_.subject_to(x_(casadi::Slice(), 0) == p_);
    opti_.subject_to(casadi::Opti::bounded(-0.15, espsilon_, 0));

    //init other pos to (500,500)
    others_position=opti_.parameter(2, other_num);
    opti_.set_value(others_position, casadi::DM::repmat(std::vector<double>(2, 500), 1, other_num));

    // add collision free constraint
    for (int id = 0; id<other_num; id++)
        for (int i = 0; i < other_constraint_horizon; i++) {
            auto cur_other_dist = casadi::MX::norm_2(x_(casadi::Slice(0, 2), i) - others_position(casadi::Slice(), id));
            opti_.subject_to(cur_other_dist >= dist_other_th + espsilon_);
        }

    solver_options_["ipopt.print_level"] = 0;
    solver_options_["ipopt.sb"] = "yes";
    solver_options_["ipopt.max_iter"] = 2000;
    solver_options_["ipopt.tol"] = 1e-8;
    solver_options_["print_time"] = 0;
    solver_options_["ipopt.acceptable_obj_change_tol"] = 1e-6;
    opti_.solver("ipopt", solver_options_);

    // state and control weight
    opti_.set_value(Q,casadi::DM::diag(casadi::DM::vertcat({1.0, 1.0, 0.0})));
    opti_.set_value(R,casadi::DM::diag(casadi::DM::vertcat({1.0, 5.0})));

    // parameter initialize
    opti_.set_value(_x_ref, casadi::DM::zeros(3, T_ + 1));
    opti_.set_value(_u_ref, casadi::DM::zeros(2, T_));

    u_init = casadi::DM::repmat({0, 0}, 1, T_);
    x_init = casadi::DM::repmat({0.0, 0.0, 0.0},1,T_ + 1);
}

void NMPCController::setOthersState(Eigen::Vector2d pos, int id) {
    opti_.set_value(others_position(casadi::Slice(), id), std::vector<double>{pos.x(), pos.y()});
}

//set local goal position,yaw
void NMPCController::setReference(const std::vector<double> x_ref, const std::vector<double> u_ref) {
    opti_.set_value(_x_ref,casadi::DM::repmat(x_ref, 1, T_ + 1));
    opti_.set_value(_u_ref,casadi::DM::repmat(u_ref, 1, T_));
}

//update current state(x,y,yaw) and control(linear,angular velocity)
void NMPCController::setXUInit(const std::vector<double> x, const std::vector<double> u) {
    u_init=casadi::DM::repmat({u[0], u[1]}, 1, T_);
    x_init=casadi::DM::repmat({x[0], x[1], x[2]}, 1, T_ + 1);
    opti_.set_value(p_, x);
}

void NMPCController::solve() {
    opti_.set_initial(x_, x_init);
    opti_.set_initial(u_, u_init);

    casadi::OptiSol solution = opti_.solve();

    //get first control
    casadi::Matrix<double> u0 = solution.value(u_)(casadi::Slice(), 0);
    control0 = u0.get_elements();

    //get all state to show path
    casadi::Matrix<double> all_x = solution.value(x_);
    state = all_x.get_elements();

    // warm start
    // use first state and all control
    x_init = casadi::DM::repmat(solution.value(x_)(casadi::Slice(), 0), 1, T_ + 1);
    u_init = solution.value(u_)(casadi::Slice(), casadi::Slice());
}

casadi::MX NMPCController::dynamics_(const casadi::MX &x, const casadi::MX &u, const double dt) {
    casadi::MX xdot(nx_, 1);

    xdot(0) = u(0) * cos(x(2));
    xdot(1) = u(0) * sin(x(2));
    xdot(2) = u(1);
    return xdot * dt;
}

std::vector<double> NMPCController::getu() {
    return control0;
}

std::vector<double> NMPCController::getState() {
    return state;
}
