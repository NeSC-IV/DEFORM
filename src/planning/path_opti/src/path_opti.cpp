#include "path_opti/path_opti.h"

void PathOptimizer::setGridMapandObsClearance(GridMap::Ptr map_, double clearance) {
    grid_map_ = map_;
    obs_clearance=clearance;
}

int PathOptimizer::startOpt(std::vector<Eigen::Vector2d>& path) {
    vector<int> opti_vars;
    for (int i = 0; i < path.size() ; i++) {
        double dist;
        grid_map_->evaluateEDT(path[i], dist);

        double dist_err = obs_clearance - dist;
        if (dist_err > 0)
            opti_vars.push_back(i);
    }

    if (opti_vars.size() == 0) {
        return 0;
    }

    Eigen::VectorXd x(opti_vars.size() * 2);
    for(int i = 0; i < opti_vars.size(); i++)
    {
        x(2 * i) = path[opti_vars[i]].x();
        x(2 * i + 1) = path[opti_vars[i]].y();
    }

    double finalCost;

    lbfgs::lbfgs_parameter_t params;
    params.g_epsilon = 1.0e-5;
    params.mem_size = 16;
    params.min_step = 1e-32;

    int ret = lbfgs::lbfgs_optimize(x,
                                    finalCost,
                                    PathOptimizer::costFunction,
                                    nullptr,
                                    nullptr,
                                    this,
                                    params);

    if (ret < 0) {
        ROS_ERROR("[Path Opti]:%s", lbfgs::lbfgs_strerror(ret));
        return -1;
    } else {
        for (int i = 0; i < opti_vars.size(); i++)
        {
            path[opti_vars[i]].x() = x(2 * i);
            path[opti_vars[i]].y() = x(2 * i + 1);
        }
        return 0;
    }
}

double PathOptimizer::costFunction(void *instance,
                    const Eigen::VectorXd &x,
                    Eigen::VectorXd &g)
{
    PathOptimizer* opti = static_cast<PathOptimizer*>(instance);

    double wei = 1.0;
    const int n = x.size();

    Eigen::Vector2d pos(0.0, 0.0);

    double fx = 0.0;

    for (int i = 0; i < n; i += 2) {
        double dist;
        pos.x() = x(i);
        pos.y() = x(i + 1);
        opti->grid_map_->evaluateEDT(pos, dist);
        Eigen::Vector2d dist_grad;
        opti->grid_map_->evaluateFirstGrad(pos, dist_grad);

        double dist_err = opti->obs_clearance - dist;

        fx += wei * pow(dist_err, 2);
        g(i) = 2.0 * (dist-opti->obs_clearance) * dist_grad(0);
        g(i + 1) = 2.0 * (dist - opti->obs_clearance) * dist_grad(1);
    }
    return fx;
}

int PathOptimizer::monitorProgress(void *instance, const Eigen::VectorXd &x, const Eigen::VectorXd &g, const double fx,
                                   const double step, const int k, const int ls) {
    std::cout << std::setprecision(4)
              << "================================" << std::endl
              << "Iteration: " << k << std::endl
              << "Function Value: " << fx << std::endl
              << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << std::endl;
    return 0;
}
void PathOptimizer::setWeightToLocalOpt(double w1, double w2) {
    weight_[0] = w1;
    weight_[1] = w2;
}

void PathOptimizer::ESDFAndLocalPointOpt(Eigen::Vector2d& pt, double obs_clear/* half of max width */) {
    double finalCost;

    cur_local_point = pt;
    local_pt_obs_clearance = obs_clear;

    Eigen::VectorXd x(2);
    x(0) = pt.x();
    x(1) = pt.y();

    lbfgs::lbfgs_parameter_t params;
    params.g_epsilon = 1.0e-5;
    params.mem_size = 16;
    params.min_step = 1e-32;

    int ret = lbfgs::lbfgs_optimize(x,
                                    finalCost,
                                    PathOptimizer::local_point_costFunction,
                                    nullptr,
                                    nullptr,
                                    this,
                                    params);

    if (ret >= 0) {
        pt = x;
    } else {
        ROS_ERROR("[local point opt]:%s", lbfgs::lbfgs_strerror(ret));
	}
}

double PathOptimizer::local_point_costFunction(void *instance, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
    auto* opt_ptr = static_cast<PathOptimizer*>(instance);
    Eigen::Vector2d weight;
    weight << opt_ptr->weight_[0], opt_ptr->weight_[1];
    ROS_INFO_STREAM_ONCE("current weight=" << weight.transpose());

    double fx = 0.0;
    g << 0.0, 0.0;

    double dist;
    Eigen::Vector2d dist_grad;
    opt_ptr->grid_map_->evaluateEDT(x, dist);
    opt_ptr->grid_map_->evaluateFirstGrad(x, dist_grad);

    double dist_error = opt_ptr->local_pt_obs_clearance - dist;

    if (dist_error >= 0) {
        fx += weight(0) * dist_error * dist_error;
        g(0) += -2*weight(0) * dist_error * dist_grad(0);
        g(1) += -2 * weight(0) * dist_error * dist_grad(1);
    }

    Eigen::Vector2d vec_to_end_pt = x - opt_ptr->cur_local_point;
    fx += weight(1) * vec_to_end_pt.norm() * vec_to_end_pt.norm();
    g(0) += 2 * weight(1) * vec_to_end_pt.x();
    g(1) += 2 * weight(1) * vec_to_end_pt.y();

    return fx;
}
