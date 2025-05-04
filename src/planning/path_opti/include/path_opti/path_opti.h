#include <plan_env/grid_map.h>
#include <path_searching/dyn_a_star.h>
#include "path_opti/lbfgs.hpp"

class PathOptimizer
{
public:
    PathOptimizer() = default;
    void setGridMapandObsClearance(GridMap::Ptr map_, double clearance);
    int startOpt(std::vector<Eigen::Vector2d>& msg);
    void ESDFAndLocalPointOpt(Eigen::Vector2d& pt, double obs_clear);
    void setWeightToLocalOpt(double w1, double w2);
    typedef std::shared_ptr<PathOptimizer> Ptr;
private:
    static double costFunction(void *instance,
                        const Eigen::VectorXd &x,
                        Eigen::VectorXd &g);
    static int monitorProgress(void *instance,
                        const Eigen::VectorXd &x,
                        const Eigen::VectorXd &g,
                        const double fx,
                        const double step,
                        const int k,
                        const int ls);
    static double local_point_costFunction(void *instance,
                               const Eigen::VectorXd &x,
                               Eigen::VectorXd &g);
private:
    GridMap::Ptr grid_map_;
    double obs_clearance;
    double local_pt_obs_clearance;
    Eigen::Vector2d cur_local_point;
    double weight_[2];

};
