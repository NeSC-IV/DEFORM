#include "width_check/find_width.h"

void FindMaxFreeRegion::initMapandMaxGap(GridMap::Ptr occ_map_, double max_gap)
{
    grid_map_ = occ_map_;
    max_formation_gap = max_gap;
};

void FindMaxFreeRegion::updateFormationGap(double cur_gap) {
    max_formation_gap = cur_gap;
}

void FindMaxFreeRegion::setEndpointandupdateBoundary(Eigen::Vector2d odom, Eigen::Vector2d local_goal) {
    line_middle.setEndpoint(odom, local_goal);
    Eigen::Vector2d midpoint = (odom + local_goal) / 2;
    std::vector<Eigen::Vector2d> normal_pts;
    Eigen::Vector2d normal_vec(line_middle.getCoeff().x(), line_middle.getCoeff().y());
    normal_pts.resize(4);
    normal_pts[0] = odom + normal_vec / normal_vec.norm() * max_formation_gap / 2.0;
    normal_pts[1] = odom - normal_vec / normal_vec.norm() * max_formation_gap / 2.0;
    normal_pts[2] = local_goal + normal_vec / normal_vec.norm() * max_formation_gap / 2.0;
    normal_pts[3] = local_goal - normal_vec / normal_vec.norm() * max_formation_gap / 2.0;

    double min_x, min_y, max_x, max_y;
    double local_bound = sqrt(2) * max_formation_gap;
    min_x = midpoint.x() + local_bound;
    max_x = midpoint.x() - local_bound;
    min_y = midpoint.y() + local_bound;
    max_y = midpoint.y() - local_bound;

    for (int i = 0; i < normal_pts.size(); i++) {
        min_x = min(min_x, normal_pts[i].x());
        max_x = max(max_x, normal_pts[i].x());
        min_y = min(min_y, normal_pts[i].y());
        max_y = max(max_y, normal_pts[i].y());
    }
    boundary_min(0) = min_x;
    boundary_min(1) = min_y;
    boundary_max(0) = max_x;
    boundary_max(1) = max_y;
}

vector<Eigen::Vector2d> FindMaxFreeRegion::getBox() {
    std::vector<Eigen::Vector2d> tmp_box;
    Eigen::Vector2d box_pt;
    box_pt << boundary_min.x(), boundary_min.y();
    tmp_box.push_back(box_pt);
    box_pt << boundary_min.x(), boundary_max.y();
    tmp_box.push_back(box_pt);
    box_pt << boundary_max.x(), boundary_max.y();
    tmp_box.push_back(box_pt);
    box_pt << boundary_max.x(), boundary_min.y();
    tmp_box.push_back(box_pt);
    box_pt << boundary_min.x(), boundary_min.y();
    tmp_box.push_back(box_pt);
    return tmp_box;
}

bool FindMaxFreeRegion::updatePointsRegion() {
    grid_map_->getBoundaryMap(boundary_min, boundary_max, pts_all);

    pts_greater.clear();
    pts_less.clear();
    for (int pt_index = 0; pt_index < pts_all.size(); pt_index++)
    {
        int posInLine = line_middle.inLine(pts_all[pt_index]);
        if (posInLine == 0)
            return false;//obs in path
        else if (posInLine > 0)
            pts_greater.push_back(pts_all[pt_index]);
        else
            pts_less.push_back(pts_all[pt_index]);
    }
    return true;
}

void FindMaxFreeRegion::solve_max_b() {
    casadi::Opti opti_;
    casadi::MX b_;

    b_ = opti_.variable(2, 1);

    casadi::MX b_ref = opti_.parameter();
    casadi::MX max_margin = opti_.parameter();
    casadi::MX A_ref = opti_.parameter(2);
    casadi::MX obs_greater_ = opti_.parameter(2, pts_greater.size());
    casadi::MX obs_less_ = opti_.parameter(2, pts_less.size());

    casadi::MX J_ = -(b_(0) - b_(1)) * (b_(0) - b_(1));
    opti_.minimize(J_);

    opti_.subject_to((b_(1) - b_ref) * (b_(1) - b_ref) < max_margin * max_margin * casadi::MX::dot(A_ref, A_ref));
    opti_.subject_to((b_(0) - b_ref) * (b_(0) - b_ref) < max_margin * max_margin * casadi::MX::dot(A_ref, A_ref));

    opti_.subject_to(b_(0) < b_ref);
    for (int gt_index = 0; gt_index < pts_greater.size(); gt_index++)
        opti_.subject_to(b_(0) > -casadi::MX::dot(A_ref, obs_greater_(casadi::Slice(), gt_index)));

    opti_.subject_to(b_(1) > b_ref);
    for (int le_index = 0; le_index < pts_less.size(); le_index++)
        opti_.subject_to(b_(1) < -casadi::MX::dot(A_ref, obs_less_(casadi::Slice(), le_index)));

    opti_.set_value(b_ref, line_middle.getb());
    opti_.set_value(A_ref, std::vector<double>{line_middle.getCoeff().x(), line_middle.getCoeff().y()});
    opti_.set_value(max_margin, max_formation_gap * 0.5);
    for (int gt_index = 0; gt_index < pts_greater.size(); gt_index++)
        opti_.set_value(obs_greater_(casadi::Slice(), gt_index),
                        std::vector<double>{pts_greater[gt_index].x(), pts_greater[gt_index].y()});

    for (int le_index = 0; le_index < pts_less.size(); le_index++)
        opti_.set_value(obs_less_(casadi::Slice(), le_index),
                        std::vector<double>{pts_less[le_index].x(), pts_less[le_index].y()});

    casadi::Dict solver_options_;
    solver_options_["ipopt.print_level"] = 0;
    solver_options_["ipopt.sb"] = "yes";
    solver_options_["ipopt.max_iter"] = 2000;
    solver_options_["ipopt.tol"] = 1e-8;
    solver_options_["print_time"] = 0;
    solver_options_["ipopt.acceptable_obj_change_tol"] = 1e-6;
    opti_.solver("ipopt", solver_options_);

    opti_.set_initial(b_, std::vector<double>{line_middle.getb(), line_middle.getb()});

    try
    {
        casadi::OptiSol solution = opti_.solve();
        std::vector<double> result = solution.value(b_).get_elements();

        Eigen::Vector3d coeff_temp = line_middle.getCoeff();
        coeff_temp.z() = result[0];
        line_greater.setCoeff(coeff_temp);
        coeff_temp.z() = result[1];
        line_less.setCoeff(coeff_temp);
    }
    catch(casadi::CasadiException& w)
    {
        std::cout << w.what() << std::endl;
    }
}

double FindMaxFreeRegion::getWidth() {
    double d_less = std::abs(line_middle.getb() - line_less.getb()) / line_middle.getNormal().norm();
    double d_greater = std::abs(line_middle.getb() - line_greater.getb()) / line_middle.getNormal().norm();
    width = d_less + d_greater;
    return width;
}

vector<Eigen::Vector2d> & FindMaxFreeRegion::getLocalMap() {
    return pts_all;
}

vector<Eigen::Vector2d> & FindMaxFreeRegion::getLocalMapGreater() {
    return pts_greater;
}

vector<Eigen::Vector2d> & FindMaxFreeRegion::getLocalMapLess() {
    return pts_less;
}

Eigen::Vector2d FindMaxFreeRegion::getWidthBestPoint() {
    double mid_b = (line_less.getb() + line_greater.getb()) * 0.5;
    LINE mid_line;
    Eigen::Vector3d coeff(line_middle.getCoeff());
    coeff.z() = mid_b;
    mid_line.setCoeff(coeff);

    double d_mid = std::abs(line_middle.getb() - mid_line.getb()) / line_middle.getNormal().norm();
    Eigen::Vector2d pt_temp = line_middle.getPoint_right() + line_middle.getNormal() / line_middle.getNormal().norm() * d_mid;
    if (mid_line.inLine(pt_temp) != 0)
        pt_temp = line_middle.getPoint_right() - line_middle.getNormal() / line_middle.getNormal().norm() * d_mid;

   return pt_temp;
}

Eigen::Vector2d FindMaxFreeRegion::getDirection() {
    return line_middle.getNormal();
}

void FindMaxFreeRegion::test(std::vector<Eigen::Vector2d>& one_line,std::vector<Eigen::Vector2d>& other_line)
{
    //greater
    double d_greater = std::abs(line_middle.getb() - line_greater.getb()) / line_middle.getNormal().norm();
    Eigen::Vector2d pt1_temp = line_middle.getPoint_left() + line_middle.getNormal() / line_middle.getNormal().norm() * d_greater;
    if (line_greater.inLine(pt1_temp) != 0)
        pt1_temp = line_middle.getPoint_left() - line_middle.getNormal() / line_middle.getNormal().norm() * d_greater;

    Eigen::Vector2d pt2_temp = line_middle.getPoint_right() + line_middle.getNormal() / line_middle.getNormal().norm() * d_greater;

    if (line_greater.inLine(pt2_temp) != 0)
        pt2_temp = line_middle.getPoint_right() - line_middle.getNormal() / line_middle.getNormal().norm() * d_greater;
    vector<Eigen::Vector2d> greater_end;
    greater_end.resize(2);
    greater_end[0] = pt1_temp;
    greater_end[1] = pt2_temp;

    //less
    double d_less = std::abs(line_middle.getb() - line_less.getb()) / line_middle.getNormal().norm();
    pt1_temp = line_middle.getPoint_left() + line_middle.getNormal() / line_middle.getNormal().norm() * d_less;
    if (line_less.inLine(pt1_temp) != 0)
        pt1_temp = line_middle.getPoint_left() - line_middle.getNormal() / line_middle.getNormal().norm() * d_less;

    pt2_temp = line_middle.getPoint_right() + line_middle.getNormal() / line_middle.getNormal().norm() * d_less;

    if (line_less.inLine(pt2_temp) != 0)
        pt2_temp = line_middle.getPoint_right() - line_middle.getNormal() / line_middle.getNormal().norm() * d_less;
    vector<Eigen::Vector2d> less_end;
    less_end.resize(2);
    less_end[0] = pt1_temp;
    less_end[1] = pt2_temp;

    one_line = greater_end;
    other_line = less_end;
}
