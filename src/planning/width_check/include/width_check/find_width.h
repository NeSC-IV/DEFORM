/*
HOW TO USE:
    step1: initialize FindMaxFreeRegion, include grid map and maximum formation width
           initMapandMaxGap(..., ...);
    step2: check maximum free region width
           updataFormationGap(...); -- current formation width + one formation interval
           setEndPointandupdateBoundary(..., ...) -- set reference line and update Boundary box
           updatePointsRegion() -- fill environment point to boundary box, then divide the environment into two parts based on a line.
           solve_max_b() -- solve the maximum gap in two lines.
           getWidth() -- get solved width
    option step :
           getWidthBestPoint() -- update local goal to center point between two line end
    debug step (show infomation) : 
           getBox() -- return the boundary box leftdown point and rightup point.
           test(..., ...) -- run after solve_max_b(), and get solved two line that is maximum gap.

           getLocalMap() -- return the local obstacles
           getLocalMapGreater(), getLocalMapLess() -- return obstacles on one side of the line
*/


#include <casadi/casadi.hpp>
#include <plan_env/grid_map.h>
#include <Eigen/Eigen>

class LINE
{
public:
    LINE() = default;
    void setEndpoint(Eigen::Vector2d pt1, Eigen::Vector2d pt2);
    void setCoeff(Eigen::Vector3d coeff);
    Eigen::Vector3d getCoeff();
    Eigen::Vector2d getNormal();
    double getb();
    Eigen::Vector2d getPoint_left();
    Eigen::Vector2d getPoint_right();
    int inLine(Eigen::Vector2d pt);
private:
    Eigen::Vector3d coefficient;
    Eigen::Vector2d left_pt,right_pt;
};

class FindMaxFreeRegion
{
public:
    void initMapandMaxGap(GridMap::Ptr occ_map_, double max_gap);
    void setEndpointandupdateBoundary(Eigen::Vector2d odom, Eigen::Vector2d local_goal);
    bool updatePointsRegion();
    void updateFormationGap(double cur_gap);
    void solve_max_b();
    double getWidth();
    Eigen::Vector2d getDirection();
    vector<Eigen::Vector2d> & getLocalMap();
    vector<Eigen::Vector2d> &getLocalMapGreater();
    vector<Eigen::Vector2d> &getLocalMapLess();

    Eigen::Vector2d getWidthBestPoint();

    vector<Eigen::Vector2d> getBox();
    typedef std::shared_ptr<FindMaxFreeRegion> Ptr;

    void test(std::vector<Eigen::Vector2d>& one_line, std::vector<Eigen::Vector2d>& other_line);
private:
    GridMap::Ptr grid_map_;
    LINE line_middle,line_greater, line_less;
    double max_formation_gap;
    Eigen::Vector2d boundary_min, boundary_max;
    double width;
    std::vector<Eigen::Vector2d> pts_greater, pts_all, pts_less;
};

/*  ============== class LINE memeber function ===================== */

inline void LINE::setEndpoint(Eigen::Vector2d pt1,Eigen::Vector2d pt2) {
    coefficient(0) = pt2.y() - pt1.y();
    coefficient(1) = pt1.x() - pt2.x();
    coefficient(2) = pt2.x() * pt1.y() - pt1.x() * pt2.y();

    left_pt=pt1; //start
    right_pt=pt2; // goal

}

inline int LINE::inLine(Eigen::Vector2d pt) {
    Eigen::Vector2d coeff_vec(coefficient.x(), coefficient.y());
    double value = coeff_vec.dot(pt) + coefficient(2);
    if (abs(value) < 0.0002)
        return 0;
    else if(value > 0)
        return 1;
    else
        return -1;
}

inline Eigen::Vector3d LINE::getCoeff()
{
   return coefficient;
}

inline void LINE::setCoeff(Eigen::Vector3d coeff) {
    coefficient = coeff;
}

inline Eigen::Vector2d LINE::getPoint_right() {
   return right_pt;
}

inline Eigen::Vector2d LINE::getPoint_left() {
    return left_pt;
}

inline Eigen::Vector2d LINE::getNormal() {
    Eigen::Vector2d normal(coefficient.x(), coefficient.y());
    return normal;
}

inline double LINE::getb() {
    return coefficient.z();
}
