#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

double signum(double x);

double mod(double value, double modulus);

double intbound(double s, double ds);

class RayCaster {
private:
  /* data */
  Eigen::Vector2d start_;
  Eigen::Vector2d end_;
  Eigen::Vector2d direction_;
  Eigen::Vector2d min_;
  Eigen::Vector2d max_;
  int x_;
  int y_;
  int endX_;
  int endY_;
  double maxDist_;
  double dx_;
  double dy_;
  int stepX_;
  int stepY_;
  double tMaxX_;
  double tMaxY_;
  double tDeltaX_;
  double tDeltaY_;
  double dist_;

  int step_num_;

public:
  RayCaster(/* args */) {
  }
  ~RayCaster() {
  }

  bool setInput(const Eigen::Vector2d& start,
                const Eigen::Vector2d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  bool step(Eigen::Vector2d& ray_pt);
};

#endif  // RAYCAST_H_