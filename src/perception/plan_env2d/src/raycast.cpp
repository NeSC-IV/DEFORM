#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <plan_env/raycast.h>

int signum(int x) {
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds) {
  // Find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0) {
    return intbound(-s, -ds);
  } else {
    s = mod(s, 1);
    // problem is now s+t*ds = 1
    return (1 - s) / ds;
  }
}

bool RayCaster::setInput(const Eigen::Vector2d& start,
                         const Eigen::Vector2d& end /* , const Eigen::Vector3d& min,
                         const Eigen::Vector3d& max */) {
  start_ = start;
  end_ = end;

  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  direction_ = (end_ - start_);
  maxDist_ = direction_.squaredNorm();

  // Break out direction vector.
  dx_ = endX_ - x_;
  dy_ = endY_ - y_;

  // Direction to increment x,y,z when stepping.
  stepX_ = (int)signum((int)dx_);
  stepY_ = (int)signum((int)dy_);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);

  // The change in t when taking a step (always positive).
  tDeltaX_ = ((double)stepX_) / dx_;
  tDeltaY_ = ((double)stepY_) / dy_;

  dist_ = 0;

  step_num_ = 0;

  // Avoids an infinite loop.
  if (stepX_ == 0 && stepY_ == 0 )
    return false;
  else
    return true;
}

bool RayCaster::step(Eigen::Vector2d& ray_pt) {
  // if (x_ >= min_.x() && x_ < max_.x() && y_ >= min_.y() && y_ < max_.y() &&
  // z_ >= min_.z() && z_ <
  // max_.z())
  ray_pt = Eigen::Vector2d(x_, y_);

  // step_num_++;

  // dist_ = (Eigen::Vector3d(x_, y_, z_) - start_).squaredNorm();

  if (x_ == endX_ && y_ == endY_ ) {
    return false;
  }

  // if (dist_ > maxDist_)
  // {
  //   return false;
  // }

  // tMaxX stores the t-value at which we cross a cube boundary along the
  // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
  // chooses the closest cube boundary. Only the first case of the four
  // has been commented in detail.
  if (tMaxX_ < tMaxY_) {
      // Update which cube we are now in.
      x_ += stepX_;
      // Adjust tMaxX to the next X-oriented boundary crossing.
      tMaxX_ += tDeltaX_;
  }
  else
  {
    y_ += stepY_;
    tMaxY_ += tDeltaY_;
  }
  return true;
}

// tMaxX stores the t-value at which we cross a cube boundary along the X axis, and similarly for Y and Z. Therefore, choosing the least tMax chooses the closest cube boundary. Only the first case of the four has been commented in detail.