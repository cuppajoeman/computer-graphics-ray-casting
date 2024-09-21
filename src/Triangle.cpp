#include "Triangle.h"
#include "Ray.h"
#include <Eigen/Dense>

bool Triangle::intersect(
  const Ray & ray, const double min_t, double & t, Eigen::Vector3d & n) const
{
  auto c0 = std::get<0>(corners);
  auto c1 = std::get<1>(corners);
  auto c2 = std::get<2>(corners);
  
  auto ta = c1 - c0;
  auto tb = c2 - c0;

  // we require that e + d*t = alpha ta + beta tb + c0
  // this implies that:
  // e - c0 = alpha ta + beta tb - dt
  // that can be written in matrix form b = Ax then invert

  Eigen::Matrix3d A;
  A.col(0) = ta;
  A.col(1) = tb;
  A.col(2) = -ray.direction;

  Eigen::Vector3d b = ray.origin - c0;


  // could occur if triangle is paralell with ray
  if (not A.fullPivLu().isInvertible()) {
    return false;
  } 

  Eigen::Vector3d solution = A.colPivHouseholderQr().solve(b);

  double alpha = solution[0];
  double beta = solution[1];
  t = solution[2];

  if (t < min_t || alpha < 0 || beta < 0 || alpha + beta > 1) {
    return false;
  }

  n = ta.cross(tb).normalized();

  return true;
}
