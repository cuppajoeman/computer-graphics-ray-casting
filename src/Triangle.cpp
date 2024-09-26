#include "Triangle.h"
#include "Ray.h"
#include <Eigen/Dense>

bool Triangle::intersect(
  const Ray & ray, const double min_t, double & t, Eigen::Matrix<double, 3, 1> & n) const
{
  const Eigen::Matrix<double, 3, 1>& c0 = std::get<0>(corners);
  const Eigen::Matrix<double, 3, 1>& c1 = std::get<1>(corners);
  const Eigen::Matrix<double, 3, 1>& c2 = std::get<2>(corners);

  Eigen::Matrix<double, 3, 1> ta = c1 - c0;
  Eigen::Matrix<double, 3, 1> tb = c2 - c0;

  Eigen::Matrix<double, 3, 3> A;
  A.col(0).noalias() = ta;
  A.col(1).noalias() = tb;
  A.col(2).noalias() = -ray.direction;  // use noalias to avoid temporary

  Eigen::Matrix<double, 3, 1> b = ray.origin - c0;


// could occur if triangle is paralell with ray
  /*if (not A.fullPivLu().isInvertible()) {*/
  /*  return false;*/
  /*} */
  /**/
  /*Eigen::Vector3d solution = A.colPivHouseholderQr().solve(b);*/
  /**/
  Eigen::Matrix<double, 3, 1> solution = A.lu().solve(b);

  double alpha = solution[0];
  double beta = solution[1];
  t = solution[2];

  if (t < min_t || alpha < 0 || beta < 0 || alpha + beta > 1) {
    return false;
  }

  n.noalias() = ta.cross(tb).normalized();  // use noalias for in-place cross product

  return true;
}
