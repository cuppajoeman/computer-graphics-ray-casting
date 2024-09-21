#include "Sphere.h"
#include "Ray.h"
#include <vector>
#include <limits>
#include <cmath>

std::vector<double> solve_quadratic(double a, double b, double c) {
    std::vector<double> solutions;

    if (a == 0) {
        if (b != 0) {
            solutions.push_back(-c / b);
        }
        return solutions;
    }

    double discriminant = b * b - 4 * a * c;

    if (discriminant > 0) {
        double sqrt_discriminant = std::sqrt(discriminant);
        solutions.push_back((-b + sqrt_discriminant) / (2 * a));
        solutions.push_back((-b - sqrt_discriminant) / (2 * a));
    } else if (discriminant == 0) {
        solutions.push_back(-b / (2 * a));
    }

    return solutions;
}

bool Sphere::intersect(
  const Ray & ray, const double min_t, double & t, Eigen::Vector3d & n) const
{
  Eigen::Vector3d d = ray.direction;
  Eigen::Vector3d e = ray.origin;
  Eigen::Vector3d c = center;
  double r = radius;

  Eigen::Vector3d e_minus_c = e - c;

  std::vector<double> solutions = solve_quadratic(
    d.dot(d),
    2 * d.dot(e_minus_c),
    e_minus_c.dot(e_minus_c) - r * r
  );

  if (solutions.empty()) {
    return false;
  }

  bool at_least_one_valid_solution = false;
  double closest_t_val = std::numeric_limits<double>::infinity();

  for (double solution : solutions) {
    if (solution >= min_t) {
      at_least_one_valid_solution = true;
      if (solution < closest_t_val) {
        closest_t_val = solution;
      }
    }
  }

  if (!at_least_one_valid_solution) {
    return false;
  }

  t = closest_t_val;
  Eigen::Vector3d point_on_sphere = ray.origin + ray.direction * t;
  n = (point_on_sphere - c).normalized();

  return true;
}

