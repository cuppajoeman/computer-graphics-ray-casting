#include "Plane.h"
#include "Ray.h"

bool Plane::intersect(
  const Ray & ray, const double min_t, double & t, Eigen::Vector3d & n) const
{
  n = normal; // normal is the same everywhere on a plane.
  auto parallel = ray.direction.dot(normal) == 0;

  if (parallel) {
    bool laying_on_plane = (ray.origin - point).dot(n) == 0;
    if (laying_on_plane) {
      t = min_t;
      return true;
    } else {
      return false;
    }
  }

  auto t_of_intersection = ((point - ray.origin).dot(normal)) / (ray.direction.dot(normal));

  if (t_of_intersection > 0) {
    int grab_me = 0;
  }

  if (t_of_intersection > min_t) {
    t = t_of_intersection;
    return true;
  } else {
    return false;
  }


}

