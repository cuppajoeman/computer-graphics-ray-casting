#include "first_hit.h"
#include <Eigen/src/Core/Matrix.h>

bool first_hit(
  const Ray & ray, 
  const double min_t,
  const std::vector< std::shared_ptr<Object> > & objects,
  int & hit_id, 
  double & t,
  Eigen::Vector3d & n)
{

  assert(objects.size() != 0);

  bool hit_found = false;

  double t_P_closest_P_so_far = std::numeric_limits<double>::infinity();
  Eigen::Vector3d n_P_closest_P_so_far;
  int hit_id_P_closest_P_so_far;

  int obj_idx = 0;
  for (auto &obj : objects) {
    if (obj->intersect(ray, min_t, t,n)) {
      if (t <= t_P_closest_P_so_far) {
        hit_found = true;
        t_P_closest_P_so_far = t;
        n_P_closest_P_so_far = n;
        hit_id_P_closest_P_so_far = obj_idx;
      }
    }
    obj_idx++;
  }

  t = t_P_closest_P_so_far;
  n = n_P_closest_P_so_far;
  hit_id = hit_id_P_closest_P_so_far;

  return hit_found;
}

