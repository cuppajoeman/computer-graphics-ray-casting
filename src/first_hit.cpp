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


  ////////////////////////////////////////////////////////////////////////////
  // Replace with your code here:
  // we need to use the shape equations
  
  // don't render things have min value less than min_t
  // sphere hit check, recall a spehere is specified by (c, r) 
  // has equation (p - c) * (p - c) - r^2 = 0 means your on the surface of the sphere
  // since p = ray_orig + t * ray_dir, we can then plug that in for p to get 
  // (ray_orig + t * ray_dir - c) * (ray_orig + t * ray_dir - c) - r^2 = 0 which can then be expanded
  // ray_dir * ray_dir t^2 + 2 ray_dir * (ray_orig - c) + (ray_orig - c) * ( ray_orig - c) - r^2 = 0
  // then use the quadratic equation to solve for t = (-b +- sqrt(b^2 - 4ac)) / 2a, 
  // the surface normal of a sphere at the intersection point i is just (i - c) normalized
  
  // for a plane intersection, we recall that the equation for a plane is given by 
  // (p - p0) * n = 0, so plugging in we have (ray_orig + ray_dir * t - p0) * n = 0 so
  // t = (p0 * n - ray_orig * n) / (d * n)
  // since this could result in zero division then make sure to do the check first because there would be no intersection in that case
  
  // for a ray triangle intersection, we recall a triangle is defined by three points, firs twe solve it for the canonical triangle with vertices (0, 0), (0, 1) (1, 0) called a b c then we want to see if there is a t such that 
  // ray_orig + ray_dir * t = alpha t1 + beta t2 + a
  // ray_orig - a = alpha t1 + beat t2 - ray_dir * t 
  // which can be written as 
  //
  //  |  |  |    alp    1
  //  t1 t2 rd   bet =  0 - a
  //  |  |  |    t      1
  //    A        x      b
  //
  // x = A^-1 b using eigen invert but if no inverse exists then the triang is paralell

  return hit_found;
  ////////////////////////////////////////////////////////////////////////////
}

