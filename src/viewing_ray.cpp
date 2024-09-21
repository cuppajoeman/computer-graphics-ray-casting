#include "viewing_ray.h"

void viewing_ray(
  const Camera & camera,
  const int i,
  const int j,
  const int width,
  const int height,
  Ray & ray)
{
  ray.origin = camera.e;
  double width_O_image_plane = camera.width; 
  double height_O_image_plane = camera.height; 
  int width_P_num_pixels_O_image_plane = width;
  int height_P_num_pixels_O_image_plane = height;
  double width_O_single_pixel_O_image_plane = width_O_image_plane / width_P_num_pixels_O_image_plane;
  double height_O_single_pixel_O_image_plane = height_O_image_plane / height_P_num_pixels_O_image_plane;

  // the components of the image pixels are specified from top left with positive x and negative y this explains the following:
  double pos_x_P_camera_coords = -camera.width / 2 + width_O_single_pixel_O_image_plane * (j + 0.5);
  double pos_y_P_camera_coords = camera.height / 2 - height_O_single_pixel_O_image_plane * (i + 0.5);

  Eigen::Vector3d pixel_loc = - camera.d * camera.w + pos_x_P_camera_coords * camera.u + pos_y_P_camera_coords * camera.v;
  ray.direction = pixel_loc;
}

