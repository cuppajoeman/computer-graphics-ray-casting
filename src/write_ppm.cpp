#include "write_ppm.h"
#include <fstream>
#include <cassert>
#include <iostream>

bool write_ppm(
  const std::string & filename,
  const std::vector<unsigned char> & data,
  const int width,
  const int height,
  const int num_channels)
{
  assert(
    (num_channels == 3 || num_channels == 1) &&
    ".ppm only supports RGB or grayscale images");

  std::ofstream ppm_file(filename);

  if (!ppm_file) {
    std::cerr << "Error: Could not open file " << filename << std::endl;
    return false;
  }

  // p2 for grayscale (num_channels == 1), p3 for rgb (num_channels == 3)
  if (num_channels == 1) {
    ppm_file << "P2\n";  // grayscale ascii format
  } else if (num_channels == 3) {
    ppm_file << "P3\n";  // RGB ASCII format
  }

  // write image dimensions and maximum color value
  ppm_file << width << " " << height << "\n255";

  for (int i = 0; i < height * width * num_channels; i += num_channels) {
    if (i % width == 0) {
      ppm_file << "\n";
    }
    for (int j = 0; j < num_channels; ++j) {
      // convert each `unsigned char` to an integer and output it as a string
      ppm_file << static_cast<int>(data[i + j]) << " ";
    }
  }

  ppm_file.close();
  return true;
}
