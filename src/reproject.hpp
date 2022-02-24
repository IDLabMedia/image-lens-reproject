#pragma once

#include "config.hpp"

namespace reproject {

enum DataLayout { RGB, RGBA, RGBZ, RGBAZ };

struct Image {
  LensInfo lens;
  int width, height, channels;
  float *data;
  DataLayout data_layout;
};

enum Interpolation {
  NEAREST,
  BILINEAR,
  BICUBIC,
};

void reproject(const Image *in, Image *out, int num_samples,
               Interpolation interpolation);

void post_process(const Image *img, float exposure, float reinhard); 

} // namespace reproject
