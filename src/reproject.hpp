#pragma once

#include "config.hpp"

namespace reproject {

struct Image {
    LensInfo lens;
    int width, height, channels;
    float *data;
};

enum Interpolation {
    NEAREST,
    BILINEAR,
    BICUBIC,
};

void reproject(const Image *in, Image *out, int num_samples,
               Interpolation interpolation);

}  // namespace reproject
