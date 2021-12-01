#pragma once

#include "config.hpp"

namespace reproject {

struct Image {
    LensInfo lens;
    int width, height, channels;
    float *data;
};

void reproject(const Image *in, Image *out);

}  // namespace reproject
