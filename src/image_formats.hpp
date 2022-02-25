#pragma once

#include <string>

#include "reproject.hpp"

namespace reproject {

void save_png(const reproject::Image &img, std::string output_file);
void save_exr(const reproject::Image &img, std::string output_file);

reproject::Image read_exr(std::string input_file);
reproject::Image read_png(std::string input_file);

} // namespace reproject
