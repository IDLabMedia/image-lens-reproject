#include "image_formats.hpp"

#include <ImfArray.h>
#include <ImfInputFile.h>
#include <ImfNamespace.h>
#include <ImfRgba.h>
#include <ImfRgbaFile.h>
#include <lodepng.h>

#include <cmath>

#include "Tracy.hpp"

namespace reproject {

void save_png(const reproject::Image &output, std::string output_file) {
  ZoneScoped;
  uint8_t *image_buf = new uint8_t[output.width * output.height * 4];
  float vmax = 0.0f;
  float vmin = 1.0f;
  for (int y = 0; y < output.height; ++y) {
    for (int x = 0; x < output.width; ++x) {
      for (int c = 0; c < output.channels; ++c) {
        float s = output.data[((y * output.width) + x) * output.channels + c];
        vmax = std::max(vmax, s);
        vmin = std::min(vmin, s);
        // s = s / (s + 1.0f);
        s = std::max(0.0f, std::min(1.0f, s));
        s = std::pow(s, 1.0f / 2.2f);
        uint8_t d = uint8_t(255.9f * s);
        image_buf[((y * output.width) + x) * 4 + c] = d;
      }
      if (output.channels != 4) {
        image_buf[((y * output.width) + x) * 4 + 3] = 255;
      }
    }
  }
  {
    ZoneScopedN("lodepng::encode");
    lodepng::encode(output_file.c_str(), image_buf, output.width,
                    output.height);
  }
  //std::printf("min: %f  max: %f\n", vmin, vmax);
  delete[] image_buf;
}

reproject::Image read_png(std::string input_file) {
  ZoneScoped;
  std::vector<uint8_t> data;
  lodepng::load_file(data, input_file);

  unsigned int w, h;
  std::vector<uint8_t> color_data;
  lodepng::decode(color_data, w, h, data);

  reproject::Image input;
  input.width = w;
  input.height = h;
  input.channels = 3;
  input.data = new float[input.width * input.height * input.channels];
  {
    ZoneScopedN("convert EXR to float buffer");
    for (int y = 0; y < input.height; ++y) {
      for (int x = 0; x < input.width; ++x) {
        uint8_t *p = &color_data[(y * w + x) * 4];
        int oo = (y * input.width + x) * input.channels;
        input.data[oo + 0] = std::pow(float(p[0]) / 255.0f, 2.2f);
        input.data[oo + 1] = std::pow(float(p[1]) / 255.0f, 2.2f);
        input.data[oo + 2] = std::pow(float(p[2]) / 255.0f, 2.2f);
      }
    }
  }

  return input;
}

static void readRgba1(const char fileName[], Imf::Array2D<Imf::Rgba> &pixels,
                      int &width, int &height) {
  ZoneScoped;
  //
  // Read an RGBA image using class RgbaInputFile:
  //
  //	- open the file
  //	- allocate memory for the pixels
  //	- describe the memory layout of the pixels
  //	- read the pixels from the file
  //

  Imf::RgbaInputFile file(fileName);
  Imath::Box2i dw = file.dataWindow();

  width = dw.max.x - dw.min.x + 1;
  height = dw.max.y - dw.min.y + 1;
  pixels.resizeErase(height, width);

  file.setFrameBuffer(&pixels[0][0] - dw.min.x - dw.min.y * width, 1, width);
  file.readPixels(dw.min.y, dw.max.y);
}

reproject::Image read_exr(std::string input_file) {
  ZoneScoped;
  Imf::Array2D<Imf::Rgba> data;
  int width, height;
  readRgba1(input_file.c_str(), data, width, height);

  reproject::Image input;
  input.width = width;
  input.height = height;
  input.channels = 3;
  input.data = new float[input.width * input.height * input.channels];
  {
    ZoneScopedN("convert EXR to float buffer");
    for (int y = 0; y < input.height; ++y) {
      for (int x = 0; x < input.width; ++x) {
        Imf::Rgba p = data[y][x];
        input.data[(y * input.width + x) * input.channels + 0] = float(p.r);
        input.data[(y * input.width + x) * input.channels + 1] = float(p.g);
        input.data[(y * input.width + x) * input.channels + 2] = float(p.b);
      }
    }
  }
  return input;
}

} // namespace reproject
