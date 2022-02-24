#include "image_formats.hpp"

#include <ImfArray.h>
#include <ImfChannelList.h>
#include <ImfFrameBuffer.h>
#include <ImfHeader.h>
#include <ImfInputFile.h>
#include <ImfNamespace.h>
#include <ImfOutputFile.h>
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
  input.data_layout = reproject::RGB;

  return input;
}

reproject::Image read_exr(std::string input_file) {
  ZoneScoped;
  using namespace Imf;

  InputFile file(input_file.c_str());
  Imath::Box2i dw = file.header().dataWindow();

  reproject::Image input;
  input.width = dw.max.x - dw.min.x + 1;
  input.height = dw.max.y - dw.min.y + 1;
  input.channels = 0;

  std::vector<half *> halfData;
  std::vector<std::string> channel_names;

  FrameBuffer fb;
  bool found_A = false;
  bool found_Z = false;

  // Figure out which channels are present
  const ChannelList &channels = file.header().channels();
  for (auto it = channels.begin(); it != channels.end(); ++it) {
    std::string chname = it.name();
    channel_names.push_back(chname);
    const Channel &channel = it.channel();

    found_A |= chname == "A";
    found_Z |= chname == "Z";
  }

  if (found_A && found_Z) {
    input.data_layout = reproject::RGBAZ;
  } else if (found_A) {
    input.data_layout = reproject::RGBA;
  } else if (found_Z) {
    input.data_layout = reproject::RGBZ;
  } else {
    input.data_layout = reproject::RGB;
  }

  // Prepare the frame buffer for the selected channels.
  input.channels = channel_names.size();
  for (auto channel_name : channel_names) {
    ZoneScopedN("prepare_channel");
    half *buf = new half[input.width * input.height];
    size_t dts = sizeof(*buf);
    fb.insert(channel_name, Slice{HALF, (char *)buf, dts, input.width * dts});
    halfData.push_back(buf);
  }

  {
    ZoneScopedN("read_pixels()");
    file.setFrameBuffer(fb);
    file.readPixels(dw.min.y, dw.max.y);
  }

  {
    ZoneScopedN("convert EXR to float buffer");
    input.data = new float[input.width * input.height * input.channels];
    for (int c = 0; c < input.channels; ++c) {
      std::string chname = channel_names[c];
      int dstC = 0;

      // clang-format off
      if (chname == "R") dstC = 0;
      if (chname == "G") dstC = 1;
      if (chname == "B") dstC = 2;
      if (input.data_layout == RGBA) {
        if (chname == "A") dstC = 3;
        if (chname == "Z") dstC = -1000;
      } else if (input.data_layout == RGBZ) {
        if (chname == "A") dstC = -1000;
        if (chname == "Z") dstC = 3;
      } else if (input.data_layout == RGBAZ) {
        if (chname == "A") dstC = 3;
        if (chname == "Z") dstC = 4;
      }
      // clang-format off

      for (int y = 0; y < input.height; ++y) {
        for (int x = 0; x < input.width; ++x) {
          int ro = y * input.width + x;
          input.data[(y * input.width + x) * input.channels + dstC] =
              float(halfData[c][ro]);
        }
      }
    }
  }

  for (half *ptr : halfData) {
    ZoneScopedN("free_f16");
    delete[] ptr;
  }

  return input;
}

void save_exr(const reproject::Image &output, std::string output_file) {
  ZoneScoped;
  using namespace Imf;

  std::vector<std::string> channel_names = {"R", "G", "B", "A", "Z"};
  if (output.channels > channel_names.size()) {
    throw std::runtime_error("cannot save exr with more than 5 channels.");
  }
  std::vector<half *> halfData;
  FrameBuffer fb;
  Header header(output.width, output.height);
  for (int i = 0; i < output.channels; ++i) {
    ZoneScopedN("convert_f32_to_f16");
    header.channels().insert(channel_names[i], Channel(HALF));
    half *buf = new half[output.width * output.height];
    for (int y = 0; y < output.height; ++y) {
      for (int x = 0; x < output.width; ++x) {
        int ro = (output.width * y + x) * output.channels + i;
        buf[y * output.width + x] = output.data[ro];
      }
    }
    size_t dts = sizeof(*buf);
    Slice slice{HALF, (char *)buf, dts, dts * output.width};
    fb.insert(channel_names[i], slice);
    halfData.push_back(buf);
  }

  {
    ZoneScopedN("write");
    OutputFile of(output_file.c_str(), header);
    of.setFrameBuffer(fb);
    of.writePixels(output.height);
  }

  for (half *ptr : halfData) {
    ZoneScopedN("free_f16");
    delete[] ptr;
  }
}

} // namespace reproject
