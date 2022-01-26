#define CXXOPTS_NO_REGEX 1
#include <ImfArray.h>
#include <ImfInputFile.h>
#include <ImfNamespace.h>
#include <ImfRgba.h>
#include <ImfRgbaFile.h>
#include <lodepng.h>

#include <Tracy.hpp>
#include <cxxopts.hpp>

#include "reproject.hpp"
#include "image_formats.hpp"

int main(int argc, char **argv) {
  // clang-format off
    cxxopts::Options options(argv[0],
        "Reprojection tool for producing a variation of lens\n"
        "configurations based on one reference image given a\n"
        "known lens configuration.");
    options.add_options()
        ("i,input", "Input image to reproject",
         cxxopts::value<std::string>(), "file")
        ("o,output", "Output file",
         cxxopts::value<std::string>(), "file")
        ("s,samples", "Number of samples per dimension for interpolating",
         cxxopts::value<int>()->default_value("1"), "number")

        ("nn", "Nearest neighbor interpolation")
        ("bl", "Bilinear interpolation")
        ("bc", "Bicubic interpolation (default)")

        ("rectilinear", "Output rectilinear image with given FOV.",
         cxxopts::value<float>(), "fov")
        ("h,help", "Show help")
        ;
  // clang-format on
  cxxopts::ParseResult result;

  std::string input_file;
  std::string output_file;
  reproject::Interpolation interpolation = reproject::BICUBIC;
  int num_samples;
  try {
    result = options.parse(argc, argv);
    if (result.count("help")) {
      std::printf("%s\n", options.help().c_str());
      return 0;
    }
    input_file = result["input"].as<std::string>();
    output_file = result["output"].as<std::string>();
    num_samples = result["samples"].as<int>();
  } catch (cxxopts::OptionParseException &e) {
    std::printf("%s\n\n%s\n", e.what(), options.help().c_str());
    return 1;
  } catch (cxxopts::OptionException &e) {
    std::printf("%s\n\n%s\n", e.what(), options.help().c_str());
    return 1;
  }

  int found_interpolation_flag = 0;
  if (result.count("nn")) {
    found_interpolation_flag++;
    interpolation = reproject::NEAREST;
  }
  if (result.count("bl")) {
    found_interpolation_flag++;
    interpolation = reproject::BILINEAR;
  }
  if (result.count("bc")) {
    found_interpolation_flag++;
    interpolation = reproject::BICUBIC;
  }
  if (found_interpolation_flag > 1) {
    std::printf("Cannot specify more than one interpolation method.\n\n%s",
                options.help().c_str());
  }

  std::printf("Reading EXR files: %s\n", input_file.c_str());

  reproject::Image input = reproject::read_exr(input_file);
  input.lens.type = reproject::LensType::FISHEYE_EQUIDISTANT;
  input.lens.fisheye_equidistant.fov = M_PI;
  input.lens.sensor_width = 36.0f;
  input.lens.sensor_height = 36.0f;

  reproject::Image output;
#if 1
  output.lens.type = reproject::LensType::RECTILINEAR;
  output.lens.rectilinear.focal_length = 4.0f;
  output.lens.sensor_width = 36.0f;
  output.lens.sensor_height = 36.0f;
#else
  output.lens = input.lens;
#endif
  output.width = input.width;
  output.height = input.height;
  output.channels = input.channels;
  output.data = new float[output.width * output.height * output.channels];

  std::printf("Reprojecting...\n");
  reproject::reproject(&input, &output, num_samples, interpolation);
  std::printf("Done!\n");

  reproject::save_png(output, output_file);

  delete[] input.data;
  delete[] output.data;

  return 0;
}
