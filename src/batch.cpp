#define CXXOPTS_NO_REGEX 1
#include <Tracy.hpp>
#include <cxxopts.hpp>

#include "image_formats.hpp"
#include "reproject.hpp"
#include <cmath>
#include <ctpl_stl.h>
#include <ghc/filesystem.hpp>

namespace fs = ghc::filesystem;

int main(int argc, char **argv) {
  // clang-format off
  cxxopts::Options options(argv[0],
    "Reprojection tool for producing a variation of lens\n"
    "configurations based on one reference image given a\n"
    "known lens configuration.");
  options.add_options()
    ("cfg", "Input JSON file containing lens and camera "
            "settings of the input images.",
     cxxopts::value<std::string>(), "json-file")
    ("i,input", "Input directory containing images to reproject.",
     cxxopts::value<std::string>(), "file")
    ("o,output", "Output directory to put the reprojected images.",
     cxxopts::value<std::string>(), "file")
    ("s,samples", "Number of samples per dimension for interpolating",
     cxxopts::value<int>()->default_value("1"), "number")

    ("nn", "Nearest neighbor interpolation")
    ("bl", "Bilinear interpolation")
    ("bc", "Bicubic interpolation (default)")

    ("scale", "Output scale, as a percentage of the input size.",
     cxxopts::value<int>()->default_value("100"), "percentage")
    ("rectilinear", "Output rectilinear image with given FOV.",
     cxxopts::value<float>(), "fov")

    ("j,parallel", "Number of parallal images to process.",
     cxxopts::value<int>()->default_value("1"), "threads")
    ("h,help", "Show help")
    ;
  // clang-format on

  cxxopts::ParseResult result;
  int num_threads = 1;
  int num_samples = 1;
  std::string input_dir;
  std::string output_dir;
  std::string cfg_file;
  int scale;
  try {
    result = options.parse(argc, argv);
    if (result.count("help")) {
      std::printf("%s\n", options.help().c_str());
      return 0;
    }
    input_dir = result["input"].as<std::string>();
    output_dir = result["output"].as<std::string>();
    cfg_file = result["cfg"].as<std::string>();
    num_samples = result["samples"].as<int>();
    num_threads = result["parallel"].as<int>();
    scale = result["scale"].as<int>();
  } catch (cxxopts::OptionParseException &e) {
    std::printf("%s\n\n%s\n", e.what(), options.help().c_str());
    return 1;
  } catch (cxxopts::OptionException &e) {
    std::printf("%s\n\n%s\n", e.what(), options.help().c_str());
    return 1;
  }

  reproject::Interpolation interpolation = reproject::BICUBIC;
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
    std::printf("Cannot specify more than one interpolation method.\n\n");
    std::printf("%s", options.help().c_str());
  }

  fs::directory_iterator end;
  fs::directory_iterator it{fs::path(input_dir)};

  ctpl::thread_pool pool(num_threads);
  for (; it != end; ++it) {
    if (it->is_regular_file()) {
      fs::path p = *it;
      if (p.extension() == ".exr" || p.extension() == ".png") {
        pool.push([p, num_samples, interpolation, output_dir, scale](int) {
          std::printf("%s\n", p.c_str());

          reproject::Image input = reproject::read_exr(p.string());
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
#endif
          output.width = input.width * scale / 100;
          output.height = input.height * scale / 100;
          output.channels = input.channels;
          output.data =
              new float[output.width * output.height * output.channels];

          reproject::reproject(&input, &output, num_samples, interpolation);

          fs::path output_path =
              output_dir / p.filename().replace_extension(".png");

          reproject::save_png(output, output_path.string());

          delete[] input.data;
          delete[] output.data;
        });
      }
    }
  }

  pool.stop(true);

  return 0;
}
