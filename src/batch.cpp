#define CXXOPTS_NO_REGEX 1
#include <Tracy.hpp>
#include <cxxopts.hpp>

#include "image_formats.hpp"
#include "reproject.hpp"
#include <cmath>
#include <ctpl_stl.h>
#include <ghc/filesystem.hpp>
#include <nlohmann/json.hpp>

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

    ("scale", "Output scale, as a percentage of the input size. "
     "It is recommended to increase --samples to prevent aliassing "
     "in case you are downscaling. Eg: --scale 50 --samples 2 "
     "or --scale 25 --samples 4",
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

  nlohmann::json cfg;
  std::ifstream cfg_ifstream{cfg_file};
  cfg_ifstream >> cfg;
  cfg_ifstream.close();

  nlohmann::json camera_cfg = cfg["camera"];
  std::printf("Found camera config: %s\n", camera_cfg.dump(1).c_str());
  std::string camera_type = camera_cfg["/type"_json_pointer];
  reproject::LensInfo input_lens;
  if (camera_type == "PANO") {
    camera_type = camera_cfg["panorama_type"].get<std::string>();
    if (camera_type == "FISHEYE_EQUIDISTANT") {
      input_lens.type = reproject::FISHEYE_EQUIDISTANT;
      input_lens.fisheye_equidistant.fov =
          camera_cfg["fisheye_fov"].get<float>();
    } else if (camera_type == "FISHEYE_EQUISOLID") {
      input_lens.type = reproject::FISHEYE_EQUISOLID;
      input_lens.fisheye_equisolid.focal_length =
          camera_cfg["fisheye_lens"].get<float>();
      input_lens.fisheye_equisolid.fov = camera_cfg["fisheye_fov"].get<float>();
    } else if (camera_type == "EQUIRECTANGULAR") {
      // TODO
      return 1;
    }
  } else if (camera_type == "PERSP") {
    // TODO
    return 1;
  } else {
    return 1;
  }
  input_lens.sensor_width = cfg["sensor_size"][0].get<float>();
  input_lens.sensor_height = cfg["sensor_size"][1].get<float>();
  std::printf("camera_type: %s\n", camera_type.c_str());

  fs::directory_iterator end;
  fs::directory_iterator it{fs::path(input_dir)};

  ctpl::thread_pool pool(num_threads);
  for (; it != end; ++it) {
    if (it->is_regular_file()) {
      fs::path p = *it;
      if (p.extension() == ".exr" || p.extension() == ".png") {
        pool.push([p, num_samples, interpolation, output_dir, scale,
                   input_lens](int) {
          std::printf("%s\n", p.c_str());

          reproject::Image input = reproject::read_exr(p.string());
          input.lens = input_lens;

          reproject::Image output;
#if 1
          output.lens.type = reproject::LensType::RECTILINEAR;
          output.lens.rectilinear.focal_length = 12.0f;
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
