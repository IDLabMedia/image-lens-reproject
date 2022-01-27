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
    ("input-cfg", "Input JSON file containing lens and camera "
                  "settings of the input images.",
     cxxopts::value<std::string>(), "json-file")
    ("output-cfg", "Output JSON file containing lens and camera "
                   "settings of the input images.",
     cxxopts::value<std::string>(), "json-file")
    ("i,input-dir", "Input directory containing images to reproject.",
     cxxopts::value<std::string>(), "file")
    ("o,output-dir", "Output directory to put the reprojected images.",
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

    ("rectilinear", "Output rectilinear image with given "
                    "focal_length,sensor_width tuple.",
     cxxopts::value<std::string>(), "focal_length,sensor_width")

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
  std::string input_cfg_file;
  std::string output_cfg_file;
  int scale;
  try {
    result = options.parse(argc, argv);
    if (result.count("help")) {
      std::printf("%s\n", options.help().c_str());
      return 0;
    }
    input_dir = result["input-dir"].as<std::string>();
    output_dir = result["output-dir"].as<std::string>();
    input_cfg_file = result["input-cfg"].as<std::string>();
    output_cfg_file = result["output-cfg"].as<std::string>();
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
  std::ifstream cfg_ifstream{input_cfg_file};
  cfg_ifstream >> cfg;
  cfg_ifstream.close();

  nlohmann::json out_cfg = cfg;

  nlohmann::json &camera_cfg = cfg["camera"];
  std::printf("Found camera config: %s\n", camera_cfg.dump(1).c_str());
  std::string camera_type = camera_cfg["/type"_json_pointer];

  reproject::LensInfo input_lens;
  input_lens.sensor_width = cfg["sensor_size"][0].get<float>();
  input_lens.sensor_height = cfg["sensor_size"][1].get<float>();

  int res_x = cfg["resolution"][0].get<int>();
  int res_y = cfg["resolution"][1].get<int>();

  std::printf("camera_type: %s\n", camera_type.c_str());

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
      std::printf("Error: Equirectangular camera not implemented\n");
      return 1;
    }
  } else if (camera_type == "PERSP") {
    input_lens.type = reproject::RECTILINEAR;
    std::string lens_unit = camera_cfg["lens_unit"].get<std::string>();
    if (lens_unit == "MILLIMETERS") {
      input_lens.rectilinear.focal_length =
          camera_cfg["focal_length"].get<float>();
    } else if (lens_unit == "FOV") {
      float angle = camera_cfg["angle"].get<float>();
      // sensor_width = focal_length * tan(fov/2)
      std::printf("Warning: relying on 'angle' is unsafe. Angle is assumed "
                  "to be based on the width of the sensor.\n");

      input_lens.rectilinear.focal_length =
          input_lens.sensor_width / std::tan(0.5f * angle);
    } else {
      std::printf("Error: Unknown lens unit: %s\n", lens_unit.c_str());
      return 1;
    }
  } else {
    std::printf("Error: Unknown camera type: %s\n", camera_type.c_str());
    return 1;
  }

  reproject::LensInfo output_lens;
  if (result.count("rectilinear")) {
    std::string lstr = result["rectilinear"].as<std::string>();
    int comma = lstr.find(",");
    if (comma == std::string::npos) {
      std::printf("Error: Required format for --rectilinear x,y\n");
      return 1;
    }
    output_lens.type = reproject::RECTILINEAR;
    output_lens.rectilinear.focal_length =
        std::atof(lstr.substr(0, comma).c_str());
    output_lens.sensor_width = std::atof(lstr.substr(comma + 1).c_str());
    output_lens.sensor_height =
        (float)res_y / (float)res_x * output_lens.sensor_width;

    // write to out_cfg
    out_cfg["camera"] = nlohmann::json::object();
    out_cfg["camera"]["type"] = "PERSP";
    out_cfg["camera"]["lens_unit"] = "MILLIMETERS";
    out_cfg["camera"]["focal_length"] = output_lens.rectilinear.focal_length;
    out_cfg["sensor_size"][0] = output_lens.sensor_width;
    out_cfg["sensor_size"][1] = output_lens.sensor_height;

    out_cfg["camera"]["projection_matrix"] = nlohmann::json::array();
    float proj[16] = {0.0f};
    proj[0] = output_lens.rectilinear.focal_length / output_lens.sensor_width;
    proj[5] = output_lens.rectilinear.focal_length / output_lens.sensor_height;
    proj[15] = 1.0f;
    for (int r = 0; r < 4; ++r) {
      out_cfg["camera"]["projection_matrix"].push_back(nlohmann::json::array());
      for (int c = 0; c < 4; ++c) {
        out_cfg["camera"]["projection_matrix"][r].push_back(proj[r * 4 + c]);
      }
    }
  }

  std::ofstream cfg_ofstream{output_cfg_file};
  cfg_ofstream << out_cfg.dump(2);
  cfg_ofstream.close();

  fs::directory_iterator end;
  fs::directory_iterator it{fs::path(input_dir)};

  ctpl::thread_pool pool(num_threads);
  for (; it != end; ++it) {
    if (it->is_regular_file()) {
      fs::path p = *it;
      if (p.extension() == ".exr" || p.extension() == ".png") {
        pool.push([p, num_samples, interpolation, output_dir, scale, input_lens,
                   output_lens](int) {
          std::printf("%s\n", p.c_str());

          reproject::Image input = reproject::read_exr(p.string());
          input.lens = input_lens;

          reproject::Image output;
          output.lens = output_lens;

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
