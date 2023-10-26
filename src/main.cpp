#define CXXOPTS_NO_REGEX 1
#include <cxxopts.hpp>
#include <tracy/Tracy.hpp>

#include "image_formats.hpp"
#include "reproject.hpp"
#include <atomic>
#include <cmath>
#include <ctpl_stl.h>
#include <ghc/filesystem.hpp>
#include <nlohmann/json.hpp>

namespace fs = ghc::filesystem;

int parse_rectilinear(const std::string &lstr, float res_x, float res_y,
                      reproject::LensInfo &li) {
  int comma = lstr.find(",");
  if (comma == std::string::npos) {
    std::printf(
        "Error: Required format for --rectilinear focal_len,sensor_width\n");
    return 1;
  }
  auto &lir = li.rectilinear;
  li.type = reproject::RECTILINEAR;
  lir.focal_length = std::atof(lstr.substr(0, comma).c_str());
  li.sensor_width = std::atof(lstr.substr(comma + 1).c_str());
  li.sensor_height = (float)res_y / (float)res_x * li.sensor_width;
  return 0;
}

int parse_equisolid(const std::string &lstr, float res_x, float res_y,
                    reproject::LensInfo &li) {
  int comma1 = lstr.find(",");
  int comma2 = lstr.find(",", comma1 + 1);
  if (comma1 == std::string::npos || comma2 == std::string::npos) {
    std::printf(
        "Error: Required format for --equisolid focal_len,sensor_width,fov\n");
    return 1;
  }
  auto &lifes = li.fisheye_equisolid;
  li.type = reproject::FISHEYE_EQUISOLID;
  lifes.focal_length = std::atof(lstr.substr(0, comma1).c_str());
  lifes.fov = std::atof(lstr.substr(comma2 + 1).c_str());
  li.sensor_width = std::atof(lstr.substr(comma1 + 1, comma2).c_str());
  li.sensor_height = (float)res_y / (float)res_x * li.sensor_width;
  return 0;
}

int parse_equidistant(const std::string &lstr, float res_x, float res_y,
                      reproject::LensInfo &li) {
  li.type = reproject::FISHEYE_EQUIDISTANT;
  li.fisheye_equidistant.fov = std::atof(lstr.c_str());
  li.sensor_width = 36.0f;
  li.sensor_height = 36.0f;
  return 0;
}

int parse_equirectangular(const std::string &lstr, float res_x, float res_y,
                          reproject::LensInfo &li) {
  auto &lier = li.equirectangular;
  li.type = reproject::EQUIRECTANGULAR;
  if (lstr == "full") {
    lier.longitude_min = -M_PI;
    lier.longitude_max = M_PI;
    lier.latitude_min = -M_PI * 0.5f;
    lier.latitude_max = M_PI * 0.5f;
  } else {
    int argidx = 0;
    for (size_t b = 0, e = lstr.find(",");;
         b = e + 1, e = lstr.find(",", e + 1)) {
      std::string arg = lstr.substr(b, e - b);
      double fa = std::atof(arg.c_str());
      // clang-format off
      switch (argidx) {
        case 0: lier.longitude_min = fa; break;
        case 1: lier.longitude_max = fa; break;
        case 2: lier.latitude_min = fa; break;
        case 3: lier.latitude_max = fa; break;
      }
      // clang-format on

      argidx++;
      if (e == std::string::npos) {
        break;
      }
    }
    if (argidx != 4) {
      std::printf("Error: expected 4 arguments for equirectangular, got %d.\n",
                  argidx);
      return 1;
    }
  }
  li.sensor_width = li.sensor_height = 0;
  return 0;
}

// Function to multiply two 3x3 matrices
void multiplyMatrices(const float a[9], const float b[9], float result[9]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      result[i * 3 + j] = 0;
      for (int k = 0; k < 3; ++k) {
        result[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
      }
    }
  }
}

// Function to compute the rotation matrix from Euler angles
void computeRotationMatrix(float pan, float pitch, float roll,
                           float matrix[9]) {
  // clang-format off
  float rot_x = pitch;
  float rot_y = pan;
  float rot_z = roll;
  // Rotation matrix around x-axis
  float R_x[9] = {
    1, 0, 0,
    0, std::cos(rot_x), -std::sin(rot_x),
    0, std::sin(rot_x), std::cos(rot_x)
  };

  // Rotation matrix around y-axis
  float R_y[9] = {
    std::cos(rot_y), 0, std::sin(rot_y),
    0, 1, 0,
    -std::sin(rot_y), 0, std::cos(rot_y)
  };

  // Rotation matrix around z-axis
  float R_z[9] = {
    std::cos(rot_z), -std::sin(rot_z), 0,
    std::sin(rot_z), std::cos(rot_z), 0,
    0, 0, 1
  };

  // Compute R = R_y * R_x * R_z
  float temp[9];
  multiplyMatrices(R_x, R_z, temp); // temp = R_x * R_z
  multiplyMatrices(R_y, temp, matrix); // matrix = R_y * temp
  // clang-format on
}

int main(int argc, char **argv) {
  ZoneScoped;

  reproject::test_conversion_math();

  // clang-format off
  cxxopts::Options options(argv[0],
    "Reprojection tool for producing a variation of lens\n"
    "configurations based on one reference image given a\n"
    "known lens configuration.");
  options.add_options("Input/output")
    ("input-cfg", "Input JSON file containing lens and camera "
                  "settings of the input images.",
     cxxopts::value<std::string>(), "json-file")
    ("output-cfg", "Output JSON file containing lens and camera "
                   "settings of the output images.",
     cxxopts::value<std::string>(), "json-file")
    ("no-configs", "Work without reading and writing config files. "
                   "Requires you to specify the input lens through the input-optics "
                   "flags (staring with -i-...) and the expected resolution of the "
                   "input images here.",
     cxxopts::value<std::string>(), "width,height")
    ("i,input-dir", "Input directory containing images to reproject.",
     cxxopts::value<std::string>(), "file")
    ("single", "A single input file to convert.",
     cxxopts::value<std::string>(), "file")
    ("o,output-dir", "Output directory to put the reprojected images.",
     cxxopts::value<std::string>(), "file")
    ("exr", "Output EXR files. Color and depth.")
    ("png", "Output PNG files. Color only.")
    ;


  options.add_options("Filter files")
    ("filter-prefix", "Only include files starting with",
     cxxopts::value<std::string>()->default_value(""), "prefix")
    ("filter-suffix", "Only include files ending with",
     cxxopts::value<std::string>()->default_value(""), "suffix")
    ;

  options.add_options("Sampling")
    ("s,samples", "Number of samples per dimension for interpolating",
     cxxopts::value<int>()->default_value("1"), "number")

    ("nn", "Nearest neighbor interpolation")
    ("bl", "Bilinear interpolation")
    ("bc", "Bicubic interpolation (default)")

    ("scale", "Output scale, as a fraction of the input size. "
     "It is recommended to increase --samples to prevent aliassing "
     "in case you are downscaling. Eg: --scale 0.5 --samples 2 "
     "or --scale 0.33334 --samples 3 or --scale 0.25 --samples 4. "
     "Final dimensions are rounded towards zero.",
     cxxopts::value<double>()->default_value("1.0"), "percentage")
    ("output-resolution", "A fixed output resolution. Overwrites the behavior of the 'scale' parameter.",
     cxxopts::value<std::string>(), "width,height")
    ;

  options.add_options("Input optics.\n"
      "   These are usually inferred by the config JSONs. When specifying\n"
      "   --no-configs, lens information needs to be passed through these\n"
      "   command line options")
    ("i-rectilinear", "Input rectilinear images with given "
                      "focal_length,sensor_width tuple.",
     cxxopts::value<std::string>(), "focal_length,sensor_width")
    ("i-equisolid", "Input equisolid images with given "
                    "focal_length,sensor_width,fov tuple.",
     cxxopts::value<std::string>(), "focal_length,sensor_width,fov")
    ("i-equidistant", "Input equidistant images with given "
                      "fov value.",
     cxxopts::value<std::string>(), "fov")
    ("i-equirectangular", "Input equirectangular images with given longitude "
                          "min,max and latitude min,max value or 'full'.",
     cxxopts::value<std::string>(), "long_min,long_max,lat_min,lat_max (radians)")
    ;

  options.add_options("Output optics")
    ("no-reproject", "Do not reproject at all.")
    ("rectilinear", "Output rectilinear images with given "
                    "focal_length,sensor_width tuple.",
     cxxopts::value<std::string>(), "focal_length,sensor_width")
    ("equisolid", "Output equisolid images with given "
                  "focal_length,sensor_width,fov tuple.",
     cxxopts::value<std::string>(), "focal_length,sensor_width,fov")
    ("equidistant", "Output equidistant images with given "
                    "fov value.",
     cxxopts::value<std::string>(), "fov")
    ("equirectangular", "Output equirectangular images with given longitude "
                        "min,max and latitude min,max value or 'full'.",
     cxxopts::value<std::string>(), "longitude_min,longitude_max,latitude_min,latitude_max")
    ("rotation", "Specify a rotation",
     cxxopts::value<std::string>()->default_value("0.0"), "pan, pitch, roll (degrees)")
    ;

  options.add_options("Color processing")
    ("exposure", "Exposure compensation in stops (EV) to brigthen "
                 "or darken the pictures.",
     cxxopts::value<double>()->default_value("0.0"), "EV")
    ("reinhard", "Use reinhard tonemapping with given maximum value "
                 "(after exposure processing) on the output images.",
     cxxopts::value<double>()->default_value("1.0"), "max")
    ;


  options.add_options("Runtime")
    ("skip-if-exists", "Skip if the output file already exists.")
    ("j,parallel", "Number of parallel images to process.",
     cxxopts::value<int>()->default_value("1"), "threads")
    ("dry-run", "Do not actually reproject images. Only produce config.")
    ("h,help", "Show help")
    ;
  // clang-format on

  cxxopts::ParseResult result;
  int num_threads = 1;
  int num_samples = 1;
  std::string input_single;
  std::string input_dir;
  std::string output_dir;
  std::string input_cfg_file;
  std::string output_cfg_file;
  double scale = 0.0;
  int ores_x = 0;
  int ores_y = 0;
  double exposure = 1.0;
  double reinhard = 1.0;
  bool dry_run = false;
  bool reproject = true;
  bool skip_if_exists = false;
  float *rotation_matrix = nullptr;
  try {
    result = options.parse(argc, argv);
    if (result.count("help")) {
      std::printf("%s\n", options.help().c_str());
      return 0;
    }
    if (result.count("input-dir") && result.count("single")) {
      std::printf("Error: cannot specify both --input-dir and --single.\n");
      std::printf("%s\n", options.help().c_str());
      return 1;
    } else {
      if (result.count("input-dir")) {
        input_dir = result["input-dir"].as<std::string>();
      } else if (result.count("single")) {
        input_single = result["single"].as<std::string>();
      } else {
        std::printf("Error: No input specified.\n");
        return 1;
      }
    }
    output_dir = result["output-dir"].as<std::string>();
    num_samples = result["samples"].as<int>();
    num_threads = result["parallel"].as<int>();
    if (result.count("output-resolution")) {
      std::string arg = result["output-resolution"].as<std::string>();
      int comma = arg.find(",");
      if (comma == std::string::npos || comma == arg.length() - 1 ||
          comma == 0) {
        std::printf("Error: Specify both width and height, separated by a "
                    "comma in output-resolution.\n");
        return 1;
      }
      ores_x = std::atoi(arg.substr(0, comma).c_str());
      ores_y = std::atoi(arg.substr(comma + 1).c_str());
    } else {
      scale = result["scale"].as<double>();
    }

    std::string euler_angles = result["rotation"].as<std::string>();
    {
      int comma0 = euler_angles.find(',');
      int comma1 = euler_angles.find(',', comma0 + 1);
      float pan =
          std::atof(euler_angles.substr(0, comma0).c_str()) / 180.0 * M_PI;
      float pitch = std::atof(euler_angles.substr(comma0 + 1, comma1).c_str()) /
                    180.0 * M_PI;
      float roll =
          std::atof(euler_angles.substr(comma1 + 1).c_str()) / 180.0 * M_PI;

      rotation_matrix = new float[9];
      computeRotationMatrix(pan, pitch, roll, rotation_matrix);
    }

    exposure = std::pow(2.0, result["exposure"].as<double>());
    reinhard = result["reinhard"].as<double>();
    if (result.count("no-reproject")) {
      reproject = false;
    }
  } catch (cxxopts::exceptions::exception &e) {
    std::printf("%s\n\n%s\n", e.what(), options.help().c_str());
    return 1;
  }

  if (result.count("dry-run")) {
    dry_run = true;
  }
  if (result.count("skip-if-exists")) {
    skip_if_exists = true;
  }

  bool store_png = false;
  bool store_exr = false;
  if (result.count("exr")) {
    store_exr = true;
  }
  if (result.count("png")) {
    store_png = true;
  }

  if (!store_exr && !store_png) {
    std::printf("Error: Did not specify any output format.\n"
                "Choose --png or --exr. (both are possible).\n");
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

  std::string filter_prefix = result["filter-prefix"].as<std::string>();
  std::string filter_suffix = result["filter-suffix"].as<std::string>();

  nlohmann::json out_cfg;
  reproject::LensInfo input_lens;
  reproject::LensInfo output_lens;
  int ires_x = 0, ires_y = 0;

  if (result.count("no-configs")) {
    // parse input resolution
    std::string lstr = result["no-configs"].as<std::string>();
    int comma = lstr.find(",");
    ires_x = std::atoi(lstr.substr(0, comma).c_str());
    ires_y = std::atoi(lstr.substr(comma + 1).c_str());

    // parse input lens type
    int input_lens_types_found = 0;
    if (result.count("i-rectilinear")) {
      std::string lstr = result["i-rectilinear"].as<std::string>();
      if (parse_rectilinear(lstr, ires_x, ires_y, input_lens) != 0) {
        return 1;
      }
      input_lens_types_found++;
    }
    if (result.count("i-equisolid")) {
      std::string lstr = result["i-equisolid"].as<std::string>();
      if (parse_equisolid(lstr, ires_x, ires_y, input_lens) != 0) {
        return 1;
      }
      input_lens_types_found++;
    }
    if (result.count("i-equidistant")) {
      std::string lstr = result["i-equidistant"].as<std::string>();
      if (parse_equidistant(lstr, ires_x, ires_y, input_lens) != 0) {
        return 1;
      }
      input_lens_types_found++;
    }
    if (result.count("i-equirectangular")) {
      std::string lstr = result["i-equirectangular"].as<std::string>();
      if (parse_equirectangular(lstr, ires_x, ires_y, input_lens) != 0) {
        return 1;
      }
      input_lens_types_found++;
    }

    if (input_lens_types_found > 1) {
      std::printf("Error: only specify one input lens type: [--i-rectilinear, "
                  "--i-equisolid, --i-equidistant, --i-equirectangular].\n");
      return 1;
    }
  } else {
    input_cfg_file = result["input-cfg"].as<std::string>();
    output_cfg_file = result["output-cfg"].as<std::string>();
    nlohmann::json cfg;
    std::ifstream cfg_ifstream{input_cfg_file};
    cfg_ifstream >> cfg;
    cfg_ifstream.close();

    out_cfg = cfg;
    std::printf("Found camera config: %s\n", cfg["camera"].dump(1).c_str());
    ires_x = cfg["resolution"][0].get<int>();
    ires_y = cfg["resolution"][1].get<int>();

    input_lens = reproject::extract_lens_info_from_config(cfg);
  }

  // Parse output lens parameters
  int output_lens_types_found = 0;

  if (ores_x == 0 && ores_y == 0) {
    ores_x = int(ires_x * scale);
    ores_y = int(ires_y * scale);
  }

  if (result.count("rectilinear")) {
    std::string lstr = result["rectilinear"].as<std::string>();
    if (parse_rectilinear(lstr, ores_x, ores_y, output_lens) != 0) {
      return 1;
    }
    output_lens_types_found++;
  }
  if (result.count("equisolid")) {
    std::string lstr = result["equisolid"].as<std::string>();
    if (parse_equisolid(lstr, ores_x, ores_y, output_lens) != 0) {
      return 1;
    }
    output_lens_types_found++;
  }
  if (result.count("equidistant")) {
    std::string lstr = result["equidistant"].as<std::string>();
    if (parse_equidistant(lstr, ores_x, ores_y, output_lens) != 0) {
      return 1;
    }
    output_lens_types_found++;
  }
  if (result.count("equirectangular")) {
    std::string lstr = result["equirectangular"].as<std::string>();
    if (parse_equirectangular(lstr, ores_x, ores_y, output_lens)) {
      return 1;
    }
    output_lens_types_found++;
  }

  if (!reproject) {
    output_lens = input_lens;
    output_lens_types_found++;
  }

  if (output_lens_types_found > 1) {
    std::printf(
        "Error: only specify one output lens type: [--rectilinear, "
        "--equisolid, --equidistant, --equirectangular, --no-reproject].\n");
    return 1;
  }

  std::printf("Creating directory: %s\n", output_dir.c_str());
  fs::create_directory(output_dir);

  if (result.count("no-configs")) {

  } else {
    // store in out_cfg
    reproject::store_lens_info_in_config(output_lens, out_cfg);
    out_cfg["resolution"][0] = ores_x;
    out_cfg["resolution"][1] = ores_y;

    if (out_cfg.contains("frames")) {
      for (int i = 0; i < out_cfg["frames"].size(); ++i) {
        std::string name = out_cfg["frames"][i]["name"].get<std::string>();

        bool remove = false;
        if (name.size() < filter_prefix.size() ||
            name.size() < filter_suffix.size()) {
          remove = true;
        } else if (name.substr(0, filter_prefix.size()) != filter_prefix) {
          remove = true;
        } else if (name.substr(name.size() - filter_suffix.size()) !=
                   filter_suffix) {
          remove = true;
        }
        if (remove) {
          out_cfg["frames"].erase(i--);
        }
      }
    }

    std::printf("Saving output config: %s\n", output_cfg_file.c_str());
    std::ofstream cfg_ofstream{output_cfg_file};
    cfg_ofstream << out_cfg.dump(2);
    cfg_ofstream.close();
  }

  if (dry_run) {
    std::printf("Dry-run. Exiting.\n");
    return 0;
  }

  int count = 0;
  std::atomic_int done_count{0};
  ctpl::thread_pool pool(num_threads);

  std::function<void(std::string)> submit_file = [&](fs::path p) {
    pool.push([p, num_samples, interpolation, output_dir, scale, ores_x, ores_y,
               input_lens, output_lens, rotation_matrix, &done_count, &count,
               reproject, exposure, reinhard, store_exr, store_png,
               skip_if_exists](int) {
      ZoneScopedN("process_file");
      try {
        fs::path output_path_base = output_dir / p.filename();
        fs::path output_path_png = output_path_base.replace_extension(".png");
        fs::path output_path_exr = output_path_base.replace_extension(".exr");

        bool exists = true;
        if (store_png && !fs::exists(output_path_png)) {
          exists = false;
        }
        if (store_exr && !fs::exists(output_path_exr)) {
          exists = false;
        }
        if (exists && skip_if_exists) {
          std::printf("Skipping '%s'. Already exists.\n",
                      output_path_png.c_str());
          done_count++;
          return;
        }

        reproject::Image input;
        if (p.extension() == ".exr") {
          input = reproject::read_exr(p.string());
        } else if (p.extension() == ".png") {
          input = reproject::read_png(p.string());
        } else if (p.extension() == ".jpeg" || p.extension() == ".jpg") {
          input = reproject::read_jpeg(p.string());
        } else {
          std::printf("Input format not supported: %s\n",
                      p.extension().c_str());
        }
        input.lens = input_lens;

        reproject::Image output;
        output.lens = output_lens;

        if (ores_x != -1 && ores_y != -1) {
          output.width = ores_x;
          output.height = ores_y;
        } else {
          output.width = int(input.width * scale);
          output.height = int(input.height * scale);
        }
        output.channels = input.channels;
        output.data_layout = input.data_layout;
        output.data = new float[output.width * output.height * output.channels];

        if (!reproject && scale == 1.0) {
          uint64_t bytes = output.width * output.height;
          bytes *= output.channels * sizeof(float);
          std::memcpy(output.data, input.data, bytes);
        } else {
          reproject::reproject(&input, &output, num_samples, interpolation,
                               rotation_matrix);
        }

        if (exposure != 1.0 || reinhard != 1.0) {
          reproject::post_process(&output, exposure, reinhard);
        }

        if (store_png) {
          reproject::save_png(output, output_path_png.string());
        }
        if (store_exr) {
          reproject::save_exr(output, output_path_exr.string());
        }

        delete[] input.data;
        delete[] output.data;

        int dc = ++done_count;
        std::printf("%4d / %4d: %s\n", dc, count, p.stem().c_str());
      } catch (const std::exception &e) {
        std::printf("Error: %s\n", e.what());
      }
    });
    count++;
  };

  if (!input_dir.empty()) {
    fs::directory_iterator end;
    fs::directory_iterator it{fs::path(input_dir)};

    std::vector<fs::path> paths;
    for (; it != end; ++it) {
      if (it->is_regular_file()) {
        paths.push_back(*it);
      }
    }
    std::sort(paths.begin(), paths.end());

    for (fs::path &p : paths) {
      std::string fn = p.filename().string();
      if (fn.size() < filter_prefix.size() ||
          fn.size() < filter_suffix.size()) {
        continue;
      }
      if (fn.substr(0, filter_prefix.size()) != filter_prefix) {
        continue;
      }
      if (fn.substr(fn.size() - filter_suffix.size()) != filter_suffix) {
        continue;
      }
      if (p.extension() == ".exr" || p.extension() == ".png") {
        submit_file(p.string());
      }
    }
  } else if (!input_single.empty()) {
    fs::path p{input_single};
    submit_file(p.string());
  }

  pool.stop(true);

  return 0;
}
