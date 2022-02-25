#include "config.hpp"

#include <nlohmann/json.hpp>

namespace reproject {

LensInfo extract_lens_info_from_config(const nlohmann::json &cfg) {
  const nlohmann::json &camera_cfg = cfg["camera"];
  std::string camera_type = camera_cfg["/type"_json_pointer];

  reproject::LensInfo lens;
  lens.sensor_width = cfg["sensor_size"][0].get<float>();
  lens.sensor_height = cfg["sensor_size"][1].get<float>();

  int res_x = cfg["resolution"][0].get<int>();
  int res_y = cfg["resolution"][1].get<int>();

  if (camera_type == "PANO") {
    camera_type = camera_cfg["panorama_type"].get<std::string>();
    if (camera_type == "FISHEYE_EQUIDISTANT") {
      lens.type = reproject::FISHEYE_EQUIDISTANT;
      lens.fisheye_equidistant.fov = camera_cfg["fisheye_fov"].get<float>();
    } else if (camera_type == "FISHEYE_EQUISOLID") {
      lens.type = reproject::FISHEYE_EQUISOLID;
      lens.fisheye_equisolid.focal_length =
          camera_cfg["fisheye_lens"].get<float>();
      lens.fisheye_equisolid.fov = camera_cfg["fisheye_fov"].get<float>();
    } else if (camera_type == "EQUIRECTANGULAR") {
      lens.type = reproject::EQUIRECTANGULAR;
      auto &leq = lens.equirectangular;
      leq.latitude_min = camera_cfg["latitude_min"].get<float>();
      leq.latitude_max = camera_cfg["latitude_max"].get<float>();
      leq.longitude_min = camera_cfg["longitude_min"].get<float>();
      leq.longitude_max = camera_cfg["longitude_max"].get<float>();
    }
  } else if (camera_type == "PERSP") {
    lens.type = reproject::RECTILINEAR;
    std::string lens_unit = camera_cfg["lens_unit"].get<std::string>();
    if (lens_unit == "MILLIMETERS") {
      lens.rectilinear.focal_length = camera_cfg["focal_length"].get<float>();
    } else if (lens_unit == "FOV") {
      float angle = camera_cfg["angle"].get<float>();
      // sensor_width = focal_length * tan(fov/2)
      std::printf("Warning: relying on 'angle' is unsafe. Angle is assumed "
                  "to be based on the width of the sensor.\n");

      lens.rectilinear.focal_length =
          lens.sensor_width / std::tan(0.5f * angle);
    } else {
      throw std::invalid_argument("Unknown lens_unit");
    }
  } else {
    throw std::invalid_argument("Unknown camera_type");
  }
  return lens;
}

void store_lens_info_in_config(const LensInfo &ol, nlohmann::json &out_cfg) {
  out_cfg["camera"] = nlohmann::json::object();
  out_cfg["sensor_size"][0] = ol.sensor_width;
  out_cfg["sensor_size"][1] = ol.sensor_height;

  if (ol.type == LensType::RECTILINEAR) {
    auto &olr = ol.rectilinear;
    out_cfg["camera"]["type"] = "PERSP";
    out_cfg["camera"]["lens_unit"] = "MILLIMETERS";
    out_cfg["camera"]["focal_length"] = olr.focal_length;

    out_cfg["camera"]["projection_matrix"] = nlohmann::json::array();
    float proj[16] = {0.0f};
    proj[0] = 2.0f * olr.focal_length / ol.sensor_width;
    proj[5] = 2.0f * olr.focal_length / ol.sensor_height;
    proj[15] = 1.0f;
    for (int r = 0; r < 4; ++r) {
      out_cfg["camera"]["projection_matrix"].push_back(nlohmann::json::array());
      for (int c = 0; c < 4; ++c) {
        out_cfg["camera"]["projection_matrix"][r].push_back(proj[r * 4 + c]);
      }
    }
  } else if (ol.type == LensType::FISHEYE_EQUISOLID) {
    auto &olfes = ol.fisheye_equisolid;
    out_cfg["camera"]["type"] = "PANO";
    out_cfg["camera"]["panorama_type"] = "FISHEYE_EQUISOLID";
    out_cfg["camera"]["fisheye_lens"] = olfes.focal_length;
    out_cfg["camera"]["fisheye_fov"] = olfes.fov;
  } else if (ol.type == LensType::FISHEYE_EQUIDISTANT) {
    auto &olfed = ol.fisheye_equidistant;
    out_cfg["camera"]["type"] = "PANO";
    out_cfg["camera"]["panorama_type"] = "FISHEYE_EQUIDISTANT";
    out_cfg["camera"]["fisheye_fov"] = olfed.fov;
  } else if (ol.type == LensType::EQUIRECTANGULAR) {
    auto &olr = ol.equirectangular;
    out_cfg["camera"]["type"] = "PANO";
    out_cfg["camera"]["panorama_type"] = "RECTILINEAR";
    out_cfg["camera"]["latitude_min"] = olr.latitude_min;
    out_cfg["camera"]["latitude_max"] = olr.latitude_max;
    out_cfg["camera"]["longitude_min"] = olr.latitude_min;
    out_cfg["camera"]["longitude_max"] = olr.latitude_max;
  } else {
    throw std::invalid_argument("Unsupported lens type.");
  }
}

} // namespace reproject
