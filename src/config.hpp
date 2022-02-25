#pragma once

#include <nlohmann/json_fwd.hpp>

namespace reproject {

enum LensType {
  RECTILINEAR,
  FISHEYE_EQUIDISTANT,
  FISHEYE_EQUISOLID,
  FISHEYE_STEREOGRAPHIC,
  EQUIRECTANGULAR
};

struct LensInfo {
  LensType type;
  union {
    struct {
      float focal_length;
    } rectilinear;
    struct {
      float fov;
    } fisheye_equidistant;
    struct {
      float focal_length;
      float fov;
    } fisheye_equisolid;
    struct {
      float latitude_min;
      float latitude_max;
      float longitude_min;
      float longitude_max;
    } equirectangular;
  };
  float sensor_width;
  float sensor_height;
};

/**
 * Extracts the lens information from a config file, as produced by the Blender
 * addon.
 * @throws std::invalid_argument if the given json tree cannot be extracted
 *         correctly.
 */
LensInfo extract_lens_info_from_config(const nlohmann::json &config);

void store_lens_info_in_config(const LensInfo &lens, nlohmann::json &config);

} // namespace reproject
