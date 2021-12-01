#pragma once

namespace reproject {

enum LensType {
    RECTILINEAR,
    FISHEYE_EQUIDISTANT,
    FISHEYE_EQUISOLID,
    FISHEYE_STEREOGRAPHIC,
    EQUIRECTANGULAR_360
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
    };
    float sensor_width;
    float sensor_height;
};

}  // namespace reproject
