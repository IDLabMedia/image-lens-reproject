#include "reproject.hpp"

#include <cmath>

namespace reproject {

typedef void (*from_func_t)(const LensInfo &li, float img_w, float img_h,
                            float cx, float cy, float &alpha, float &theta);
typedef void (*to_func_t)(const LensInfo &li, float img_w, float img_h,
                          float cx, float cy, float &alpha, float &theta);

inline void sample_bilinear(const Image *img, float sx, float sy, float *out) {
    int lx = sx;
    int ux = std::min(img->width - 1, int(sx + 1.0f));
    int ly = sy;
    int uy = std::min(img->height - 1, int(sy + 1.0f));
    float fx = sx - lx;
    float fy = sy - ly;
    float cfx = 1.0f - fx;
    float cfy = 1.0f - fy;

    int pitch = img->width * img->channels;
    for (int c = 0; c < img->channels; ++c) {
        float ll = img->data[ly * pitch + lx * img->channels + c];
        float lu = img->data[ly * pitch + ux * img->channels + c];
        float ul = img->data[uy * pitch + lx * img->channels + c];
        float uu = img->data[uy * pitch + ux * img->channels + c];

        // interpolate horizontally
        float l = fx * lu + cfx * ll;
        float u = fx * uu + cfx * ul;

        // interpolate vertically
        float r = fy * u + cfy * l;
        out[c] = r;
    }
}

// === RECTILINEAR ===

inline void rectilinear_to_spherical(const LensInfo &li, float img_w,
                                     float img_h, float cx, float cy,
                                     float &alpha, float &theta) {
    float r_px = std::sqrt(cx * cx + cy * cy);  // [px]
    alpha = std::atan2(cy, cx);
    float factor = li.sensor_width / img_w;  // [mm / px]
    // r_px = r_mm / factor
    // r_mm = focal_length_mm * tan(theta)
    // r_px / factor = focal_length * tan(theta)
    // r_px * focal_length / factor = tan(theta)
    theta = std::atan(r_px * li.rectilinear.focal_length * factor);
}

inline void spherical_to_rectilinear(const LensInfo &li, float img_w,
                                     float img_h, float alpha, float theta,
                                     float &cx, float &cy) {
    float x = std::cos(alpha);
    float y = std::sin(alpha);
    float r_mm = li.rectilinear.focal_length * std::tan(theta);
    float r_px = r_mm / li.sensor_width * img_w;
    cx = r_px * x;
    cy = r_px * y;
}

// === EQUIDISTANT ===

inline void equidistant_to_spherical(const LensInfo &li, float img_w,
                                     float img_h, float cx, float cy,
                                     float &alpha, float &theta) {
    float r_px = std::sqrt(cx * cx + cy * cy);  // [px]
    alpha = std::atan2(cy, cx);
    float factor = li.sensor_width / img_w;  // [mm / px]
    // r_px = r_mm / factor
    // r_mm = focal_length_mm * theta
    // r_px / factor = focal_length * theta
    // r_px * focal_length / factor = theta
    theta = r_px * li.rectilinear.focal_length * factor;
}

inline void spherical_to_equidistant(const LensInfo &li, float img_w,
                                     float img_h, float alpha, float theta,
                                     float &cx, float &cy) {
    float x = std::cos(alpha);
    float y = std::sin(alpha);
    float r_mm = li.rectilinear.focal_length * theta;
    float r_px = r_mm / li.sensor_width * img_w;
    cx = r_px * x;
    cy = r_px * y;
}

template <from_func_t ff, to_func_t tf>
void reproject_from_to(const Image *in, Image *out) {
    int pitch = out->width * out->channels;
    for (int y = 0; y < out->height; ++y) {
        for (int x = 0; x < out->width; ++x) {
            // Center around (0,0)
            float cx = (x + 0.5f) - out->width * 0.5f;
            float cy = (y + 0.5f) - out->height * 0.5f;

            float alpha;
            float theta;
            ff(out->lens, out->width, out->height, cx, cy, alpha, theta);

            float sx, sy;  // source coordinate on input image
            tf(in->lens, in->width, in->height, alpha, theta, sx, sy);

            // bilinear interpolation from source image
            float *dst = &out->data[y * pitch + x * out->channels];
            sample_bilinear(in, sx, sy, dst);
        }
    }
}

template <from_func_t ff>
void reproject_from(const Image *in, Image *out) {
    if (in->lens.type == RECTILINEAR) {
        reproject_from_to<ff, spherical_to_rectilinear>(in, out);
    } else if (in->lens.type == FISHEYE_EQUIDISTANT) {
        reproject_from_to<ff, spherical_to_equidistant>(in, out);
    }
}

void reproject(const Image *in, Image *out) {
    if (out->lens.type == RECTILINEAR) {
        reproject_from<rectilinear_to_spherical>(in, out);
    } else if (out->lens.type == FISHEYE_EQUIDISTANT) {
        reproject_from<equidistant_to_spherical>(in, out);
    }
}

}  // namespace reproject
