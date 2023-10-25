#include "reproject.hpp"

#include <algorithm>
#include <cmath>

#include <tracy/Tracy.hpp>

namespace reproject {

/**
 * cx, cy - Center of the pixel, in a coordinate system where the center of the
 * image is at (0,0), the corners are at (-/+0.5*w, -/+0.5*h).
 *
 * For oaspherical (optical-axis spherical):
 *
 *    alpha - Rotation around the optical axis.
 *    theta - Angle away from the optical axis, outward.
 *
 * For equirectangular:
 *
 *    alpha - Horizontal angle in radians.
 *    theta - Vertical angle in radians.
 */
typedef void (*target_to_vec_t)(const LensInfo &li, float img_w, float img_h,
                                float cx, float cy, float &x, float &y,
                                float &z);
typedef void (*vec_to_source_t)(const LensInfo &li, float img_w, float img_h,
                                float x, float y, float z, float &cx,
                                float &cy);

typedef void (*sample_func_t)(const Image *img, float sx, float sy, float *out);

template <typename T> inline T clamp(T x, T min, T max) {
  return std::max(min, std::min(max, x));
}

// === SAMPLING ===

template <bool LoopHorizontally>
inline void sample_nearest(const Image *img, float sx, float sy, float *out) {
  int lx;
  if constexpr (LoopHorizontally) {
    lx = (int(sx + 0.5f) + img->width) % img->width;
  } else {
    lx = clamp(int(sx + 0.5f), 0, img->width - 1);
  }
  int ly = clamp(int(sy + 0.5f), 0, img->height - 1);

  int pitch = img->width * img->channels;
  for (int c = 0; c < img->channels; ++c) {
    out[c] = img->data[ly * pitch + lx * img->channels + c];
  }
}

template <bool LoopHorizontally>
inline void sample_bilinear(const Image *img, float sx, float sy, float *out) {
  // clang-format off
  int lx, ux;
  if constexpr (LoopHorizontally) {
    lx = (int(sx)        + img->width) % img->width;
    ux = (int(sx + 1.0f) + img->width) % img->width;
  } else {
    lx = clamp(int(sx)       , 0, img->width - 1);
    ux = clamp(int(sx + 1.0f), 0, img->width - 1);
  }
  int ly = clamp(int(sy)       , 0, img->height - 1);
  int uy = clamp(int(sy + 1.0f), 0, img->height - 1);
  // clang-format on

  float fx = std::max(0.0f, std::min(1.0f, sx - lx));
  float fy = std::max(0.0f, std::min(1.0f, sy - ly));
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

inline float cubicInterpolate(float p[4], float x) {
  // clang-format off
  return p[1] + 0.5f * x * (p[2] - p[0] + x *
          (2.0f * p[0] - 5.0f * p[1] + 4.0f * p[2] - p[3] + x *
           (3.0f * (p[1] - p[2]) + p[3] - p[0])));
  // clang-format on
}

inline float bicubicInterpolate(float p[4][4], float x, float y) {
  float arr[4];
  arr[0] = cubicInterpolate(p[0], y);
  arr[1] = cubicInterpolate(p[1], y);
  arr[2] = cubicInterpolate(p[2], y);
  arr[3] = cubicInterpolate(p[3], y);
  return cubicInterpolate(arr, x);
}

template <bool LoopHorizontally>
inline void sample_bicubic(const Image *img, float sx, float sy, float *out) {
  // clang-format off
  int x0, x1, x2, x3;
  if constexpr (LoopHorizontally) {
    x0 = (int(sx - 1.0f) + img->width) % img->width;
    x1 = (int(sx       ) + img->width) % img->width;
    x2 = (int(sx + 1.0f) + img->width) % img->width;
    x3 = (int(sx + 2.0f) + img->width) % img->width;
  } else {
    x0 = clamp(int(sx - 1.0f), 0, img->width - 1);
    x1 = clamp(int(sx       ), 0, img->width - 1);
    x2 = clamp(int(sx + 1.0f), 0, img->width - 1);
    x3 = clamp(int(sx + 2.0f), 0, img->width - 1);
  }
  int y0 = clamp(int(sy - 1.0f), 0, img->height - 1);
  int y1 = clamp(int(sy       ), 0, img->height - 1);
  int y2 = clamp(int(sy + 1.0f), 0, img->height - 1);
  int y3 = clamp(int(sy + 2.0f), 0, img->height - 1);
  // clang-format on

  float fx = std::max(0.0f, std::min(1.0f, sx - x1));
  float fy = std::max(0.0f, std::min(1.0f, sy - y1));

  int pitch = img->width * img->channels;
  for (int c = 0; c < img->channels; ++c) {
    float p[4][4];
#define FETCH(xi, yi)                                                          \
  p[xi][yi] = img->data[y##yi * pitch + x##xi * img->channels + c]
    // clang-format off
    FETCH(0, 0); FETCH(1, 0); FETCH(2, 0); FETCH(3, 0);
    FETCH(0, 1); FETCH(1, 1); FETCH(2, 1); FETCH(3, 1);
    FETCH(0, 2); FETCH(1, 2); FETCH(2, 2); FETCH(3, 2);
    FETCH(0, 3); FETCH(1, 3); FETCH(2, 3); FETCH(3, 3);
    // clang-format on
#undef FETCH

    out[c] = bicubicInterpolate(p, fx, fy);
  }
}

// === RECTILINEAR ===

inline void rectilinear_to_vec(const LensInfo &li, float img_w, float img_h,
                               float cx, float cy, float &x, float &y,
                               float &z) {
  x = cx / img_w * li.sensor_width / li.rectilinear.focal_length;
  y = cy / img_h * li.sensor_height / li.rectilinear.focal_length;
  z = -1.0f;
}

inline void vec_to_rectilinear(const LensInfo &li, float img_w, float img_h,
                               float x, float y, float z, float &cx,
                               float &cy) {
  x /= -z;
  y /= -z;
  cx = x * img_w / li.sensor_width * li.rectilinear.focal_length;
  cy = y * img_h / li.sensor_height * li.rectilinear.focal_length;
}

// === EQUIDISTANT ===

inline void equidistant_to_vec(const LensInfo &li, float img_w, float img_h,
                               float cx, float cy, float &x, float &y,
                               float &z) {

  float r_px = std::sqrt(cx * cx + cy * cy); // [px]

  float r_mm = r_px / img_w * li.sensor_width;
  float focal_length = li.sensor_width / li.fisheye_equidistant.fov;
  // r_mm = f * theta
  // theta = r_mm / f
  float theta = r_mm / focal_length;
  float s = std::sin(theta) / r_px;
  x = s * cx;
  y = s * cy;
  z = std::cos(theta);
}

inline void vec_to_equidistant(const LensInfo &li, float img_w, float img_h,
                               float x, float y, float z, float &cx,
                               float &cy) {
  x /= -z;
  y /= -z;
  float r = std::sqrt(x * x + y * y);
  float theta = std::atan(r);

  float factor = li.sensor_width / img_w; // [mm / px]
  float focal_length = li.sensor_width / li.fisheye_equidistant.fov;

  float r_mm = focal_length * theta;
  float r_px = r_mm / li.sensor_width * img_w;

  cx = x / r * r_px;
  cy = y / r * r_px;

  // TODO Validate
}

inline void equidistant_to_oaspherical(const LensInfo &li, float img_w,
                                       float img_h, float cx, float cy,
                                       float &alpha, float &theta) {
  float r_px = std::sqrt(cx * cx + cy * cy); // [px]

  float r_mm = r_px / img_w * li.sensor_width;
  float focal_length = li.sensor_width / li.fisheye_equidistant.fov;
  // r_mm = f * theta
  // theta = r_mm / f
  theta = r_mm / focal_length;
  alpha = std::atan2(cy, cx);
}

inline void oaspherical_to_equidistant(const LensInfo &li, float img_w,
                                       float img_h, float alpha, float theta,
                                       float &cx, float &cy) {
  float x = std::cos(alpha);
  float y = std::sin(alpha);

  // r_mm = f * theta
  // r_px = (r_mm / sensor_size) * image_size
  // r_px = (f * theta / sensor_size) * image_size

  // sensor_size/2 = f * (fov/2)
  // f = sensor_size / fov

  float factor = li.sensor_width / img_w; // [mm / px]
  float focal_length = li.sensor_width / li.fisheye_equidistant.fov;

  float r_mm = focal_length * theta;
  float r_px = r_mm / li.sensor_width * img_w;
  cx = r_px * x;
  cy = r_px * y;
}

// === EQUIRECTANGULAR ===

inline void equirectangular_to_vec(const LensInfo &li, float img_w, float img_h,
                                   float cx, float cy, float &x, float &y,
                                   float &z) {
  // clang-format off
  float longitude_span = li.equirectangular.longitude_max - li.equirectangular.longitude_min;
  float latitude_span  = li.equirectangular.latitude_max  - li.equirectangular.latitude_min;
  float longitude = ((cx / img_w) + 0.5f) * longitude_span + li.equirectangular.longitude_min;
  float latitude  = ((cy / img_h) + 0.5f) * latitude_span  + li.equirectangular.latitude_min;
  // clang-format on
  x = std::sin(longitude);
  z = -std::cos(longitude);
  y = std::sin(latitude);
}

inline void vec_to_equirectangular(const LensInfo &li, float img_w, float img_h,
                                   float x, float y, float z, float &cx,
                                   float &cy) {
  float theta = -std::atan2(-x, -z);
  float phi = std::asin(y / std::sqrt(x * x + y * y + z * z));

  // clang-format off
  float longitude_span = li.equirectangular.longitude_max - li.equirectangular.longitude_min;
  float latitude_span  = li.equirectangular.latitude_max  - li.equirectangular.latitude_min;
  cx = ((theta - li.equirectangular.longitude_min) / longitude_span - 0.5f) * img_w;
  cy = ((phi   - li.equirectangular.latitude_min ) / latitude_span  - 0.5f) * img_h;
  // clang-format on
}

template <target_to_vec_t tgt2vec, vec_to_source_t vec2src, sample_func_t sf>
void reproject_from_to_with_interpolation(const Image *in, Image *out,
                                          int num_samples,
                                          const float *rotation_matrix) {
  ZoneScoped;

  int pitch = out->width * out->channels;
  float normalize = (1.0f / (num_samples * num_samples));
  float *buffer = new float[2 * out->channels];
  float *sample_accumulator = buffer;
  float *sample = buffer + out->channels;
  for (int y = 0; y < out->height; ++y) {
    for (int x = 0; x < out->width; ++x) {
      // Take center of pixel, and center entire image around (0,0)
      float cx = (x + 0.5f) - out->width * 0.5f;
      float cy = (y + 0.5f) - out->height * 0.5f;

      for (int c = 0; c < out->channels; ++c) {
        sample_accumulator[c] = 0.0f;
      }

      for (int ssx = 0; ssx < num_samples; ++ssx) {
        float scx = cx + (ssx + 1.0f) / (num_samples + 1.0f) - 0.5f;

        for (int ssy = 0; ssy < num_samples; ++ssy) {
          float scy = cy + (ssy + 1.0f) / (num_samples + 1.0f) - 0.5f;

          float vx, vy, vz;
          tgt2vec(out->lens, out->width, out->height, scx, scy, vx, vy, vz);

          if (rotation_matrix != nullptr) {
            const float *rm = rotation_matrix;
            float nx = rm[0] * vx + rm[1] * vy + rm[2] * vz;
            float ny = rm[3] * vx + rm[4] * vy + rm[5] * vz;
            float nz = rm[6] * vx + rm[7] * vy + rm[8] * vz;
            vx = nx;
            vy = ny;
            vz = nz;
          }

          float sx, sy; // source coordinate on input image
          vec2src(in->lens, in->width, in->height, vx, vy, vz, sx, sy);

          // if (x == out->width / 2 && y == out->height / 2) {
          //   std::printf("x,y:%d,%d; (w,h:%d,%d)  vec:%12f,%12f%12f;
          //   cx,cy:%f,%f, sx,sy:%12f,%12f\n", x, y, out->width, out->height,
          //   vx, vy, vz, cx, cy, sx, sy);
          // }

          // convert back to top-left aligned coordinates
          sx = (sx - 0.5f) + in->width * 0.5f;
          sy = (sy - 0.5f) + in->height * 0.5f;

          // if (x == out->width / 2 && y == out->height / 2) {
          //   std::printf("    sx,sy:%12f,%12f (w,h:%d,%d)\n", sx, sy,
          //   in->width, in->height);
          // }

          // bilinear interpolation from source image
          sf(in, sx, sy, sample);

          for (int c = 0; c < out->channels; ++c) {
            sample_accumulator[c] += sample[c];
          }
        }
        float *dst = &out->data[y * pitch + x * out->channels];
        for (int c = 0; c < out->channels; ++c) {
          dst[c] = sample_accumulator[c] * normalize;
        }
      }
    }
  }
  delete[] buffer;
}

template <target_to_vec_t tgt2vec, vec_to_source_t vec2src,
          bool LoopHorizontally>
void reproject_from_to(const Image *in, Image *out, int num_samples,
                       const float *rotation_matrix, Interpolation im) {
  if (im == NEAREST) {
    reproject_from_to_with_interpolation<tgt2vec, vec2src,
                                         sample_nearest<LoopHorizontally>>(
        in, out, num_samples, rotation_matrix);
  } else if (im == BILINEAR) {
    reproject_from_to_with_interpolation<tgt2vec, vec2src,
                                         sample_bilinear<LoopHorizontally>>(
        in, out, num_samples, rotation_matrix);
  } else if (im == BICUBIC) {
    reproject_from_to_with_interpolation<tgt2vec, vec2src,
                                         sample_bicubic<LoopHorizontally>>(
        in, out, num_samples, rotation_matrix);
  }
}

/**
 * Reproject a given image to an already-specified (target) lens-type, as per
 * the templated target_to_aospherical. This function switches over the input
 * lens type.
 */
template <target_to_vec_t tgt2vec>
void reproject_to(const Image *in, Image *out, int num_samples,
                  const float *rotation_matrix, Interpolation im) {
  if (in->lens.type == RECTILINEAR) {
    reproject_from_to<tgt2vec, vec_to_rectilinear, false>(in, out, num_samples,
                                                          rotation_matrix, im);
  } else if (in->lens.type == FISHEYE_EQUIDISTANT) {
    reproject_from_to<tgt2vec, vec_to_equidistant, false>(in, out, num_samples,
                                                          rotation_matrix, im);
  } else if (in->lens.type == EQUIRECTANGULAR) {

    float long_range = in->lens.equirectangular.longitude_max -
                       in->lens.equirectangular.longitude_min;
    if (std::abs(long_range - (2 * M_PI)) < 1e-5f) {
      reproject_from_to<tgt2vec, vec_to_equirectangular, true>(
          in, out, num_samples, rotation_matrix, im);
    } else {
      reproject_from_to<tgt2vec, vec_to_equirectangular, false>(
          in, out, num_samples, rotation_matrix, im);
    }
  }
}

void reproject_with_sample_method(const Image *in, Image *out, int num_samples,
                                  const float *rotation_matrix,
                                  Interpolation im) {}

void reproject(const Image *in, Image *out, int num_samples, Interpolation im,
               const float *rotation_matrix) {
  ZoneScoped;
  if (out->lens.type == RECTILINEAR) {
    reproject_to<rectilinear_to_vec>(in, out, num_samples, rotation_matrix, im);
  } else if (out->lens.type == FISHEYE_EQUIDISTANT) {
    reproject_to<equidistant_to_vec>(in, out, num_samples, rotation_matrix, im);
  } else if (out->lens.type == EQUIRECTANGULAR) {
    reproject_to<equirectangular_to_vec>(in, out, num_samples, rotation_matrix,
                                         im);
  }
}

void post_process(const Image *img, float exposure, float reinhard) {
  ZoneScoped;
  int ch = std::min(img->channels, 3);
  int i = 0;
  for (int y = 0; y < img->height; ++y) {
    for (int x = 0; x < img->width; ++x) {
      for (int c = 0; c < ch; ++c) {
        float v = img->data[i];
        v *= exposure;
        v = v * (1.0f + v / (reinhard * reinhard)) / (1.0f + v);
        img->data[i] = v;
        i++;
      }
      i += img->channels - ch;
    }
  }
}

constexpr auto test_eq = [](std::string name_a, float a, std::string name_b,
                            float b) {
  if (std::abs(a - b) > 1e-5f) {
    std::printf("ERROR: %s = %12f, %s = %12f\n", name_a.c_str(), a,
                name_b.c_str(), b);
  }
};

bool check_all_equal(
    std::vector<std::tuple<std::string, float, std::string, float>> pairs,
    std::string test, float i0, float i1) {
  bool good = true;
  for (auto &pair : pairs) {
    float a = std::get<1>(pair);
    float b = std::get<3>(pair);
    if (std::abs(a - b) > 1e-5f) {
      if (good) {
        std::printf("Test %s (input: %f %f) failed:\n", test.c_str(), i0, i1);
      }
      good = false;

      std::printf("  %s=%12f, %s=%12f\n", std::get<0>(pair).c_str(), a,
                  std::get<2>(pair).c_str(), b);
    }
  }
  return good;
}

void test_conversion_math() { ZoneScoped; }

} // namespace reproject
