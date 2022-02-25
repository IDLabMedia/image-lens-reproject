#include "reproject.hpp"

#include <algorithm>
#include <cmath>

#include <Tracy.hpp>

namespace reproject {

typedef void (*from_func_t)(const LensInfo &li, float img_w, float img_h,
                            float cx, float cy, float &alpha, float &theta);
typedef void (*to_func_t)(const LensInfo &li, float img_w, float img_h,
                          float cx, float cy, float &alpha, float &theta);

typedef void (*sample_func_t)(const Image *img, float sx, float sy, float *out);

template <typename T> inline T clamp(T x, T min, T max) {
  return std::max(min, std::min(max, x));
}

inline void sample_nearest(const Image *img, float sx, float sy, float *out) {
  int lx = clamp(int(sx + 0.5f), 0, img->width - 1);
  int ly = clamp(int(sy + 0.5f), 0, img->height - 1);

  int pitch = img->width * img->channels;
  for (int c = 0; c < img->channels; ++c) {
    out[c] = img->data[ly * pitch + lx * img->channels + c];
  }
}

inline void sample_bilinear(const Image *img, float sx, float sy, float *out) {
  // clang-format off
    int lx = clamp(int(sx)       , 0, img->width - 1);
    int ux = clamp(int(sx + 1.0f), 0, img->width - 1);
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
    return p[1] + 0.5 * x * (p[2] - p[0] + x *
            (2.0 * p[0] - 5.0 * p[1] + 4.0 * p[2] - p[3] + x *
             (3.0 * (p[1] - p[2]) + p[3] - p[0])));
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

inline void sample_bicubic(const Image *img, float sx, float sy, float *out) {
  // clang-format off
    int x0 = clamp(int(sx - 1.0f), 0, img->width - 1);
    int x1 = clamp(int(sx       ), 0, img->width - 1);
    int x2 = clamp(int(sx + 1.0f), 0, img->width - 1);
    int x3 = clamp(int(sx + 2.0f), 0, img->width - 1);
    int y0 = clamp(int(sy - 1.0f), 0, img->height - 1);
    int y1 = clamp(int(sy       ), 0, img->height - 1);
    int y2 = clamp(int(sy + 1.0f), 0, img->height - 1);
    int y3 = clamp(int(sy + 2.0f), 0, img->height - 1);
  // clang-format on
  //
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

inline void rectilinear_to_spherical(const LensInfo &li, float img_w,
                                     float img_h, float cx, float cy,
                                     float &alpha, float &theta) {
  float r_px = std::sqrt(cx * cx + cy * cy); // [px]
  alpha = std::atan2(cy, cx);
  float factor = li.sensor_width / img_w; // [mm / px]
  // r_mm = focal_length_mm * tan(theta)
  // TODO validate
  theta = std::atan(r_px / li.rectilinear.focal_length * factor);
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
  // TODO valide
}

// === EQUIDISTANT ===

inline void equidistant_to_spherical(const LensInfo &li, float img_w,
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

inline void spherical_to_equidistant(const LensInfo &li, float img_w,
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

template <from_func_t ff, to_func_t tf, sample_func_t sf>
void reproject_from_to(const Image *in, Image *out, int num_samples) {
  ZoneScoped;

  int pitch = out->width * out->channels;
  float normalize = (1.0f / (num_samples * num_samples));
  for (int y = 0; y < out->height; ++y) {
    for (int x = 0; x < out->width; ++x) {
      // Center around (0,0)
      float cx = (x + 0.5f) - out->width * 0.5f;
      float cy = (y + 0.5f) - out->height * 0.5f;

      float sample_accumulator[out->channels];
      for (int c = 0; c < out->channels; ++c) {
        sample_accumulator[c] = 0.0f;
      }

      for (int ssx = 0; ssx < num_samples; ++ssx) {
        float scx = cx + (ssx + 1.0f) / (num_samples + 1.0f) - 0.5f;

        for (int ssy = 0; ssy < num_samples; ++ssy) {
          float scy = cy + (ssy + 1.0f) / (num_samples + 1.0f) - 0.5f;

          float alpha;
          float theta;
          ff(out->lens, out->width, out->height, scx, scy, alpha, theta);

          float sx, sy; // source coordinate on input image
          tf(in->lens, in->width, in->height, alpha, theta, sx, sy);

          // convert back to top-left aligned coordinates
          sx = (sx - 0.5f) + in->width * 0.5f;
          sy = (sy - 0.5f) + in->height * 0.5f;

          // bilinear interpolation from source image
          float sample[out->channels];
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
}

template <from_func_t ff, sample_func_t sf>
void reproject_from(const Image *in, Image *out, int num_samples) {
  if (in->lens.type == RECTILINEAR) {
    reproject_from_to<ff, spherical_to_rectilinear, sf>(in, out, num_samples);
  } else if (in->lens.type == FISHEYE_EQUIDISTANT) {
    reproject_from_to<ff, spherical_to_equidistant, sf>(in, out, num_samples);
  } else if (in->lens.type == EQUIRECTANGULAR) {
    throw std::runtime_error("Equirectangular not supported.");
  }
}

template <sample_func_t sf>
void reproject_with_sample_method(const Image *in, Image *out,
                                  int num_samples) {
  if (out->lens.type == RECTILINEAR) {
    reproject_from<rectilinear_to_spherical, sf>(in, out, num_samples);
  } else if (out->lens.type == FISHEYE_EQUIDISTANT) {
    reproject_from<equidistant_to_spherical, sf>(in, out, num_samples);
  } else if (out->lens.type == EQUIRECTANGULAR) {
    throw std::runtime_error("Equirectangular not supported.");
  }
}

void reproject_nearest_neighbor(const Image *in, Image *out, int num_samples) {
  ZoneScoped;
  reproject_with_sample_method<sample_nearest>(in, out, num_samples);
}
void reproject_bilinear(const Image *in, Image *out, int num_samples) {
  ZoneScoped;
  reproject_with_sample_method<sample_bilinear>(in, out, num_samples);
}
void reproject_bicubic(const Image *in, Image *out, int num_samples) {
  ZoneScoped;
  reproject_with_sample_method<sample_bicubic>(in, out, num_samples);
}
void reproject(const Image *in, Image *out, int num_samples, Interpolation im) {
  if (im == NEAREST) {
    reproject_nearest_neighbor(in, out, num_samples);
  } else if (im == BILINEAR) {
    reproject_bilinear(in, out, num_samples);
  } else if (im == BICUBIC) {
    reproject_bicubic(in, out, num_samples);
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

} // namespace reproject
