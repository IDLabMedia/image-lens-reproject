#define CXXOPTS_NO_REGEX 1
#include <ImfArray.h>
#include <ImfInputFile.h>
#include <ImfNamespace.h>
#include <ImfRgba.h>
#include <ImfRgbaFile.h>
#include <lodepng.h>

#include <cxxopts.hpp>

#include "reproject.hpp"

void readRgba1(const char fileName[], Imf::Array2D<Imf::Rgba> &pixels,
               int &width, int &height) {
    //
    // Read an RGBA image using class RgbaInputFile:
    //
    //	- open the file
    //	- allocate memory for the pixels
    //	- describe the memory layout of the pixels
    //	- read the pixels from the file
    //

    Imf::RgbaInputFile file(fileName);
    Imath::Box2i dw = file.dataWindow();

    width = dw.max.x - dw.min.x + 1;
    height = dw.max.y - dw.min.y + 1;
    pixels.resizeErase(height, width);

    file.setFrameBuffer(&pixels[0][0] - dw.min.x - dw.min.y * width, 1, width);
    file.readPixels(dw.min.y, dw.max.y);
}

int main(int argc, char **argv) {
    // clang-format off
    cxxopts::Options options(argv[0],
        "Reprojection tool for producing a variation of lens\n"
        "configurations based on one reference image given a\n"
        "known lens configuration.");
    options.add_options()
        ("i,input", "Input image to reproject",
         cxxopts::value<std::string>(), "file")
        ("o,output", "Output file",
         cxxopts::value<std::string>(), "file")
        ("rectilinear", "Output rectilinear image with given FOV.",
         cxxopts::value<float>(), "fov")
        ("h,help", "Show help")
        ;
    // clang-format on
    cxxopts::ParseResult result;
    try {
        result = options.parse(argc, argv);
        if (result.count("help")) {
            std::printf("%s\n", options.help().c_str());
            return 0;
        }
    } catch (cxxopts::OptionParseException &e) {
        std::printf("%s\n\n%s\n", e.what(), options.help().c_str());
        return 1;
    }

    std::string input_file = result["input"].as<std::string>();
    std::printf("Reading EXR files: %s\n", input_file.c_str());
    Imf::Array2D<Imf::Rgba> data;
    int width, height;
    readRgba1(input_file.c_str(), data, width, height);
    std::printf("Read some data : %d x %d\n", width, height);
    int x = width / 2, y = height / 2;
    std::printf("%f; %f; %f; %f\n", float(data[y][x].r), float(data[y][x].g),
                float(data[y][x].b), float(data[y][x].a));

    reproject::Image input;
    input.lens.type = reproject::LensType::FISHEYE_EQUIDISTANT;
    input.lens.fisheye_equidistant.fov = M_PI;
    input.lens.sensor_width = 36.0f;
    input.lens.sensor_height = 36.0f;
    input.width = width;
    input.height = height;
    input.channels = 3;
    input.data = new float[input.width * input.height * input.channels];
    for (int y = 0; y < input.height; ++y) {
        for (int x = 0; x < input.width; ++x) {
            Imf::Rgba p = data[y][x];
            input.data[(y * input.width + x) * input.channels + 0] = float(p.r);
            input.data[(y * input.width + x) * input.channels + 1] = float(p.g);
            input.data[(y * input.width + x) * input.channels + 2] = float(p.b);
        }
    }

    reproject::Image output;
#if 1
    output.lens.type = reproject::LensType::RECTILINEAR;
    output.lens.rectilinear.focal_length = 18.0f;
    output.lens.sensor_width = 36.0f;
    output.lens.sensor_height = 36.0f;
#else
    output.lens = input.lens;
#endif
    output.width = width;
    output.height = height;
    output.channels = input.channels;
    output.data = new float[output.width * output.height * output.channels];

    std::printf("Reprojecting...\n");
    reproject::reproject(&input, &output);
    std::printf("Done!\n");

    std::string output_file = result["output"].as<std::string>();
    uint8_t *image_buf = new uint8_t[output.width * output.height * 4];
    float vmax = 0.0f;
    float vmin = 1.0f;
    for (int y = 0; y < output.height; ++y) {
        for (int x = 0; x < output.width; ++x) {
            for (int c = 0; c < output.channels; ++c) {
                float s =
                    output.data[((y * output.width) + x) * output.channels + c];
                vmax = std::max(vmax, s);
                vmin = std::min(vmin, s);
                //s = s / (s + 1.0f);
                s = std::max(0.0f, std::min(1.0f, s));
                uint8_t d = uint8_t(255.9f * s);
                image_buf[((y * output.width) + x) * 4 + c] = d;
            }
            if (output.channels != 4) {
                image_buf[((y * output.width) + x) * 4 + 3] = 255;
            }
        }
    }
    lodepng::encode(output_file.c_str(), image_buf, output.width,
                    output.height);
    std::printf("min: %f  max: %f\n", vmin, vmax);

    return 0;
}
