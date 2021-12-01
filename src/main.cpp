#define CXXOPTS_NO_REGEX 1
#include <cxxopts.hpp>

#include "reproject.hpp"

int main(int argc, char** argv) {
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
    } catch (cxxopts::OptionParseException& e) {
        std::printf("%s\n\n%s\n", e.what(), options.help().c_str());
        return 1;
    }




    return 0;
}
