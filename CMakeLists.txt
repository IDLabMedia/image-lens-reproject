cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0022 NEW)
cmake_policy(SET CMP0079 NEW)

project(reproject VERSION 0.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/)
set(CMAKE_CXX_STANDARD 17)

find_package(SDL2 REQUIRED)

add_subdirectory(lib/filesystem)
add_subdirectory(lib/cxxopts)
add_subdirectory(lib/glad)

add_subdirectory(lib/tracy)
add_library(TracyHeaders INTERFACE)
target_include_directories(TracyHeaders INTERFACE "lib/tracy/")
add_definitions(-DTRACY_ENABLE)

add_library(lodepng STATIC "lib/lodepng/lodepng.cpp")
target_include_directories(lodepng PUBLIC "lib/lodepng/")

add_subdirectory(lib/Imath)

set(OPENEXR_INSTALL OFF CACHE BOOL "" FORCE)
set(OPENEXR_INSTALL_TOOLS OFF CACHE BOOL "" FORCE)
set(OPENEXR_INSTALL_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(lib/openexr)
# Cleanup their mess
target_link_libraries(OpenEXRCore PUBLIC Imath::Imath)

add_executable(reproject
    "src/main.cpp"
    "src/reproject.cpp"
    "src/image_formats.cpp"
    )
target_include_directories(reproject PUBLIC "src")
target_link_libraries(reproject PUBLIC
    TracyHeaders
    ghc_filesystem
    lodepng
    SDL2::SDL2
    glad
    SDL2::SDL2main
    cxxopts
    TracyClient
    OpenEXR::OpenEXR
    )
