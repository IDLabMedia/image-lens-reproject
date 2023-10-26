cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0022 NEW)
cmake_policy(SET CMP0079 NEW)

project(reproject VERSION 0.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(lib/filesystem)
add_subdirectory(lib/cxxopts)

add_subdirectory(lib/tracy)
add_library(TracyHeaders INTERFACE)
target_include_directories(TracyHeaders INTERFACE "lib/tracy/")
if ($TRACY_ENABLE)
    add_definitions(-DTRACY_ENABLE)
endif()

add_library(lodepng STATIC "lib/lodepng/lodepng.cpp")
target_include_directories(lodepng PUBLIC "lib/lodepng/")

add_subdirectory(lib/Imath)

set(OPENEXR_INSTALL OFF CACHE BOOL "" FORCE)
set(OPENEXR_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(OPENEXR_INSTALL_TOOLS OFF CACHE BOOL "" FORCE)
set(OPENEXR_INSTALL_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(lib/openexr)
target_link_libraries(OpenEXRCore PUBLIC Imath::Imath)

add_library(ctpl INTERFACE)
target_include_directories(ctpl INTERFACE "lib/ctpl/")

set(JSON_MultipleHeaders ON CACHE BOOL "" FORCE)
add_subdirectory(lib/json)

find_package(JPEG)

set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "\$ORIGIN")
add_executable(reproject
    "src/main.cpp"
    "src/reproject.cpp"
    "src/image_formats.cpp"
    "src/config.cpp"
    )
target_include_directories(reproject PUBLIC "src")
target_link_libraries(reproject PUBLIC
    TracyHeaders
    ghc_filesystem
    lodepng
    cxxopts
    TracyClient
    OpenEXR::OpenEXR
    ctpl
    nlohmann_json
    JPEG::JPEG
    )
set_target_properties(reproject PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
target_compile_definitions(reproject PRIVATE -D_USE_MATH_DEFINES)



foreach(LIBRARY OpenEXRCore OpenEXR Imath Iex IlmThread)
  set_target_properties(${LIBRARY} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
endforeach()
