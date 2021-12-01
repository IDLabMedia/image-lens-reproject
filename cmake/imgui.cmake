add_library(imgui STATIC
    lib/imgui/imgui.cpp
    lib/imgui/imgui_widgets.cpp
    lib/imgui/imgui_draw.cpp)
target_include_directories(imgui PUBLIC lib/imgui)

add_library(imgui-sdl STATIC
    lib/imgui/examples/imgui_impl_sdl.cpp)
target_link_libraries(imgui-sdl PUBLIC SDL2::SDL2)
target_include_directories(imgui-sdl PRIVATE lib/imgui/)

add_library(imgui-opengl3 STATIC
    lib/imgui/examples/imgui_impl_opengl3.cpp)
target_link_libraries(imgui-opengl3 PUBLIC glad)
target_include_directories(imgui-opengl3 PRIVATE lib/imgui/)
target_compile_definitions(imgui-opengl3 PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLAD)
