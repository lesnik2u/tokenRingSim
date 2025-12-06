cmake_minimum_required(VERSION 3.25)
project(TokenRing LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Suppress developer warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON)

# Add local module path if we were using modules, but we aren't yet.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# macOS specific
if(APPLE)
    add_compile_definitions(GL_SILENCE_DEPRECATION)
endif()

# Dependencies
include(FetchContent)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.5
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        docking
)
FetchContent_MakeAvailable(imgui)

FetchContent_Declare(
    rlimgui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(rlimgui)

# UI Backend Library
add_library(UI_Backend STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${rlimgui_SOURCE_DIR}/rlImGui.cpp
)

target_include_directories(UI_Backend PUBLIC
    ${imgui_SOURCE_DIR}
    ${rlimgui_SOURCE_DIR}
)

target_link_libraries(UI_Backend PUBLIC raylib)

# Main Executable
add_executable(TokenRing
    src/main.cpp
    src/core/Entity.cpp
    src/core/Node.cpp
    src/core/Token.cpp
    src/core/Ring.cpp
    src/graphics/Visualizer.cpp
)

# Include directories
target_include_directories(TokenRing PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(TokenRing PRIVATE UI_Backend raylib)

# Copy compile_commands.json for clangd
add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMENT "Updating compile_commands.json in project root"
)