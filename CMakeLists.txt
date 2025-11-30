cmake_minimum_required(VERSION 4.2)
project(TokenRing LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON)

set(CMAKE_POLICY_VERSION 4.2)

if(APPLE)
    add_compile_definitions(GL_SILENCE_DEPRECATION)
endif()

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
    GIT_TAG        v1.92.5
)
FetchContent_MakeAvailable(imgui)

FetchContent_Declare(
    rlimgui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(rlimgui)

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

add_executable(TokenRing
    src/main.cpp
    src/Entity.cpp
    src/Node.cpp
    src/Token.cpp
    src/Ring.cpp
    src/Visualizer.cpp
)

target_link_libraries(TokenRing PRIVATE UI_Backend)

target_include_directories(TokenRing PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMENT "Updating compile_commands.json in project root (clangd support)"
)

