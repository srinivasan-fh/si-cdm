cmake_minimum_required(VERSION 3.24)

# Deployment target must be set before project() to take effect on Xcode.
set(CMAKE_OSX_DEPLOYMENT_TARGET "12.0" CACHE STRING "Minimum macOS version")

project(SICDM
    VERSION 0.1.0
    DESCRIPTION "SI CDM - high-performance CAD/CAM workbench"
    LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(SICDM_BUILD_APP   "Build the desktop application" ON)
option(SICDM_BUILD_TESTS "Build unit tests" ON)
option(SICDM_ENABLE_LTO  "Enable link-time optimisation in Release" ON)
option(SICDM_NATIVE_ARCH "Tune for the build machine's CPU (-march=native)" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerOptions)

# ---------------------------------------------------------------------------
# stb_image (single header, public domain / MIT) is vendored in third_party/stb
# at commit 2c980bb59875b0d32144a71867fbdebb2f77cd20, for reference-image import.

# ---------------------------------------------------------------------------
# sicdm_core: platform-independent kernel (math, camera, geometry, scene).
# No windowing or GPU dependencies so it can be unit tested headless.
add_library(sicdm_core STATIC
    src/core/Camera.cpp
    src/core/Geometry.cpp
    src/core/Scene.cpp
    src/core/Command.cpp
    src/core/Extrude.cpp
    src/core/CommandEngine.cpp
    src/core/FileIO.cpp
    src/core/Json.cpp
    src/core/Mcp.cpp
    src/core/StbImage.cpp
    src/core/Sketch.cpp
    src/ui/Tools.cpp
)
target_include_directories(sicdm_core PUBLIC src)
target_include_directories(sicdm_core SYSTEM PRIVATE third_party/stb)
sicdm_set_compile_options(sicdm_core)

# sicdm-mcp: headless MCP server over stdio (no window, no GPU).
add_executable(sicdm_mcp tools/sicdm_mcp.cpp)
set_target_properties(sicdm_mcp PROPERTIES OUTPUT_NAME "sicdm-mcp")
target_link_libraries(sicdm_mcp PRIVATE sicdm_core)
sicdm_set_compile_options(sicdm_mcp)

# ---------------------------------------------------------------------------
if(SICDM_BUILD_APP)
    include(Dependencies)

    set(SICDM_APP_SOURCES
        src/main.cpp
        src/app/Application.cpp
        src/app/McpHttpServer.cpp
        src/app/SketchController.cpp
        src/render/Renderer.cpp
        src/render/Shader.cpp
        src/ui/FileDialog.cpp
        src/ui/LandingUI.cpp
        src/ui/Toolbox.cpp
        src/ui/ToolIcons.cpp
        src/ui/ViewCube.cpp
    )

    if(APPLE)
        set(SICDM_BUNDLE_NAME "SI CDM")
        set(SICDM_BUNDLE_ID "com.sicdm.workbench")
        add_executable(sicdm MACOSX_BUNDLE ${SICDM_APP_SOURCES})
        set_target_properties(sicdm PROPERTIES
            OUTPUT_NAME "SICDM"
            MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/resources/macos/Info.plist.in"
            MACOSX_BUNDLE_BUNDLE_NAME "${SICDM_BUNDLE_NAME}"
            MACOSX_BUNDLE_GUI_IDENTIFIER "${SICDM_BUNDLE_ID}"
            MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
            MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
            XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER "${SICDM_BUNDLE_ID}"
            XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "-"
            XCODE_ATTRIBUTE_ENABLE_HARDENED_RUNTIME "YES"
            XCODE_ATTRIBUTE_GCC_OPTIMIZATION_LEVEL[variant=Release] "3"
            XCODE_ATTRIBUTE_LLVM_LTO[variant=Release] "YES_THIN"
            XCODE_ATTRIBUTE_DEAD_CODE_STRIPPING "YES"
            XCODE_GENERATE_SCHEME ON
        )
    else()
        add_executable(sicdm ${SICDM_APP_SOURCES})
    endif()

    target_compile_definitions(sicdm PRIVATE SICDM_VERSION="${PROJECT_VERSION}")
    find_package(Threads REQUIRED)
    target_link_libraries(sicdm PRIVATE sicdm_core sicdm_imgui glfw OpenGL::GL Threads::Threads)
    sicdm_set_compile_options(sicdm)

    if(CMAKE_GENERATOR STREQUAL "Xcode")
        source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src PREFIX "Sources" FILES ${SICDM_APP_SOURCES})
    endif()
endif()

# ---------------------------------------------------------------------------
if(SICDM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
