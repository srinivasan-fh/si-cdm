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

if(MINGW)
    # Self-contained executables: no libstdc++ / libgcc / winpthread DLLs to ship beside them.
    add_link_options(-static -static-libgcc -static-libstdc++)
endif()

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
    src/core/Units.cpp
    src/core/Extrude.cpp
    src/core/Motion.cpp
    src/core/Pose.cpp
    src/core/Physics.cpp
    src/core/Sculpt.cpp
    src/core/Wearable.cpp
    src/core/Print.cpp
    src/core/Section.cpp
    src/core/Cam.cpp
    src/core/CamCommand.cpp
    src/core/CamEdit.cpp
    src/core/CamLibrary.cpp
    src/core/FeatureCommand.cpp
    src/core/Constraints.cpp
    src/core/Brep.cpp
    src/core/Step.cpp
    src/core/Recovery.cpp
    src/core/CamVerify.cpp
    src/core/Iges.cpp
    src/core/Assembly.cpp
    src/core/Explode.cpp
    src/core/DirectEdit.cpp
    src/core/FaceCommand.cpp
    src/core/SectionView.cpp
    src/core/Zip.cpp
    src/core/Xml.cpp
    src/core/MeshImport.cpp
    src/core/Weldment.cpp
    src/core/Route.cpp
    src/core/RouteCommand.cpp
    src/core/Mold.cpp
    src/core/Fastener.cpp
    src/core/FastenerCommand.cpp
    src/core/Study.cpp
    src/core/StudyCommand.cpp
    src/core/Stackup.cpp
    src/core/StackupCommand.cpp
    src/core/Armature.cpp
    src/core/ArmatureCommand.cpp
    src/core/Gizmo.cpp
    src/core/InsertBrush.cpp
    src/core/InsertCommand.cpp
    src/core/ModelExport.cpp
    src/core/Video.cpp
    src/core/Selection.cpp
    src/core/Keymap.cpp
    src/core/MoldCommand.cpp
    src/core/WeldmentCommand.cpp
    src/core/Bvh.cpp
    src/core/SketchEdit.cpp
    src/core/AssemblyCommand.cpp
    src/core/Drawing.cpp
    src/core/Tolerances.cpp
    src/core/DrawingCommand.cpp
    src/core/SketchCommand.cpp
    src/core/Features.cpp
    src/core/HoleWizard.cpp
    src/core/Variables.cpp
    src/core/SheetMetal.cpp
    src/core/Fea.cpp
    src/core/FeaReport.cpp
    src/core/Multires.cpp
    src/core/Retopo.cpp
    src/core/UvMap.cpp
    src/core/Translate.cpp
    src/core/Material.cpp
    src/core/Render.cpp
    src/core/Collide.cpp
    src/core/Components.cpp
    src/core/ComponentCommand.cpp
    src/core/Pdm.cpp
    src/core/PdmCommand.cpp
    src/core/lang/German.cpp
    src/core/Library.cpp
    src/core/Lathe.cpp
    src/core/LatheCommand.cpp
    src/core/Ed25519.cpp
    src/core/Licence.cpp
    src/core/Preferences.cpp
    src/core/Rig.cpp
    src/core/RobotExport.cpp
    src/core/Multibody.cpp
    src/core/RobotRun.cpp
    src/core/Terrain.cpp
    src/core/Vehicle.cpp
    src/core/Sensors.cpp
    src/core/Kinematics.cpp
    src/core/RobotEnv.cpp
    src/core/ForceField.cpp
    src/core/ImageWrite.cpp
    src/core/RobotCommand.cpp
    src/core/UpdateCheck.cpp
    src/core/CrashReport.cpp
    src/core/Plugin.cpp
    src/core/CommandEngine.cpp
    src/core/FileIO.cpp
    src/core/Json.cpp
    src/core/Mcp.cpp
    src/core/StbImage.cpp
    src/core/Sketch.cpp
    src/core/TextOutline.cpp
    src/core/ExactMesh.cpp
    src/core/Surfacing.cpp
    src/core/PickFeature.cpp
    src/core/SketchShapes.cpp
    src/core/PointCloud.cpp
    src/ui/Tools.cpp
)
target_include_directories(sicdm_core PUBLIC src)
target_compile_definitions(sicdm_core PUBLIC SICDM_VERSION="${PROJECT_VERSION}")

# The third-party notices, compiled in (About, NOTICES).
file(READ "${PROJECT_SOURCE_DIR}/resources/THIRD-PARTY-NOTICES.txt" SICDM_NOTICES_TEXT)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/resources/THIRD-PARTY-NOTICES.txt")
configure_file(src/core/Notices.cpp.in "${CMAKE_CURRENT_BINARY_DIR}/generated/Notices.cpp" @ONLY)
target_sources(sicdm_core PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated/Notices.cpp")
# The user guide and the example scripts, built in for the in-app help.
file(GLOB SICDM_DOC_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/docs/*.md" "${CMAKE_CURRENT_SOURCE_DIR}/docs/examples/*.txt")
add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/EmbeddedDocs.cpp"
    COMMAND ${CMAKE_COMMAND} -DDOCS_DIR=${CMAKE_CURRENT_SOURCE_DIR}/docs -DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/generated/EmbeddedDocs.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/EmbedDocs.cmake
    DEPENDS ${SICDM_DOC_SOURCES} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/EmbedDocs.cmake"
    COMMENT "Building the user guide in"
    VERBATIM)
target_sources(sicdm_core PRIVATE src/core/HelpDocs.cpp "${CMAKE_CURRENT_BINARY_DIR}/generated/EmbeddedDocs.cpp")
# The fonts, built in for the UI and for raised and cut text on parts (TextOutline).
include(EmbedFonts)
sicdm_embed_core_fonts(SICDM_CORE_FONT_SOURCE)
target_sources(sicdm_core PRIVATE ${SICDM_CORE_FONT_SOURCE})

# Licences: the vendor's Ed25519 public key (64 hex digits) that licence files are checked
# with. Empty: the build checks none and runs as the Community edition. Make a key pair
# with sicdm-licence keygen.
set(SICDM_LICENCE_PUBLIC_KEY "" CACHE STRING "Ed25519 public key (64 hex digits) for licence files")
if(SICDM_LICENCE_PUBLIC_KEY AND NOT SICDM_LICENCE_PUBLIC_KEY MATCHES "^[0-9a-fA-F]+$")
    message(FATAL_ERROR "SICDM_LICENCE_PUBLIC_KEY must be 64 hex digits")
endif()
string(LENGTH "${SICDM_LICENCE_PUBLIC_KEY}" _sicdm_key_len)
if(SICDM_LICENCE_PUBLIC_KEY AND NOT _sicdm_key_len EQUAL 64)
    message(FATAL_ERROR "SICDM_LICENCE_PUBLIC_KEY must be 64 hex digits")
endif()
set_source_files_properties(src/core/Licence.cpp PROPERTIES COMPILE_DEFINITIONS "SICDM_LICENCE_PUBLIC_KEY=\"${SICDM_LICENCE_PUBLIC_KEY}\"")
target_include_directories(sicdm_core SYSTEM PRIVATE third_party/stb)
sicdm_set_compile_options(sicdm_core)

# sicdm-mcp: headless MCP server over stdio (no window, no GPU).
add_executable(sicdm_mcp tools/sicdm_mcp.cpp)
set_target_properties(sicdm_mcp PROPERTIES OUTPUT_NAME "sicdm-mcp")
target_link_libraries(sicdm_mcp PRIVATE sicdm_core)
sicdm_set_compile_options(sicdm_mcp)

# sicdm-licence: makes key pairs and issues licence files (for the vendor).
add_executable(sicdm_licence tools/sicdm_licence.cpp)
set_target_properties(sicdm_licence PROPERTIES OUTPUT_NAME "sicdm-licence")
target_link_libraries(sicdm_licence PRIVATE sicdm_core)
sicdm_set_compile_options(sicdm_licence)

# ---------------------------------------------------------------------------
if(SICDM_BUILD_APP)
    include(Dependencies)
    include(EmbedFonts)
    sicdm_embed_fonts(SICDM_FONT_SOURCE)

    set(SICDM_APP_SOURCES
        src/main.cpp
        src/app/Application.cpp
        src/app/McpHttpServer.cpp
        src/app/OpenFiles.cpp
        src/app/SketchController.cpp
        src/app/PenInput.cpp
        src/app/Splash.cpp
        src/render/GL.cpp
        src/render/Renderer.cpp
        src/render/Shader.cpp
        src/ui/FileDialog.cpp
        src/ui/CamPanel.cpp
        src/ui/HelpWindow.cpp
        src/ui/PreferencesWindow.cpp
        src/ui/ProductWindow.cpp
        src/ui/AppearanceWindow.cpp
        src/ui/LandingUI.cpp
        src/ui/Toolbox.cpp
        src/ui/KeyInput.cpp
        src/ui/ToolIcons.cpp
        src/ui/Theme.cpp
        src/ui/ViewCube.cpp
    )

    if(APPLE)
        enable_language(OBJCXX)
        list(APPEND SICDM_APP_SOURCES src/app/PenInput_mac.mm) # tablet pressure (Cocoa events)
        set(SICDM_BUNDLE_NAME "SI CDM")
        set(SICDM_BUNDLE_ID "com.sicdm.workbench")
        # App icon (Dock, Finder, Launchpad), rendered by tools/make_icon.py.
        set(SICDM_ICNS "${CMAKE_CURRENT_SOURCE_DIR}/resources/macos/SICDM.icns")
        set_source_files_properties(${SICDM_ICNS} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
        add_executable(sicdm MACOSX_BUNDLE ${SICDM_APP_SOURCES} ${SICDM_FONT_SOURCE} ${SICDM_ICNS})
        set_target_properties(sicdm PROPERTIES
            OUTPUT_NAME "SICDM"
            MACOSX_BUNDLE_ICON_FILE "SICDM.icns"
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
    elseif(WIN32)
        # Icon and version information; a GUI program, so no console window opens with it.
        set(SICDM_ICO "${CMAKE_CURRENT_SOURCE_DIR}/resources/windows/sicdm.ico")
        configure_file(resources/windows/sicdm.rc.in "${CMAKE_CURRENT_BINARY_DIR}/sicdm.rc" @ONLY)
        add_executable(sicdm WIN32 ${SICDM_APP_SOURCES} ${SICDM_FONT_SOURCE} "${CMAKE_CURRENT_BINARY_DIR}/sicdm.rc")
        if(MSVC)
            target_link_options(sicdm PRIVATE /ENTRY:mainCRTStartup) # keep main() as the entry point
        endif()
    else()
        add_executable(sicdm ${SICDM_APP_SOURCES} ${SICDM_FONT_SOURCE})
    endif()

    find_package(Threads REQUIRED)
    target_link_libraries(sicdm PRIVATE sicdm_core sicdm_imgui glfw OpenGL::GL Threads::Threads)
    if(UNIX AND NOT APPLE)
        target_link_libraries(sicdm PRIVATE ${CMAKE_DL_LIBS}) # PenInput opens libXi at run time
    endif()
    target_include_directories(sicdm SYSTEM PRIVATE third_party/stb) # window icon / splash logo decoding
    sicdm_set_compile_options(sicdm)

    if(CMAKE_GENERATOR STREQUAL "Xcode")
        source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src PREFIX "Sources" FILES ${SICDM_APP_SOURCES})
        source_group("Generated" FILES ${SICDM_FONT_SOURCE})
        source_group("Resources" FILES ${SICDM_ICNS})
    endif()
endif()

# ---------------------------------------------------------------------------
include(Packaging)

# ---------------------------------------------------------------------------
if(SICDM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
