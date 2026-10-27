add_executable(sicdm_tests
    test_main.cpp
    test_math.cpp
    test_camera.cpp
    test_geometry.cpp
    test_picking.cpp
    test_tools.cpp
    test_command.cpp
    test_fileio.cpp
    test_engine.cpp
    test_mcp.cpp
    test_extrude.cpp
    test_motion.cpp
    test_library.cpp
    test_units.cpp
    test_pose.cpp
    test_physics.cpp
    test_sculpt.cpp
    test_wearable.cpp
    test_print.cpp
    test_cam.cpp
    test_features.cpp
    test_constraints.cpp
    test_step.cpp
    test_recovery.cpp
    test_camverify.cpp
    test_iges.cpp
    test_drawing.cpp
    test_assembly.cpp
    test_performance.cpp
    test_partdesign.cpp
    test_surfacing.cpp
    test_sketch_edit.cpp
    test_lathe.cpp
    test_system.cpp
    test_help.cpp
    test_plugin.cpp
    test_preferences.cpp
    test_robotics.cpp
    test_undo.cpp
)
target_link_libraries(sicdm_tests PRIVATE sicdm_core)
target_include_directories(sicdm_tests SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/third_party/stb) # reading back written images
# A program that crashes on purpose, and the licence tool, run by the tests.
add_executable(crash_probe crash_probe.cpp)
target_link_libraries(crash_probe PRIVATE sicdm_core)
add_executable(sample_plugin sample_plugin.cpp)
add_dependencies(sicdm_tests crash_probe sicdm_licence sample_plugin)
target_compile_definitions(sicdm_tests PRIVATE SICDM_CRASH_PROBE="$<TARGET_FILE:crash_probe>" SICDM_LICENCE_TOOL="$<TARGET_FILE:sicdm_licence>"
    SICDM_SAMPLE_PLUGIN="$<TARGET_FILE:sample_plugin>")
sicdm_set_compile_options(sicdm_tests)
add_test(NAME sicdm_tests COMMAND sicdm_tests)
# Tests that read example files from the source tree.
target_compile_definitions(sicdm_tests PRIVATE SICDM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
