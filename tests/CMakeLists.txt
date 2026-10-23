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
    test_sketch_edit.cpp
)
target_link_libraries(sicdm_tests PRIVATE sicdm_core)
sicdm_set_compile_options(sicdm_tests)
add_test(NAME sicdm_tests COMMAND sicdm_tests)
# Tests that read example files from the source tree.
target_compile_definitions(sicdm_tests PRIVATE SICDM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
