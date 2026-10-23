# Shared warning and optimisation flags for first-party targets.

include(CheckIPOSupported)
check_ipo_supported(RESULT SICDM_IPO_SUPPORTED OUTPUT _ipo_msg LANGUAGES CXX)

function(sicdm_set_compile_options target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /Zc:__cplusplus
            /fp:precise $<$<CONFIG:Release>:/O2>) # precise: no contraction, as below
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion
            # No fused multiply-adds: geometric predicates (booleans, coplanarity) must
            # round the same on every CPU, or Apple Silicon and x86 disagree.
            -ffp-contract=off
            $<$<CONFIG:Release>:-O3 -fno-math-errno>)
        if(SICDM_NATIVE_ARCH AND NOT CMAKE_GENERATOR STREQUAL "Xcode")
            target_compile_options(${target} PRIVATE -march=native)
        endif()
    endif()

    if(SICDM_ENABLE_LTO AND SICDM_IPO_SUPPORTED)
        set_property(TARGET ${target} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    endif()
endfunction()
