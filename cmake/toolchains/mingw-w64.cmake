# Cross-compiling for 64-bit Windows with MinGW-w64 (from Linux):
#   cmake -B build/mingw -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/mingw-w64.cmake
# The posix thread model is needed for std::thread.
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(SICDM_MINGW_PREFIX x86_64-w64-mingw32)
find_program(CMAKE_C_COMPILER NAMES ${SICDM_MINGW_PREFIX}-gcc-posix ${SICDM_MINGW_PREFIX}-gcc REQUIRED)
find_program(CMAKE_CXX_COMPILER NAMES ${SICDM_MINGW_PREFIX}-g++-posix ${SICDM_MINGW_PREFIX}-g++ REQUIRED)
find_program(CMAKE_RC_COMPILER NAMES ${SICDM_MINGW_PREFIX}-windres windres)
set(CMAKE_FIND_ROOT_PATH /usr/${SICDM_MINGW_PREFIX})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
