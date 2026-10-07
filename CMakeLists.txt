cmake_minimum_required(VERSION 3.20)
project(RxTwister VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(RX_FETCH_DEPS "Download missing dependencies with FetchContent" ON)
option(RX_WITH_ASSIMP "Enable Assimp (FBX, DAE, 3DS, X, ... import/export)" ON)
option(RX_WITH_TBB "Use oneTBB for parallelism when available" ON)
option(RX_WITH_DRACO "Enable Draco decompression for glTF" ON)
option(RX_NATIVE_ARCH "Optimise for the build machine's CPU (-march=native)" OFF)
option(RX_BUILD_TESTS "Build tests" ON)

include(cmake/Dependencies.cmake)

add_library(rxcore
  src/core/util.cpp
  src/tex/image_ops.cpp
  src/tex/textures.cpp
  src/tex/binder.cpp
  src/tex/normal_bake.cpp
  src/geom/mesh_ops.cpp
  src/geom/subdivide.cpp
  src/geom/simplify.cpp
  src/geom/remesh.cpp
  src/geom/bvh.cpp
  src/io/common.cpp
  src/io/gltf_read.cpp
  src/io/gltf_write.cpp
  src/io/obj_io.cpp
  src/io/stl_ply_io.cpp
  src/io/usd_write.cpp
  src/io/assimp_io.cpp
  src/io/blender_io.cpp
  src/pipeline.cpp
  src/cli_args.cpp
  src/c_api.cpp)
target_include_directories(rxcore PUBLIC include PRIVATE src ${RX_STB_INCLUDE})
target_link_libraries(rxcore PUBLIC ${RX_DEP_LIBS} Threads::Threads)
target_compile_definitions(rxcore PRIVATE ${RX_DEP_DEFS} _USE_MATH_DEFINES NOMINMAX)
if(MSVC)
  target_compile_options(rxcore PRIVATE /W3 /O2 /utf-8 /bigobj)
else()
  target_compile_options(rxcore PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Wno-sign-compare)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(rxcore PRIVATE -Wno-class-memaccess)
  endif()
  if(RX_NATIVE_ARCH)
    target_compile_options(rxcore PRIVATE -march=native)
  endif()
endif()

add_executable(rxtwister apps/cli/main.cpp)
target_include_directories(rxtwister PRIVATE src)
target_link_libraries(rxtwister PRIVATE rxcore)

# Shared library with the C API for engine / GUI bindings
add_library(rxtwister_c SHARED src/c_api.cpp)
target_include_directories(rxtwister_c PRIVATE src)
target_link_libraries(rxtwister_c PRIVATE rxcore)
set_target_properties(rxtwister_c PROPERTIES OUTPUT_NAME rxtwister CXX_VISIBILITY_PRESET hidden)

if(RX_BUILD_TESTS)
  enable_testing()
  add_executable(rx_tests tests/test_main.cpp)
  target_include_directories(rx_tests PRIVATE src)
  target_link_libraries(rx_tests PRIVATE rxcore)
  target_compile_definitions(rx_tests PRIVATE ${RX_DEP_DEFS})
  add_test(NAME rx_tests COMMAND rx_tests ${CMAKE_BINARY_DIR}/test_out)
endif()

include(GNUInstallDirs)
install(TARGETS rxtwister rxtwister_c RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/rx/c_api.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rx)

message(STATUS "Rx Twister: assimp=${RX_HAVE_ASSIMP} webp=${RX_HAVE_WEBP} tbb=${RX_HAVE_TBB} draco=${RX_HAVE_DRACO} zlib=${RX_HAVE_ZLIB}")

if(RX_BUILD_TESTS)
  add_executable(rx_make_big tests/make_big.cpp)
  target_include_directories(rx_make_big PRIVATE src)
  target_link_libraries(rx_make_big PRIVATE rxcore)
endif()
