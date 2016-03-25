cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

file (GLOB SRC_FILES "${PROJECT_SOURCE_DIR}/src/*.cc")
file (GLOB INC_FILES "${PROJECT_SOURCE_DIR}/include/*.h")

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${INC_FILES})

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX ".d")

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})
set_target_properties(rgbd_tum PROPERTIES DEBUG_POSTFIX ".d")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})
set_target_properties(stereo_kitti PROPERTIES DEBUG_POSTFIX ".d")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})
set_target_properties(mono_tum PROPERTIES DEBUG_POSTFIX ".d")

add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})
set_target_properties(mono_kitti PROPERTIES DEBUG_POSTFIX ".d")

add_executable(mono_video Examples/Monocular/mono_video.cc)
target_link_libraries(mono_video ${PROJECT_NAME})
set_target_properties(mono_video PROPERTIES DEBUG_POSTFIX ".d")

add_executable(dbow2_demo Examples/DBoW2_example/demo.cc)
target_link_libraries(dbow2_demo ${PROJECT_NAME})
set_target_properties(dbow2_demo PROPERTIES DEBUG_POSTFIX ".d")

add_executable(dbow2_txt2yaml Examples/DBoW2_example/txt2yaml.cc)
target_link_libraries(dbow2_txt2yaml ${PROJECT_NAME})
set_target_properties(dbow2_txt2yaml PROPERTIES DEBUG_POSTFIX ".d")

add_executable(map_simple_example Examples/Map/simple_example.cc)
target_link_libraries(map_simple_example ${PROJECT_NAME})
set_target_properties(map_simple_example PROPERTIES DEBUG_POSTFIX ".d")

