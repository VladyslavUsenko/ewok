cmake_minimum_required(VERSION 2.8.3)
project(ewok_ring_buffer)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_definitions(-std=c++11 -g -O3 -ftree-vectorize -march=native)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR} ${OCTOMAP_INCLUDE_DIRS})

catkin_simple()

cs_add_executable(ring_buffer_example src/ring_buffer_example.cpp)
cs_add_executable(tum_rgbd_ring_buffer_example src/tum_rgbd_ring_buffer_example.cpp)
target_link_libraries(tum_rgbd_ring_buffer_example ${OCTOMAP_LIBRARIES})

catkin_add_gtest(test_ring_buffer_base test/ring-buffer-base-test.cpp)

cs_install()
cs_export()
