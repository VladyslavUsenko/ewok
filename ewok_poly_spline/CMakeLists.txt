cmake_minimum_required(VERSION 3.16)
project(ewok_poly_spline)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

catkin_simple()


catkin_add_gtest(test_polynomial test/polynomial-test.cpp)
catkin_add_gtest(test_uniform_bspline test/uniform-bspline-test.cpp)

cs_install()
cs_export()
