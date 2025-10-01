# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(openarm_teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Find external packages
# -----------------------------
find_package(OpenArmCAN REQUIRED)
# find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

# -----------------------------
# Create static library
# -----------------------------
add_library(
  openarm_teleop_lib STATIC
  src/controller/dynamics.cpp src/controller/control.cpp
  src/openarm_port/openarm_init.cpp src/openarm_port/joint_mapper.cpp)

target_include_directories(openarm_teleop_lib
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(
  openarm_teleop_lib
  PUBLIC OpenArmCAN::openarm_can Eigen3::Eigen ${orocos_kdl_LIBRARIES}
         ${kdl_parser_LIBRARIES} urdfdom::urdfdom_model yaml-cpp::yaml-cpp)

# -----------------------------
# Executables
# -----------------------------
add_executable(gravity_comp control/gravity_compasation.cpp)
add_executable(comm_test control/openarm_communication_test.cpp)
add_executable(unilateral_control control/openarm_unilateral_control.cpp)
add_executable(bilateral_control control/openarm_bilateral_control.cpp)

target_link_libraries(gravity_comp PRIVATE openarm_teleop_lib)
target_link_libraries(comm_test PRIVATE openarm_teleop_lib)
target_link_libraries(unilateral_control PRIVATE openarm_teleop_lib)
target_link_libraries(bilateral_control PRIVATE openarm_teleop_lib)
