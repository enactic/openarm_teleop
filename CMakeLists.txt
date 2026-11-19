# Copyright 2025-2026 Enactic, Inc.
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

include(CTest)
include(GNUInstallDirs)

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

if(NOT TARGET kdl_parser::kdl_parser)
  # For old kdl-parser such as libkdl-parser-dev 1.14.2 in Debian.
  add_library(kdl_parser::kdl_parser INTERFACE IMPORTED)
  target_link_libraries(kdl_parser::kdl_parser
                        INTERFACE ${kdl_parser_LIBRARIES})
  target_include_directories(kdl_parser::kdl_parser
                             INTERFACE ${kdl_parser_INCLUDE_DIRS})
endif()
target_link_libraries(
  openarm_teleop_lib
  PUBLIC OpenArmCAN::openarm_can Eigen3::Eigen ${orocos_kdl_LIBRARIES}
         kdl_parser::kdl_parser urdfdom::urdfdom_model yaml-cpp::yaml-cpp)

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

# -----------------------------
# Install
# -----------------------------
# These executables aren't for users. They are run by a launcher.
install(TARGETS gravity_comp unilateral_control bilateral_control
        DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/openarm-teleop)
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR}/openarm-teleop)

# Generate URDFs by openarm_description. They need a ROS 2 environment.
find_package(openarm_description REQUIRED)
find_program(XACRO xacro REQUIRED)
set(OPENARM_XACRO ${openarm_description_DIR}/../urdf/openarm.urdf.xacro)
set(OPENARM_URDFS)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/urdf)
# OpenArm 2.0 and OpenArm 1.0
foreach(MAJOR 2 1)
  set(OPENARM_URDF ${CMAKE_CURRENT_BINARY_DIR}/urdf/openarm-${MAJOR}.0.urdf)
  add_custom_command(
    OUTPUT ${OPENARM_URDF}
    COMMAND ${XACRO} ${OPENARM_XACRO} robot_preset:=v${MAJOR}/default_bimanual
            -o ${OPENARM_URDF}
    DEPENDS ${OPENARM_XACRO}
    VERBATIM)
  list(APPEND OPENARM_URDFS ${OPENARM_URDF})
endforeach()
add_custom_target(urdf ALL DEPENDS ${OPENARM_URDFS})
install(FILES ${OPENARM_URDFS}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/openarm-teleop/urdf)

# openarm-teleop finds the above files by relative paths from itself.
file(RELATIVE_PATH OPENARM_TELEOP_RELATIVE_LIBEXECDIR
     ${CMAKE_INSTALL_FULL_BINDIR}
     ${CMAKE_INSTALL_FULL_LIBEXECDIR}/openarm-teleop)
file(RELATIVE_PATH OPENARM_TELEOP_RELATIVE_DATADIR ${CMAKE_INSTALL_FULL_BINDIR}
     ${CMAKE_INSTALL_FULL_DATADIR}/openarm-teleop)
configure_file(bin/openarm-teleop.in openarm-teleop @ONLY)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/openarm-teleop
        DESTINATION ${CMAKE_INSTALL_BINDIR})

# -----------------------------
# Tests
# -----------------------------
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
