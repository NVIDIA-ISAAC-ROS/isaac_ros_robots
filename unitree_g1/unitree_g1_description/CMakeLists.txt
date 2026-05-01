# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(unitree_g1_description)

# Find dependencies.
find_package(ament_cmake REQUIRED)

include(FetchContent)

# Fetch Unitree ROS files for URDF (needed for RViz visualization).
fetchcontent_declare(
  unitree_ros
  URL https://github.com/unitreerobotics/unitree_ros/archive/refs/heads/master.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

fetchcontent_makeavailable(unitree_ros)

# Download G1 MuJoCo models directly from GitHub (raw files).
# The models are in the main branch but not included in release zips yet.
set(GITHUB_RAW_BASE "https://raw.githubusercontent.com/unitreerobotics/unitree_mujoco/main/unitree_robots/g1")
set(G1_DOWNLOAD_DIR "${CMAKE_CURRENT_BINARY_DIR}/g1_download")
file(MAKE_DIRECTORY ${G1_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${G1_DOWNLOAD_DIR}/meshes)

# Download XML files.
file(DOWNLOAD
  "${GITHUB_RAW_BASE}/g1_29dof.xml"
  "${G1_DOWNLOAD_DIR}/g1_29dof.xml"
  SHOW_PROGRESS
)

file(DOWNLOAD
  "${GITHUB_RAW_BASE}/scene_29dof.xml"
  "${G1_DOWNLOAD_DIR}/scene_29dof.xml"
  SHOW_PROGRESS
)

# Download meshes (list all mesh files).
set(MESH_NAMES
  "head_link.STL"
  "left_ankle_pitch_link.STL"
  "left_ankle_roll_link.STL"
  "left_elbow_link.STL"
  "left_hand_index_0_link.STL"
  "left_hand_index_1_link.STL"
  "left_hand_middle_0_link.STL"
  "left_hand_middle_1_link.STL"
  "left_hand_palm_link.STL"
  "left_hand_thumb_0_link.STL"
  "left_hand_thumb_1_link.STL"
  "left_hand_thumb_2_link.STL"
  "left_hip_pitch_link.STL"
  "left_hip_roll_link.STL"
  "left_hip_yaw_link.STL"
  "left_knee_link.STL"
  "left_rubber_hand.STL"
  "left_shoulder_pitch_link.STL"
  "left_shoulder_roll_link.STL"
  "left_shoulder_yaw_link.STL"
  "left_wrist_pitch_link.STL"
  "left_wrist_roll_link.STL"
  "left_wrist_yaw_link.STL"
  "logo_link.STL"
  "pelvis.STL"
  "pelvis_contour_link.STL"
  "right_ankle_pitch_link.STL"
  "right_ankle_roll_link.STL"
  "right_elbow_link.STL"
  "right_hand_index_0_link.STL"
  "right_hand_index_1_link.STL"
  "right_hand_middle_0_link.STL"
  "right_hand_middle_1_link.STL"
  "right_hand_palm_link.STL"
  "right_hand_thumb_0_link.STL"
  "right_hand_thumb_1_link.STL"
  "right_hand_thumb_2_link.STL"
  "right_hip_pitch_link.STL"
  "right_hip_roll_link.STL"
  "right_hip_yaw_link.STL"
  "right_knee_link.STL"
  "right_rubber_hand.STL"
  "right_shoulder_pitch_link.STL"
  "right_shoulder_roll_link.STL"
  "right_shoulder_yaw_link.STL"
  "right_wrist_pitch_link.STL"
  "right_wrist_roll_link.STL"
  "right_wrist_yaw_link.STL"
  "torso_link.STL"
  "waist_roll_link.STL"
  "waist_support_link.STL"
  "waist_yaw_link.STL"
)

message(STATUS "Downloading G1 mesh files from GitHub...")
foreach(MESH_NAME ${MESH_NAMES})
  file(DOWNLOAD
    "${GITHUB_RAW_BASE}/meshes/${MESH_NAME}"
    "${G1_DOWNLOAD_DIR}/meshes/${MESH_NAME}"
    STATUS DOWNLOAD_STATUS
  )
  list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
  if(NOT STATUS_CODE EQUAL 0)
    message(WARNING "Failed to download ${MESH_NAME}, may not be critical")
  endif()
endforeach()

# Source paths.
set(G1_MUJOCO_DIR "${G1_DOWNLOAD_DIR}")
set(PATCH_FILE "${CMAKE_CURRENT_SOURCE_DIR}/patches/add_g1_29_dof_with_hand.patch")

# Create working directory for patched files.
set(MUJOCO_WORK_DIR "${CMAKE_CURRENT_BINARY_DIR}/mujoco_work")
file(MAKE_DIRECTORY ${MUJOCO_WORK_DIR})
file(MAKE_DIRECTORY ${MUJOCO_WORK_DIR}/unitree_robots/g1)

# Copy source files to working directory.
configure_file(
  ${G1_MUJOCO_DIR}/g1_29dof.xml
  ${MUJOCO_WORK_DIR}/unitree_robots/g1/g1_29dof.xml
  COPYONLY
)

configure_file(
  ${G1_MUJOCO_DIR}/scene_29dof.xml
  ${MUJOCO_WORK_DIR}/unitree_robots/g1/scene_29dof.xml
  COPYONLY
)

# Copy meshes.
file(MAKE_DIRECTORY ${MUJOCO_WORK_DIR}/unitree_robots/g1/meshes)
foreach(MESH_NAME ${MESH_NAMES})
  configure_file(
    ${G1_MUJOCO_DIR}/meshes/${MESH_NAME}
    ${MUJOCO_WORK_DIR}/unitree_robots/g1/meshes/${MESH_NAME}
    COPYONLY
  )
endforeach()

# Apply patch to create with_hand versions.
# Use --forward to skip already-applied hunks, and -r /dev/null to avoid .rej files.
message(STATUS "Applying patch to create G1 with hand models...")
execute_process(
  COMMAND patch --forward --reject-file=/dev/null -p0 -i ${PATCH_FILE}
  WORKING_DIRECTORY ${MUJOCO_WORK_DIR}
  RESULT_VARIABLE PATCH_RESULT
  OUTPUT_VARIABLE PATCH_OUTPUT
  ERROR_VARIABLE PATCH_ERROR
  OUTPUT_QUIET
  ERROR_QUIET
)

# Check if patched files exist (patch may return error even if successful).
if(EXISTS "${MUJOCO_WORK_DIR}/unitree_robots/g1/g1_29dof_with_hand.xml" AND
  EXISTS "${MUJOCO_WORK_DIR}/unitree_robots/g1/scene_29dof_with_hand.xml")
  message(STATUS "Patch applied successfully (or already applied)")
else()
  message(FATAL_ERROR "Failed to apply patch: patched files do not exist\n${PATCH_ERROR}")
endif()

# Create install directories.
set(INSTALL_MJCF_DIR "${CMAKE_CURRENT_BINARY_DIR}/install_mjcf")
file(MAKE_DIRECTORY ${INSTALL_MJCF_DIR})
file(MAKE_DIRECTORY ${INSTALL_MJCF_DIR}/meshes)

# Copy all MJCF files to install directory.
# Base model.
configure_file(
  ${MUJOCO_WORK_DIR}/unitree_robots/g1/g1_29dof.xml
  ${INSTALL_MJCF_DIR}/g1_29dof.xml
  COPYONLY
)

# Base scene.
configure_file(
  ${MUJOCO_WORK_DIR}/unitree_robots/g1/scene_29dof.xml
  ${INSTALL_MJCF_DIR}/scene_29dof.xml
  COPYONLY
)

# With hand model (created by patch).
configure_file(
  ${MUJOCO_WORK_DIR}/unitree_robots/g1/g1_29dof_with_hand.xml
  ${INSTALL_MJCF_DIR}/g1_29dof_with_hand.xml
  COPYONLY
)

# With hand scene (created by patch).
configure_file(
  ${MUJOCO_WORK_DIR}/unitree_robots/g1/scene_29dof_with_hand.xml
  ${INSTALL_MJCF_DIR}/scene_29dof_with_hand.xml
  COPYONLY
)

# Copy meshes to install directory.
foreach(MESH_NAME ${MESH_NAMES})
  configure_file(
    ${MUJOCO_WORK_DIR}/unitree_robots/g1/meshes/${MESH_NAME}
    ${INSTALL_MJCF_DIR}/meshes/${MESH_NAME}
    COPYONLY
  )
endforeach()

# Also create meshes directory for RViz/URDF (separate from MJCF).
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/meshes)
foreach(MESH_NAME ${MESH_NAMES})
  configure_file(
    ${MUJOCO_WORK_DIR}/unitree_robots/g1/meshes/${MESH_NAME}
    ${CMAKE_CURRENT_BINARY_DIR}/meshes/${MESH_NAME}
    COPYONLY
  )
endforeach()

# Process URDF from unitree_ros for RViz visualization.
set(G1_ROS_DIR "${unitree_ros_SOURCE_DIR}/robots/g1_description")
set(G1_URDF_FILE "g1_29dof_with_hand_rev_1_0.urdf")

# Read and fix mesh paths in URDF.
file(READ ${G1_ROS_DIR}/${G1_URDF_FILE} OFFICIAL_URDF_CONTENT)

# Fix mesh paths to use package:// URLs.
string(REPLACE
  "filename=\"meshes/"
  "filename=\"package://unitree_g1_description/meshes/"
  OFFICIAL_URDF_CONTENT "${OFFICIAL_URDF_CONTENT}")

# Remove _rev_1_0 suffix from mesh filenames to match unitree_mujoco mesh names.
string(REGEX REPLACE
  "_rev_1_0\\.STL"
  ".STL"
  OFFICIAL_URDF_CONTENT "${OFFICIAL_URDF_CONTENT}")

# Write fixed URDF.
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/urdf)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/urdf/g1_29dof_with_hand_rev_1_0_fixed.urdf
  "${OFFICIAL_URDF_CONTENT}")

# Install custom URDF xacro files from source.
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install fixed URDF from build directory.
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/urdf/g1_29dof_with_hand_rev_1_0_fixed.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install patched MJCF files and meshes.
install(DIRECTORY ${INSTALL_MJCF_DIR}/
  DESTINATION share/${PROJECT_NAME}/mjcf
)

# Install meshes for RViz/URDF.
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install patch file for reference.
install(FILES patches/add_g1_29_dof_with_hand.patch
  DESTINATION share/${PROJECT_NAME}/patches
)

ament_package()
