# Unitree G1 Description Package

This package provides URDF and MuJoCo MJCF description files for the Unitree G1 humanoid robot.

## Overview

The package automatically downloads the official Unitree G1 MuJoCo models from the [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) repository and applies patches to add:
- Dexterous hand models (finger joints)
- RealSense D435 camera setup
- Updated sensor configurations for compatibility

## Build Process

During the CMake configure step, the package:

1. **Downloads** the following from `unitree_mujoco`:
   - `g1_29dof.xml` - Base robot model
   - `scene_29dof.xml` - Base scene configuration
   - `meshes/*.STL` - All mesh files

2. **Applies patch** `patches/add_g1_29_dof_with_hand.patch` to create:
   - `g1_29dof_with_hand.xml` - Robot with dexterous hands
   - `scene_29dof_with_hand.xml` - Scene with hand-enabled robot

3. **Installs** all files to `share/unitree_g1_description/mjcf/`

## Available Models

After installation, the following MuJoCo models are available:

**Robot-only models:**
- `g1_29dof.xml` - Base 29-DOF model without hands
- `g1_29dof_with_hand.xml` - Full model with dexterous hands (43 DOF)

**Scene models (recommended - includes robot + environment):**
- `scene_29dof.xml` - Scene with base model (ground, lighting, etc.)
- `scene_29dof_with_hand.xml` - Scene with full model (ground, lighting, etc.)

**Meshes:**
- `meshes/` - All STL mesh files (51 files)

## Usage

### With ros2_control

The xacro files in `urdf/` integrate with ros2_control.

**Recommended: Use scene file for complete environment:**

```xml
<xacro:include filename="$(find unitree_g1_description)/urdf/g1_ros2_control.urdf.xacro"/>
<xacro:g1_ros2_control
  mujoco_model_path="$(find unitree_g1_description)/mjcf/scene_29dof_with_hand.xml"
  enable_viewer="true"/>
```

### In Launch Files

**Recommended: Use scene file:**

```python
from ament_index_python.packages import get_package_share_directory

description_pkg = get_package_share_directory("unitree_g1_description")
# Use scene file (includes robot + ground + lighting)
mujoco_model_path = os.path.join(description_pkg, "mjcf", "scene_29dof_with_hand.xml")
```

**Alternative: Robot-only (no environment):**

```python
# Use robot-only file (no ground plane or lighting)
mujoco_model_path = os.path.join(description_pkg, "mjcf", "g1_29dof_with_hand.xml")
```

## Modifications from Upstream

The patch (`patches/add_g1_29_dof_with_hand.patch`) adds:

### Hand Joints (per hand)
- 3x thumb joints (thumb_0, thumb_1, thumb_2)
- 2x index finger joints (index_0, index_1)
- 2x middle finger joints (middle_0, middle_1)

### Camera Setup
- RealSense D435 stereo camera
  - Left camera (depth)
  - Right camera (depth)
  - RGB camera
  - Mounted on torso

### Sensor Updates
- Renamed IMU sensors for compatibility with ros2_control
- Added velocity and acceleration sensors

## Synchronization with Bazel

This ROS/CMake setup mirrors the Bazel configuration in `third_party/unitree_mujoco/`, ensuring:
- Same upstream source (unitree_mujoco GitHub)
- Same patch file
- Consistent model behavior across Holoscan and ROS2 builds

## Dependencies

- `ament_cmake` - Build system
- `xacro` - URDF processing
- `patch` - Apply model modifications (system utility)

## Files

```
unitree_g1_description/
├── CMakeLists.txt              # Build configuration
├── package.xml                  # ROS package manifest
├── README.md                    # This file
├── patches/
│   └── add_g1_29_dof_with_hand.patch  # Model modifications
└── urdf/
    ├── g1_ros2_control.urdf.xacro           # ros2_control integration
    └── g1_with_ros2_control_full.urdf.xacro # Full robot description
```

## License

Apache-2.0
