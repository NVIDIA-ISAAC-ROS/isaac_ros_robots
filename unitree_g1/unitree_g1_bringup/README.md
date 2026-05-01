# Unitree G1 Bringup

Launch files and configurations for bringing up the Unitree G1 robot in MuJoCo or on real hardware.

## Contents

- **launch/**: Launch files
  - `g1_mujoco.launch.py`: Launch G1 robot with mock hardware (for development/testing)

- **config/**: Configuration files
  - `controller_manager.yaml`: ros2_control controller manager configuration
  - `g1.rviz`: RViz configuration for visualization

## Usage

### Launch with Mock Hardware

```bash
ros2 launch unitree_g1_bringup g1_mujoco.launch.py
```

Options:
- `use_rviz:=true/false` - Enable/disable RViz visualization (default: true)

### Expected Topics

When running, the following topics will be available:
- `/joint_states` - All 29 joint positions, velocities, and efforts (200 Hz)
- `/imu_sensor_broadcaster/imu` - IMU sensor data (200 Hz)
- `/tf` and `/tf_static` - Robot transforms

## Dependencies

- `unitree_g1_description` - Robot description files
- `controller_manager` - ros2_control controller manager
- `joint_state_broadcaster` - Joint state publisher
- `imu_sensor_broadcaster` - IMU sensor publisher
- `robot_state_publisher` - Robot state publisher

