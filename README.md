# BRACU Hydra Robot - Pick and Place System

A complete ROS2 pick-and-place system integrating a 4-DOF robot arm with Intel RealSense D435i camera, YOLOv8 object detection, and ESP32-based hardware control.

![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Python 3.12](https://img.shields.io/badge/Python-3.12-green)
![ESP32](https://img.shields.io/badge/ESP32-S3-red)

## 🎯 Features

- **Vision System**: Intel RealSense D435i RGB-D camera integration
- **Object Detection**: Custom YOLOv8 model for real-time object detection
- **Motion Planning**: MoveIt2 with custom IK solver for 4-DOF robot
- **TF Integration**: Automatic 3D object localization in robot frame
- **Pick & Place**: Automated object picking with collision avoidance
- **Hardware Control**: ESP32-S3 firmware for 4 arm joints + 2 gripper servos
- **Teleoperation**: Multiple teleop modes (serial, gamepad, smooth control)
- **Real-time Control**: Butter-smooth motion with input filtering

## 📋 Table of Contents

- [System Requirements](#system-requirements)
- [Hardware Setup](#hardware-setup)
- [Installation](#installation)
- [Configuration](#configuration)
- [Running the System](#running-the-system)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)
- [Development](#development)

## 💻 System Requirements

### Software
- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy Jalisco
- **Python**: 3.12+
- **CUDA**: 11.8+ (optional, for GPU acceleration)

### Hardware
- **Robot**: 4-DOF articulated arm (BRACU Hydra) + 2 gripper servos
- **Controller**: ESP32-S3 microcontroller
- **Camera**: Intel RealSense D435i
- **Computer**: 8GB RAM minimum, 16GB recommended
- **USB**: USB 3.0 ports for camera
- **Gamepad**: Xbox/PS4 controller (optional, for teleoperation)

## 🔧 Hardware Setup

### Camera Mounting
1. Mount RealSense D435i with clear view of workspace
2. Connect via USB 3.0 cable
3. Recommended mounting height: 0.5-1.0m above workspace
4. Angle camera downward toward pick area

### Robot Connection
1. Connect robot controller to computer
2. Verify serial/USB connection
3. Test joint control: `ros2 topic echo /joint_states`

### ESP32-S3 Firmware

The robot uses an ESP32-S3 for servo control with simple serial protocol:

**Servo Configuration:**
| Joint | GPIO | Range | Home |
|-------|------|-------|------|
| Base | 4 | 80-130° | 98° |
| Joint 1 | 5 | 20-85° | 85° |
| Joint 2 | 6 | 60-85° | 85° |
| Joint 3 | 7 | 0-180° | 90° |
| Gripper 1 | 15 | 0-60° | 0° |
| Gripper 2 | 16 | 0-100° | 0° |

**Protocol:**
```
Command: "J <base> <j1> <j2> <j3> <grip1> <grip2>\n" (degrees)
Response: "S <base> <j1> <j2> <j3> <grip1> <grip2>\n" (current positions)
```

**Flash firmware:**
```bash
cd esp32_firmware
pio run --target upload
```

## 🚀 Installation

### 1. Install ROS2 Jazzy

```bash
# Add ROS2 repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop-full
```

### 2. Install Dependencies

```bash
# ROS2 packages
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-visual-tools \
  ros-jazzy-rviz-visual-tools \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-tf2-tools \
  python3-colcon-common-extensions

# RealSense SDK
sudo apt install -y \
  ros-jazzy-realsense2-camera \
  ros-jazzy-realsense2-description \
  librealsense2-dkms \
  librealsense2-utils

# Python dependencies
pip3 install ultralytics opencv-python scipy numpy
```

### 3. Clone and Build

```bash
# Create workspace
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src

# Clone repository
git clone https://github.com/tushhh00/haydra_bracu.git
cd ..

# Build workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### 4. Camera Verification

```bash
# Test RealSense camera
realsense-viewer

# Check camera devices
rs-enumerate-devices

# Verify /dev/video* devices
ls /dev/video*
```

## ⚙️ Configuration

### 1. Camera Calibration

Edit [src/robot_moveit_config/launch/pick_place_system.launch.py](src/robot_moveit_config/launch/pick_place_system.launch.py):

```python
# Adjust camera TF based on your mounting
'camera_x': '0.0',      # X offset from base_link
'camera_y': '0.35',     # Y offset (distance in front)
'camera_z': '0.8',      # Z offset (height)
'camera_roll': '0.0',
'camera_pitch': '0.785',  # 45 degrees downward
'camera_yaw': '0.0',
```

### 2. YOLO Model

Place your trained YOLO model:
```bash
# Default path
src/yolo/best.pt
```

Update path in launch file if different:
```python
'model_path': '/path/to/your/model.pt'
```

### 3. Robot Kinematics

Verify joint limits in [src/robot_description/urdf/robot.urdf.xacro](src/robot_description/urdf/robot.urdf.xacro):
- Check joint ranges match your hardware
- Verify link lengths
- Confirm joint directions

### 4. IK Solver Tuning

Edit [src/robot_moveit_config/scripts/robot_ik_solver.py](src/robot_moveit_config/scripts/robot_ik_solver.py):

```python
# Error tolerance (currently 2cm)
best_err < 0.02  # Decrease for precision, increase for reachability

# Optimization iterations
'maxiter': 1000  # Increase if solutions fail
```

## 🎮 Running the System

### Complete System Launch

```bash
# Terminal 1: Source and launch complete system
cd ~/ws_moveit
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_moveit_config pick_place_system.launch.py publish_static_camera_tf:=true
```

**This launches:**
- Robot hardware interface
- MoveIt2 motion planning
- RealSense camera driver
- YOLO object detection
- TF broadcaster (camera → base_link)

### Manual Pick Execution

```bash
# Terminal 2: Run pick script
cd ~/ws_moveit
source install/setup.bash

# Option 1: Using custom IK solver
python3 src/robot_moveit_config/scripts/pick_with_ik.py

# Option 2: Using joint-space control (more reliable)
python3 src/robot_moveit_config/scripts/pick_detected_object.py

# Option 3: With specific coordinates
python3 src/robot_moveit_config/scripts/pick_with_ik.py -x 0.3 -y 0.4 -z 0.5

# Option 4: Predefined positions
python3 src/robot_moveit_config/scripts/pick_with_ik.py --home
python3 src/robot_moveit_config/scripts/pick_with_ik.py --ready
```

### 🎮 Teleoperation Modes

#### Serial Teleop (Real Hardware)
Direct serial control for ESP32-connected robot with gamepad:

```bash
# Connect gamepad and ESP32, then run:
ros2 run robot_moveit_config serial_teleop.py
```

**Controls (Xbox):**
- Left Stick X/Y: Base / Joint 1
- Right Stick X/Y: Joint 3 / Joint 2
- LB / RB: Gripper 1 Close/Open
- LT / RT: Gripper 2 Close/Open
- A: Enable | B: E-Stop | Start: Home

#### Smooth Gamepad Teleop (Simulation)
Butter-smooth motion control for MoveIt simulation:

```bash
ros2 run robot_moveit_config smooth_teleop.py
```

#### MoveIt Servo Teleop
Real-time Cartesian control with MoveIt Servo:

```bash
ros2 launch robot_moveit_config teleop_gamepad.launch.py
```

### Testing Components

#### Camera Only
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  enable_color:=true \
  pointcloud.enable:=true
```

#### YOLO Detection Only
```bash
cd ~/ws_moveit/src/yolo
python3 yolo_detector.py
```

#### View TF Frames
```bash
# List all transforms
ros2 run tf2_ros tf2_echo base_link camera_link

# Monitor detected object
ros2 run tf2_ros tf2_echo base_link detected_object

# Visualize TF tree
ros2 run tf2_tools view_frames
evince frames_*.pdf
```

#### Test IK Solver
```bash
cd ~/ws_moveit/src/robot_moveit_config/scripts
python3 robot_ik_solver.py

# Enter positions interactively:
# Pos: 0.0 0.4 0.6
# Pos: 0.2 0.3 0.5
```

## 🐛 Troubleshooting

### Camera Issues

#### "No devices detected"
```bash
# Check USB connection
lsusb | grep Intel

# Restart udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Physical reset: Unplug and replug USB cable
```

#### "Device or resource busy"
```bash
# Kill existing processes
pkill -9 realsense
pkill -9 rs-enumerate-devices

# Reset USB device
sudo rmmod uvcvideo
sudo modprobe uvcvideo

# Physical reset required in some cases
```

### Motion Planning Issues

#### "Unable to sample valid states for goal tree"
**Cause**: 4-DOF robot cannot reach 6-DOF poses

**Solution**: Use joint-space control instead:
```bash
python3 src/robot_moveit_config/scripts/pick_detected_object.py
```

#### "IK returned UNREACHABLE"
**Causes**:
1. Position outside robot workspace
2. Joint limits preventing reach
3. IK solver tolerance too strict

**Solutions**:
```bash
# Test reachability
python3 src/robot_moveit_config/scripts/robot_ik_solver.py

# Adjust IK tolerance in robot_ik_solver.py:
best_err < 0.03  # Increase from 0.02 to 0.03

# Add more initial guesses for optimization
```

#### "MoveItPy not initialized"
**Cause**: MoveItPy conflicts with action-based planning

**Solution**: Use `pick_detected_object.py` which uses MoveGroup action directly.

### Detection Issues

#### "No detections" / Low confidence
```bash
# Check YOLO model
ls -lh src/yolo/best.pt

# Test YOLO independently
cd src/yolo
python3 yolo_detector.py

# Verify camera feed
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
```

#### "TF transform not found"
```bash
# Verify TF is publishing
ros2 topic echo /tf_static | grep camera

# Check TF tree
ros2 run tf2_tools view_frames

# Restart with static TF enabled
ros2 launch robot_moveit_config pick_place_system.launch.py publish_static_camera_tf:=true
```

### Build Errors

```bash
# Clean and rebuild
cd ~/ws_moveit
rm -rf build install log
colcon build --symlink-install

# If Python path issues
export PYTHONPATH=$PYTHONPATH:$(pwd)/install/lib/python3.12/site-packages
```

## 📁 Project Structure

```
haydra_bracu/
├── esp32_firmware/                  # ESP32-S3 servo controller
│   └── src/
│       └── main.cpp                # Firmware with serial protocol
│
├── src/
│   ├── robot_description/          # Robot URDF/xacro
│   │   ├── urdf/
│   │   │   ├── robot.urdf.xacro   # Main robot description
│   │   │   └── robot.ros2_control.xacro
│   │   └── meshes/                 # Visual/collision meshes
│   │
│   ├── robot_moveit_config/        # MoveIt configuration
│   │   ├── config/
│   │   │   ├── kinematics.yaml    # IK solver config
│   │   │   ├── joint_limits.yaml
│   │   │   ├── servo_config.yaml  # MoveIt Servo settings
│   │   │   └── moveit_controllers.yaml
│   │   ├── launch/
│   │   │   ├── pick_place_system.launch.py  # Main launch file
│   │   │   ├── teleop_gamepad.launch.py     # Gamepad teleop
│   │   │   └── moveit.launch.py
│   │   └── scripts/
│   │       ├── serial_teleop.py             # ESP32 serial control
│   │       ├── smooth_teleop.py             # Smooth gamepad control
│   │       ├── gamepad_teleop.py            # MoveIt Servo teleop
│   │       ├── real_hardware_teleop.py      # micro-ROS teleop
│   │       ├── pick_detected_object.py      # Joint-space picker
│   │       ├── pick_with_ik.py              # IK-based picker
│   │       └── robot_ik_solver.py           # Custom IK solver
│   │
│   └── yolo/                        # Object detection
│       ├── yolo_detector.py        # ROS2 YOLO node
│       └── best.pt                 # Trained model
│
├── launch_moveit_demo.sh           # Quick launch script
├── launch_system.sh                # Full system launch
├── launch_real_hardware.sh         # Real robot launch
└── README.md                       # This file
```

## 🔍 Key Files Explained

### [esp32_firmware/src/main.cpp](esp32_firmware/src/main.cpp)
ESP32-S3 firmware for servo control:
- Controls 4 arm joints + 2 gripper servos
- Simple serial protocol (no micro-ROS needed)
- Smooth motion with acceleration limiting
- Auto-home on startup

### [serial_teleop.py](src/robot_moveit_config/scripts/serial_teleop.py)
Bridges ROS2 and ESP32 hardware:
- Gamepad input to servo commands
- Coordinate space conversion (URDF ↔ Servo)
- Real-time joint state publishing
- Gripper control with LB/RB and LT/RT

### [pick_place_system.launch.py](src/robot_moveit_config/launch/pick_place_system.launch.py)
Main launch file combining all components:
- Robot hardware interface
- MoveIt2 motion planning
- RealSense camera
- YOLO detection
- Static TF publisher

### [robot_ik_solver.py](src/robot_moveit_config/scripts/robot_ik_solver.py)
Custom numerical IK solver for 4-DOF robot:
- Forward kinematics
- Inverse kinematics using scipy optimization
- Workspace analysis
- Interactive testing mode

### [pick_detected_object.py](src/robot_moveit_config/scripts/pick_detected_object.py)
Joint-space motion control (most reliable):
- Uses MoveIt2 planning service
- Direct joint goals
- Synchronous execution

### [pick_with_ik.py](src/robot_moveit_config/scripts/pick_with_ik.py)
Cartesian motion control:
- Uses custom IK solver
- Position-based goals
- Command-line arguments for coordinates
- Predefined positions (home, ready)

## 🛠️ Development

### Adding New Detection Classes

1. Train YOLO model with new classes
2. Update model: `src/yolo/best.pt`
3. Restart YOLO node

### Adjusting Robot Workspace

1. Edit joint limits: `src/robot_description/urdf/robot.urdf.xacro`
2. Update IK limits: `src/robot_moveit_config/scripts/robot_ik_solver.py`
3. Rebuild: `colcon build --packages-select robot_description robot_moveit_config`

### Camera Position Changes

1. Update launch file camera TF parameters
2. Or use dynamic TF: Remove `publish_static_camera_tf:=true`
3. Calibrate using ArUco markers or checkerboard

### Custom Pick Strategies

Create new script in `src/robot_moveit_config/scripts/`:
```python
#!/usr/bin/env python3
import rclpy
from your_custom_logic import PickStrategy

# Implement custom behavior
```

## 📊 Robot Workspace

The 4-DOF robot has a limited workspace:

- **Height (Z)**: 0.07m to 0.98m
- **Horizontal reach**: ~0.53m radius
- **Arm orientation**: Faces +Y direction when joint_1=0

**Reachable zones:**
- Front center (0.0, 0.4, 0.6): ✓
- Extended front (0.0, 0.45, 0.65): ✓
- Sides (±0.2, 0.3, 0.5): ✓ (may need base rotation)

**Unreachable zones:**
- Too low (z < 0.3m): Limited
- Behind robot (y < 0): Not reachable
- Far sides (|x| > 0.3m): Limited

## 📝 ROS2 Topics

```bash
# Camera
/camera/camera/color/image_raw          # RGB image
/camera/camera/depth/image_rect_raw     # Depth image
/camera/camera/depth/color/points       # Point cloud

# Detection
/yolo/detections                        # Detection array
/detected_object/pose                   # Object pose in camera frame

# Robot
/joint_states                           # Current joint positions
/arm_controller/follow_joint_trajectory # Joint trajectory action

# TF
/tf                                     # Dynamic transforms
/tf_static                              # Static transforms (camera)
```

## 🎓 Usage Tips

1. **Always verify camera connection** before launching
2. **Test IK reachability** before pick attempts
3. **Use joint-space control** for more reliable motion
4. **Calibrate camera TF** for accurate 3D localization
5. **Monitor TF transforms** to debug positioning issues

## 📚 References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)
- [MoveIt2 Tutorials](https://moveit.picknik.ai/main/index.html)
- [RealSense ROS2 Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/)

## 🤝 Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature-name`
3. Commit changes: `git commit -am 'Add feature'`
4. Push to branch: `git push origin feature-name`
5. Submit pull request

## 📄 License

This project is licensed under the MIT License.

## 👥 Authors

- BRACU Robotics Team

## 🙏 Acknowledgments

- MoveIt2 community
- Intel RealSense team
- Ultralytics YOLO team

---

**Last Updated**: January 2026  
**ROS2 Version**: Jazzy Jalisco  
**Status**: Active Development
