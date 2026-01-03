# ESP32-S3 + micro-ROS Setup Guide for 4-DOF Robotic Arm

## Hardware Requirements

- ESP32-S3 DevKitC-1 (or similar)
- 4x Servo motors (or stepper motors with drivers)
- Power supply for motors (separate from ESP32)
- USB cable for programming and communication

## Wiring (Default - Adjust in firmware)

### Servo Mode:
| Joint | GPIO Pin | Description |
|-------|----------|-------------|
| Joint 1 | GPIO 4 | Base rotation |
| Joint 2 | GPIO 5 | Shoulder |
| Joint 3 | GPIO 6 | Elbow |
| Joint 4 | GPIO 7 | Wrist |

### Stepper Mode:
| Joint | STEP Pin | DIR Pin |
|-------|----------|---------|
| Joint 1 | GPIO 4 | GPIO 15 |
| Joint 2 | GPIO 5 | GPIO 16 |
| Joint 3 | GPIO 6 | GPIO 17 |
| Joint 4 | GPIO 7 | GPIO 18 |

## Software Setup

### 1. Install PlatformIO

```bash
# Install PlatformIO CLI
pip install platformio

# Or use VS Code extension
code --install-extension platformio.platformio-ide
```

### 2. Build and Flash ESP32 Firmware

```bash
cd /home/peru0002/hydra_ws/src/haydra_bracu/esp32_firmware

# Build
pio run

# Upload (connect ESP32 via USB)
pio run -t upload

# Monitor serial output (optional)
pio device monitor
```

### 3. Install micro-ROS Agent on PC

```bash
# Option A: From apt (if available)
sudo apt install ros-jazzy-micro-ros-agent

# Option B: Build from source
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
cd ~/microros_ws
colcon build
source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash
```

### 4. Set Serial Port Permissions

```bash
# Add user to dialout group (permanent)
sudo usermod -a -G dialout $USER

# Log out and back in, or run:
newgrp dialout

# Quick fix (temporary)
sudo chmod 666 /dev/ttyUSB0
```

## Running the System

### Option A: Using Launch Script

```bash
cd /home/peru0002/hydra_ws
chmod +x launch_real_hardware.sh
./launch_real_hardware.sh
```

### Option B: Manual Steps

```bash
# Terminal 1: Start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Terminal 2: Start gamepad
ros2 run joy joy_node

# Terminal 3: Start teleop
cd /home/peru0002/hydra_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/haydra_bracu/src/robot_moveit_config/scripts/real_hardware_teleop.py
```

## Gamepad Controls

| Button/Stick | Action |
|--------------|--------|
| Left Stick X | Joint 1 (base rotation) |
| Left Stick Y | Joint 2 (shoulder) |
| Right Stick Y | Joint 3 (elbow) |
| Right Stick X | Joint 4 (wrist) |
| LT | Slow speed |
| RT | Fast speed |
| A | Enable control |
| B | Emergency stop |
| Start | Go to home position |
| Back | Calibrate (set as zero) |

## Troubleshooting

### ESP32 Not Detected
```bash
# Check USB connection
ls -la /dev/ttyUSB* /dev/ttyACM*

# Check dmesg for USB events
dmesg | tail -20
```

### micro-ROS Agent Not Connecting
```bash
# Check if ESP32 is publishing
ros2 topic list
# Should see /joint_states if connected

# Check agent output for errors
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6
```

### Servo Jitter
- Add capacitors (100µF) near servo power pins
- Use separate power supply for servos
- Reduce `max_velocity` in teleop parameters

### Stepper Missing Steps
- Reduce `max_velocity`
- Check motor current settings on driver
- Verify `GEAR_RATIO` in firmware matches your setup

## Firmware Configuration

Edit `arm_controller.ino`:

```cpp
// Choose motor type
#define USE_SERVO true  // false for steppers

// Adjust pins to your wiring
const int SERVO_PINS[NUM_JOINTS] = {4, 5, 6, 7};

// Servo calibration (microseconds)
const int SERVO_MIN_US = 500;   // Adjust for your servos
const int SERVO_MAX_US = 2500;

// Stepper settings (if using steppers)
const int STEPS_PER_REV = 200;
const int MICROSTEPS = 16;
const float GEAR_RATIO[NUM_JOINTS] = {5.0, 5.0, 3.0, 1.0};

// Joint limits (radians) - match your robot URDF
const float JOINT_MIN[NUM_JOINTS] = {-1.745, -0.873, -0.873, -1.571};
const float JOINT_MAX[NUM_JOINTS] = {1.745, 1.745, 1.745, 1.571};
```

## WiFi Mode (Alternative to USB)

Uncomment in firmware setup():
```cpp
set_microros_wifi_transports("YOUR_WIFI_SSID", "YOUR_PASSWORD", "PC_IP_ADDRESS", 8888);
```

Run agent in UDP mode:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
