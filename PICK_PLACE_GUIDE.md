# Pick and Place System - Step by Step Guide

This guide explains how to use the YOLO + RealSense + MoveIt pick and place system.

---

## Step 1: Measure Your Camera Position

Before running, you need to know where your camera is relative to the robot base.

**Measure these values (in meters and radians):**

| Parameter | Description | How to Measure |
|-----------|-------------|----------------|
| `camera_x` | Distance in front of robot base | Use a ruler/tape measure |
| `camera_y` | Distance to the left (positive) or right (negative) | Use a ruler/tape measure |
| `camera_z` | Height above robot base | Use a ruler/tape measure |
| `camera_pitch` | Tilt angle (radians) | If camera points down 30°, use 0.52 rad |
| `camera_roll` | Roll angle (usually 0) | Usually 0 |
| `camera_yaw` | Rotation angle (radians) | If camera faces robot, use 3.14 rad |

**Common setups:**
- Camera 50cm in front, 80cm high, tilted down 30°:
  ```
  camera_x:=0.5 camera_y:=0.0 camera_z:=0.8 camera_pitch:=0.52
  ```

---

## Step 2: Connect Hardware

1. **Plug in RealSense camera** to USB 3.0 port (blue port)
2. **Power on the robot** (or use simulation)
3. **Verify camera is detected:**
   ```bash
   ls /dev/video*
   ```
   You should see multiple video devices.

---

## Step 3: Launch the System

Open a terminal and run:

```bash
cd ~/ws_moveit

# Launch with your measured camera values
./launch_moveit_demo.sh \
    publish_static_camera_tf:=true \
    camera_x:=0.5 \
    camera_y:=0.0 \
    camera_z:=0.8 \
    camera_pitch:=0.52
```

**Wait for these messages:**
- ✅ `RealSense Node Is Up!`
- ✅ `3D Object Detector initialized!`
- ✅ `Camera intrinsics received`
- ✅ `You can start planning now!`

---

## Step 4: Verify Detection is Working

Open a **new terminal** and check detections:

```bash
cd ~/ws_moveit
source install/setup.bash

# See detection poses (in robot base frame)
ros2 topic echo /detected_object/pose
```

You should see output like:
```yaml
header:
  frame_id: "base_link"
pose:
  position:
    x: 0.738
    y: 0.024
    z: 0.694
```

**These x, y, z values are in meters relative to your robot base!**

---

## Step 5: Verify TF is Correct

Check the transform chain:

```bash
# See all frames
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

The TF tree should look like:
```
world
  └── base_link
        ├── link1 → link2 → ... (robot)
        └── camera_link
              └── camera_color_optical_frame
```

---

## Step 6: Plan Motion in RViz

1. **RViz should open automatically** with the MoveIt plugin

2. **In the MotionPlanning panel:**
   - Select "Planning Group": `manipulator`
   - Use the interactive marker (colored ball) to drag the end effector
   - Click **"Plan"** to see the planned path
   - Click **"Execute"** to move the robot

3. **To move to a detected object position:**
   - Note the x, y, z from Step 4
   - In RViz, enter these values in "Goal State" → "Position"
   - Add some height (e.g., +0.1m to z) for approach
   - Plan and execute

---

## Step 7: Use Python Script for Automated Pick

### Option A: Simple Echo and Manual Planning

```bash
# Terminal 1: Run the system (already running from Step 3)

# Terminal 2: Watch detections
ros2 topic echo /detected_object/pose --once
```

Copy the x, y, z values and use them in RViz.

### Option B: Use the Pick Script

```bash
cd ~/ws_moveit
source install/setup.bash
python3 src/robot_moveit_config/scripts/pick_detected_object.py
```

Follow the on-screen menu:
- Press `s` to show latest detection
- Press `p` to plan pick motion

### Option C: Write Custom Code

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MyPicker(Node):
    def __init__(self):
        super().__init__('my_picker')
        self.sub = self.create_subscription(
            PoseStamped,
            '/detected_object/pose',  # Already in base_link frame!
            self.detection_callback,
            10
        )
    
    def detection_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.get_logger().info(f'Object at: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        # Send this pose to MoveIt for planning!

rclpy.init()
node = MyPicker()
rclpy.spin(node)
```

---

## Step 8: Fine-Tune Camera Position

If objects appear in wrong locations:

1. **Place an object at a known position** (e.g., 50cm in front of robot)

2. **Check the detection:**
   ```bash
   ros2 topic echo /detected_object/pose --once
   ```

3. **Compare detected vs actual position** and adjust camera_x, camera_y, camera_z

4. **Restart with new values:**
   ```bash
   # Ctrl+C to stop, then:
   ./launch_moveit_demo.sh \
       publish_static_camera_tf:=true \
       camera_x:=0.55 \   # Adjusted value
       camera_z:=0.75     # Adjusted value
   ```

---

## Quick Reference

### Topics

| Topic | Description |
|-------|-------------|
| `/detected_object/pose` | Object pose in `base_link` frame (USE THIS!) |
| `/detected_object/pose_camera` | Object pose in camera frame |
| `/yolo/image_3d_detections` | Debug image with annotations |
| `/camera/camera/color/image_raw` | RGB image |
| `/camera/camera/aligned_depth_to_color/image_raw` | Depth image |

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `publish_static_camera_tf` | false | Set to `true` for table-mounted camera |
| `camera_x` | 0.5 | X position (meters) |
| `camera_y` | 0.0 | Y position (meters) |
| `camera_z` | 0.8 | Z position (meters) |
| `camera_pitch` | 0.0 | Pitch angle (radians) |
| `camera_roll` | 0.0 | Roll angle (radians) |
| `camera_yaw` | 0.0 | Yaw angle (radians) |
| `target_class` | "" | Filter detections (e.g., "Tomato") |

### Useful Commands

```bash
# List all topics
ros2 topic list

# See detection rate
ros2 topic hz /detected_object/pose

# View camera image
ros2 run rqt_image_view rqt_image_view

# Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

## Troubleshooting

### Problem: "TF not available" warning
**Solution:** Make sure `publish_static_camera_tf:=true` is set

### Problem: Camera errors "Device busy"
**Solution:** Unplug and replug the USB cable, then restart

### Problem: Detections are in wrong position
**Solution:** Re-measure camera position and update camera_x/y/z values

### Problem: No detections
**Solution:** 
1. Check camera is working: `ros2 topic hz /camera/camera/color/image_raw`
2. Make sure objects are in camera view
3. Check YOLO model supports the object class

---

## Example Workflow

```bash
# Terminal 1: Launch system
cd ~/ws_moveit
./launch_moveit_demo.sh publish_static_camera_tf:=true camera_x:=0.5 camera_z:=0.8 camera_pitch:=0.5

# Terminal 2: Monitor detections
cd ~/ws_moveit && source install/setup.bash
ros2 topic echo /detected_object/pose

# Terminal 3: (Optional) View camera feed
ros2 run rqt_image_view rqt_image_view /yolo/image_3d_detections
```

Then use RViz to plan and execute motions to the detected positions!
