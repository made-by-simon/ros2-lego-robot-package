# LEGO Hub ROS 2 Interface

ROS 2 Humble package for LEGO Robot Inventor Hub via Bluetooth.

## Prerequisites

- WSL 2 with Ubuntu 22.04
- ROS 2 Humble
- Bluetooth adapter
- Python packages: `bleak>=0.20.0`

## Installation

### 1. WSL Bluetooth Setup (One-Time)

Windows PowerShell (Admin):
```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list
usbipd bind --busid 1-4
usbipd attach --wsl --busid 1-4
```

WSL Terminal:
```bash
sudo apt install -y bluetooth bluez python3-pip
sudo systemctl start bluetooth
pip3 install bleak>=0.20.0
```

### 2. Install ROS 2 Package

```bash
cd ~/ros2_ws/src
git clone <repo> lego_hub_ros2
cd ~/ros2_ws
colcon build --packages-select lego_hub_ros2
source install/setup.bash
```

### 3. Upload Hub Program

1. Visit https://code.pybricks.com
2. Connect to hub
3. Copy contents of `config/hub_program.py`
4. Upload to hub
5. Disconnect from Pybricks Code

### 4. Launch

```bash
ros2 launch lego_hub_ros2 hub.launch.py
```

Press hub button to start. Device detection will run automatically.

## Usage

### Topics

Published:
- `/hub/imu` (sensor_msgs/Imu) - IMU data
- `/hub/battery` (std_msgs/Float32) - Battery voltage in volts
- `/hub/motor_{port}/state` (sensor_msgs/JointState) - Motor state
- `/hub/color_{port}/color` (std_msgs/ColorRGBA) - Detected color
- `/hub/color_{port}/reflection` (std_msgs/Int32) - Reflection percentage
- `/hub/color_{port}/hsv` (geometry_msgs/Vector3) - HSV values
- `/hub/ultrasonic_{port}/distance` (sensor_msgs/Range) - Distance in meters

Subscribed:
- `/hub/led` (std_msgs/ColorRGBA) - Set LED color
- `/hub/motor_{port}/velocity` (std_msgs/Float32) - Speed in deg/s
- `/hub/motor_{port}/position` (geometry_msgs/Vector3) - Target position (x=angle, y=speed)

### Services

- `/hub/motor_{port}/stop` (std_srvs/Trigger) - Stop specific motor
- `/hub/stop_all` (std_srvs/Trigger) - Emergency stop all motors

### Examples

Motor control:
```bash
ros2 topic pub --once /hub/motor_a/velocity std_msgs/Float32 "{data: 180.0}"
ros2 topic pub --once /hub/motor_a/position geometry_msgs/Vector3 "{x: 360.0, y: 200.0}"
ros2 service call /hub/motor_a/stop std_srvs/srv/Trigger
```

LED control:
```bash
ros2 topic pub --once /hub/led std_msgs/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}"
```

View sensors:
```bash
ros2 topic echo /hub/imu
ros2 topic echo /hub/battery
ros2 topic list
```

## Daily Workflow

Each restart:
1. Attach Bluetooth: `usbipd attach --wsl --busid 1-4` (Windows Admin)
2. Launch node: `ros2 launch lego_hub_ros2 hub.launch.py`
3. Press hub button

## Troubleshooting

Hub not found:
- Verify `hciconfig` shows adapter
- Confirm hub program uploaded
- Ensure Pybricks Code disconnected

Build fails:
- Check `__init__.py` exists in `lego_hub_ros2/lego_hub_ros2/`
- Verify resource file: `resource/lego_hub_ros2`

Connection drops:
- Reduce distance to hub
- Check for Bluetooth interference

## Package Structure

```
lego_hub_ros2/
├── config/
│   └── hub_program.py          # MicroPython code for hub
├── launch/
│   └── hub.launch.py           # Launch file
├── lego_hub_ros2/
│   ├── __init__.py            # Package initialization
│   └── hub_node.py            # Main ROS 2 node
├── resource/
│   └── lego_hub_ros2          # Resource marker
├── package.xml                 # Package manifest
├── setup.py                    # Setup script
└── README.md                   # This file
```

## License

MIT
