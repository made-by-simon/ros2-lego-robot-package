# Quick Start Guide

## Prerequisites

- WSL 2 with Ubuntu 22.04
- ROS 2 Humble installed
- Bluetooth adapter

## Setup (15 minutes)

### 1. WSL Bluetooth (One-Time, 5 min)

**Windows PowerShell (Admin):**
```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list  # Find Bluetooth BUSID
usbipd bind --busid 1-4  # Replace with your BUSID
usbipd attach --wsl --busid 1-4
```

**WSL Terminal:**
```bash
sudo apt install -y bluetooth bluez python3-pip
sudo systemctl start bluetooth
pip3 install bleak>=0.20.0
```

### 2. Install Package (5 min)

```bash
cd ~/ros2_ws/src
# Copy lego_hub_ros2 folder here
cd ~/ros2_ws
colcon build --packages-select lego_hub_ros2
source install/setup.bash
```

### 3. Upload Hub Program (5 min)

1. Open https://code.pybricks.com
2. Connect to hub
3. Copy all of `config/hub_program.py`
4. Upload to hub
5. **Disconnect from Pybricks Code**

### 4. Test (2 min)

```bash
ros2 launch lego_hub_ros2 hub.launch.py
```

Press hub button. You should see "Ready!"

```bash
# Test motor (replace 'a' with your port)
ros2 topic pub --once /hub/motor_a/velocity std_msgs/Float32 "{data: 180.0}"
```

## Daily Usage

**Every restart:**
1. Windows Admin: `usbipd attach --wsl --busid 1-4`
2. WSL: `ros2 launch lego_hub_ros2 hub.launch.py`
3. Press hub button

## Common Commands

```bash
# List topics
ros2 topic list

# Motor control
ros2 topic pub --once /hub/motor_a/velocity std_msgs/Float32 "{data: 360.0}"

# Stop motor
ros2 service call /hub/motor_a/stop std_srvs/srv/Trigger

# LED control
ros2 topic pub --once /hub/led std_msgs/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0}"

# View sensors
ros2 topic echo /hub/imu
ros2 topic echo /hub/battery
```

## Troubleshooting

**Hub not found:**
- Check: `hciconfig` shows adapter
- Verify hub program uploaded
- Confirm Pybricks Code disconnected

**Build fails:**
- Check `__init__.py` exists in `lego_hub_ros2/lego_hub_ros2/`
- Check resource file: `resource/lego_hub_ros2`

**Connection drops:**
- Reduce distance to hub
- Check for interference
