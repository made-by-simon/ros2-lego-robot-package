# LEGO Hub ROS 2 Interface

ROS 2 Humble package for interfacing with LEGO Robot Inventor Hub via Bluetooth.

## Quick Setup

### Prerequisites

- WSL 2 with Ubuntu 22.04
- ROS 2 Humble
- Bluetooth adapter

### 1. WSL Bluetooth Setup (One-Time)

**Windows PowerShell (Admin):**
```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list  # Find Bluetooth BUSID (e.g., 1-4)
usbipd bind --busid 1-4
usbipd attach --wsl --busid 1-4
```

**WSL Terminal:**
```bash
sudo apt install -y bluetooth bluez python3-pip
sudo systemctl start bluetooth
pip3 install bleak>=0.20.0
```

### 2. Install Package

```bash
cd ~/ros2_ws/src
git clone <repo> lego_hub_ros2
cd ~/ros2_ws
colcon build --packages-select lego_hub_ros2
source install/setup.bash
```

### 3. Upload Hub Program

1. Open https://code.pybricks.com
2. Connect to hub
3. Copy contents of `config/hub_program.py`
4. Upload to hub
5. **Disconnect from Pybricks Code**

### 4. Launch

```bash
ros2 launch lego_hub_ros2 hub.launch.py
```

Press hub button to start. You should see device detection and "Ready!"

## Usage

### Topics

**Published:**
- `/hub/imu` - IMU data
- `/hub/battery` - Battery voltage (V)
- `/hub/motor_{port}/state` - Motor position, velocity, effort

**Subscribed:**
- `/hub/led` - LED color (ColorRGBA)
- `/hub/motor_{port}/velocity` - Speed command (Float32, deg/s)
- `/hub/motor_{port}/position` - Position command (Vector3: x=angle, y=speed)

### Services

- `/hub/motor_{port}/stop` - Stop motor
- `/hub/stop_all` - Emergency stop

### Examples

```bash
# Motor control
ros2 topic pub --once /hub/motor_a/velocity std_msgs/Float32 "{data: 180.0}"

# LED control
ros2 topic pub --once /hub/led std_msgs/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0}"

# View sensors
ros2 topic echo /hub/imu
ros2 topic echo /hub/battery
```

## Daily Workflow

**Each time you restart:**

1. Attach Bluetooth: `usbipd attach --wsl --busid 1-4` (Windows Admin)
2. Launch: `ros2 launch lego_hub_ros2 hub.launch.py`
3. Press hub button

## Troubleshooting

**Hub not found:**
- Check `hciconfig` shows adapter
- Verify hub program uploaded
- Ensure Pybricks Code disconnected

**Build fails:**
- Verify `__init__.py` exists in `lego_hub_ros2/` folder
- Check resource marker exists: `resource/lego_hub_ros2`

**Connection drops:**
- Reduce distance to hub
- Check for Bluetooth interference

## License

MIT
