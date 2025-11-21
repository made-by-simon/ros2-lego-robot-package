# LEGO Robot Inventor Hub - ROS 2 Interface

ROS 2 package for interfacing with LEGO Robot Inventor Hub via Bluetooth Low Energy.

## Features

- **Future-Proof Architecture**: Extensible protocol that rarely needs updates
- **Universal Device Detection**: Automatically detects all connected devices
- **Dynamic ROS Interface**: Creates topics/services based on detected devices
- **Plug-and-Play**: Works with any combination of devices on any ports
- **Zero Configuration**: No code changes needed for different robot configurations
- **Comprehensive Motor Control**: DC power, velocity, position, and angle commands
- **Status Monitoring**: Connection status and hub health published to ROS
- 20Hz sensor update rate with automatic reconnection

## System Requirements

- Ubuntu 22.04 (WSL compatible)
- ROS 2 Humble
- Python 3.10+
- Bluetooth adapter

## Design Philosophy

**The hub code is designed to be uploaded once and used for years.** The extensible architecture means:

- Most new features can be added **PC-side only** (no hub updates needed)
- Protocol supports future expansion without breaking changes
- Dynamic interface creation adapts to any robot configuration
- Clean separation: hub handles devices, PC handles intelligence

See `ARCHITECTURE.md` for detailed design documentation and `EXTENSION_GUIDE.md` for how to extend the system.

## Installation

### 1. Install Dependencies

```bash
# Install bleak for BLE communication
pip install bleak>=0.20.0

# Install Pybricks on your LEGO Hub (follow https://pybricks.com)
```

### 2. WSL Bluetooth Setup (Important!)

Since you're using WSL, you need to enable Bluetooth passthrough:

```bash
# Install usbipd-win on Windows (PowerShell as Admin):
winget install --interactive --exact dorssel.usbipd-win

# In WSL, install required tools:
sudo apt install linux-tools-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20

# On Windows PowerShell (as Admin), find your Bluetooth adapter:
usbipd list

# Attach Bluetooth adapter to WSL (replace BUSID with your device):
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>

# In WSL, verify Bluetooth is available:
hciconfig
bluetoothctl
```

Alternatively, you can run the ROS 2 node on Windows natively if Bluetooth passthrough is problematic.

### 3. Build Package

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src
git clone <your-repo> lego_hub_interface  # Or copy the package

# Build
cd ~/ros2_ws
colcon build --packages-select lego_hub_interface
source install/setup.bash
```

## Usage

### Step 1: Upload Hub Program

1. Open Pybricks Code (https://code.pybricks.com)
2. Connect to your LEGO Robot Inventor Hub
3. Copy the contents of `hub_programs/hub_listener.py`
4. Upload to the hub
5. **Disconnect from Pybricks Code** (important!)

### Step 2: Launch ROS 2 Node

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch the interface
ros2 launch lego_hub_interface hub_interface.launch.py

# Or with custom hub name:
ros2 launch lego_hub_interface hub_interface.launch.py hub_name:="My Hub"
```

### Step 3: Start Hub Program

Press the button on your LEGO hub to start the previously uploaded program. You should see "Hub ready" in the ROS terminal.

### Step 4: Interact with Topics

The node automatically creates topics based on detected devices. First, check what's available:

```bash
# List all topics
ros2 topic list

# Example output might show:
# /hub/imu
# /hub/battery_voltage
# /hub/motor_a/state
# /hub/motor_b/state
# /hub/color_c/color
# /hub/ultrasonic_d/distance
# etc.
```

**General Commands:**

```bash
# View IMU data
ros2 topic echo /hub/imu

# View battery voltage
ros2 topic echo /hub/battery_voltage

# Set LED color (works for any configuration)
ros2 topic pub /hub/led/command std_msgs/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}"

# Stop all motors
ros2 service call /hub/stop_all_motors std_srvs/srv/Trigger

# Request device configuration
ros2 service call /hub/request_config std_srvs/srv/Trigger
```

**Motor Commands (if you have motors):**

```bash
# View motor state (replace 'a' with your port)
ros2 topic echo /hub/motor_a/state

# DC power control: -100 to 100
ros2 topic pub /hub/motor_a/dc_power std_msgs/Float32 "{data: 50.0}"

# Velocity control: speed in deg/s
ros2 topic pub /hub/motor_a/velocity std_msgs/Float32 "{data: 360.0}"

# Position control: x=target_angle (degrees), y=speed (deg/s)
ros2 topic pub /hub/motor_a/position geometry_msgs/Vector3 "{x: 90.0, y: 200.0, z: 0.0}"

# Run for angle: x=angle_delta (degrees), y=speed (deg/s)
ros2 topic pub /hub/motor_a/run_angle geometry_msgs/Vector3 "{x: 180.0, y: 300.0, z: 0.0}"

# Stop motor (coast)
ros2 service call /hub/motor_a/stop std_srvs/srv/Trigger

# Brake motor (passive resistance)
ros2 service call /hub/motor_a/brake std_srvs/srv/Trigger

# Hold motor (active holding)
ros2 service call /hub/motor_a/hold std_srvs/srv/Trigger

# Reset motor angle to zero
ros2 service call /hub/motor_a/reset_angle std_srvs/srv/Trigger
```

**Sensor Data (if you have sensors):**

```bash
# Color sensor (replace 'c' with your port)
ros2 topic echo /hub/color_c/color        # Detected color
ros2 topic echo /hub/color_c/reflection   # Reflection percentage
ros2 topic echo /hub/color_c/hsv          # Raw HSV values (hue, saturation, value)

# Ultrasonic sensor
ros2 topic echo /hub/ultrasonic_d/distance
```

## Topics

The node dynamically creates topics based on detected devices. Topic naming follows the pattern:
`/hub/{device_type}_{port}/...`

### Always Available

- `/hub/imu` (sensor_msgs/Imu) - IMU data (acceleration, angular velocity)
- `/hub/battery_voltage` (std_msgs/Float32) - Battery voltage in volts
- `/hub/status` (std_msgs/String) - Connection status ("connected", "disconnected", "ready")
- `/hub/led/command` (std_msgs/ColorRGBA) - LED color control (subscribed)

### Motor Topics (Per Motor)

For each motor on port X (e.g., A, B, C...):

**Published:**
- `/hub/motor_{x}/state` (sensor_msgs/JointState) - Position (rad), velocity (rad/s), and load (%)

**Subscribed:**
- `/hub/motor_{x}/dc_power` (std_msgs/Float32) - DC power -100 to 100
- `/hub/motor_{x}/velocity` (std_msgs/Float32) - Speed in deg/s
- `/hub/motor_{x}/position` (geometry_msgs/Vector3) - x=target_angle (deg), y=speed (deg/s)
- `/hub/motor_{x}/run_angle` (geometry_msgs/Vector3) - x=angle_delta (deg), y=speed (deg/s)

### Color Sensor Topics (Per Sensor)

For each color sensor on port X:

**Published:**
- `/hub/color_{x}/color` (std_msgs/ColorRGBA) - Detected color (mapped to RGB)
- `/hub/color_{x}/reflection` (std_msgs/Int32) - Reflection percentage (0-100)
- `/hub/color_{x}/hsv` (geometry_msgs/Vector3) - Raw HSV values (x=hue 0-359, y=saturation 0-100, z=value 0-100)

### Ultrasonic Sensor Topics (Per Sensor)

For each ultrasonic sensor on port X:

**Published:**
- `/hub/ultrasonic_{x}/distance` (sensor_msgs/Range) - Distance in meters

## Services

### Global Services

- `/hub/stop_all_motors` (std_srvs/Trigger) - Coast stop for all motors
- `/hub/brake_all_motors` (std_srvs/Trigger) - Brake all motors (passive resistance)
- `/hub/hold_all_motors` (std_srvs/Trigger) - Hold all motors at current position
- `/hub/reconnect` (std_srvs/Trigger) - Trigger reconnection attempt
- `/hub/request_config` (std_srvs/Trigger) - Request device configuration resend
- `/hub/ping` (std_srvs/Trigger) - Ping hub to check responsiveness

### Per-Motor Services

For each motor on port X:

- `/hub/motor_{x}/stop` (std_srvs/Trigger) - Coast stop motor
- `/hub/motor_{x}/brake` (std_srvs/Trigger) - Brake motor
- `/hub/motor_{x}/hold` (std_srvs/Trigger) - Hold motor position
- `/hub/motor_{x}/reset_angle` (std_srvs/Trigger) - Reset angle to zero

## Configuration

Edit `config/hub_params.yaml`:

```yaml
lego_hub_node:
  ros__parameters:
    hub_name: "Pybricks Hub"      # Name of your hub
    reconnect_timeout: 5.0         # Seconds between reconnection attempts
    auto_configure: true           # Automatically detect devices and create interfaces
```

## Protocol Details

### Extensible Architecture

The hub code uses an **extensible protocol design** that rarely needs updates:

1. **Device Abstraction Layer**: Generic `DeviceWrapper` class handles any device type
2. **Command System**: Structured command bytes with port-based addressing
3. **Auto-Discovery**: Hub detects and reports all connected devices
4. **Future-Proof**: New device types and commands can be added PC-side without hub updates

### Device Detection

On startup, the hub automatically:
1. Scans all 6 ports (A-F)
2. Attempts to initialize each device type (Motor, Color, Ultrasonic, Force)
3. Sends configuration to ROS node
4. ROS node creates appropriate topics/services

### Command Structure

Commands use structured addressing:

**Port-Addressed Commands (0x01-0x6F):**
```
Command Byte = 0x01 + (port_index * 16) + (command_id - 1)
```

Examples:
- Motor on Port A, DC Power: `0x01`
- Motor on Port A, Run Target: `0x03`
- Motor on Port B, DC Power: `0x11`
- Motor on Port C, Stop: `0x25`

**Motor Commands (per port):**
- `0x01`: DC power (2 bytes: signed power -100 to 100)
- `0x02`: Run at speed (2 bytes: signed speed in deg/s)
- `0x03`: Run to target (4 bytes: angle, 2 bytes: speed)
- `0x04`: Run for angle (4 bytes: angle delta, 2 bytes: speed)
- `0x05`: Stop (1 byte: 0=coast, 1=brake, 2=hold)
- `0x06`: Reset angle (4 bytes: new angle)
- `0x07`: Set limits (2 bytes: max_speed, 2 bytes: acceleration)
- `0x08`: Track target (2 bytes: target)

**Global Commands (0x70-0xFF):**
- `0x70`: Stop all motors
- `0x71`: Brake all motors
- `0x72`: Hold all motors
- `0x80`: Hub LED on (3 bytes: R, G, B)
- `0x81`: Hub LED off
- `0x82`: Hub LED blink (3 bytes: RGB, 2 bytes: on_time, 2 bytes: off_time)
- `0xF0`: Request device configuration
- `0xF1`: Ping
- `0xF2`: Get protocol version
- `0xFF`: Shutdown

### Sensor Data Format

Data packets contain variable-length float arrays:
1. **IMU (9 floats)**: accel_xyz (cm/s²), gyro_xyz (deg/s), heading/pitch/roll (deg)
2. **Battery (1 float)**: voltage in mV
3. **Device data** (varies by type):
   - Motor: angle (deg), speed (deg/s), load (%)
   - Color: color_enum, reflection (%), H, S, V (HSV color model)
   - Ultrasonic: distance (mm)

### Why This Design is Extensible

- **New devices**: Add to `DEVICE_TYPES` and implement in `DeviceWrapper.read_state()`
- **New commands**: Add to command handler without changing protocol structure
- **Future sensors**: ROS node adapts automatically via device configuration
- **Protocol versioning**: Built-in version checking for compatibility

## Troubleshooting

### Hub not found

- Ensure hub is powered on
- Verify you're disconnected from Pybricks Code
- Check Bluetooth is working: `bluetoothctl` then `scan on`
- Try resetting Bluetooth: `sudo systemctl restart bluetooth`

### WSL Bluetooth issues

- Verify USB passthrough is working: `lsusb` should show your Bluetooth adapter
- Try running node on Windows natively instead of WSL
- Check Windows Bluetooth is not using the adapter

### Connection drops frequently

- Reduce distance between hub and PC
- Check for interference from other Bluetooth devices
- Increase `reconnect_timeout` in params.yaml

### Motor commands not working

- Verify hub program is running (press button on hub)
- Check topic names: `ros2 topic list`
- Ensure motor is connected to correct port
- Check hub logs via Pybricks Code terminal

## Example: Simple Controller

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        
        # This will work with ANY motor on port A
        self.motor_pub = self.create_publisher(
            Float32, '/hub/motor_a/velocity', 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/hub/imu', self.imu_callback, 10
        )
    
    def imu_callback(self, msg):
        # Control motor based on hub tilt
        accel_x = msg.linear_acceleration.x
        
        # Scale acceleration to motor speed (-100 to 100)
        speed = Float32()
        speed.data = max(-100.0, min(100.0, accel_x * 10.0))
        self.motor_pub.publish(speed)

def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example: Multi-Robot Setup

The universal design makes it easy to control multiple hubs:

```bash
# Terminal 1: First hub
ros2 launch lego_hub_interface hub_interface.launch.py hub_name:="Robot1"

# Terminal 2: Second hub  
ros2 launch lego_hub_interface hub_interface.launch.py hub_name:="Robot2"

# Now you have separate namespaces with identical interfaces
ros2 topic list
# /hub/motor_a/velocity      (for Robot1)
# /robot2/hub/motor_a/velocity   (for Robot2)
```

## License

MIT

## Author

Simon Wong (smw2@ualberta.ca)