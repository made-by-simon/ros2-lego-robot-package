# LEGO Hub ROS 2 - Quick Reference

## Startup Sequence

1. Upload `hub_listener.py` to LEGO hub via Pybricks Code
2. Disconnect from Pybricks Code
3. Launch ROS node: `ros2 launch lego_hub_interface hub_interface.launch.py`
4. Press button on hub to start program
5. Wait for "Dynamic interfaces created successfully!" message
6. Check available topics: `ros2 topic list`

## Common Commands

### Discovery

```bash
# See what devices are connected
ros2 topic list

# View device configuration in terminal output
# Look for lines like "Port A: Motor", "Port C: ColorSensor"
```

### Motors

```bash
# DC power control (-100 to 100)
ros2 topic pub /hub/motor_a/dc_power std_msgs/Float32 "{data: 50.0}"

# Velocity control (deg/s)
ros2 topic pub /hub/motor_a/velocity std_msgs/Float32 "{data: 360.0}"

# Position control (x=angle in deg, y=speed in deg/s)
ros2 topic pub /hub/motor_a/position geometry_msgs/Vector3 "{x: 180.0, y: 300.0}"

# Run for angle (x=angle_delta in deg, y=speed in deg/s)
ros2 topic pub /hub/motor_a/run_angle geometry_msgs/Vector3 "{x: 90.0, y: 200.0}"

# Monitor motor state
ros2 topic echo /hub/motor_a/state

# Stop motor (coast)
ros2 service call /hub/motor_a/stop std_srvs/srv/Trigger

# Brake motor (passive resistance)
ros2 service call /hub/motor_a/brake std_srvs/srv/Trigger

# Hold motor (active hold)
ros2 service call /hub/motor_a/hold std_srvs/srv/Trigger

# Reset angle
ros2 service call /hub/motor_a/reset_angle std_srvs/srv/Trigger
```

### Sensors

```bash
# Color sensor
ros2 topic echo /hub/color_c/color
ros2 topic echo /hub/color_c/reflection
ros2 topic echo /hub/color_c/hsv

# Ultrasonic
ros2 topic echo /hub/ultrasonic_d/distance
```

### Hub Control

```bash
# IMU data
ros2 topic echo /hub/imu

# Battery
ros2 topic echo /hub/battery_voltage

# LED control
ros2 topic pub /hub/led/command std_msgs/ColorRGBA "{r: 0.0, g: 1.0, b: 0.0}"

# Emergency stop all motors
ros2 service call /hub/stop_all_motors std_srvs/srv/Trigger
```

## Topic Naming Convention

- **Motors**: `/hub/motor_{port}/...` (port = a, b, c, d, e, f)
- **Color Sensors**: `/hub/color_{port}/...`
- **Ultrasonic Sensors**: `/hub/ultrasonic_{port}/...`
- **Force Sensors**: `/hub/force_{port}/...`
- **Hub**: `/hub/imu`, `/hub/battery_voltage`, `/hub/led/command`

## Python API Examples

### Subscribe to Motor State

```python
from sensor_msgs.msg import JointState

def callback(msg):
    angle_rad = msg.position[0]
    velocity_rad_s = msg.velocity[0]
    load_percent = msg.effort[0]
    print(f"Angle: {angle_rad}, Speed: {velocity_rad_s}")

self.create_subscription(JointState, '/hub/motor_a/state', callback, 10)
```

### Control Motor Velocity

```python
from std_msgs.msg import Float32

pub = self.create_publisher(Float32, '/hub/motor_a/velocity', 10)

msg = Float32()
msg.data = 360.0  # deg/s
pub.publish(msg)
```

### Control Motor with DC Power

```python
from std_msgs.msg import Float32

pub = self.create_publisher(Float32, '/hub/motor_a/dc_power', 10)

msg = Float32()
msg.data = 75.0  # -100 to 100
pub.publish(msg)
```

### Control Motor Position

```python
from geometry_msgs.msg import Vector3

pub = self.create_publisher(Vector3, '/hub/motor_a/position', 10)

msg = Vector3()
msg.x = 90.0   # Target angle in degrees
msg.y = 200.0  # Speed (deg/s)
pub.publish(msg)
```

### Read Color Sensor

```python
from std_msgs.msg import ColorRGBA, Int32
from geometry_msgs.msg import Vector3

# Detected color (mapped)
self.create_subscription(ColorRGBA, '/hub/color_c/color', color_cb, 10)

# Reflection percentage
self.create_subscription(Int32, '/hub/color_c/reflection', refl_cb, 10)

# Raw HSV (hue 0-359, saturation 0-100, value 0-100)
self.create_subscription(Vector3, '/hub/color_c/hsv', hsv_cb, 10)
```

### Read IMU

```python
from sensor_msgs.msg import Imu

def imu_callback(msg):
    accel_x = msg.linear_acceleration.x  # m/s²
    gyro_z = msg.angular_velocity.z      # rad/s

self.create_subscription(Imu, '/hub/imu', imu_callback, 10)
```

### Call Service

```python
from std_srvs.srv import Trigger

client = self.create_client(Trigger, '/hub/motor_a/stop')
client.wait_for_service()

request = Trigger.Request()
future = client.call_async(request)
```

## Troubleshooting

### No topics appearing

- Ensure hub program is running (press button)
- Check ROS node output for "Dynamic interfaces created successfully!"
- Try: `ros2 service call /hub/request_config std_srvs/srv/Trigger`

### Motor not responding

- Verify correct port name (check `ros2 topic list`)
- Ensure motor is properly connected to hub
- Check if motor is stopped/held: try `ros2 service call /hub/motor_a/stop std_srvs/srv/Trigger`

### Connection drops

- Check Bluetooth range
- Verify no other apps (like Pybricks Code) are connected
- Increase `reconnect_timeout` in config

### WSL Bluetooth issues

- Verify USB passthrough: `lsusb` should show Bluetooth adapter
- Check `hciconfig` shows adapter
- Consider running on Windows natively

## Unit Conversions

The ROS interface uses standard SI units:

| Hub Units | ROS Units | Conversion |
|-----------|-----------|------------|
| Motor angle (deg) | rad | `× 0.0174533` |
| Motor speed (deg/s) | rad/s | `× 0.0174533` |
| Acceleration (cm/s²) | m/s² | `× 0.01` |
| Angular velocity (deg/s) | rad/s | `× 0.0174533` |
| Distance (mm) | m | `× 0.001` |
| Voltage (mV) | V | `× 0.001` |
| Color RGB (0-100) | normalized | `÷ 100` |

## Command Opcodes (Advanced)

If you need to send raw commands:

### Motors (0x01-0x42)
- `0x01-0x06`: Velocity (ports A-F)
- `0x11-0x16`: Position (ports A-F)
- `0x21-0x26`: Reset angle (ports A-F)
- `0x31-0x36`: Stop specific motor
- `0x40`: Stop all
- `0x41`: Brake all
- `0x42`: Hold all

### Hub (0x50-0xFF)
- `0x50`: LED RGB
- `0x51`: LED off
- `0xF0`: Request config
- `0xFF`: Shutdown

## Best Practices

1. **Always check topic list** after connection to see available devices
2. **Use services for one-time actions** (stop, reset), topics for continuous control
3. **Monitor battery voltage** - hub will disconnect when low
4. **Implement timeout logic** in your controllers for safety
5. **Test with small speeds first** before running at full power
6. **Use position control** for precise movements, velocity for continuous operation
7. **Subscribe to motor state** to verify commands are executed

## Performance Tips

- Hub updates at 20Hz (50ms period)
- Motor commands are executed immediately
- Position commands are non-blocking (motor moves in background)
- Multiple position commands will override previous targets
- LED changes take ~10ms
- Sensor readings are always fresh from last cycle

## Additional Resources

- Pybricks Documentation: https://docs.pybricks.com
- ROS 2 Humble Docs: https://docs.ros.org/en/humble/
- Bleak (BLE Library): https://github.com/hbldh/bleak