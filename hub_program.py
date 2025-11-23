"""
MicroPython script for LEGO Robot Inventor Hub.
Enables ROS 2 interface via Bluetooth with automatic device detection.
"""

from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from usys import stdin, stdout
from uselect import poll
import ustruct


# Initialize hub.
hub = InventorHub()

# Device type constants.
MOTOR = 1
COLOR = 2
ULTRASONIC = 3

PORTS = [Port.A, Port.B, Port.C, Port.D, Port.E, Port.F]
PORT_NAMES = ["A", "B", "C", "D", "E", "F"]


def detect_devices():
    """Detect all connected devices on all ports."""
    devices = []
    
    for i, port in enumerate(PORTS):
        device = None
        device_type = 0
        
        # Try each device type in order.
        try:
            device = Motor(port)
            device_type = MOTOR
        except:
            try:
                device = ColorSensor(port)
                device_type = COLOR
            except:
                try:
                    device = UltrasonicSensor(port)
                    device_type = ULTRASONIC
                except:
                    pass
        
        devices.append((i, device_type, device))
        
        if device:
            type_names = ["Motor", "Color Sensor", "Ultrasonic Sensor"]
            print(f"Port {PORT_NAMES[i]}: {type_names[device_type - 1]}")
    
    return devices


def send_configuration(devices):
    """Send device configuration to ROS node."""
    num_devices = sum(1 for _, dt, _ in devices if dt > 0)
    
    config = bytes([num_devices])
    for port_idx, dev_type, _ in devices:
        if dev_type > 0:
            config += bytes([port_idx, dev_type])
    
    stdout.buffer.write(b"CFG" + ustruct.pack(">H", len(config)) + config)


def read_sensors(devices):
    """Read all sensor data and return as float list."""
    data = []
    
    # Hub IMU data (9 floats).
    imu = hub.imu.acceleration()
    gyro = hub.imu.angular_velocity()
    data.extend([imu[0], imu[1], imu[2], gyro[0], gyro[1], gyro[2]])
    data.extend([hub.imu.heading(), hub.imu.tilt()[0], hub.imu.tilt()[1]])
    
    # Battery voltage (1 float).
    data.append(hub.battery.voltage())
    
    # Device-specific data.
    for _, dev_type, dev in devices:
        if dev_type == MOTOR:
            data.extend([dev.angle(), dev.speed(), dev.load()])
        
        elif dev_type == COLOR:
            # Map color to integer code.
            from pybricks.parameters import Color as C
            color_map = {
                None: -1, C.BLACK: 0, C.VIOLET: 1, C.BLUE: 3,
                C.CYAN: 4, C.GREEN: 5, C.YELLOW: 6, C.ORANGE: 8,
                C.RED: 9, C.WHITE: 10, C.NONE: -1
            }
            hsv = dev.hsv()
            data.extend([
                color_map.get(dev.color(), -1),
                dev.reflection(),
                hsv.h, hsv.s, hsv.v
            ])
        
        elif dev_type == ULTRASONIC:
            distance = dev.distance()
            data.append(distance if distance else -1)
    
    return data


def handle_motor_command(device, cmd_id, params):
    """Execute motor command."""
    try:
        if cmd_id == 1 and len(params) >= 2:  # DC power.
            power = ustruct.unpack(">h", params[:2])[0]
            device.dc(power)
            return True
        
        elif cmd_id == 2 and len(params) >= 2:  # Run at speed.
            speed = ustruct.unpack(">h", params[:2])[0]
            device.run(speed)
            return True
        
        elif cmd_id == 3 and len(params) >= 4:  # Run to target.
            angle = ustruct.unpack(">i", params[:4])[0]
            speed = ustruct.unpack(">h", params[4:6])[0] if len(params) >= 6 else 200
            device.run_target(speed, angle, wait=False)
            return True
        
        elif cmd_id == 4 and len(params) >= 4:  # Run by angle.
            angle = ustruct.unpack(">i", params[:4])[0]
            speed = ustruct.unpack(">h", params[4:6])[0] if len(params) >= 6 else 200
            device.run_angle(speed, angle, wait=False)
            return True
        
        elif cmd_id == 5:  # Stop.
            stop_type = params[0] if len(params) >= 1 else 0
            [device.stop, device.brake, device.hold][stop_type]()
            return True
        
        elif cmd_id == 6:  # Reset angle.
            angle = ustruct.unpack(">i", params[:4])[0] if len(params) >= 4 else 0
            device.reset_angle(angle)
            return True
    
    except Exception as e:
        print(f"Motor command error: {e}")
    
    return False


def handle_command(cmd, devices):
    """Process incoming command."""
    if len(cmd) < 1:
        return False
    
    cmd_type = cmd[0]
    
    # Device commands (0x01-0x6F).
    if 0x01 <= cmd_type <= 0x6F:
        port_idx = (cmd_type - 0x01) // 16
        cmd_id = (cmd_type - 0x01) % 16 + 1
        
        if port_idx < len(devices):
            _, dev_type, dev = devices[port_idx]
            params = cmd[1:]
            
            if dev_type == MOTOR:
                return handle_motor_command(dev, cmd_id, params)
    
    # Global commands (0x70-0xFF).
    elif cmd_type == 0x70:  # Stop all motors.
        for _, dt, d in devices:
            if dt == MOTOR:
                d.stop()
        return True
    
    elif cmd_type == 0x80 and len(cmd) >= 4:  # LED.
        hub.light.on((cmd[1], cmd[2], cmd[3]))
        return True
    
    elif cmd_type == 0xFF:  # Shutdown.
        stdout.buffer.write(b"BYE\n")
        return "shutdown"
    
    return False


def main():
    """Main program loop."""
    # Detect devices.
    print("Detecting devices...")
    devices = detect_devices()
    
    # Send configuration.
    send_configuration(devices)
    
    # Setup input polling.
    keyboard = poll()
    keyboard.register(stdin)
    
    print("\nReady.")
    hub.light.on(Color.GREEN)
    
    # Main loop.
    while True:
        # Read and send sensor data.
        sensor_data = read_sensors(devices)
        packed = ustruct.pack(f">{len(sensor_data)}f", *sensor_data)
        stdout.buffer.write(b"SNS" + ustruct.pack(">H", len(packed)) + packed)
        
        # Check for commands.
        if keyboard.poll(0):
            header = stdin.buffer.read(3)
            
            if header == b"CMD":
                length = ustruct.unpack(">H", stdin.buffer.read(2))[0]
                cmd = stdin.buffer.read(length)
                
                result = handle_command(cmd, devices)
                
                if result == "shutdown":
                    break
                
                stdout.buffer.write(b"ACK\n" if result else b"NAK\n")
        
        wait(50)
    
    hub.light.off()


if __name__ == "__main__":
    main()
