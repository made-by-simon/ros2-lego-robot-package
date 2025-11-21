"""
MicroPython script that runs on Lego Robot Inventor Hub to allow efficient and 
adaptable operation with a ROS 2 node. Simplifies interaction with hardware 
(motors and sensors included in Lego set 51515) to a standardized byte code format. 
Automatically detects and mananges all connected devices. 
"""

from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from usys import stdin, stdout
from uselect import poll
import ustruct 


hub = InventorHub()

# Device types. 
MOTOR, COLOR, ULTRASONIC = 1, 2, 3
PORTS = [Port.A, Port.B, Port.C, Port.D, Port.E, Port.F]
PORT_NAMES = ["A", "B", "C", "D", "E", "F"]

# Detect all devices. 
devices = []
for i, port in enumerate(PORTS): 
    device = None
    device_type = 0
    # Nested try/except flow to find the correct device type. 
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
        print(f"Identified {['Motor', 'Color', 'Ultrasonic'][device_type-1]}{' Sensor' if device_type >= 2 else ''} at Port {PORT_NAMES[i]}")

# Transmit connected devices. 
num_devices = 0
for _, dev_type, _ in devices:
    if dev_type > 0:
        num_devices += 1
config = bytes([num_devices])
for port_idx, dev_type, _ in devices:
    if dev_type > 0:
        config += bytes([port_idx, dev_type])
stdout.buffer.write(b"CFG" + ustruct.pack('>H', len(config)) + config)

# Setup keyboard polling. 
keyboard = poll()
keyboard.register(stdin)
print("\nReady.")
hub.light.on(Color.GREEN)

# Main loop. 
while True:
    # Read and send sensor data. 
    data = []

    imu = hub.imu.acceleration()
    gyro = hub.imu.angular_velocity()
    data.extend([imu[0], imu[1], imu[2], gyro[0], gyro[1], gyro[2]])
    data.extend([hub.imu.heading(), hub.imu.tilt()[0], hub.imu.tilt()[1]])
    data.append(hub.battery.voltage())
    
    for _, dev_type, dev in devices:
        if dev_type == MOTOR:
            data.extend([dev.angle(), dev.speed(), dev.load()])
        elif dev_type == COLOR:
            from pybricks.parameters import Color as C
            color_map = {None: -1, C.BLACK: 0, C.VIOLET: 1, C.BLUE: 3, 
                        C.CYAN: 4, C.GREEN: 5, C.YELLOW: 6, C.ORANGE: 8, 
                        C.RED: 9, C.WHITE: 10, C.NONE: -1}
            hsv = dev.hsv()
            data.extend([color_map.get(dev.color(), -1), dev.reflection(), 
                        hsv.h, hsv.s, hsv.v])
        elif dev_type == ULTRASONIC:
            d = dev.distance()
            data.append(d if d else -1)

    packed = ustruct.pack(f'>{len(data)}f', *data)
    stdout.buffer.write(b"SNS" + ustruct.pack('>H', len(packed)) + packed)
    
    # Check for commands. 
    if keyboard.poll(0):
        header = stdin.buffer.read(3)
        if header == b"CMD":
            length = ustruct.unpack('>H', stdin.buffer.read(2))[0]
            cmd = stdin.buffer.read(length)
            
            if len(cmd) < 1:
                continue
            
            cmd_type = cmd[0]
            success = False
            
            # Device commands (0x01-0x6F). 
            if 0x01 <= cmd_type <= 0x6F:
                port_idx = (cmd_type - 0x01) // 16
                cmd_id = (cmd_type - 0x01) % 16 + 1
                
                if port_idx < len(devices):
                    _, dev_type, dev = devices[port_idx]
                    params = cmd[1:]
                    
                    try:
                        if dev_type == MOTOR:
                            if cmd_id == 1 and len(params) >= 2:  # DC
                                dev.dc(ustruct.unpack('>h', params[:2])[0])
                                success = True
                            elif cmd_id == 2 and len(params) >= 2:  # Run speed
                                dev.run(ustruct.unpack('>h', params[:2])[0])
                                success = True
                            elif cmd_id == 3 and len(params) >= 4:  # Run target
                                angle = ustruct.unpack('>i', params[:4])[0]
                                speed = ustruct.unpack('>h', params[4:6])[0] if len(params) >= 6 else 200
                                dev.run_target(speed, angle, wait=False)
                                success = True
                            elif cmd_id == 4 and len(params) >= 4:  # Run angle
                                angle = ustruct.unpack('>i', params[:4])[0]
                                speed = ustruct.unpack('>h', params[4:6])[0] if len(params) >= 6 else 200
                                dev.run_angle(speed, angle, wait=False)
                                success = True
                            elif cmd_id == 5:  # Stop
                                stop_type = params[0] if len(params) >= 1 else 0
                                [dev.stop, dev.brake, dev.hold][stop_type]()
                                success = True
                            elif cmd_id == 6:  # Reset angle
                                angle = ustruct.unpack('>i', params[:4])[0] if len(params) >= 4 else 0
                                dev.reset_angle(angle)
                                success = True
                    except:
                        pass
            
            # Global commands (0x70-0xFF). 
            elif cmd_type == 0x70:  # Stop all. 
                for _, dt, d in devices:
                    if dt == MOTOR:
                        d.stop()
                success = True
            elif cmd_type == 0x80 and len(cmd) >= 4:  # LED. 
                hub.light.on((cmd[1], cmd[2], cmd[3]))
                success = True
            elif cmd_type == 0xFF:  # Shutdown. 
                stdout.buffer.write(b"BYE\n")
                break
            
            stdout.buffer.write(b"ACK\n" if success else b"NAK\n")
    
    wait(50)

hub.light.off()
