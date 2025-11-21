"""
Universal ROS 2 Node for LEGO Robot Inventor Hub Interface
Extensible protocol handler with future-proof design
"""
import asyncio
import struct
from threading import Thread, Event, Lock
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Imu, Range, JointState
from std_msgs.msg import ColorRGBA, Float32, Int32, Bool, String
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger

from bleak import BleakScanner, BleakClient


# Device type constants
DEVICE_TYPES = {
    0: 'NONE',
    1: 'MOTOR',
    2: 'COLOR',
    3: 'ULTRASONIC',
    4: 'FORCE',
}

PORT_NAMES = ['A', 'B', 'C', 'D', 'E', 'F']


class MotorCommand(IntEnum):
    """Motor command IDs"""
    DC_POWER = 0x01
    RUN_SPEED = 0x02
    RUN_TARGET = 0x03
    RUN_ANGLE = 0x04
    STOP = 0x05
    RESET_ANGLE = 0x06
    SET_LIMITS = 0x07
    TRACK_TARGET = 0x08


class GlobalCommand(IntEnum):
    """Global command IDs"""
    STOP_ALL = 0x70
    BRAKE_ALL = 0x71
    HOLD_ALL = 0x72
    LED_ON = 0x80
    LED_OFF = 0x81
    LED_BLINK = 0x82
    REQUEST_CONFIG = 0xF0
    PING = 0xF1
    GET_VERSION = 0xF2
    SHUTDOWN = 0xFF


def compute_command_byte(port_idx, cmd_id):
    """Compute command byte from port index and command ID"""
    return 0x01 + port_idx * 16 + (cmd_id - 1)


class LegoHubNode(Node):
    """Universal ROS 2 node with extensible protocol"""
    
    PYBRICKS_COMMAND_UUID = "c5f50002-8280-46da-89f4-6d8051e4aeef"
    
    def __init__(self):
        super().__init__('lego_hub_node')
        
        # Parameters
        self.declare_parameter('hub_name', 'Pybricks Hub')
        self.declare_parameter('reconnect_timeout', 5.0)
        self.declare_parameter('auto_configure', True)
        self.declare_parameter('update_rate', 20.0)
        
        self.hub_name = self.get_parameter('hub_name').value
        self.reconnect_timeout = self.get_parameter('reconnect_timeout').value
        self.auto_configure = self.get_parameter('auto_configure').value
        
        # Device configuration
        self.device_config = []
        self.config_received = Event()
        self.config_lock = Lock()
        
        # Dynamic interfaces
        self.publishers = {}
        self.subscribers = {}
        self.services = {}
        
        # Static publishers
        self.imu_pub = self.create_publisher(Imu, 'hub/imu', 10)
        self.battery_pub = self.create_publisher(Float32, 'hub/battery_voltage', 10)
        self.status_pub = self.create_publisher(String, 'hub/status', 10)
        
        # Static subscribers
        self.create_subscription(ColorRGBA, 'hub/led/command', self.led_callback, 10)
        
        # Static services
        self.create_service(Trigger, 'hub/stop_all_motors', self.stop_all_callback)
        self.create_service(Trigger, 'hub/brake_all_motors', self.brake_all_callback)
        self.create_service(Trigger, 'hub/hold_all_motors', self.hold_all_callback)
        self.create_service(Trigger, 'hub/reconnect', self.reconnect_callback)
        self.create_service(Trigger, 'hub/request_config', self.request_config_callback)
        self.create_service(Trigger, 'hub/ping', self.ping_callback)
        
        # BLE state
        self.client = None
        self.connected = False
        self.connection_event = Event()
        
        # Async communication
        self.loop = None
        self.ble_thread = None
        self.command_queue = None
        
        self.start_ble_thread()
        
        self.get_logger().info("Universal LEGO Hub Node initialized (Extensible Protocol v1.0)")
        self.get_logger().info(f"Searching for '{self.hub_name}'...")
    
    def start_ble_thread(self):
        """Start BLE communication thread"""
        self.ble_thread = Thread(target=self._run_ble_loop, daemon=True)
        self.ble_thread.start()
    
    def _run_ble_loop(self):
        """Run asyncio event loop"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.command_queue = asyncio.Queue()
        self.loop.run_until_complete(self._ble_main())
    
    async def _ble_main(self):
        """Main BLE loop"""
        while rclpy.ok():
            try:
                await self._connect_to_hub()
                if self.connected:
                    if self.auto_configure:
                        self.get_logger().info("Waiting for device configuration...")
                        await asyncio.wait_for(
                            asyncio.get_event_loop().run_in_executor(None, self.config_received.wait),
                            timeout=10.0
                        )
                        self._setup_dynamic_interfaces()
                    
                    await self._communication_loop()
            except asyncio.TimeoutError:
                self.get_logger().error("Timeout waiting for configuration")
                self.connected = False
            except Exception as e:
                self.get_logger().error(f"BLE error: {e}")
                self.connected = False
            
            await asyncio.sleep(self.reconnect_timeout)
    
    async def _connect_to_hub(self):
        """Connect to hub"""
        self.get_logger().info("Scanning for hub...")
        
        device = await BleakScanner.find_device_by_name(self.hub_name, timeout=10.0)
        
        if device is None:
            self.get_logger().warn(f"Hub '{self.hub_name}' not found")
            return
        
        self.get_logger().info(f"Found hub at {device.address}, connecting...")
        
        self.client = BleakClient(device, disconnected_callback=self._handle_disconnect)
        await self.client.connect()
        await self.client.start_notify(self.PYBRICKS_COMMAND_UUID, self._handle_rx)
        
        self.connected = True
        self.connection_event.set()
        
        # Publish status
        status_msg = String()
        status_msg.data = "connected"
        self.status_pub.publish(status_msg)
        
        self.get_logger().info("Connected! Press button on hub to start.")
        await asyncio.sleep(0.5)
    
    def _handle_disconnect(self, client):
        """Handle disconnection"""
        self.get_logger().warn("Hub disconnected")
        self.connected = False
        self.connection_event.clear()
        self.config_received.clear()
        
        status_msg = String()
        status_msg.data = "disconnected"
        self.status_pub.publish(status_msg)
    
    def _handle_rx(self, sender, data: bytearray):
        """Handle incoming BLE data"""
        if data[0] == 0x01:
            payload = data[1:]
            self._process_hub_data(payload)
    
    def _process_hub_data(self, data):
        """Process hub data"""
        if len(data) < 3:
            return
        
        header = data[:3]
        
        if header == b"RDY":
            self.get_logger().info("Hub ready - waiting for program start")
        elif header == b"ACK":
            pass
        elif header == b"NAK":
            self.get_logger().warn("Hub rejected command")
        elif header == b"BYE":
            self.get_logger().info("Hub program ended")
        elif header == b"PNG":
            self.get_logger().info("Ping response received")
        elif header == b"ERR":
            if len(data) >= 4:
                error_code = data[3]
                self.get_logger().error(f"Hub error code: {error_code}")
        elif header == b"VER":
            if len(data) >= 5:
                version = data[5]
                self.get_logger().info(f"Hub protocol version: {version}")
        elif header == b"CFG":
            if len(data) >= 5:
                data_length = struct.unpack('>H', data[3:5])[0]
                config_data = data[5:5+data_length]
                self._parse_device_config(config_data)
        elif header == b"SNS":
            if len(data) >= 5:
                data_length = struct.unpack('>H', data[3:5])[0]
                sensor_data = data[5:5+data_length]
                self._parse_sensor_data(sensor_data)
    
    def _parse_device_config(self, data):
        """Parse device configuration"""
        with self.config_lock:
            self.device_config = []
            
            if len(data) < 1:
                return
            
            num_devices = data[0]
            self.get_logger().info(f"Received configuration: {num_devices} devices")
            
            for i in range(num_devices):
                if len(data) >= 1 + (i + 1) * 2:
                    port_idx = data[1 + i * 2]
                    device_type = data[2 + i * 2]
                    self.device_config.append((port_idx, device_type))
                    
                    port_name = PORT_NAMES[port_idx] if port_idx < len(PORT_NAMES) else "?"
                    device_name = DEVICE_TYPES.get(device_type, "Unknown")
                    self.get_logger().info(f"  Port {port_name}: {device_name}")
            
            self.config_received.set()
    
    def _setup_dynamic_interfaces(self):
        """Create interfaces based on detected devices"""
        with self.config_lock:
            for port_idx, device_type in self.device_config:
                port_name = PORT_NAMES[port_idx].lower()
                device_type_name = DEVICE_TYPES.get(device_type, 'NONE')
                
                if device_type_name == 'MOTOR':
                    self._setup_motor_interface(port_name, port_idx)
                elif device_type_name == 'COLOR':
                    self._setup_color_interface(port_name, port_idx)
                elif device_type_name == 'ULTRASONIC':
                    self._setup_ultrasonic_interface(port_name, port_idx)
        
        self.get_logger().info("Dynamic interfaces created successfully!")
        
        status_msg = String()
        status_msg.data = "ready"
        self.status_pub.publish(status_msg)
    
    def _setup_motor_interface(self, port_name, port_idx):
        """Setup motor interfaces"""
        prefix = f'hub/motor_{port_name}'
        
        # Publishers
        self.publishers[f'{prefix}/state'] = self.create_publisher(JointState, f'{prefix}/state', 10)
        
        # Subscribers with lambdas that capture port_idx
        self.subscribers[f'{prefix}/dc_power'] = self.create_subscription(
            Float32, f'{prefix}/dc_power',
            lambda msg, idx=port_idx: self._motor_dc_callback(msg, idx), 10
        )
        
        self.subscribers[f'{prefix}/velocity'] = self.create_subscription(
            Float32, f'{prefix}/velocity',
            lambda msg, idx=port_idx: self._motor_velocity_callback(msg, idx), 10
        )
        
        self.subscribers[f'{prefix}/position'] = self.create_subscription(
            Vector3, f'{prefix}/position',
            lambda msg, idx=port_idx: self._motor_position_callback(msg, idx), 10
        )
        
        self.subscribers[f'{prefix}/run_angle'] = self.create_subscription(
            Vector3, f'{prefix}/run_angle',
            lambda msg, idx=port_idx: self._motor_run_angle_callback(msg, idx), 10
        )
        
        # Services
        self.services[f'{prefix}/stop'] = self.create_service(
            Trigger, f'{prefix}/stop',
            lambda req, resp, idx=port_idx: self._motor_stop_callback(req, resp, idx)
        )
        
        self.services[f'{prefix}/brake'] = self.create_service(
            Trigger, f'{prefix}/brake',
            lambda req, resp, idx=port_idx: self._motor_brake_callback(req, resp, idx)
        )
        
        self.services[f'{prefix}/hold'] = self.create_service(
            Trigger, f'{prefix}/hold',
            lambda req, resp, idx=port_idx: self._motor_hold_callback(req, resp, idx)
        )
        
        self.services[f'{prefix}/reset_angle'] = self.create_service(
            Trigger, f'{prefix}/reset_angle',
            lambda req, resp, idx=port_idx: self._motor_reset_callback(req, resp, idx)
        )
        
        self.get_logger().info(f"  Created motor interface: {prefix}")
    
    def _setup_color_interface(self, port_name, port_idx):
        """Setup publishers for color sensor"""
        prefix = f'hub/color_{port_name}'
        
        self.publishers[f'{prefix}/color'] = self.create_publisher(
            ColorRGBA, f'{prefix}/color', 10
        )
        self.publishers[f'{prefix}/reflection'] = self.create_publisher(
            Int32, f'{prefix}/reflection', 10
        )
        self.publishers[f'{prefix}/hsv'] = self.create_publisher(
            Vector3, f'{prefix}/hsv', 10
        )
        
        self.get_logger().info(f"  Created color sensor interface: {prefix}")
    
    def _setup_ultrasonic_interface(self, port_name, port_idx):
        """Setup ultrasonic sensor interfaces"""
        prefix = f'hub/ultrasonic_{port_name}'
        
        self.publishers[f'{prefix}/distance'] = self.create_publisher(Range, f'{prefix}/distance', 10)
        
        self.get_logger().info(f"  Created ultrasonic sensor interface: {prefix}")
    
    def _setup_force_interface(self, port_name, port_idx):
        """Setup force sensor interfaces"""
        prefix = f'hub/force_{port_name}'
        
        self.publishers[f'{prefix}/force'] = self.create_publisher(Float32, f'{prefix}/force', 10)
        self.publishers[f'{prefix}/pressed'] = self.create_publisher(Bool, f'{prefix}/pressed', 10)
        
        self.get_logger().info(f"  Created force sensor interface: {prefix}")
    
    def _parse_sensor_data(self, data):
        """Parse sensor data"""
        try:
            num_floats = len(data) // 4
            if num_floats == 0:
                return
            
            values = struct.unpack(f'>{num_floats}f', data[:num_floats * 4])
            idx = 0
            
            # IMU (9 floats)
            if num_floats < 10:
                return
            
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'hub_imu'
            imu_msg.linear_acceleration.x = values[idx] / 100.0
            imu_msg.linear_acceleration.y = values[idx+1] / 100.0
            imu_msg.linear_acceleration.z = values[idx+2] / 100.0
            imu_msg.angular_velocity.x = values[idx+3] * 0.0174533
            imu_msg.angular_velocity.y = values[idx+4] * 0.0174533
            imu_msg.angular_velocity.z = values[idx+5] * 0.0174533
            self.imu_pub.publish(imu_msg)
            idx += 9
            
            # Battery (1 float)
            battery_msg = Float32()
            battery_msg.data = values[idx] / 1000.0
            self.battery_pub.publish(battery_msg)
            idx += 1
            
            # Device-specific data
            with self.config_lock:
                for port_idx, device_type in self.device_config:
                    port_name = PORT_NAMES[port_idx].lower()
                    device_type_name = DEVICE_TYPES.get(device_type, 'NONE')
                    
                    if device_type_name == 'MOTOR' and idx + 2 < num_floats:
                        angle = values[idx]
                        speed = values[idx+1]
                        load = values[idx+2]
                        
                        pub_key = f'hub/motor_{port_name}/state'
                        if pub_key in self.publishers:
                            joint_msg = JointState()
                            joint_msg.header.stamp = self.get_clock().now().to_msg()
                            joint_msg.name = [f'motor_{port_name}']
                            joint_msg.position = [angle * 0.0174533]
                            joint_msg.velocity = [speed * 0.0174533]
                            joint_msg.effort = [load]
                            self.publishers[pub_key].publish(joint_msg)
                        
                        idx += 3
                    
                    elif device_type_name == 'COLOR' and idx + 4 < num_floats:
                        color_val = int(values[idx])
                        reflection = int(values[idx+1])
                        h = values[idx+2]  # Hue (0-359)
                        s = values[idx+3]  # Saturation (0-100)
                        v = values[idx+4]  # Value/Brightness (0-100)
                        
                        prefix = f'hub/color_{port_name}'
                        
                        if f'{prefix}/color' in self.publishers and color_val >= 0:
                            color_msg = ColorRGBA()
                            # Map Pybricks color enum to normalized RGB
                            color_map = {
                                -1: (0.0, 0.0, 0.0),  # None
                                0: (0.0, 0.0, 0.0),   # Black
                                1: (0.5, 0.0, 0.5),   # Violet
                                3: (0.0, 0.0, 1.0),   # Blue
                                4: (0.0, 1.0, 1.0),   # Cyan
                                5: (0.0, 1.0, 0.0),   # Green
                                6: (1.0, 1.0, 0.0),   # Yellow
                                8: (1.0, 0.5, 0.0),   # Orange
                                9: (1.0, 0.0, 0.0),   # Red
                                7: (1.0, 1.0, 1.0),   # White
                            }
                            rgb_mapped = color_map.get(color_val, (0.5, 0.5, 0.5))
                            color_msg.r, color_msg.g, color_msg.b = rgb_mapped
                            color_msg.a = 1.0
                            self.publishers[f'{prefix}/color'].publish(color_msg)
                        
                        if f'{prefix}/reflection' in self.publishers:
                            refl_msg = Int32()
                            refl_msg.data = reflection
                            self.publishers[f'{prefix}/reflection'].publish(refl_msg)
                        
                        if f'{prefix}/hsv' in self.publishers:
                            hsv_msg = Vector3()
                            hsv_msg.x = h
                            hsv_msg.y = s
                            hsv_msg.z = v
                            self.publishers[f'{prefix}/hsv'].publish(hsv_msg)
                        
                        idx += 5
                    
                    elif device_type_name == 'ULTRASONIC' and idx < num_floats:
                        distance = values[idx]
                        
                        pub_key = f'hub/ultrasonic_{port_name}/distance'
                        if pub_key in self.publishers and distance >= 0:
                            range_msg = Range()
                            range_msg.header.stamp = self.get_clock().now().to_msg()
                            range_msg.header.frame_id = f'ultrasonic_{port_name}'
                            range_msg.radiation_type = Range.ULTRASOUND
                            range_msg.min_range = 0.01
                            range_msg.max_range = 2.5
                            range_msg.range = distance / 1000.0
                            self.publishers[pub_key].publish(range_msg)
                        
                        idx += 1
        
        except Exception as e:
            self.get_logger().error(f"Error parsing sensor data: {e}")
    
    async def _communication_loop(self):
        """Main communication loop"""
        while self.connected and rclpy.ok():
            try:
                cmd = await asyncio.wait_for(self.command_queue.get(), timeout=0.1)
                await self._send_command(cmd)
            except asyncio.TimeoutError:
                pass
            
            await asyncio.sleep(0.01)
    
    async def _send_command(self, command_data):
        """Send command to hub"""
        if not self.connected or not self.client:
            return
        
        msg = b"CMD" + struct.pack('>H', len(command_data)) + command_data
        payload = bytes([0x00]) + msg
        await self.client.write_gatt_char(self.PYBRICKS_COMMAND_UUID, payload)
    
    def _queue_command(self, command_data):
        """Queue command (thread-safe)"""
        if self.loop and self.command_queue:
            asyncio.run_coroutine_threadsafe(self.command_queue.put(command_data), self.loop)
    
    # Motor callback methods
    def _motor_dc_callback(self, msg, port_idx):
        """DC power control"""
        cmd_byte = compute_command_byte(port_idx, MotorCommand.DC_POWER)
        cmd = struct.pack('>Bh', cmd_byte, int(msg.data))
        self._queue_command(cmd)
    
    def _motor_velocity_callback(self, msg, port_idx):
        """Velocity control"""
        cmd_byte = compute_command_byte(port_idx, MotorCommand.RUN_SPEED)
        cmd = struct.pack('>Bh', cmd_byte, int(msg.data))
        self._queue_command(cmd)
    
    def _motor_position_callback(self, msg, port_idx):
        """Position control (x=angle, y=speed)"""
        cmd_byte = compute_command_byte(port_idx, MotorCommand.RUN_TARGET)
        angle = int(msg.x)
        speed = int(msg.y) if msg.y != 0 else 200
        cmd = struct.pack('>BiH', cmd_byte, angle, speed)
        self._queue_command(cmd)
    
    def _motor_run_angle_callback(self, msg, port_idx):
        """Run for angle (x=angle, y=speed)"""
        cmd_byte = compute_command_byte(port_idx, MotorCommand.RUN_ANGLE)
        angle = int(msg.x)
        speed = int(msg.y) if msg.y != 0 else 200
        cmd = struct.pack('>BiH', cmd_byte, angle, speed)
        self._queue_command(cmd)
    
    def _motor_stop_callback(self, request, response, port_idx):
        """Stop motor"""
        cmd_byte = compute_command_byte(port_idx, MotorCommand.STOP)
        cmd = struct.pack('>BB', cmd_byte, 0)
        self._queue_command(cmd)
        response.success = True
        response.message = f"Motor {PORT_NAMES[port_idx]} stopped"
        return response
    
    def _motor_brake_callback(self, request, response, port_idx):
        """Brake motor"""
        cmd_byte = compute_command_byte(port_idx, MotorCommand.STOP)
        cmd = struct.pack('>BB', cmd_byte, 1)
        self._queue_command(cmd)
        response.success = True
        response.message = f"Motor {PORT_NAMES[port_idx]} braking"
        return response
    
    def _motor_hold_callback(self, request, response, port_idx):
        """Hold motor"""
        cmd_byte = compute_command_byte(port_idx, MotorCommand.STOP)
        cmd = struct.pack('>BB', cmd_byte, 2)
        self._queue_command(cmd)
        response.success = True
        response.message = f"Motor {PORT_NAMES[port_idx]} holding"
        return response
    
    def _motor_reset_callback(self, request, response, port_idx):
        """Reset motor angle"""
        cmd_byte = compute_command_byte(port_idx, MotorCommand.RESET_ANGLE)
        cmd = struct.pack('>Bi', cmd_byte, 0)
        self._queue_command(cmd)
        response.success = True
        response.message = f"Motor {PORT_NAMES[port_idx]} angle reset"
        return response
    
    # Global callback methods
    def led_callback(self, msg):
        """LED control"""
        cmd = struct.pack('>BBBB', GlobalCommand.LED_ON,
                         int(msg.r * 255), int(msg.g * 255), int(msg.b * 255))
        self._queue_command(cmd)
    
    def stop_all_callback(self, request, response):
        """Stop all motors"""
        cmd = struct.pack('>B', GlobalCommand.STOP_ALL)
        self._queue_command(cmd)
        response.success = True
        response.message = "All motors stopped"
        return response
    
    def brake_all_callback(self, request, response):
        """Brake all motors"""
        cmd = struct.pack('>B', GlobalCommand.BRAKE_ALL)
        self._queue_command(cmd)
        response.success = True
        response.message = "All motors braking"
        return response
    
    def hold_all_callback(self, request, response):
        """Hold all motors"""
        cmd = struct.pack('>B', GlobalCommand.HOLD_ALL)
        self._queue_command(cmd)
        response.success = True
        response.message = "All motors holding"
        return response
    
    def request_config_callback(self, request, response):
        """Request configuration"""
        cmd = struct.pack('>B', GlobalCommand.REQUEST_CONFIG)
        self._queue_command(cmd)
        response.success = True
        response.message = "Configuration requested"
        return response
    
    def ping_callback(self, request, response):
        """Ping hub"""
        cmd = struct.pack('>B', GlobalCommand.PING)
        self._queue_command(cmd)
        response.success = True
        response.message = "Ping sent"
        return response
    
    def reconnect_callback(self, request, response):
        """Trigger reconnection"""
        response.success = not self.connected
        response.message = "Reconnection queued" if response.success else "Already connected"
        return response
    
    def destroy_node(self):
        """Cleanup"""
        if self.connected:
            cmd = struct.pack('>B', GlobalCommand.SHUTDOWN)
            self._queue_command(cmd)
        
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LegoHubNode()
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()