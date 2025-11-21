"""
Simplified ROS 2 Node for LEGO Hub Interface
"""
import asyncio
import struct
from threading import Thread, Event

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range, JointState
from std_msgs.msg import ColorRGBA, Float32, Int32
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger

from bleak import BleakScanner, BleakClient

DEVICE_NAMES = {0: 'NONE', 1: 'MOTOR', 2: 'COLOR', 3: 'ULTRASONIC'}
PORT_NAMES = ['A', 'B', 'C', 'D', 'E', 'F']
COMMAND_UUID = "c5f50002-8280-46da-89f4-6d8051e4aeef"


class LegoHubNode(Node):
    def __init__(self):
        super().__init__('lego_hub_node')
        
        self.declare_parameter('hub_name', 'Pybricks Hub')
        self.hub_name = self.get_parameter('hub_name').value
        
        self.device_config = []
        self.config_received = Event()
        self.pubs = {}
        
        # Static publishers
        self.imu_pub = self.create_publisher(Imu, 'hub/imu', 10)
        self.battery_pub = self.create_publisher(Float32, 'hub/battery_voltage', 10)
        
        # Static subscribers
        self.create_subscription(ColorRGBA, 'hub/led/command', self.led_cb, 10)
        
        # Static services
        self.create_service(Trigger, 'hub/stop_all_motors', 
                          lambda req, res: self._srv(res, 0x70, "Motors stopped"))
        
        self.client = None
        self.connected = False
        self.loop = None
        self.cmd_queue = None
        
        Thread(target=self._ble_thread, daemon=True).start()
        self.get_logger().info(f"Searching for '{self.hub_name}'...")
    
    def _ble_thread(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.cmd_queue = asyncio.Queue()
        self.loop.run_until_complete(self._ble_main())
    
    async def _ble_main(self):
        while rclpy.ok():
            try:
                device = await BleakScanner.find_device_by_name(self.hub_name, timeout=10.0)
                if not device:
                    await asyncio.sleep(5)
                    continue
                
                self.client = BleakClient(device, disconnected_callback=self._disconnected)
                await self.client.connect()
                await self.client.start_notify(COMMAND_UUID, self._handle_rx)
                
                self.connected = True
                self.get_logger().info("Connected! Press hub button.")
                
                await asyncio.wait_for(
                    self.loop.run_in_executor(None, self.config_received.wait), 
                    timeout=10.0
                )
                self._setup_interfaces()
                
                while self.connected and rclpy.ok():
                    try:
                        cmd = await asyncio.wait_for(self.cmd_queue.get(), timeout=0.1)
                        msg = b"CMD" + struct.pack('>H', len(cmd)) + cmd
                        await self.client.write_gatt_char(COMMAND_UUID, bytes([0x00]) + msg)
                    except asyncio.TimeoutError:
                        pass
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                self.connected = False
            
            await asyncio.sleep(5)
    
    def _disconnected(self, client):
        self.connected = False
        self.config_received.clear()
        self.get_logger().warn("Disconnected")
    
    def _handle_rx(self, sender, data: bytearray):
        if data[0] != 0x01:
            return
        
        payload = data[1:]
        if len(payload) < 3:
            return
        
        if payload[:3] == b"CFG" and len(payload) >= 5:
            length = struct.unpack('>H', payload[3:5])[0]
            cfg = payload[5:5+length]
            
            self.device_config = []
            num_devices = cfg[0]
            for i in range(num_devices):
                if len(cfg) >= 1 + (i + 1) * 2:
                    port_idx = cfg[1 + i * 2]
                    dev_type = cfg[2 + i * 2]
                    self.device_config.append((port_idx, dev_type))
                    self.get_logger().info(
                        f"  Port {PORT_NAMES[port_idx]}: {DEVICE_NAMES.get(dev_type, 'Unknown')}"
                    )
            self.config_received.set()
        
        elif payload[:3] == b"SNS" and len(payload) >= 5:
            length = struct.unpack('>H', payload[3:5])[0]
            sensor_data = payload[5:5+length]
            self._parse_sensors(sensor_data)
    
    def _setup_interfaces(self):
        for port_idx, dev_type in self.device_config:
            port = PORT_NAMES[port_idx].lower()
            
            if dev_type == 1:  # Motor
                self.pubs[f'motor_{port}/state'] = self.create_publisher(
                    JointState, f'hub/motor_{port}/state', 10
                )
                self.create_subscription(Float32, f'hub/motor_{port}/velocity',
                    lambda msg, idx=port_idx: self._queue(struct.pack('>Bh', 
                        0x01 + idx * 16 + 1, int(msg.data))), 10
                )
                self.create_subscription(Vector3, f'hub/motor_{port}/position',
                    lambda msg, idx=port_idx: self._queue(struct.pack('>BiH', 
                        0x01 + idx * 16 + 2, int(msg.x), int(msg.y) if msg.y else 200)), 10
                )
                self.create_service(Trigger, f'hub/motor_{port}/stop',
                    lambda req, res, idx=port_idx: self._srv(res, 0x01 + idx * 16 + 4, 
                        f"Motor {PORT_NAMES[idx]} stopped")
                )
            
            elif dev_type == 2:  # Color
                self.pubs[f'color_{port}/color'] = self.create_publisher(
                    ColorRGBA, f'hub/color_{port}/color', 10
                )
                self.pubs[f'color_{port}/reflection'] = self.create_publisher(
                    Int32, f'hub/color_{port}/reflection', 10
                )
                self.pubs[f'color_{port}/hsv'] = self.create_publisher(
                    Vector3, f'hub/color_{port}/hsv', 10
                )
            
            elif dev_type == 3:  # Ultrasonic
                self.pubs[f'ultrasonic_{port}/distance'] = self.create_publisher(
                    Range, f'hub/ultrasonic_{port}/distance', 10
                )
        
        self.get_logger().info("Ready!")
    
    def _parse_sensors(self, data):
        try:
            num_floats = len(data) // 4
            values = struct.unpack(f'>{num_floats}f', data[:num_floats * 4])
            idx = 0
            
            # IMU (9 floats)
            if num_floats < 10:
                return
            
            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = 'hub_imu'
            imu.linear_acceleration.x = values[idx] / 100.0
            imu.linear_acceleration.y = values[idx+1] / 100.0
            imu.linear_acceleration.z = values[idx+2] / 100.0
            imu.angular_velocity.x = values[idx+3] * 0.0174533
            imu.angular_velocity.y = values[idx+4] * 0.0174533
            imu.angular_velocity.z = values[idx+5] * 0.0174533
            self.imu_pub.publish(imu)
            idx += 9
            
            # Battery
            self.battery_pub.publish(Float32(data=values[idx] / 1000.0))
            idx += 1
            
            # Devices
            for port_idx, dev_type in self.device_config:
                port = PORT_NAMES[port_idx].lower()
                
                if dev_type == 1 and idx + 2 < num_floats:  # Motor
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    js.name = [f'motor_{port}']
                    js.position = [values[idx] * 0.0174533]
                    js.velocity = [values[idx+1] * 0.0174533]
                    js.effort = [values[idx+2]]
                    if f'motor_{port}/state' in self.pubs:
                        self.pubs[f'motor_{port}/state'].publish(js)
                    idx += 3
                
                elif dev_type == 2 and idx + 4 < num_floats:  # Color
                    color_val = int(values[idx])
                    if f'color_{port}/color' in self.pubs and color_val >= 0:
                        color_map = {0: (0, 0, 0), 1: (0.5, 0, 0.5), 3: (0, 0, 1),
                                   4: (0, 1, 1), 5: (0, 1, 0), 6: (1, 1, 0),
                                   8: (1, 0.5, 0), 9: (1, 0, 0), 10: (1, 1, 1)}
                        r, g, b = color_map.get(color_val, (0.5, 0.5, 0.5))
                        self.pubs[f'color_{port}/color'].publish(
                            ColorRGBA(r=r, g=g, b=b, a=1.0)
                        )
                    
                    if f'color_{port}/reflection' in self.pubs:
                        self.pubs[f'color_{port}/reflection'].publish(
                            Int32(data=int(values[idx+1]))
                        )
                    
                    if f'color_{port}/hsv' in self.pubs:
                        self.pubs[f'color_{port}/hsv'].publish(
                            Vector3(x=values[idx+2], y=values[idx+3], z=values[idx+4])
                        )
                    idx += 5
                
                elif dev_type == 3 and idx < num_floats:  # Ultrasonic
                    if f'ultrasonic_{port}/distance' in self.pubs and values[idx] >= 0:
                        rng = Range()
                        rng.header.stamp = self.get_clock().now().to_msg()
                        rng.header.frame_id = f'ultrasonic_{port}'
                        rng.radiation_type = Range.ULTRASOUND
                        rng.min_range = 0.01
                        rng.max_range = 2.5
                        rng.range = values[idx] / 1000.0
                        self.pubs[f'ultrasonic_{port}/distance'].publish(rng)
                    idx += 1
        
        except Exception as e:
            self.get_logger().error(f"Parse error: {e}")
    
    def _queue(self, cmd):
        if self.loop and self.cmd_queue:
            asyncio.run_coroutine_threadsafe(self.cmd_queue.put(cmd), self.loop)
    
    def _srv(self, response, cmd_byte, msg):
        self._queue(struct.pack('>B', cmd_byte))
        response.success = True
        response.message = msg
        return response
    
    def led_cb(self, msg):
        self._queue(struct.pack('>BBBB', 0x80, int(msg.r * 255), 
                               int(msg.g * 255), int(msg.b * 255)))
    
    def destroy_node(self):
        if self.connected:
            self._queue(struct.pack('>B', 0xFF))
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LegoHubNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()