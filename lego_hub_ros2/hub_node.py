 #!/usr/bin/env python3

"""ROS 2 node for LEGO Robot Inventor Hub interface via Bluetooth."""

import asyncio
import struct
from threading import Event, Thread

from bleak import BleakClient, BleakScanner
from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, Range
from std_msgs.msg import ColorRGBA, Float32, Int32
from std_srvs.srv import Trigger


DEVICE_NAMES = {0: "NONE", 1: "MOTOR", 2: "COLOR", 3: "ULTRASONIC"}
PORT_NAMES = ["A", "B", "C", "D", "E", "F"]
COMMAND_UUID = "c5f50002-8280-46da-89f4-6d8051e4aeef"


class LegoHubNode(Node):
    """ROS 2 node for interfacing with LEGO Robot Inventor Hub."""

    def __init__(self):
        """Initialize the LEGO Hub node."""
        super().__init__("lego_hub_node")

        self.declare_parameter("hub_name", "Pybricks Hub")
        self.hub_name = self.get_parameter("hub_name").value

        self.device_config = []
        self.config_received = Event()
        self.pubs = {}
        self.client = None
        self.connected = False
        self.loop = None
        self.cmd_queue = None

        # Static publishers
        self.imu_pub = self.create_publisher(Imu, "hub/imu", 10)
        self.battery_pub = self.create_publisher(Float32, "hub/battery", 10)

        # Static subscribers
        self.create_subscription(ColorRGBA, "hub/led", self._led_callback, 10)

        # Static services
        self.create_service(
            Trigger,
            "hub/stop_all",
            lambda req, res: self._service_response(res, 0x70, "Motors stopped"),
        )

        Thread(target=self._bluetooth_thread, daemon=True).start()
        self.get_logger().info(f"Searching for '{self.hub_name}'...")

    def _bluetooth_thread(self):
        """Run Bluetooth operations in separate thread."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.cmd_queue = asyncio.Queue()
        self.loop.run_until_complete(self._bluetooth_main())

    async def _bluetooth_main(self):
        """Main Bluetooth connection loop."""
        while rclpy.ok():
            try:
                device = await BleakScanner.find_device_by_name(
                    self.hub_name, timeout=10.0
                )
                if not device:
                    await asyncio.sleep(5)
                    continue

                self.client = BleakClient(
                    device, disconnected_callback=self._on_disconnect
                )
                await self.client.connect()
                await self.client.start_notify(COMMAND_UUID, self._handle_data)

                self.connected = True
                self.get_logger().info("Connected! Press hub button to start.")

                await asyncio.wait_for(
                    self.loop.run_in_executor(None, self.config_received.wait),
                    timeout=10.0,
                )
                self._setup_dynamic_interfaces()

                while self.connected and rclpy.ok():
                    try:
                        cmd = await asyncio.wait_for(
                            self.cmd_queue.get(), timeout=0.1
                        )
                        msg = b"CMD" + struct.pack(">H", len(cmd)) + cmd
                        await self.client.write_gatt_char(
                            COMMAND_UUID, bytes([0x00]) + msg
                        )
                    except asyncio.TimeoutError:
                        pass

            except Exception as e:
                self.get_logger().error(f"Connection error: {e}")
                self.connected = False

            await asyncio.sleep(5)

    def _on_disconnect(self, client):
        """Handle disconnection."""
        self.connected = False
        self.config_received.clear()
        self.get_logger().warn("Disconnected from hub")

    def _handle_data(self, sender, data: bytearray):
        """Handle incoming data from hub."""
        if data[0] != 0x01:
            return

        payload = data[1:]
        if len(payload) < 3:
            return

        if payload[:3] == b"CFG" and len(payload) >= 5:
            self._handle_configuration(payload)
        elif payload[:3] == b"SNS" and len(payload) >= 5:
            length = struct.unpack(">H", payload[3:5])[0]
            sensor_data = payload[5 : 5 + length]
            self._parse_sensor_data(sensor_data)

    def _handle_configuration(self, payload):
        """Parse and store device configuration."""
        length = struct.unpack(">H", payload[3:5])[0]
        cfg = payload[5 : 5 + length]

        self.device_config = []
        num_devices = cfg[0]

        for i in range(num_devices):
            if len(cfg) >= 1 + (i + 1) * 2:
                port_idx = cfg[1 + i * 2]
                dev_type = cfg[2 + i * 2]
                self.device_config.append((port_idx, dev_type))
                self.get_logger().info(
                    f"  Port {PORT_NAMES[port_idx]}: "
                    f"{DEVICE_NAMES.get(dev_type, 'Unknown')}"
                )

        self.config_received.set()

    def _setup_dynamic_interfaces(self):
        """Create topics and services based on detected devices."""
        for port_idx, dev_type in self.device_config:
            port = PORT_NAMES[port_idx].lower()

            if dev_type == 1:
                self._setup_motor_interface(port_idx, port)
            elif dev_type == 2:
                self._setup_color_interface(port)
            elif dev_type == 3:
                self._setup_ultrasonic_interface(port)

        self.get_logger().info("Ready!")

    def _setup_motor_interface(self, port_idx, port):
        """Create motor topics and services."""
        self.pubs[f"motor_{port}/state"] = self.create_publisher(
            JointState, f"hub/motor_{port}/state", 10
        )

        self.create_subscription(
            Float32,
            f"hub/motor_{port}/velocity",
            lambda msg, idx=port_idx: self._queue_command(
                struct.pack(">Bh", 0x01 + idx * 16 + 1, int(msg.data))
            ),
            10,
        )

        self.create_subscription(
            Vector3,
            f"hub/motor_{port}/position",
            lambda msg, idx=port_idx: self._queue_command(
                struct.pack(
                    ">Bih",
                    0x01 + idx * 16 + 2,
                    int(msg.x),
                    int(msg.y) if msg.y else 200,
                )
            ),
            10,
        )

        self.create_service(
            Trigger,
            f"hub/motor_{port}/stop",
            lambda req, res, idx=port_idx: self._service_response(
                res, 0x01 + idx * 16 + 4, f"Motor {PORT_NAMES[idx]} stopped"
            ),
        )

    def _setup_color_interface(self, port):
        """Create color sensor topics."""
        self.pubs[f"color_{port}/color"] = self.create_publisher(
            ColorRGBA, f"hub/color_{port}/color", 10
        )
        self.pubs[f"color_{port}/reflection"] = self.create_publisher(
            Int32, f"hub/color_{port}/reflection", 10
        )
        self.pubs[f"color_{port}/hsv"] = self.create_publisher(
            Vector3, f"hub/color_{port}/hsv", 10
        )

    def _setup_ultrasonic_interface(self, port):
        """Create ultrasonic sensor topics."""
        self.pubs[f"ultrasonic_{port}/distance"] = self.create_publisher(
            Range, f"hub/ultrasonic_{port}/distance", 10
        )

    def _parse_sensor_data(self, data):
        """Parse and publish sensor data."""
        try:
            num_floats = len(data) // 4
            values = struct.unpack(f">{num_floats}f", data[: num_floats * 4])
            idx = 0

            if num_floats < 10:
                return

            # IMU data (9 floats)
            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = "hub_imu"
            imu.linear_acceleration.x = values[idx] / 100.0
            imu.linear_acceleration.y = values[idx + 1] / 100.0
            imu.linear_acceleration.z = values[idx + 2] / 100.0
            imu.angular_velocity.x = values[idx + 3] * 0.0174533
            imu.angular_velocity.y = values[idx + 4] * 0.0174533
            imu.angular_velocity.z = values[idx + 5] * 0.0174533
            self.imu_pub.publish(imu)
            idx += 9

            # Battery voltage (1 float)
            self.battery_pub.publish(Float32(data=values[idx] / 1000.0))
            idx += 1

            # Device-specific data
            for port_idx, dev_type in self.device_config:
                port = PORT_NAMES[port_idx].lower()

                if dev_type == 1 and idx + 2 < num_floats:
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    js.name = [f"motor_{port}"]
                    js.position = [values[idx] * 0.0174533]
                    js.velocity = [values[idx + 1] * 0.0174533]
                    js.effort = [values[idx + 2]]
                    if f"motor_{port}/state" in self.pubs:
                        self.pubs[f"motor_{port}/state"].publish(js)
                    idx += 3

                elif dev_type == 2 and idx + 4 < num_floats:
                    self._publish_color_data(port, values, idx)
                    idx += 5

                elif dev_type == 3 and idx < num_floats:
                    if f"ultrasonic_{port}/distance" in self.pubs and values[idx] >= 0:
                        rng = Range()
                        rng.header.stamp = self.get_clock().now().to_msg()
                        rng.header.frame_id = f"ultrasonic_{port}"
                        rng.radiation_type = Range.ULTRASOUND
                        rng.min_range = 0.01
                        rng.max_range = 2.5
                        rng.range = values[idx] / 1000.0
                        self.pubs[f"ultrasonic_{port}/distance"].publish(rng)
                    idx += 1

        except Exception as e:
            self.get_logger().error(f"Sensor parsing error: {e}")

    def _publish_color_data(self, port, values, idx):
        """Publish color sensor data."""
        color_val = int(values[idx])

        if f"color_{port}/color" in self.pubs and color_val >= 0:
            color_map = {
                0: (0, 0, 0),
                1: (0.5, 0, 0.5),
                3: (0, 0, 1),
                4: (0, 1, 1),
                5: (0, 1, 0),
                6: (1, 1, 0),
                8: (1, 0.5, 0),
                9: (1, 0, 0),
                10: (1, 1, 1),
            }
            r, g, b = color_map.get(color_val, (0.5, 0.5, 0.5))
            self.pubs[f"color_{port}/color"].publish(ColorRGBA(r=r, g=g, b=b, a=1.0))

        if f"color_{port}/reflection" in self.pubs:
            self.pubs[f"color_{port}/reflection"].publish(
                Int32(data=int(values[idx + 1]))
            )

        if f"color_{port}/hsv" in self.pubs:
            self.pubs[f"color_{port}/hsv"].publish(
                Vector3(x=values[idx + 2], y=values[idx + 3], z=values[idx + 4])
            )

    def _queue_command(self, cmd):
        """Queue command for sending to hub."""
        if self.loop and self.cmd_queue:
            asyncio.run_coroutine_threadsafe(self.cmd_queue.put(cmd), self.loop)

    def _service_response(self, response, cmd_byte, msg):
        """Helper for service responses."""
        self._queue_command(struct.pack(">B", cmd_byte))
        response.success = True
        response.message = msg
        return response

    def _led_callback(self, msg):
        """Handle LED color command."""
        self._queue_command(
            struct.pack(
                ">BBBB", 0x80, int(msg.r * 255), int(msg.g * 255), int(msg.b * 255)
            )
        )

    def destroy_node(self):
        """Clean shutdown."""
        if self.connected:
            self._queue_command(struct.pack(">B", 0xFF))
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = LegoHubNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
