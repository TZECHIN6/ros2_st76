# ***************************************************************************************
# *
# *    Author: TzeChing Luk
# *    Date: 2023-10-18
# *
# ***************************************************************************************

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8MultiArray

import struct

class ST76Node(Node):
    def __init__(self):
        super().__init__('st76_node')
        self.buffer = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # 9 bytes length read data
        self.serial_subscriptions = self.create_subscription(
            UInt8MultiArray, '/serial_read', self.serial_read_callback, 100)
        self.serial_subscriptions
        self.serial_publisher = self.create_publisher(UInt8MultiArray, 'serial_write', 10)
        self.encoder_publisher = self.create_publisher(Float32, '/encoder_read', 10)
        timer_period = 0.1  # [s]
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def calculate_crc16(self, data: list[int]) -> list:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001  # 0xA001 is the bit-reversed form of the polynomial 0x8005
                else:
                    crc >>= 1
        crc_bytes = [(crc & 0xFF), ((crc >> 8) & 0xFF)]
        return crc_bytes
    
    def serial_read_callback(self, msg: UInt8MultiArray):
        for data in msg.data:
            self.buffer.pop(0)
            self.buffer.append(data)
            crc = self.calculate_crc16(self.buffer[:-2])
            # If it is a valid byte array, parse the payload
            if self.buffer[7] == crc[0] and self.buffer[8] == crc[1]:
                payload = struct.unpack('!i', bytes(self.buffer[3:7]))[0]
                msg_encoder = Float32()
                msg_encoder.data = float(payload)/100.0  # 2 decimals
                self.encoder_publisher.publish(msg_encoder)
        return
    
    def timer_callback(self):
        msg = UInt8MultiArray()
        command = struct.pack('6B', 0x01, 0x03, 0x00, 0x21, 0x00, 0x02)
        crc = self.calculate_crc16(command)
        msg.data = command + bytes(crc)
        self.serial_publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    st76_node = ST76Node()

    rclpy.spin(st76_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    st76_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()