# ***************************************************************************************
# *
# *    Author: TzeChing Luk
# *    Date: 2023-10-31
# *
# ***************************************************************************************

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from autoware_auto_vehicle_msgs.msg import VelocityReport


class VehicleSpeedReportPublisher(Node):
    def __init__(self):
        super().__init__('vehicle_speed_report_publisher_node')
        self.prev_encoder_value = None
        self.prev_encoder_time = None
        self.encoder_ratio = 1.0

        self.velocity_status_pub_ = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)
        self.encoder_subscription_ = self.create_subscription(Float32, '/encoder_read', self.encoder_callback, 10)
        self.encoder_subscription_
    
    def encoder_callback(self, msg: Float32):
        current_time = self.get_clock().now().to_msg()
        current_encoder_value = msg.data
        if self.prev_encoder_time == None or self.prev_encoder_value == None:
            self.prev_encoder_value = current_encoder_value
            self.prev_encoder_time = current_time
            return
        velocity_msg = VelocityReport()
        velocity_msg.header.stamp = current_time
        velocity_msg.header.frame_id = 'base_link'
        time_difference = self.time_difference(current_time, self.prev_encoder_time)
        velocity_msg.longitudinal_velocity = (current_encoder_value - self.prev_encoder_value) * self.encoder_ratio / time_difference  # [m/s]
        self.velocity_status_pub_.publish(velocity_msg)
        # Update previous time and encoder reading
        self.prev_encoder_value = current_encoder_value
        self.prev_encoder_time = current_time

    def time_difference(self, current_time , previous_time) -> float:
        current_time = current_time.sec + current_time.nanosec * 1e-9
        previous_time = previous_time.sec + previous_time.nanosec * 1e-9
        time_diff = current_time - previous_time
        return time_diff


def main(args=None):
    rclpy.init(args=args)

    vehicle_speed_report_publisher = VehicleSpeedReportPublisher()

    rclpy.spin(vehicle_speed_report_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_speed_report_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

