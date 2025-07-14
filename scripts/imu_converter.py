#!/usr/bin/python3

from transporter_imu.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy


class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.create_subscription(Float64MultiArray,'/bno055_cubemx',self.imu_converter_callback,qos_profile)
        self.imu_publisher = self.create_publisher(Imu, '/imu_converted', 10)


    def imu_converter_callback(self, msg):  
        imu_msg = Imu()
        imu_msg.orientation.x = msg.data[0]
        imu_msg.orientation.y = msg.data[1]
        imu_msg.orientation.z = msg.data[2]
        imu_msg.orientation.w = msg.data[3]
        imu_msg.angular_velocity.x = msg.data[10]
        imu_msg.angular_velocity.y = msg.data[11]
        imu_msg.angular_velocity.z = msg.data[12]
        imu_msg.linear_acceleration.x = msg.data[7]
        imu_msg.linear_acceleration.y = msg.data[8]
        imu_msg.linear_acceleration.z = msg.data[9]

        self.imu_publisher.publish(imu_msg)




def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()