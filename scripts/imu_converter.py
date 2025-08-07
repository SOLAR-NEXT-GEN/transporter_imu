#!/usr/bin/python3
from transporter_imu.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Header
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf_transformations
import math
import numpy as np

class BnoPublisher(Node):
    def __init__(self):
        super().__init__('bno_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.raw_data = Float64MultiArray()
        self.create_subscription(Float64MultiArray,'/bno055_data',self.imu_converter_callback,qos_profile)
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        self.create_timer(0.01, self.timer_callback)
        
        # Rotation matrix for gyro
        self.gyro_rotation_matrix = np.array([
            [0,  -1,  0],
            [-1,  0,  0],
            [0,   0,  1]
        ])
        
        # Rotation matrix for euler
        self.euler_rotation_matrix = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ])
    
    def convert_angle(self, angle):
        if angle > math.pi:
            return angle - 2 * math.pi
        return angle
    
    def apply_rotation(self, x, y, z, rotation_matrix):
        vector = np.array([x, y, z])
        rotated = rotation_matrix @ vector
        return rotated[0], rotated[1], rotated[2]
            
    def imu_converter_callback(self, msg):  
        self.raw_data = msg
        
    def timer_callback(self):
        if len(self.raw_data.data) < 16:
            return
            
        imu_msg = Imu()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        
        euler_x = self.raw_data.data[13]
        euler_y = self.raw_data.data[14] 
        euler_z = self.convert_angle(self.raw_data.data[15])
        euler_rot_x, euler_rot_y, euler_rot_z = self.apply_rotation(euler_x, euler_y, euler_z, self.euler_rotation_matrix)
        
        quaternion = tf_transformations.quaternion_from_euler(
            0.0,
            0.0,
            euler_rot_z
        )
        
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        
        gyro_x = self.raw_data.data[7]
        gyro_y = self.raw_data.data[8]
        gyro_z = self.raw_data.data[9]
        gyro_rot_x, gyro_rot_y, gyro_rot_z = self.apply_rotation(gyro_x, gyro_y, gyro_z, self.gyro_rotation_matrix)
        
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = gyro_rot_z
        
        accel_x = self.raw_data.data[4]
        accel_y = self.raw_data.data[5]
        accel_z = self.raw_data.data[6]
        
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[0] = 0.01
        
        self.imu_publisher.publish(imu_msg)
        print(f"E:[{euler_rot_x:.1f},{euler_rot_y:.1f},{euler_rot_z:.1f}] "
              f"G:[{gyro_rot_x:.1f},{gyro_rot_y:.1f},{gyro_rot_z:.1f}] "
              f"A:[{accel_x:.1f},{accel_y:.1f},{accel_z:.1f}]")

def main(args=None):
    rclpy.init(args=args)
    node = BnoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()