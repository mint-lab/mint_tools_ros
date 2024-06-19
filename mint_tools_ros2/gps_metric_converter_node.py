import numpy as np
from pyproj import Transformer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

class EKF_node(Node):
    def __init__(self):
        super().__init__('gps_metric_converter_node')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox/fix',
            self.fix_callback,
            10)

        self.pub_pose = self.create_publisher(PoseStamped, '/ekf_joint', 10)
        self.init_latlong = (37.5586385, 127.04869079999999)
        self.init_position = np.array(self.conv_latlon_to_utm(*self.init_latlong))
        self.position = np.empty((0,2))

    def fix_callback(self, data):
            latitude = data.latitude
            longitude = data.longitude
            altitude = data.altitude
            self.position = np.array(self.conv_latlon_to_utm(latitude, longitude))

            rel_position = self.position - self.init_position

            nav_msg = PoseStamped()
            nav_msg.header.stamp = data.header.stamp
            nav_msg.header.frame_id = 'map'
            nav_msg.pose.position.x = float(rel_position[0])
            nav_msg.pose.position.y = float(rel_position[1])
            nav_msg.pose.position.z = 0.0
            nav_msg.pose.orientation.x = 0.
            nav_msg.pose.orientation.y = 0.
            nav_msg.pose.orientation.z = 0.
            nav_msg.pose.orientation.w = 1.
            self.pub_pose.publish(nav_msg)

    def conv_latlon_to_utm(self, latitude, longitude):
        transformer = Transformer.from_crs('EPSG:4326', 'EPSG:5186')  # EPSG:4326 = GPS 기준 좌표계, EPSG:32752 = 서울(52S) 기준 좌표계
        utm_y, utm_x = transformer.transform(latitude, longitude)
        return utm_x, utm_y

def main():
    rclpy.init()
    node = EKF_node()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
