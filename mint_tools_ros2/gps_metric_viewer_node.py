from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from pyproj import Transformer
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix


class MetricViewer(Node):
    """ROS2 node to visualize the localization results in metric coordinates"""

    def __init__(self):
        """A constructor"""
        super().__init__("GPS_Metric_Viewer")
        # Parse command line arguments
        raw_topic_name_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="The topic name for the raw data",
        )
        self.raw_topic_name = (
            self.declare_parameter(
                name="raw_topic_name",
                value="/ublox/fix",
                descriptor=raw_topic_name_descriptor,
            )
            .get_parameter_value()
            .string_value
        )
        localizer_topic_name_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="The topic name for the applied localization algorithm",
        )
        self.localizer_topic_name = (
            self.declare_parameter(
                name="localizer_topic_name",
                value="/surface_gps/pose",
                descriptor=localizer_topic_name_descriptor,
            )
            .get_parameter_value()
            .string_value
        )
        self.buffer_size = (
            self.declare_parameter("buffer_size", 100)
            .get_parameter_value()
            .integer_value
        )

        self.raw_buffer = deque([], maxlen=self.buffer_size)
        self.localizer_buffer = deque([], maxlen=self.buffer_size)

        # Create subscriber
        self.raw_subscriber = self.create_subscription(
            NavSatFix, self.raw_topic_name, self.raw_callback, 10
        )
        self.localizer_subscriber = self.create_subscription(
            PoseStamped, self.localizer_topic_name, self.localizer_callback, 10
        )
        self.get_logger().info(
            f"Subscribed to the topic for the raw data: {self.raw_topic_name}"
        )
        self.get_logger().info(
            f"Subscribed to the topic for the applied localization algorithm: {self.localizer_topic_name}"
        )

        self.origin = ()
        self.initialize_plot()

    def initialize_plot(self):
        """Initialize the plot"""
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.line_width = 2
        self.orientation_step = 100
        self.orientation_length = 0.2
        self.orientation_width = 0.02
        self.orientation_alpha = 0.5

        self.raw_plot, = self.ax.plot([], [], "b.", linewidth=self.line_width, label="Raw GPS data")
        self.localizer_plot, = self.ax.plot([], [], "r-", linewidth=self.line_width, label="Localized data")

        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.grid(True)
        self.ax.axis("equal")
        self.ax.legend()
        plt.show()

        self._epsg_convertor = Transformer.from_crs('EPSG:4326', 'EPSG:5186')

    def raw_callback(self, raw_msg: NavSatFix):
        """A callback function to process the raw GPS data"""
        if not self.origin:
            self.origin = (raw_msg.latitude, raw_msg.longitude)
            y, x = self._epsg_convertor.transform(*self.origin)
            self._epsg_offset = np.array([x, y, 0])

        y, x = self._epsg_convertor.transform(raw_msg.latitude, raw_msg.longitude)
        self.raw_buffer.append(
            (
                raw_msg.header.stamp.sec + raw_msg.header.stamp.nanosec * 1e-9,
                x - self._epsg_offset[0],
                y - self._epsg_offset[1],
                raw_msg.altitude,
            )
        )
        self.visualize_pose()

    def localizer_callback(self, msg: PoseStamped):
        """A callback function to process the localization results"""
        self.localizer_buffer.append(
            (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            )
        )
        print(self.localizer_buffer[-1])
        self.visualize_pose()

    def visualize_pose(self):
        """Visualize the localization results"""
        # Update raw data
        if len(self.raw_buffer) > 0:
            data_x, data_y = zip(*[(item[1], item[2]) for item in self.raw_buffer])
            self.raw_plot.set_data(data_x, data_y)

        # Update localized data
        if len(self.localizer_buffer) > 0:
            localizer_x, localizer_y = zip(*[(item[1], item[2]) for item in self.localizer_buffer])
            self.localizer_plot.set_data(localizer_x, localizer_y)


        # Draw the 2D XY plot
        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.01)

        # Draw the 2D time-Z plot


def main():
    rclpy.init()
    node = MetricViewer()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
