from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
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
            self.declare_parameter("buffer_size", 20)
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

        self.gps_mode = {
            "SPS": "black",
            "DGPS": "red",
            "RTK-FIX": "green",
            "RTK-FLOAT": "blue",
        }
        self.raw_colors = deque(
            [], maxlen=self.buffer_size
        )  # for expression of gps mode

        # Raw GPS data
        self.raw_plot = self.ax.scatter([], [], c=self.raw_colors)
        (self.localizer_plot,) = self.ax.plot(
            [], [], "m-", linewidth=self.line_width, label="Localized data"
        )
        self.euler_x_plot = self.ax.quiver(
            [],
            [],
            [],
            [],
            color="r",
            width=self.orientation_width,
            alpha=self.orientation_alpha,
            edgecolor=None,
        )
        self.euler_y_plot = self.ax.quiver(
            [],
            [],
            [],
            [],
            color="g",
            width=self.orientation_width,
            alpha=self.orientation_alpha,
            edgecolor=None,
        )

        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.grid(True)
        self.ax.axis("equal")

        legend_elements = [
            Line2D(
                [0],
                [0],
                marker="o",
                color="w",
                label="Raw GPS: " + key,
                markerfacecolor=value,
                markersize=10,
            )
            for key, value in self.gps_mode.items()
        ]
        self.ax.legend(
            handles=self.ax.get_legend_handles_labels()[0] + legend_elements,
            loc="upper right",
        )
        plt.show()

        self._epsg_convertor = Transformer.from_crs("EPSG:4326", "EPSG:5186")

    def raw_callback(self, raw_msg: NavSatFix):
        """A callback function to process the raw GPS data"""
        if not self.origin:
            self.origin = (raw_msg.latitude, raw_msg.longitude)
            self.get_logger().info(f"Origin: {self.origin}")
            y, x = self._epsg_convertor.transform(*self.origin)
            self._epsg_offset = np.array([x, y, 0])

        if raw_msg.status.status == 0:
            self.color = self.gps_mode["SPS"]  # black
        elif raw_msg.status.status == 1:
            self.color = self.gps_mode["DGPS"]  # red
        elif raw_msg.status.status == 2:
            # estimated position error in driver.py @ nmea_navsat_driver
            if raw_msg.position_covariance[8] < 0.02:
                self.color = self.gps_mode["RTK-FIX"]  # green
            else:
                self.color = self.gps_mode["RTK-FLOAT"]  # blue
        self.raw_colors.append(self.color)

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
        self.visualize_pose()

    def visualize_pose(self):
        """Visualize the localization results"""
        # Update raw data
        if len(self.raw_buffer) > 0:
            data_x, data_y = zip(*[(item[1], item[2]) for item in self.raw_buffer])
            self.raw_plot.set_offsets(np.c_[data_x, data_y])
            self.raw_plot.set_color(self.raw_colors)

        # Update localized data
        if len(self.localizer_buffer) > 0:
            (
                localizer_x,
                localizer_y,
                localizer_qx,
                localizer_qy,
                localizer_qz,
                localizer_qw,
            ) = zip(
                *[
                    (item[1], item[2], item[4], item[5], item[6], item[7])
                    for item in self.localizer_buffer
                ]
            )
            self.localizer_plot.set_data(localizer_x, localizer_y)

            # Update orientation
            q = [
                (qx, qy, qz, qw)
                for qx, qy, qz, qw in zip(
                    localizer_qx, localizer_qy, localizer_qz, localizer_qw
                )
            ]

            self._count = 0
            if self._count % self.orientation_step == 0:
                # R = Rotation.from_quat(q[-1]).as_matrix()
                # dx = R @ np.array([self.orientation_length, 0, 0])
                # dy = R @ np.array([0, self.orientation_length, 0])
                # plt.arrow(
                #     localizer_x[-1],
                #     localizer_y[-1],
                #     dx[0],
                #     dx[1],
                #     color="r",
                #     width=self.orientation_width,
                #     alpha=self.orientation_alpha,
                #     edgecolor=None,
                # )
                # plt.arrow(
                #     localizer_x[-1],
                #     localizer_y[-1],
                #     dy[0],
                #     dy[1],
                #     color="g",
                #     width=self.orientation_width,
                #     alpha=self.orientation_alpha,
                #     edgecolor=None,
                # )
                # self.euler_x_plot.set_offsets(
                #             (localizer_x[-1], localizer_y[-1]),
                #             (localizer_x[-1], localizer_y[-1])
                # )
                # self.euler_y_plot.set_offsets(
                #             (localizer_x[-1], localizer_y[-1]),
                #             (localizer_x[-1], localizer_y[-1])
                # )
                th_x, th_y, th_z = Rotation.from_quat(q[-1]).as_euler("xyz")
                # self.euler_x_plot.set_UVC(
                #             localizer_x[-1],
                #             localizer_y[-1],
                # )
                # self.euler_y_plot.set_UVC(
                #             localizer_x[-1],
                #             localizer_y[-1],
                # )
            self._count += 1

        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_xbound(min(data_x) - 1, max(data_x) + 1)
        self.ax.set_ybound(min(data_y) - 1, max(data_y) + 1)
        # self.ax.set_aspect("equal")

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
