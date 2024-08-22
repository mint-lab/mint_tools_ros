import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter


def load_float_data(bag_file: str, topic_names: list) -> dict:
    from pathlib import Path
    from rosbags.highlevel import AnyReader

    """Read `bag_file` and extract float data named as `topic_name`.

    Parameters
    ----------
    bag_file : str
        ROS bag file
    topic_names : list
        Float data topic names

    Returns
    -------
    dict
        A dictionary of float data with the topic names as keys
    """
    file_path = Path(os.path.expanduser(bag_file))
    topic_names = [
        "/" + topic_name if not topic_name.startswith("/") else topic_name
        for topic_name in topic_names
    ]

    dataset = {}
    with AnyReader([file_path]) as reader:
        for topic_name in topic_names:
            dataset[topic_name] = []
            connections = [x for x in reader.connections if x.topic == topic_name]
            for connection, timestamp, rawdata in reader.messages(
                connections=connections
            ):
                msg = reader.deserialize(rawdata, connection.msgtype)
                time_sec = timestamp / 10**9
                if connection.msgtype == "sensor_msgs/msg/FluidPressure":
                    if "zed" in topic_name:
                        msg.fluid_pressure /= 100
                    dataset[topic_name].append(
                        (time_sec, msg.fluid_pressure, msg.variance)
                    )
                elif connection.msgtype == "sensor_msgs/msg/Imu":
                    dataset[topic_name].append(
                        (
                            time_sec,
                            np.array(
                                [
                                    msg.orientation.x,
                                    msg.orientation.y,
                                    msg.orientation.z,
                                    msg.orientation.w,
                                ]
                            ),
                            np.array(msg.orientation_covariance),
                        )
                    )
                elif connection.msgtype == "sensor_msgs/msg/MagneticField":
                    dataset[topic_name].append(
                        (
                            time_sec,
                            np.array(
                                [
                                    msg.magnetic_field.x,
                                    msg.magnetic_field.y,
                                    msg.magnetic_field.z,
                                ]
                            ),
                            np.array(msg.magnetic_field_covariance),
                        )
                    )
                elif connection.msgtype == "geometry_msgs/msg/TwistStamped":
                    dataset[topic_name].append(
                        (
                            time_sec,
                            np.array(
                                [
                                    msg.twist.linear.x,
                                    msg.twist.linear.y,
                                    msg.twist.linear.z,
                                ]
                            ),
                            np.array(
                                [
                                    msg.twist.angular.x,
                                    msg.twist.angular.y,
                                    msg.twist.angular.z,
                                ]
                            ),
                        )
                    )
                elif connection.msgtype == "sensor_msgs/msg/Temperature":
                    dataset[topic_name].append(
                        (time_sec, msg.temperature, msg.variance)
                    )

    return dataset


if __name__ == "__main__":
    topic_dict = {
        "RTK GPS velocity (m/s)": "/ublox/vel",
        "Normal GPS velocity (m/s)": "/ascen/vel",
        "AHRS quaternion (rad)": "/imu/data",
        # "AHRS magnetic field (T)": "/imu/mag",
        "ZED pressure (Pa)": "/zed/zed_node/atm_press",
        "ZED quaternion (rad)": "/zed/zed_node/imu/data",
        "ZED magnetic field (T)": "/zed/zed_node/imu/mag",
        "Barometer pressure (hPa)": "/magic_wand/pressure",
        "Barometer temperature (\N{DEGREE SIGN}C)": "/magic_wand/temperature",
    }

    visualization_options = {
        "line_width": 2,
        "line_color": "blue",
    }

    dataset = load_float_data(
        "~/HYWC_linear_0819_2/",
        list(topic_dict.values()),
    )

    # Draw float data
    for key, value in topic_dict.items():
        fig, ax = plt.subplots()

        ax.set_xlabel("Time [sec]")
        ax.set_ylabel(key)
        ax.set_title(value)
        ax.grid(color="gray", linestyle="--", alpha=0.2)

        if "pressure" in key or "temperature" in key:
            ax.plot(
                [time for time, *_ in dataset[value]],
                [data for _, data, _ in dataset[value]],
                color=visualization_options["line_color"],
                linewidth=visualization_options["line_width"],
            )
            ax.yaxis.set_major_locator(MultipleLocator(0.1))
            ax.yaxis.set_major_formatter(FormatStrFormatter("%4.1f"))
            ax.yaxis.set_minor_locator(MultipleLocator(0.02))

        elif "quaternion" in key:
            ax.plot(
                [time for time, *_ in dataset[value]],
                [x for _, (x, y, z, w), _ in dataset[value]],
                label="x",
                color="red",
                linewidth=visualization_options["line_width"],
            )
            ax.plot(
                [time for time, *_ in dataset[value]],
                [y for _, (x, y, z, w), _ in dataset[value]],
                label="y",
                color="green",
                linewidth=visualization_options["line_width"],
            )
            ax.plot(
                [time for time, *_ in dataset[value]],
                [z for _, (x, y, z, w), _ in dataset[value]],
                label="z",
                color="blue",
                linewidth=visualization_options["line_width"],
            )
            ax.plot(
                [time for time, *_ in dataset[value]],
                [w for _, (x, y, z, w), _ in dataset[value]],
                label="w",
                color="black",
                linewidth=visualization_options["line_width"],
            )
            ax.yaxis.set_major_locator(MultipleLocator(0.1))
            ax.yaxis.set_major_formatter(FormatStrFormatter("%3.2f"))
            ax.yaxis.set_minor_locator(MultipleLocator(0.05))
            ax.legend()

        elif "magnetic" in key:
            ax.plot(
                [time for time, *_ in dataset[value]],
                [x for _, (x, y, z), _ in dataset[value]],
                label="x",
                color="red",
                linewidth=visualization_options["line_width"],
            )
            ax.plot(
                [time for time, *_ in dataset[value]],
                [y for _, (x, y, z), _ in dataset[value]],
                label="y",
                color="green",
                linewidth=visualization_options["line_width"],
            )
            ax.plot(
                [time for time, *_ in dataset[value]],
                [z for _, (x, y, z), _ in dataset[value]],
                label="z",
                color="blue",
                linewidth=visualization_options["line_width"],
            )
            ax.yaxis.set_major_locator(MultipleLocator(0.0005))
            ax.yaxis.set_major_formatter(FormatStrFormatter("%5.4f"))
            ax.yaxis.set_minor_locator(MultipleLocator(0.0001))
            ax.legend()

        elif "velocity" in key:
            ax.plot(
                [time for time, *_ in dataset[value]],
                [x for _, (x, y, z), *_ in dataset[value]],
                label="linear_x",
                color="red",
                linewidth=visualization_options["line_width"],
            )
            ax.plot(
                [time for time, *_ in dataset[value]],
                [y for _, (x, y, z), *_ in dataset[value]],
                label="linear_y",
                color="green",
                linewidth=visualization_options["line_width"],
            )
            ax.yaxis.set_major_locator(MultipleLocator(0.5))
            ax.yaxis.set_major_formatter(FormatStrFormatter("%3.1f"))
            ax.yaxis.set_minor_locator(MultipleLocator(0.1))
            ax.legend()

        plt.tight_layout()

    plt.show()
