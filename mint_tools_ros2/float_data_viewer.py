import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter


class Colors:
    """Color class for terminal output."""

    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"
    RESET = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"
    END = "\033[0m"


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
        available_topics = list(conn.topic for conn in reader.connections)
        print(
            f"Available topics in ROS bag file:\n"
            + "\n".join(f" - {topic}" for topic in sorted(available_topics))
        )
        print()
        for topic_name in topic_names:
            if topic_name not in available_topics:
                print(
                    f"{Colors.YELLOW}{Colors.BOLD}[Warning]{Colors.BOLD}{Colors.END}"
                    f' Topic {Colors.YELLOW}{Colors.BOLD}"{topic_name}"{Colors.BOLD}{Colors.END}'
                    f" is not available in the ROS bag file."
                )
                topic_names.remove(topic_name)
        print(
            f"The topic lists to be plotted"
            f' are {Colors.GREEN}{Colors.BOLD}"{topic_names}"{Colors.BOLD}{Colors.END}'
        )

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
                elif connection.msgtype == "geometry_msgs/msg/PoseStamped":
                    dataset[topic_name].append(
                        (
                            time_sec,
                            np.array(
                                [
                                    msg.pose.position.x,
                                    msg.pose.position.y,
                                    msg.pose.position.z,
                                ]
                            ),
                            np.array(
                                [
                                    msg.pose.orientation.x,
                                    msg.pose.orientation.y,
                                    msg.pose.orientation.z,
                                    msg.pose.orientation.w,
                                ]
                            ),
                        )
                    )

    return dataset


# parsing arguments
def parse_args():
    """Parse command line arguments.

    Returns
    -------
    argparse.Namespace
        Parsed arguments
    """
    import argparse

    parser = argparse.ArgumentParser(
        description="Float data viewer (e.g. pressure, temperature, quaternion, magnetic field, velocity)"
    )
    parser.add_argument("bag_file", type=str, help="ROS bag file directory")
    parser.add_argument(
        "topic_names", type=str, nargs="+", help="Float data topic names"
    )
    return parser.parse_args()


if __name__ == "__main__":
    topic_filter = {
        "RTK GPS velocity [m/s]": "/ublox/vel",
        "Normal GPS velocity [m/s]": "/ascen/vel",
        "AHRS quaternion [rad]": "/imu/data",
        "AHRS magnetic field [uT]": "/imu/mag",
        "ZED pressure [Pa]": "/zed/zed_node/atm_press",
        "ZED quaternion [rad]": "/zed/zed_node/imu/data",
        "ZED magnetic field [uT]": "/zed/zed_node/imu/mag",
        "Old ZED pressure [hPa]": "/zed2/zed_node/pressure",
        "Old ZED quaternion [rad]": "/zed2/zed_node/imu/data",
        "Old ZED magnetic field [uT]": "/zed2/zed_node/imu/mag",
        "Barometer pressure [hPa]": "/magic_wand/pressure",
        "Barometer temperature [\N{DEGREE SIGN}C]": "/magic_wand/temperature",
        "Wire encoder position [m]": "/jjjwire_robot_position",
    }

    visualization_options = {
        "line_width": 2,
        "line_color": "blue",
    }

    # Parse arguments
    args = parse_args()
    bag_file = args.bag_file
    topic_names = args.topic_names

    print(f"* Bag_file: {bag_file}")
    print(f"* Topic_names: {topic_names}", end="\n\n")

    # Load float data
    topic_list = [
        topic_name for topic_name in topic_names if topic_name in topic_filter.values()
    ]
    dataset = load_float_data(bag_file, topic_list)

    # Draw float data
    for key, value in topic_filter.items():
        if value not in dataset.keys():
            continue
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
            for i in range(4):
                ax.plot(
                    [time - dataset[value][0][0] for time, *_ in dataset[value]],
                    [data[i] for _, data, _ in dataset[value]],
                    label=f"{['x', 'y', 'z', 'w'][i]}",
                    color=["red", "green", "blue", "black"][i],
                    linewidth=visualization_options["line_width"],
                )
            ax.yaxis.set_major_locator(MultipleLocator(0.1))
            ax.yaxis.set_major_formatter(FormatStrFormatter("%3.2f"))
            ax.yaxis.set_minor_locator(MultipleLocator(0.05))
            ax.legend()

        elif "magnetic" in key:
            for i in range(3):
                ax.plot(
                    [time - dataset[value][0][0] for time, *_ in dataset[value]],
                    [data[i] * 1e6 for _, data, _ in dataset[value]],
                    label=f"{['x', 'y', 'z'][i]}",
                    color=["red", "green", "blue"][i],
                    linewidth=visualization_options["line_width"],
                )
            ax.yaxis.set_major_locator(MultipleLocator(10))
            ax.yaxis.set_minor_locator(MultipleLocator(1))
            ax.legend()

        elif "velocity" in key:
            for i in range(3):
                ax.plot(
                    [time for time, *_ in dataset[value]],
                    [data[i] for _, data, _ in dataset[value]],
                    label=f"{['x', 'y', 'z'][i]}",
                    color=["red", "green", "blue"][i],
                    linewidth=visualization_options["line_width"],
                )

            ax.yaxis.set_major_locator(MultipleLocator(0.5))
            ax.yaxis.set_major_formatter(FormatStrFormatter("%3.1f"))
            ax.yaxis.set_minor_locator(MultipleLocator(0.1))
            ax.legend()

        elif "position" in key:
            xyz_data = np.array([data for _, data, _ in dataset[value]])

            # XY plot
            ax.plot(
                xyz_data[:, 0],
                xyz_data[:, 1],
                color=visualization_options["line_color"],
                linewidth=visualization_options["line_width"],
            )
            ax.set_xlabel("X [m]")
            ax.set_ylabel("Y [m]")
            ax.axis("equal")
            ax.grid(True)
            ax.set_title(f"{key} XY plot")

            # Time Z plot
            fig, ax = plt.subplots()
            t0 = dataset[value][0][0]
            ax.plot(
                [time - t0 for time, *_ in dataset[value]],
                xyz_data[:, 2],
                color=visualization_options["line_color"],
                linewidth=visualization_options["line_width"],
            )
            ax.set_xlabel("Time [sec]")
            ax.set_ylabel("Z [m]")
            ax.grid(True)
            ax.set_title(f"{key} Time-Z plot")

        plt.tight_layout()

    plt.show()
