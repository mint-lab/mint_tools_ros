import numpy as np
import matplotlib.pyplot as plt
from gps_map_viewer import load_gps_data
from pyproj import Transformer
import argparse


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="GPS data plots in metric coordinates")
    parser.add_argument("ros_bag_file", type=str, help="ROS bag file")
    parser.add_argument("ros_topic_name", type=str, help="GPS topic name")
    parser.add_argument(
        "--epsg_latlon",
        "-l",
        type=str,
        default="EPSG:4326",
        help="EPSG code of GPS data (source)",
    )
    parser.add_argument(
        "--epsg_metric",
        "-m",
        type=str,
        default="EPSG:5186",
        help="EPSG code of metric data (target)",
    )
    parser.add_argument(
        "--line_color", "-c", type=str, default="red", help="GPS line color"
    )
    parser.add_argument(
        "--line_weight", "-w", type=float, default=2, help="GPS line weight"
    )
    parser.add_argument(
        "--alt_cov_threshold",
        "-t",
        type=float,
        default=0.014,
        help="Altitude covariance threshold",
    )
    args = parser.parse_args()

    label_fontsize = 18
    legend_fontsize = 16
    ticks_fontsize = 16

    # Load GPS data from a ROS bag file
    gps_data = load_gps_data(args.ros_bag_file, args.ros_topic_name)

    # GPS mode color map
    gps_mode = {
        "SPS": "black",
        "DGPS": "red",
        "RTK-Float": "blue",
        "RTK-Fix": "green",
    }

    # Convert GPS data from `epsg_latlon` to `epsg_metric`
    transformer = Transformer.from_crs(args.epsg_latlon, args.epsg_metric)
    # time, x, y, z, cov, mode, color
    xyz_data = np.array(
        [
            (
                (time,)
                + transformer.transform(lat, lon)[::-1]
                + (alt,)
                + (cov,)
                + (mode,)
                + (
                    (
                        gps_mode["SPS"]
                        if mode == 0
                        else (
                            gps_mode["DGPS"]
                            if mode == 1
                            else (
                                gps_mode["RTK-Fix"]
                                if mode == 2 and cov[-1] < args.alt_cov_threshold
                                else gps_mode["RTK-Float"]
                            )
                        )
                    ),
                )
            )
            for time, lat, lon, alt, cov, mode in gps_data
        ],
        dtype=object,
    )

    # Prepare the data for plotting
    xyz_data[:, 0:4] -= xyz_data[0, 0:4]  # Set the first data as the origin
    xyz_data[:, 0] /= 1e9  # Convert time from ns to s

    # Draw the 2D XY plot
    fig = plt.figure()
    plt.scatter(
        xyz_data[:, 1],
        xyz_data[:, 2],
        color=xyz_data[:, 6],
        linewidth=args.line_weight,
    )
    plt.xlabel(r"$X\,[m]$", fontsize=label_fontsize)
    plt.ylabel(r"$Y\,[m]$", fontsize=label_fontsize)
    plt.xticks(fontsize=ticks_fontsize)
    # Set num of ticks for y-axis to 5
    plt.locator_params(axis="y", nbins=5)
    plt.yticks(fontsize=ticks_fontsize)
    plt.grid(True)
    plt.title(rf"GPS data $X$-$Y$ plot")
    plt.axis("equal")
    plt.tight_layout()
    plt.legend(
        handles=[
            plt.Line2D([0], [0], marker="o", linestyle="None", color=color, label=mode)
            for mode, color in gps_mode.items()
        ],
        fontsize=legend_fontsize,
    )

    # Draw the 2D time-Z plot
    fig = plt.figure()
    plt.scatter(
        xyz_data[:, 0],
        xyz_data[:, 3],
        color=xyz_data[:, 6],
        linewidth=args.line_weight,
    )
    plt.xlabel(r"$Time\,[s]$", fontsize=label_fontsize)
    plt.ylabel(r"$Z\,[m]$", fontsize=label_fontsize)
    plt.xticks(fontsize=ticks_fontsize)
    plt.yticks(fontsize=ticks_fontsize)
    plt.grid(True)
    plt.title(rf"GPS data $Time$-$Z$ plot")
    plt.tight_layout()
    plt.legend(
        handles=[
            plt.Line2D([0], [0], marker="o", linestyle="None", color=color, label=mode)
            for mode, color in gps_mode.items()
        ],
        fontsize=legend_fontsize,
    )

    # Draw the 3D XYZ plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        xyz_data[:, 1],
        xyz_data[:, 2],
        xyz_data[:, 3],
        color=xyz_data[:, 6],
        linewidth=args.line_weight,
    )
    ax.set_xlabel(r"$X\,[m]$")
    ax.set_ylabel(r"$Y\,[m]$")
    ax.set_zlabel(r"$Z\,[m]$")
    ax.grid(True)
    ax.set_title(rf"GPS data $X$-$Y$-$Z$ plot")
    plt.tight_layout()
    ax.legend(
        handles=[
            plt.Line2D([0], [0], marker="o", linestyle="None", color=color, label=mode)
            for mode, color in gps_mode.items()
        ]
    )

    plt.show()
