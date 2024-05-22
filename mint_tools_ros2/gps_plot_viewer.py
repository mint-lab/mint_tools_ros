import numpy as np
import matplotlib.pyplot as plt
from gps_map_viewer import load_gps_data
from pyproj import Transformer
import argparse


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='GPS data plots in metric coordinates')
    parser.add_argument('ros_bag_file',         type=str,   help='ROS bag file')
    parser.add_argument('ros_topic_name',       type=str,   help='GPS topic name')
    parser.add_argument('--epsg_latlon',  '-l', type=str,   default='EPSG:4326', help='EPSG code of GPS data (source)')
    parser.add_argument('--epsg_metric',  '-m', type=str,   default='EPSG:5186', help='EPSG code of metric data (target)')
    parser.add_argument('--line_color',   '-c', type=str,   default='red', help='GPS line color')
    parser.add_argument('--line_weight',  '-w', type=float, default=2, help='GPS line weight')
    args = parser.parse_args()

    # Load GPS data from a ROS bag file
    gps_data = load_gps_data(args.ros_bag_file, args.ros_topic_name)

    # Convert GPS data from `epsg_latlon` to `epsg_metric`
    transformer = Transformer.from_crs(args.epsg_latlon, args.epsg_metric)
    xyz_data = np.array([((time,) + transformer.transform(lat, lon)[::-1] + (alt,)) for time, lat, lon, alt, *_ in gps_data])
    xyz_data -= xyz_data[0] # Set the first data as the origin

    # Draw the 2D XY plot
    fig = plt.figure()
    plt.plot(xyz_data[:, 1], xyz_data[:, 2], color=args.line_color, linewidth=args.line_weight)
    plt.xlabel('X [m]')
    plt.ylabel('X [m]')
    plt.grid(True)
    plt.axis('equal')
    plt.tight_layout()

    # Draw the 2D time-Z plot
    fig = plt.figure()
    plt.plot(xyz_data[:, 0], xyz_data[:, 3], color=args.line_color, linewidth=args.line_weight)
    plt.xlabel('Time [s]')
    plt.ylabel('Z [m]')
    plt.grid(True)
    plt.tight_layout()

    # Draw the 3D XYZ plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xyz_data[:, 1], xyz_data[:, 2], xyz_data[:, 3], color=args.line_color, linewidth=args.line_weight)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.grid(True)
    ax.set_aspect('equal')
    plt.tight_layout()

    plt.show()