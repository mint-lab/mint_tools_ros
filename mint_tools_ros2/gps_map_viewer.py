from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
import folium
import argparse


def load_gps_data(bag_file, topic_name):
    ''' Read `bag_file` and extract GPS data named as `topic_name`'''
    bag_path = Path(bag_file)
    if not topic_name.startswith('/'):
        topic_name = '/' + topic_name

    gps_data = []
    with AnyReader([bag_path], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            gps_data.append((timestamp, msg.latitude, msg.longitude, msg.altitude, msg.position_covariance, msg.status.status))
    return gps_data


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='A GPS data viewer on OpenStreetMap')
    parser.add_argument('ros_bag_file',         type=str,   help='ROS bag file')
    parser.add_argument('ros_topic_name',       type=str,   help='GPS topic name')
    parser.add_argument('--output_file',  '-o', type=str,   default='gps_map_viewer.html', help='Output HTML file')
    parser.add_argument('--zoom_level',   '-z', type=int,   default=18, help='The initial zoom level of the map')
    parser.add_argument('--line_color',   '-c', type=str,   default='red', help='GPS line color')
    parser.add_argument('--line_weight',  '-w', type=float, default=2, help='GPS line weight')
    args = parser.parse_args()

    # Load GPS data from a ROS bag file
    gps_data = load_gps_data(args.ros_bag_file, args.ros_topic_name)

    # Draw `gps_data` on a map
    latlon = [(lat, lon) for _, lat, lon, *_ in gps_data]
    map = folium.Map(location=latlon[0], zoom_start=args.zoom_level)
    folium.PolyLine(latlon, color=args.line_color, weight=args.line_weight, opacity=1).add_to(map)
    map.save(args.output_file)