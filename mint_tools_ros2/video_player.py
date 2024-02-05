import sys
import time
import math
import rclpy
import cv2

import sensor_msgs.msg
from cv_bridge import CvBridge

def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)
    node = rclpy.create_node('video_capture_ros2')
    node_logger = node.get_logger()

    filename = 0
    if len(args) > 1:
      filename = args[1]
      try:
        filename = int(filename)
      except ValueError:
        pass
    cap = cv2.VideoCapture(filename)
    if not cap.isOpened():
      node_logger.error(f'Cannot read "{filename}"')

    topic = 'video_player/image'
    pub_img = node.create_publisher(sensor_msgs.msg.Image, topic, 1)
    pub_img_compressed = node.create_publisher(sensor_msgs.msg.CompressedImage, topic + '/compressed', 1)

    time.sleep(1)
    cvb = CvBridge()

    while rclpy.ok():
      try:
        success, img = cap.read()
        if not success:
          node_logger.info('Terminiate (due to an invalid image)')
          break
        pub_img.publish(cvb.cv2_to_imgmsg(img))
        pub_img_compressed.publish(cvb.cv2_to_compressed_imgmsg(img))
        time.sleep(1/30)
      except KeyboardInterrupt:
        node_logger.info('Terminate (due to keyboard interrupt)')
        break

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()