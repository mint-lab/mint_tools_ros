import sys
import time
import cv2
import rclpy
from rclpy.node import Node
import sensor_msgs.msg
from cv_bridge import CvBridge

class VideoCaptureNode(Node):
    """VideoCaptureNode Class for compressed images"""
    def __init__(self, filename=0):
        super().__init__('video_capture_ros2')
        self.filename = filename
        self.cv_bridge = CvBridge()
        rgb_topic = 'video_player/image' # RGB image topic name
        gray_topic = 'video_player/gray' # Grayscale image topic name
        self.publisher_img = self.create_publisher(sensor_msgs.msg.Image, rgb_topic, 1)
        self.publisher_gray = self.create_publisher(sensor_msgs.msg.Image, gray_topic, 1)
        self.publisher_compressed_img = self.create_publisher(sensor_msgs.msg.CompressedImage, rgb_topic + '/compressed', 1)
        self.publisher_compressed_gray = self.create_publisher(sensor_msgs.msg.CompressedImage, gray_topic + '/compressed', 1)
        self.initialize_capture()

    def initialize_capture(self):
        """Initialize video capture objects"""
        self.cap = cv2.VideoCapture(self.filename)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot read "{self.filename}"')
        else:
            self.get_logger().info(f'Successfully opened "{self.filename}"')

    def run(self):
        """Run video capture node with compressed topic"""
        while rclpy.ok():
            success, img = self.cap.read()
            if not success:
                self.get_logger().info('Terminate (due to an invalid image)')
                break

            # Converts OpenCV Images(numpy.ndarray) to ROS 2 Messages(sensor_msgs.msg.Image)
            img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
            self.publisher_img.publish(img_msg)

            # Converts to compressed images
            compressed_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(img, dst_format="jpeg")
            self.publisher_compressed_img.publish(compressed_img_msg)

            # Converts to grayscale images
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray_msg = self.cv_bridge.cv2_to_imgmsg(gray_img, encoding="mono8")
            self.publisher_gray.publish(gray_msg)

            # Converts to compressed grayscale images
            compressed_gray_msg = self.cv_bridge.cv2_to_compressed_imgmsg(gray_img, dst_format="jpeg")
            self.publisher_compressed_gray.publish(compressed_gray_msg)

            time.sleep(1 / 30)

def main(args=None):
    rclpy.init(args=args)
    filename = sys.argv[1] if len(sys.argv) > 1 else 0
    video_capture_node = VideoCaptureNode(filename=filename)
    try:
        video_capture_node.run()
    except KeyboardInterrupt:
        video_capture_node.get_logger().info('Terminate (due to keyboard interrupt)')
    finally:
        video_capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
