from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2
import rclpy
from cv_bridge import CvBridge

class CompressImg(Node):
    def __init__(self):
        super().__init__('Img_Compressor')
        self.declare_parameter('origin_img', '/rgb_img')
        self.img_topic = self.get_parameter('origin_img').value
        self.sub_img = self.create_subscription(Image, self.img_topic, 
                                                self.compress_callback, 1)
        self.pub_img = self.create_publisher(CompressedImage, 'comp_img', 1)
        self.cv_bridge = CvBridge()

    def compress_callback(self, msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        comp_img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(image, dst_format="jpeg")
        self.pub_img.publish(comp_img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CompressImg()
    rclpy.spin(node)

    node.destory_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()