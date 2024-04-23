from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2
import rclpy
from cv_bridge import CvBridge

class DecompressImg(Node):
    def __init__(self):
        super().__init__('Img_Decompressor')
        self.declare_parameter('comp_img', '/comp_img')
        self.img_topic = self.get_parameter('comp_img').value
        self.sub_img = self.create_subscription(CompressedImage, self.img_topic, 
                                                self.decompress_callback, 10)
        self.pub_img = self.create_publisher(Image, 'decomp_img', 10)
        self.cv_bridge = CvBridge()

    def decompress_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        decomp_img = self.cv_bridge.cv2_to_imgmsg(image_np)
        self.pub_img.publish(decomp_img)

def main(args=None):
    rclpy.init(args=args)
    node = DecompressImg()
    rclpy.spin(node)

    node.destory_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()