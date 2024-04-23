from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2
import rclpy
from cv_bridge import CvBridge

class ImageViewer(Node):
    def __init__(self):
        super().__init__('ImageViewer')
        self.declare_parameter('comp_img', '/comp_img')
        self.declare_parameter('raw_img', '/raw_img')
        self.comp_img_topic = self.get_parameter('comp_img').value
        self.origin_img_topic = self.get_parameter('raw_img').value
        self.sub_comp_img = self.create_subscription(CompressedImage, self.comp_img_topic, 
                                                self.comp_img_callback, 10)
        self.sub_comp_img = self.create_subscription(Image, self.origin_img_topic, 
                                                self.raw_img_callback, 10)
        self.pub_img = self.create_publisher(Image, 'decomp_img', 10)
        self.cv_bridge = CvBridge()

    def comp_img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow("Compressed Image", image_np)
        self.check_exit()

    def raw_img_callback(self, msg):
        if msg.encoding == 'bgr8':
            image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding == 'rgb8':
            image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)[:, :, ::-1]
        else:
            return
        cv2.imshow("Raw Image", image_np)
        self.check_exit()

    def check_exit(self):
        key = cv2.waitKey(1)
        if key==ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()