#This file attemps to parse a compressed image into a OpenCv image for 
#Image processesing and Artificial Vision

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import rclpy
from rclpy.node import Node


class SubscriberNode(Node):

    def __init__(self):
        super().__init__('image_preprocessing')

        self.topic_name = '/image_raw/compressed'
        self.subscriber = self.create_subscription(CompressedImage,
                                                   self.topic_name,
                                                   self.listener_callback,
                                                   20
                                                   )
        self.subscriber

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error: {e}")
        
        binary_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        blurred_img = cv2.GaussianBlur(binary_img, (5,5), 0)

        canny_img = cv2.Canny(blurred_img, 100, 200)
        try:
            cv2.imshow('canny', canny_img)
            cv2.waitKey(1)
        except KeyboardInterrupt:
            print("Shutting down Camera")
        


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    image_subscriber = SubscriberNode()
    rclpy.spin(image_subscriber)

    # Shutdown when done
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()