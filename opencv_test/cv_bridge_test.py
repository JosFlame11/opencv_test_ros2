#Test for cv_bridge
#This will only publish the camera sensor into the ROS topic

import cv2

import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

class PublsiherNodeClass(Node):

    # Constructor
    def __init__(self):
        super().__init__('image_publisher')

        self.cameraDevice = 0
        self.camera = cv2.VideoCapture(self.cameraDevice)

        self.bridgeObject = CvBridge()

        self.topicName = 'camera_raw'

        self.queue_size = 20

        self.puiblisher = self.create_publisher(Image, self.topicName, self.queue_size)

        self.periodCommunication = 0.02

        # Timer
        self.timer = self.create_timer(self.periodCommunication,  self.callbackFunction)

        self.i = 0

    # Callback Function for the Timer
    def callbackFunction(self):

        success, frame = self.camera.read()

        frame = cv2.resize(frame, (820, 640), interpolation=cv2.INTER_CUBIC)

        if success == True:
            try:
                ros2_message = self.bridgeObject.cv2_to_imgmsg(frame)
            except CvBridgeError as e:
                self.get_logger().error(f"Error: {e}")
                
            self.puiblisher.publish(ros2_message)


        self.get_logger().info('Publishing image number %d' %self.i)
        self.i += 1


def main(args=None):
    #initialize rclpy
    rclpy.init(args=args)

    publisherObject = PublsiherNodeClass()

    rclpy.spin(publisherObject)

    #destroy
    publisherObject.destroy_node()

    #shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()