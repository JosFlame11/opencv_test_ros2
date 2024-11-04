import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class SubscriberNode(Node):

    def __init__(self):
        super().__init__('image_preprocessing')

        self.topic_name = '/image_raw/compressed'
        self.subscriber = self.create_subscription(
            CompressedImage,
            self.topic_name,
            self.listener_callback,
            20
        )
        self.bridge = CvBridge()

        # Initialize ROI settings with six points
        self.roi_top_left = [0.3, 0.55]
        self.roi_top_right = [0.7, 0.55]
        self.roi_middle_left = [0.0, 1.0]    # New middle left point
        self.roi_middle_right = [1.0, 1.0]   # New middle right point
        self.roi_bottom_left = [0.0, 1.0]
        self.roi_bottom_right = [1.0, 1.0]

        """This values apply for simulation only
        when dealing with the real robot, adjust at needed.
        Also if wanted to convert this values to the array just multiply the 
        width (first column) * value, and height (second column) * value"""    


        # Create sliders to adjust the ROI dynamically
        self.init_roi_gui()

    def init_roi_gui(self):
        # Create a window for the sliders
        cv2.namedWindow("ROI Settings")

        # Add sliders for each ROI vertex, each controlling an x or y coordinate (0 to 100 scale)
        cv2.createTrackbar("Top Left X", "ROI Settings", int(self.roi_top_left[0] * 100), 100, self.update_top_left_x)
        cv2.createTrackbar("Top Left Y", "ROI Settings", int(self.roi_top_left[1] * 100), 100, self.update_top_left_y)
        cv2.createTrackbar("Top Right X", "ROI Settings", int(self.roi_top_right[0] * 100), 100, self.update_top_right_x)
        cv2.createTrackbar("Top Right Y", "ROI Settings", int(self.roi_top_right[1] * 100), 100, self.update_top_right_y)
        cv2.createTrackbar("Middle Left X", "ROI Settings", int(self.roi_middle_left[0] * 100), 100, self.update_middle_left_x)
        cv2.createTrackbar("Middle Left Y", "ROI Settings", int(self.roi_middle_left[1] * 100), 100, self.update_middle_left_y)
        cv2.createTrackbar("Middle Right X", "ROI Settings", int(self.roi_middle_right[0] * 100), 100, self.update_middle_right_x)
        cv2.createTrackbar("Middle Right Y", "ROI Settings", int(self.roi_middle_right[1] * 100), 100, self.update_middle_right_y)
        cv2.createTrackbar("Bottom Left X", "ROI Settings", int(self.roi_bottom_left[0] * 100), 100, self.update_bottom_left_x)
        cv2.createTrackbar("Bottom Left Y", "ROI Settings", int(self.roi_bottom_left[1] * 100), 100, self.update_bottom_left_y)
        cv2.createTrackbar("Bottom Right X", "ROI Settings", int(self.roi_bottom_right[0] * 100), 100, self.update_bottom_right_x)
        cv2.createTrackbar("Bottom Right Y", "ROI Settings", int(self.roi_bottom_right[1] * 100), 100, self.update_bottom_right_y)

    # Update functions for each slider
    def update_top_left_x(self, value):
        self.roi_top_left[0] = value / 100.0
    def update_top_left_y(self, value):
        self.roi_top_left[1] = value / 100.0
    def update_top_right_x(self, value):
        self.roi_top_right[0] = value / 100.0
    def update_top_right_y(self, value):
        self.roi_top_right[1] = value / 100.0
    def update_middle_left_x(self, value):
        self.roi_middle_left[0] = value / 100.0
    def update_middle_left_y(self, value):
        self.roi_middle_left[1] = value / 100.0
    def update_middle_right_x(self, value):
        self.roi_middle_right[0] = value / 100.0
    def update_middle_right_y(self, value):
        self.roi_middle_right[1] = value / 100.0
    def update_bottom_left_x(self, value):
        self.roi_bottom_left[0] = value / 100.0
    def update_bottom_left_y(self, value):
        self.roi_bottom_left[1] = value / 100.0
    def update_bottom_right_x(self, value):
        self.roi_bottom_right[0] = value / 100.0
    def update_bottom_right_y(self, value):
        self.roi_bottom_right[1] = value / 100.0

    def listener_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error: {e}")
            return
        
        # Detect and display lines
        frame_with_lines, masked = self.detect_line(frame)
        cv2.imshow('Processed Frame', frame_with_lines)
        cv2.imshow('ROI frame', masked)
        cv2.waitKey(1)


    def detect_line(self, frame):
        # Convert image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        
        # Apply Canny edge detector
        edges = cv2.Canny(blurred, 100, 200)
        
        # Define the region of interest (ROI) dynamically from sliders
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # Create polygon based on current slider settings
        roi_points = np.array([[
            (int(self.roi_bottom_left[0] * width), int(self.roi_bottom_left[1] * height)),
            (int(self.roi_middle_left[0] * width), int(self.roi_middle_left[1] * height)),
            (int(self.roi_top_left[0] * width), int(self.roi_top_left[1] * height)),
            (int(self.roi_top_right[0] * width), int(self.roi_top_right[1] * height)),
            (int(self.roi_middle_right[0] * width), int(self.roi_middle_right[1] * height)),
            (int(self.roi_bottom_right[0] * width), int(self.roi_bottom_right[1] * height))
        ]], np.int32)
        
        # Apply mask to focus on the ROI
        cv2.fillPoly(mask, roi_points, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Draw the ROI perimeter on the original frame for visualization
        cv2.polylines(frame, roi_points, isClosed=True, color=(0, 0, 255), thickness=2)

        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 25, minLineLength=100, maxLineGap=50)
        
        # Draw the detected lines on the original image
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
        
        return frame, masked_edges
        


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    image_subscriber = SubscriberNode()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Shutdown when done
        if rclpy.ok():
            image_subscriber.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()