import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from picamera2 import Picamera2  # Raspberry Pi camera
from cv_bridge import CvBridge
import numpy as np

class CameraNode(Node):
    """A ROS 2 Node that captures images from a Raspberry Pi Camera and publishes them."""
    
    def __init__(self):
        """Initialize the camera node and start publishing images."""
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.timer = self.create_timer(0.033, self.publish_camera_image)  # ~30 FPS
        self.bridge = CvBridge()

        # Initialize Pi Camera
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"size": (640, 480)}))
        self.picam2.start()

        self.get_logger().info("Camera Node has been started with Raspberry Pi Camera Module 3.")

    def publish_camera_image(self):
        """Capture an image from the Pi Camera and publish it as a ROS Image message."""
        frame = self.picam2.capture_array()

        if frame is None:
            self.get_logger().error('Failed to capture image from Pi Camera')
            return

        # Validate the captured frame
        if frame.ndim != 3 or frame.shape[2] != 3:
            self.get_logger().error('Captured image is not valid')
            return

        try:
            # Convert image to ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_.publish(msg)
            self.get_logger().debug('Published camera image.')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def destroy_node(self):
        """Stop the Pi Camera and clean up resources when the node is destroyed."""
        self.picam2.stop()  # Stop the Pi Camera
        super().destroy_node()
        self.get_logger().info("Camera Node has been shut down and Pi Camera stopped.")

def main(args=None):
    """Main entry point for the Camera Node."""
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    
