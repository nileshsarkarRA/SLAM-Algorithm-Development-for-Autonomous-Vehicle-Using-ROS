import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # Set up the publisher for cmd_vel (to control the robot)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribe to the LIDAR scan topic to receive LIDAR data
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Parameter for minimum safe distance from obstacles (in meters)
        self.stop_distance = self.declare_parameter('stop_distance', 0.15).value  # 15 cm

        # Variable to track the robot's current state
        self.is_turning = False

    def scan_callback(self, msg):
        # Filter out invalid LIDAR readings (inf, NaN) from the ranges list
        ranges = [distance for distance in msg.ranges if not (distance == float('inf') or distance == float('nan'))]
        if len(ranges) == 0:
            self.get_logger().warn('No valid LIDAR readings.')
            return  # No valid readings, ignore this callback

        # Find the closest obstacle distance
        closest_distance = min(ranges)

        # Determine the movement based on the closest distance
        if closest_distance < self.stop_distance:
            self.turn_car(closest_distance)
        else:
            self.move_forward()

    def stop_car(self):
        """Stop the car by sending zero velocity commands."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)
        self.get_logger().info('Stopping the car.')

    def turn_car(self, closest_distance):
        """Turn the car to avoid obstacles."""
        self.stop_car()  # Stop before making a turn for safety
        turn_msg = Twist()
        turn_msg.linear.x = 0.0
        # Increase turning speed as the robot gets closer to an obstacle
        turn_msg.angular.z = 1.0 - (closest_distance / self.stop_distance)
        self.publisher_.publish(turn_msg)
        self.get_logger().info(f'Turning: Closest obstacle at {closest_distance:.2f} meters.')

    def move_forward(self):
        """Move the car forward if no obstacles are detected."""
        forward_msg = Twist()
        forward_msg.linear.x = 0.2  # Forward speed (adjust as needed)
        forward_msg.angular.z = 0.0
        self.publisher_.publish(forward_msg)
        self.get_logger().info('Moving forward.')

def main(args=None):
    rclpy.init(args=args)

    # Create the LIDAR node
    lidar_node = LidarNode()

    # Spin the node to keep the program alive and responsive to callbacks
    rclpy.spin(lidar_node)

    # Cleanup and shutdown
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


   
