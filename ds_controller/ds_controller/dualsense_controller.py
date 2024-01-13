import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class DualSenseController(Node):
    def __init__(self):
        super().__init__('dualsense_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

    def joy_callback(self, msg):
        twist = Twist()

        x_vel = msg.axes[1] * 0.2
        z_vel = msg.axes[3] * 1.0

        #Eliminate drift
        if x_vel < 0.018 and x_vel > -0.001:
            x_vel = 0.0
        if z_vel > -0.011 and z_vel < 0.001:
            z_vel = 0.0

        # Map the DualSense controller inputs to linear and angular velocities
        # This is just an example, replace with your actual mapping
        twist.linear.x = x_vel  # Forward/backward
        twist.angular.z = z_vel  # Left/right

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    dualsense_controller = DualSenseController()

    rclpy.spin(dualsense_controller)

    dualsense_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()