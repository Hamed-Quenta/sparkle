import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float64
import Jetson.GPIO as GPIO
import datetime
import time

WHEEL_DIAMETER = 0.1
WHEEL_BASE = 0.281

# Timestamp funtion
def TimestampMilisec64():
    return float((datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1)).total_seconds() * 1000)

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        GPIO.setmode(GPIO.BOARD)

        # Left motor connections
        self.left_motor_in1 = 22  # GPIO pin for left motor direction
        self.left_motor_in2 = 21
        self.left_motor_en_pin = 15  # GPIO pin for left motor EN
        
        # Right motor connections
        self.right_motor_in1 = 23  # GPIO pin for right motor direction
        self.right_motor_in2 = 24
        self.right_motor_en_pin = 33  # GPIO pin for right motor EN

        # Set up left motor GPIO pins
        GPIO.setup(self.left_motor_en_pin, GPIO.OUT)
        self.left_motor_pwm = GPIO.PWM(self.left_motor_en_pin, 15000)  # 1000 Hz PWM frequency
        self.left_motor_pwm.start(0)  # start with duty cycle 0
        
        GPIO.setup(self.right_motor_en_pin, GPIO.OUT)
        self.right_motor_pwm = GPIO.PWM(self.right_motor_en_pin, 15000)
        self.right_motor_pwm.start(0)

        GPIO.setup(self.left_motor_in1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.left_motor_in2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_motor_in1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_motor_in2, GPIO.OUT, initial=GPIO.LOW)
        
        # Variables
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        self.prev_time_left = TimestampMilisec64()
        self.prev_time_right = TimestampMilisec64()

        self.speed_left = 0
        self.speed_right = 0

        self.kp_left = 50.0
        self.ki_left = 10.0
        self.kd_left = 0.45

        self.kp_right = 55.0
        self.ki_right = 9.0
        self.kd_right = 0.21

        self.cumulative_error_left = 0
        self.cumulative_error_right = 0

        self.prev_error_left = 0
        self.prev_error_right = 0

        self.pwm_left = 0.0
        self.pwm_right = 0.0

        self.setpoint_left = 0.0
        self.setpoint_right = 0.0

        self.wheel_diameter = WHEEL_DIAMETER
        self.wheel_base = WHEEL_BASE

        # Create publisher and subscriber
        self.subscription_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback_cmd_vel, 10)
        self.left_speed_publisher_ = self.create_publisher(Float64, '/speed_left', 10)
        self.right_speed_publisher_ = self.create_publisher(Float64, '/speed_right', 10)
        self.left_setpoint_publisher_ = self.create_publisher(Float64, '/setpoint_left', 10)
        self.right_setpoint_publisher_ = self.create_publisher(Float64, '/setpoint_right', 10)

        # self.speed_publisher_ = self.create_publisher(Twist, '/speed', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.subscription_left_ticks = self.create_subscription(
            Int16, '/left_ticks', self.listener_callback_left_ticks, qos_profile)
        self.subscription_right_ticks = self.create_subscription(
            Int16, '/right_ticks', self.listener_callback_right_ticks, qos_profile)
        
    def computePID_left(self, input_speed, setpoint, elapsed_time):
        error = setpoint - input_speed
        self.cumulative_error_left += error * elapsed_time
        derivative_error = (error - self.prev_error_left) / elapsed_time

        # limit the cumulative error
        if self.cumulative_error_left > 1:
            self.cumulative_error_left = 1
        elif self.cumulative_error_left < -1:
            self.cumulative_error_left = -1

        if setpoint == 0:
            self.cumulative_error_left = 0
            error = 0

        output = self.kp_left*error + self.ki_left*self.cumulative_error_left + self.kd_left*derivative_error

        self.prev_error_left = error

        return output
    
    def computePID_right(self, input_speed, setpoint, elapsed_time):
        error = setpoint - input_speed
        self.cumulative_error_right += error * elapsed_time
        derivative_error = (error - self.prev_error_right) / elapsed_time

        # limit the cumulative error
        if self.cumulative_error_right > 1:
            self.cumulative_error_right = 1
        elif self.cumulative_error_right < -1:
            self.cumulative_error_right = -1
        
        if setpoint == 0:
            self.cumulative_error_right = 0
            error = 0

        output = self.kp_right*error + self.ki_right*self.cumulative_error_right + self.kd_right*derivative_error

        self.prev_error_right = error

        return output

    def listener_callback_cmd_vel(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.setpoint_left = (2.0 * linear_x - angular_z * self.wheel_base) / 2.0
        self.setpoint_right = (2.0 * linear_x + angular_z * self.wheel_base) / 2.0
        # self.setpoint_left = linear_x * 60 / (3.14159265359 * self.wheel_diameter) - angular_z * self.wheel_base * 60 / (3.14159265359 * self.wheel_diameter * 2)
        # self.setpoint_right = linear_x * 60 / (3.14159265359 * self.wheel_diameter) + angular_z * self.wheel_base * 60 / (3.14159265359 * self.wheel_diameter * 2)

    def listener_callback_left_ticks(self, msg):
        # Manage rollover and rollunder when we get outside the 16-bit integer range
        diff_ticks = (65535 + msg.data - self.prev_left_ticks) % 65535
        if diff_ticks > 10000:
            diff_ticks -= 65535
        
        elapsed_time =  round((TimestampMilisec64() - self.prev_time_left) / 1000, 3)

        if elapsed_time > 0.0:
            self.speed_left = float(3.14159265359 * self.wheel_diameter * diff_ticks / (131.25 * 64 * 0.03)) # m/s
            # self.speed_left = float(60 * diff_ticks / (131.25 * 64 * 0.03)) # rpm
        else:
            self.speed_left = 0.0
            self.setpoint_left = 0.0

        self.prev_left_ticks = msg.data
        self.prev_time_left = TimestampMilisec64()

        # Set the direction of the motors
        if self.setpoint_left > 0:
            GPIO.output(self.left_motor_in1, GPIO.LOW)
            GPIO.output(self.left_motor_in2, GPIO.HIGH)
        elif self.setpoint_left < 0:
            GPIO.output(self.left_motor_in1, GPIO.HIGH)
            GPIO.output(self.left_motor_in2, GPIO.LOW)
        elif self.setpoint_left == 0:
            GPIO.output(self.left_motor_in1, GPIO.LOW)
            GPIO.output(self.left_motor_in2, GPIO.LOW)
        else:
            GPIO.output(self.left_motor_in1, GPIO.LOW)
            GPIO.output(self.left_motor_in2, GPIO.LOW)

        if elapsed_time > 0.0:
            self.pwm_left = self.computePID_left(abs(self.speed_left), abs(self.setpoint_left), 0.03)
            self.pwm_left = int(abs(3*self.pwm_left + 30))

            if self.pwm_left > 85:
                self.pwm_left = 85
            elif self.pwm_left <= 0:
                self.pwm_left = 0

            self.left_motor_pwm.ChangeDutyCycle(self.pwm_left)
        else:
            self.left_motor_pwm.ChangeDutyCycle(0)

    def listener_callback_right_ticks(self, msg):
        # Manage rollover and rollunder when we get outside the 16-bit integer range
        diff_ticks = (65535 + msg.data - self.prev_right_ticks) % 65535
        if diff_ticks > 10000:
            diff_ticks -= 65535

        elapsed_time =  round((TimestampMilisec64() - self.prev_time_right) / 1000, 3)

        if elapsed_time > 0.0:
            self.speed_right = 3.14159265359 * self.wheel_diameter * diff_ticks / (131.25 * 64 * 0.03) # m/s
            # self.speed_right = float(60 * diff_ticks / (131.25 * 64 * 0.03)) # rpm
        else:
            self.speed_right = 0.0
            self.setpoint_right = 0.0

        self.prev_right_ticks = msg.data
        self.prev_time_right = TimestampMilisec64()

        # Set the direction of the motors
        if self.setpoint_right > 0:
            GPIO.output(self.right_motor_in1, GPIO.HIGH)
            GPIO.output(self.right_motor_in2, GPIO.LOW)
        elif self.setpoint_right < 0:
            GPIO.output(self.right_motor_in1, GPIO.LOW)
            GPIO.output(self.right_motor_in2, GPIO.HIGH)
        elif self.setpoint_right == 0:
            GPIO.output(self.right_motor_in1, GPIO.LOW)
            GPIO.output(self.right_motor_in2, GPIO.LOW)
        else:
            GPIO.output(self.right_motor_in1, GPIO.LOW)
            GPIO.output(self.right_motor_in2, GPIO.LOW)

        if elapsed_time > 0.0:
            self.pwm_right = self.computePID_right(abs(self.speed_right), abs(self.setpoint_right), 0.03)
            self.pwm_right = int(abs(3*self.pwm_right + 30))

            if self.pwm_right > 85:
                self.pwm_right = 85
            elif self.pwm_right <= 0:
                self.pwm_right = 0

            self.right_motor_pwm.ChangeDutyCycle(self.pwm_right)
        else:
            self.right_motor_pwm.ChangeDutyCycle(0)
    
    def timer_callback(self):
        left_msg = Float64()
        left_msg.data = float(self.speed_left)

        right_msg = Float64()
        right_msg.data = float(self.speed_right)

        left_setpoint_msg = Float64()
        left_setpoint_msg.data = float(self.setpoint_left)

        right_setpoint_msg = Float64()
        right_setpoint_msg.data = float(self.setpoint_right)

        # msg = Twist()
        # msg.linear.x = float(self.speed_left)
        # msg.linear.y = self.setpoint_left
        # msg.linear.z = float(self.pwm_left)
        # msg.angular.x = float(self.speed_right)
        # msg.angular.y = self.setpoint_right
        # msg.angular.z = float(self.pwm_right)

        self.left_speed_publisher_.publish(left_msg)
        self.right_speed_publisher_.publish(right_msg)
        self.left_setpoint_publisher_.publish(left_setpoint_msg)
        self.right_setpoint_publisher_.publish(right_setpoint_msg)

        # self.speed_publisher_.publish(msg)

    def on_shutdown(self):
        self.left_motor_pwm.stop()
        self.right_motor_pwm.stop()
        self.get_logger().info('Shutting down motor controller node')


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info('Keyboard Interrupt (SIGINT)')
        motor_controller.on_shutdown()
        GPIO.cleanup()
    finally:
        motor_controller.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
