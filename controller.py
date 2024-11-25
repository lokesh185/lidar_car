import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import sys
import tty
import termios

class ArduinoControlNode(Node):

    def __init__(self):
        super().__init__('arduino_control_node')

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 9600

        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            return

        self.motor_cmd_publisher = self.create_publisher(String, 'motor_cmd', 10)
        self.servo_cmd_publisher = self.create_publisher(String, 'servo_cmd', 10)

    def send_motor_command(self, speed, direction):
        motor_command = f"M {speed} {direction}"
        motor_msg = String()
        motor_msg.data = motor_command
        self.motor_cmd_publisher.publish(motor_msg)
        self.get_logger().info(f"Sent motor command: {motor_command}")

    def send_servo_command(self, angle):
        servo_command = f"S {angle}"
        servo_msg = String()
        servo_msg.data = servo_command
        self.servo_cmd_publisher.publish(servo_msg)
        self.get_logger().info(f"Sent servo command: {servo_command}")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def control_robot(self):
        self.get_logger().info("Use W, A, S, D to control the robot. Press Q to quit.")

        motor_speed = 100
        servo_angle = 90

        while True:
            key = self.get_key()

            if key == 'w':
                self.send_motor_command(motor_speed, 1)
            elif key == 's':
                self.send_motor_command(motor_speed, -1)
            elif key == 'a':
                servo_angle = max(30, servo_angle - 10)
                self.send_servo_command(servo_angle)
            elif key == 'd':
                servo_angle = min(150, servo_angle + 10)
                self.send_servo_command(servo_angle)
            elif key == 'q':
                self.get_logger().info("Exiting control.")
                break
            elif key == 'x':
                self.send_motor_command(0, 0)
                self.get_logger().info("Robot stopped.")
            else:
                self.get_logger().info(f"Invalid key: {key}. Please use W, A, S, D, or Q.")

def main(args=None):
    rclpy.init(args=args)

    arduino_control_node = ArduinoControlNode()

    try:
        arduino_control_node.control_robot()
    except KeyboardInterrupt:
        pass
    finally:
        arduino_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
