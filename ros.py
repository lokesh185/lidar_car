import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

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

        self.encoder_subscriber = self.create_subscription(
            String,
            'encoder_data',
            self.encoder_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.timer_callback) 

    def timer_callback(self):

        motor_speed = 150  # Speed: 0-255
        motor_direction = 1  # Direction: 1 = Forward, -1 = Backward
        motor_command = f"M {motor_speed} {motor_direction}"
        

        motor_msg = String()
        motor_msg.data = motor_command
        self.motor_cmd_publisher.publish(motor_msg)
        self.get_logger().info(f"Sent motor command: {motor_command}")


        servo_angle = 90 
        servo_command = f"S {servo_angle}"
        
 
        servo_msg = String()
        servo_msg.data = servo_command
        self.servo_cmd_publisher.publish(servo_msg)
        self.get_logger().info(f"Sent servo command: {servo_command}")

    def encoder_callback(self, msg):
        self.get_logger().info(f"Encoder data received: {msg.data}")
        
        try:
            if self.serial_connection.isOpen():
                self.serial_connection.write(b"R\n")
                self.get_logger().info("Requesting encoder data from Arduino...")
                time.sleep(1)  
                if self.serial_connection.in_waiting > 0:
                    encoder_data = self.serial_connection.readline().decode('utf-8').strip()
                    self.get_logger().info(f"Encoder data: {encoder_data}")
        except Exception as e:
            self.get_logger().error(f"Failed to send encoder request: {e}")

def main(args=None):
    rclpy.init(args=args)

    arduino_control_node = ArduinoControlNode()

    try:
        rclpy.spin(arduino_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        arduino_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
