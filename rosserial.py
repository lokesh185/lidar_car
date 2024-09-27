import rospy
from std_msgs.msg import String

def send_commands():
    pub = rospy.Publisher('motor_servo_command', String, queue_size=10)
    rospy.init_node('motor_command_node', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Example command format "<motor_direction>:<motor_speed>:<servo_angle>"
        command = "1:150:90"  # Example: motor direction = 1, motor speed = 150, servo angle = 90
        rospy.loginfo(command)
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_commands()
    except rospy.ROSInterruptException:
        pass
