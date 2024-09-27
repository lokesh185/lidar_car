#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <Servo.h>
#include <Encoder.h>

#define enA 9
#define in1 6
#define in2 7

Servo servo;
Encoder encoder(2, 3);
int pos;

long position = 0;
int rotation = 0;
int rotDirection = 0;

ros::NodeHandle nh;


struct MotorServoParams {
  int motor_direction;
  int motor_speed;
  int servo_angle;
};


std_msgs::Int32 encoder_msg;
ros::Publisher pub_encoder("encoder_data", &encoder_msg);


void set_values(MotorServoParams params) {
  if (rotDirection != params.motor_direction) {
    if (params.motor_direction == 1) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      rotDirection = 1;
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      rotDirection = 0;
    }
  }
  if (!(params.motor_speed > 256 || params.motor_speed < 0)) {
    analogWrite(enA, params.motor_speed);
  }

  if (params.servo_angle >= 0 && params.servo_angle <= 180) {
    servo.write(params.servo_angle);
  }
}


MotorServoParams parse_message(String s) {
  MotorServoParams params;
  int i = s.indexOf(":");
  int j = s.lastIndexOf(":");

  if (i != j) {
    params.motor_direction = s.substring(0, i).toInt();
    params.motor_speed = s.substring(i + 1, j).toInt();
    params.servo_angle = s.substring(j + 1).toInt();
  }

  return params;
}

void commandCallback(const std_msgs::String& msg) {

  MotorServoParams params = parse_message(String(msg.data.c_str()));


  set_values(params);
}

ros::Subscriber<std_msgs::String> sub_command("motor_servo_command", &commandCallback);

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  servo.attach(5);
  servo.write(0);

  nh.initNode();
  nh.advertise(pub_encoder);
  nh.subscribe(sub_command);
}

void loop() {
  encoder_msg.data = encoder.read();
  pub_encoder.publish(&encoder_msg);

  nh.spinOnce();  
  delay(100);  
}
