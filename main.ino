#include <Servo.h>
#include <Encoder.h>

#define enA 9
#define in1 6
#define in2 7

Servo servo;
Encoder encoder(2,3);
int pos;

long position = 0;	
int rotation = 0;	
int rotDirection = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  Serial.begin(115200);
  servo.attach(5);
  servo.write(0);
}
void set_values(int motor_direction,int motor_speed,int servo_angle){
  if(rotDirection != motor_direction){
    if(motor_direction == 1){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        rotDirection = 1;
    }
    else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        rotDirection = 0;
    }
  }
  if(!(motor_speed >256 || motor_speed <0)){
    analogWrite(enA, motor_speed); 
  }

  if(servo_angle >=0 &&servo_angle <=180 ){
    servo.write(servo_angle);
  }

}
void parse_set(String s){
  String to_parse = "";
  int i = s.indexOf(":");
  int j = s.lastIndexOf(":");
  if(i==j)return;
  int  md = s.substring(0,i).toInt() ,ms=s.substring(i+1,j).toInt() ,sa =s.substring(j+1).toInt();
  set_values(md,ms,sa);
  }

void loop() {
  if (Serial.available() > 0) { 
    // input = "<motor direction>:<motor speed>:<servo angle>"|"r"
    String input = Serial.readString();

    if(input[0] =='r'){
      Serial.println(encoder.read());
    }
    else {
      parse_set(input);
    }
  }
}
