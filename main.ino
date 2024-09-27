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
// void loop() {
//   if (pos >= 0 && pos <= 180){
//     myServo.write(pos);
//     Serial.print("Turned to: ");
//     Serial.println(pos);
//   }
//   else {
//     Serial.println("Invalid postition!");
//   }
// }

int rotDirection = 0;
void setup()
{
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    Serial.begin(9600);
    myServo.attach(5);
    myServo.write(0);
}

void loop()
{

    Serial.println("Enter position: ");
    while (Serial.available() == 0)
    {
    };
    pos = Serial.readString().toInt();
    analogWrite(enA, pwmOutput);
    analogWrite(enA, pwmOutput);

    if (pressed == true & rotDirection == 0)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        rotDirection = 1;
        delay(20);
    }
    // If button is pressed - change rotation direction
    if (pressed == false & rotDirection == 1)
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        rotDirection = 0;
        delay(20);
    }
}