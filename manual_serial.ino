#include <Servo.h>
#include <Encoder.h>

// Motor and Servo Pins
#define enA 11
#define in1 6
#define in2 7

// Encoder Pins
Encoder encoder(2, 3);

// Motor and Servo Configuration
Servo servo;
int motorSpeed = 0;     // Current motor speed
int motorDirection = 0; // 1 = Forward, -1 = Backward, 0 = Stopped
int servoAngle = 90;    // Servo angle (90 = center)

// Setup function
void setup() {
  Serial.begin(9600);

  // Attach servo and configure motor pins
  servo.attach(5);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  // Initialize motor and servo
  servo.write(servoAngle);
  analogWrite(enA, 0);

  Serial.println("Setup complete. Send commands as follows:");
  Serial.println("M <speed> <direction> - Motor control (speed: 0-255, direction: 1=forward, -1=backward)");
  Serial.println("S <angle> - Servo control (angle: 0-180)");
  Serial.println("R - Read encoder count");
}

// Loop function
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read a command line
    command.trim(); // Remove any trailing newline or spaces

    // Parse the command
    if (command.startsWith("M")) {
      handleMotorCommand(command);
    } else if (command.startsWith("S")) {
      handleServoCommand(command);
    } else if (command.startsWith("R")) {
      sendEncoderReading();
    } else {
      Serial.println("Invalid command. Use M, S, or R.");
    }
  }
}

// Handle motor commands
void handleMotorCommand(String command) {
  int space1 = command.indexOf(' '); // Find first space
  int space2 = command.indexOf(' ', space1 + 1); // Find second space

  if (space1 == -1 || space2 == -1) {
    Serial.println("Invalid motor command. Use: M <speed> <direction>");
    return;
  }

  // Extract speed and direction
  int speed = command.substring(space1 + 1, space2).toInt();
  int direction = command.substring(space2 + 1).toInt();

  // Validate inputs
  if (speed < 0 || speed > 255 || (direction != 1 && direction != -1)) {
    Serial.println("Invalid motor parameters. Speed: 0-255, Direction: 1=forward, -1=backward");
    return;
  }

  // Apply motor command
  motorSpeed = speed;
  motorDirection = direction;

  if (motorDirection == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (motorDirection == -1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(enA, motorSpeed);
  Serial.print("Motor speed set to ");
  Serial.print(motorSpeed);
  Serial.print(" in direction ");
  Serial.println(motorDirection == 1 ? "Forward" : "Backward");
}

// Handle servo commands
void handleServoCommand(String command) {
  int space = command.indexOf(' '); // Find space

  if (space == -1) {
    Serial.println("Invalid servo command. Use: S <angle>");
    return;
  }

  // Extract angle
  int angle = command.substring(space + 1).toInt();

  // Validate input
  if (angle < 0 || angle > 180) {
    Serial.println("Invalid servo angle. Range: 0-180");
    return;
  }

  // Apply servo command
  servoAngle = angle;
  servo.write(servoAngle);
  Serial.print("Servo angle set to ");
  Serial.println(servoAngle);
}

// Send encoder reading
void sendEncoderReading() {
  long encoderCount = encoder.read(); // Get encoder count
  Serial.print("Encoder count: ");
  Serial.println(encoderCount);
}
