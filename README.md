Robot Control System

This project demonstrates a simple robot control system that uses an Arduino and ROS2 for real-time motor and servo control. The system listens for user input via the terminal (W, A, S, D, X, Q keys) to control the robot's movement. The Arduino controls the motors and servo, while the Python ROS2 node communicates with the Arduino to send commands and receive encoder data.
Features

    Control robot movement with W (forward), S (backward), A (turn left), D (turn right), and X (stop).
    Servo control for steering (turn left and right).
    Motor control with adjustable speed.
    Encoder feedback to monitor motor position.
    Real-time communication between Python ROS2 node and Arduino via serial.
