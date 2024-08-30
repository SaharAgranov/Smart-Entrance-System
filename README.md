# Smart Entrance System

This repository contains the code and resources for a smart entrance system project. The project integrates Arduino and Python to create a system that controls a door lock based on PIR sensor, ultrasonic sensor data, webcam and facial recognition.

## Table of Contents
- [Overview](#overview)
- [Hardware Setup](#hardware-setup)
- [Software Structure](#software-structure)
- [Usage](#usage)
- [Results](#results)

## Overview
This project integrates an Arduino microcontroller with a Python-based machine learning model to create a smart entrance system. The Arduino handles PIR and ultrasonic sensor data to detect motion, while Python processes the data and controls a servo motor to lock/unlock the door.

## Hardware Setup
- **Arduino Uno:** Handles the PIR and ultrasonic sensor and controls the servo motor.
- **PIR Sensor:** Turn on and off the system based on motion.
- **Ultrasonic Sensor:** Detects the distance to an object.
- **Servo Motor:** Acts as the lock mechanism.
- **Wiring:** Provide a diagram or description of how the components are connected.

## Software Structure
- `ArduinoCode.ino`: The Arduino sketch that reads sensor data and communicates with the Python script.
- `PythonCode.ipynb`: A Jupyter notebook containing the Python code that processes sensor data and controls the servo motor.

## Usage
- **Arduino:** The Arduino code continuously reads PIR sensor and based on turn on and off the ultrasonic sensor data and sends it to the Python script via serial communication.
- **Python:** The Python script processes the data to determine if the door should be locked or unlocked based on predefined conditions.

## Results
The system successfully identifies authorized personnel and controls the door lock based on sensor input.

Wiring Diagram based on Tinkercad simulator.
![Thinkercards](https://github.com/user-attachments/assets/968cbf9a-7531-4ab6-8047-47d38fe778d4)

