# GPS-Vehicle-Tracking-System

## Overview
The GPS Vehicle Tracking System is designed to monitor vehicle location, speed, and vibrations for accident detection. The system integrates hardware components such as the u-blox NeoM7/M8 GPS module, Arduino Uno, and the ADXL355 accelerometer. The software component involves coding in the Arduino IDE, utilizing libraries like TinyGPS++ and ArduinoJSON to create JSON packets for web integration.

## Hardware Components
u-blox NeoM6 GPS Module: Provides GPS coordinates and heading information.
Arduino Uno: The main microcontroller that processes GPS data and controls other components.
ADXL355: Used for detecting vibrations related to engine start/stop and accidents.

## Software Components
Arduino IDE: Used for writing and uploading the code to the Arduino Uno.
### Libraries:
TinyGPS++: For parsing GPS data.
ArduinoJSON: For creating JSON packets to structure and send data.

## Installation and Setup

### 1. Install Required Libraries 
TinyGPS++: Handles GPS data.
ArduinoJSON: Helps in creating JSON packets.
Install these libraries via the Arduino IDE Library Manager.
### 2. Connect Hardware Components
GPS Module (u-blox NeoM6/M7/M8): Connect the SDA and SCL (I2C pins) to A4 and A5 of the Arduino Uno.
ADXL355 Accelerometer: Connect to the Arduino to detect vibrations and accelerations.
### 3. Upload the Arduino Code
Write or upload the provided code to the Arduino Uno via the Arduino IDE.

## Functionality

### 1. Acquire GPS Coordinates
The system acquires GPS coordinates from the u-blox NeoM6/M7/M8 GPS module.
### 2. GPS Heading
Determines the GPS heading using the built-in magnetometer of the GPS module.
### 3. Speed and Acceleration Calculation
Speed is calculated using the change in GPS coordinates.
The Haversine formula is used to calculate the distance between two GPS points.
Acceleration is determined using the ADXL355 accelerometer.
### 4. Vibration Detection
Detects engine start/stop and potential accidents.
The ADXL355 is calibrated for accurate vibration sensing.
### 5. JSON Packet Creation
All the data collected (GPS coordinates, speed, heading, acceleration) is structured into a comprehensive JSON packet.

## Conclusion
This GPS Vehicle Tracking System provides a robust solution for real-time vehicle monitoring, integrating crucial functionalities like location tracking, speed calculation, and accident detection. The data can be easily integrated with web services for further analysis and monitoring.
