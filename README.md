# Motor Predictive Maintenance System

An ESP32-based motor condition monitoring system that analyses vibration signals to support predictive maintenance.

## Project Overview

This project focuses on monitoring the vibration behaviour of a DC motor using sensor-based measurements and digital signal processing.

The system combines vibration sensing, embedded processing and IoT-based data visualisation.

## Hardware Components

- ESP32 Development Board
- Piezoelectric Vibration Sensor
- MPU6050 Accelerometer
- 775 DC Motor
- OLED Display (SSD1306)
- LEDs and Buzzer
- Relay Module

## Software and Technologies

- Arduino IDE
- Embedded C++
- RMS Signal Analysis
- Fast Fourier Transform (FFT)
- ThingSpeak IoT Platform

## Working Principle

1. The piezoelectric sensor detects mechanical vibrations from the motor.
2. The MPU6050 measures acceleration along three axes.
3. The ESP32 collects and processes the sensor readings.
4. RMS calculations estimate vibration intensity.
5. FFT analysis extracts dominant frequency information.
6. LEDs, buzzer and OLED provide local status indications.
7. ThingSpeak enables remote monitoring and visualisation.

## Features

- Dual-sensor vibration monitoring
- Three-axis acceleration measurement
- RMS-based vibration analysis
- FFT-based frequency analysis
- OLED-based real-time display
- LED and buzzer status indication
- IoT data visualisation using ThingSpeak

## Future Improvements

- Machine-learning-based fault classification
- Automated anomaly detection
- Long-term vibration trend analysis
- Improved predictive maintenance accuracy

## Applications

- Industrial motor monitoring
- Rotating machinery condition assessment
- Predictive maintenance systems
- Industrial IoT applications

## Author

**Lakshmi Anand**  
Electronics and Instrumentation Engineering (2nd year)

## Acknowledgement

Developed as an academic engineering project exploring the integration of sensors, signal processing, embedded systems and IoT.
