# Maze-Solving Robot (Micromouse-Inspired)

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Hardware Architecture](#hardware-architecture)
- [Chassis Design](#chassis-design)
- [Algorithm](#algorithm)
- [Bill of Materials](#bill-of-materials)
- [Installation & Usage](#installation--usage)
- [Project Team](#project-team)
- [References](#references)

---

## Introduction

This project presents a **maze-solving robot** inspired by the Micromouse competition. The robot autonomously navigates and solves mazes using an optimized flood-fill algorithm, tracing the shortest path from start to target in minimum time. The robot logic is implemented in C++ and runs on an Arduino Nano microcontroller, interfacing with infrared sensors and encoder motors for real-time navigation and wall detection.

---

## Features

- **Shortest path tracing** using optimized `flood-fill` algorithm and from start to target.
- **Real-time wall detection** with three `GP2Y0A41SK0F` infrared sensors.
- **Precise movement control** using encoder motors and PID control
- **Custom double-layer PCB** designed for compact integration
- **EEPROM-based map storage** for persistent maze data
- **3D-printed PLA chassis** for lightweight and robust structure

---

## Hardware Components

- **Microcontroller:** Arduino Nano (`ATmega328`)
- **Sensors:** 3× `GP2Y0A41SK0F` sharp infrared sensors
- **Motors:** 2× `N20` `6`V `500` RPM Encoder Motors
- **Motor Driver:** `TA6586` Dual H-Bridge
- **Battery:** 2× `3.7V` `1250`mAh LiPo cells (in series)
- **Wheels:** N20 motor rubber small wheels
- **Castor Wheel:** N20 castor robot ball wheel
- **Custom PCB:** Designed with `EasyEDA`, integrating all components
- **Custom Chassis:** Designed with `Solidworks`, and with `PolyLactic Acid`
 
 ---

## Algorithm

- **Flood-Fill Algorithm:** Optimized for speed and efficiency, with comparison to standard flood-fill for performance gains
- **PID Control:** Utilizes motor encoder feedback for precise motor control and accurate positioning
- **Wall Detection:** Real-time distance calculation and alignment using IR sensors
- **EEPROM Storage:** Maze mapping data stored and updated in Arduino Nano’s EEPROM (1 KB)
- **Reset Capability:** EEPROM data can be reset for new maze runs


---

## Installation & Usage

1. **PCB Fabrication:** Print the custom PCB using the provided schematics.
2. **Chassis Assembly:** 3D print the chassis and assemble all hardware components.
3. **Wiring:** Connect components as per the circuit diagram.
4. **Programming:** Upload the C++ code to the Arduino Nano using Arduino IDE.
5. **Operation:** Place the robot at the maze start point and power it on. The robot will autonomously solve the maze and trace the shortest path to the target.

---

## Project Team

- **Prof. Soumya Bagchi** – Assistant Professor, Department of Physics, IIT ISM Dhanbad
- **Pratyush Tripathi** – Electrical Engineering
- **Prasoon Krishna Yadav** – Electronics and Communication Engineering
- **Prayag Asrani** – Mechanical Engineering
- **Palak Kumari** – Computer Science & Engineering
- **Abhirup Choudhury** – Mathematics and Computing


---

## References

1. [Micromouse History](https://micromouseonline.com/micromouse-book/history/)
2. Monk, S. (2012). *Programming Arduino: Getting Started with Sketches*. McGraw-Hill Education.
3. [Arduino Nano Datasheet](https://docs.arduino.cc/resources/datasheets/A000005-datasheet.pdf)
4. [IEEE Micromouse History](https://www.ieee.org/)
5. [Micromouse Maze Solving Algorithm (IEEE)](https://ieeexplore.ieee.org/document/5578409)
6. Nishi, T., & Yasuda, T. (1990). *A study on algorithms for Micromouse competitions*. Robotics and Automation Journal.
7. Elger, D. (1987). *Wall-Following Algorithms for Autonomous Robots*. Control Systems Magazine.
8. Brooks, R. A. (1986). *A robust layered control system for a mobile robot*. IEEE Journal of Robotics and Automation.
9. [Arduino Nano Board Information](https://content.arduino.cc/assets/arduino_nano_size.pdf)
10. [Arduino Nano Pinout](https://docs.arduino.cc/resources/pinouts/A000005-full-pinout.pdf)
11. [TA6586 Motor Driver Datasheet](https://www.micros.com.pl/mediaserver/UITA6586_0001.pdf)
12. [GP2Y0A41SK0F IR Sensor Datasheet](https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a41sk_e.pdf)
13. [Arduino Nano Schematics](https://content.arduino.cc/assets/NanoV3.3_sch.pdf)
14. [Arduino Nano Eagle File](https://content.arduino.cc/assets/Nano-reference.zip)
15. [Flood Fill Algorithm Comparison](https://marsuniversity.github.io/ece387/FloodFill.pdf)

---

**For more details, see the full project report and source files in this repository.**
