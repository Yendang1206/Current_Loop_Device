# Digital Current Loop Device for Frequency Converters
This repository contains the design and implementation details of a digital current loop device for frequency converters, developed as part of a thesis project. This device serves as an intermediate unit between a motor and a frequency converter, featuring advanced functionalities such as fast and slow increments or decrements for user-friendly control.
## Overview
The digital current loop device converts digital inputs—representing torque (0–50 Nm) and speed (0–3000 rpm)—into 0–20 mA analog current loops. This project integrates hardware and software to achieve precise control, accurate signal processing, and robust performance in noisy environments.
## Key Features
+ Converts torque and speed inputs from a rotary encoder into 0–20 mA current loop outputs.
+ Displays real-time input values on an LCD I2C screen.
+ Offers convenient fast/slow increment and decrement control.
+ Incorporates grounding, shielding, and paired signal wiring to enhance signal stability.
## The work methods
The work methods for this project were split into three main stages: 
 + circuit design, simulation and prototyping,
 + developing an Arduino software program,
 + manufacturing the physical Printed Circuit Boards (PCB).
## Results
Demonstrated positive correlation between torque and speed inputs with the corresponding current loop values (0–20 mA).
Achieved consistent performance between breadboard prototypes and integrated PCB systems.
Eliminated capacitive crosstalk and frequency interference using proper grounding, shielding, and CAT6 cable pairing.
## How to use
1. Connect the device between the motor and the frequency converter.
2. Adjust torque and speed inputs using the rotary encoder.
3. Monitor real-time input values on the LCD I2C display.
4. Verify current loop outputs using multimeters for accuracy.
## Future recommendations
Conduct EMC testing to further enhance noise immunity and long-term stability.
