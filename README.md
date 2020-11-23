# Makelangelo-firmware #

CNC firmware for many different control boards and kinematic systems.

This project specifically deals with the firmware: the code in the brain of the robot that receives instructions and moves the motors.  It pairs really well with Makelangelo, a project to give humans a pleasant GUI.

Makelangelo-firmware supports many different robot shapes and robot brains.  (see heading below)

Originally designed to drive the http://www.makelangelo.com/ polargraph mural drawing robot.  Makelangelo is scalable: our test models have ranged from 30cm^2 to 300cm^2.

## Kinematics aka Shapes ##

Makelangelo-firmware can be recompiled to work as one of many different types of CNC:

````
#define POLARGRAPH       1  // Polargraph: wall hanging V shape like Makelangelo
#define TRADITIONALXY    3  // Traditional: classic XYZ gantry
#define COREXY           2  // CoreXY: gantry with cross-belt tensioning
#define ZARPLOTTER       4  // Zarplotter:  4 motor, x-shaped 2D motion
#define SKYCAM           5  // Skycam: 4 motor, x-shaped 3D motion
#define DELTA            6  // Delta: 3 arm delta robot, rotary action.  untested.
#define STEWART          7  // Stewart platform: 6 arm stewart platform, rotary action.  untested.
#define ARM3             8  // Arm3: 3DOF palletizing robot arm.
#define SIXI             9  // Sixi: 6DOF robot arm.
#define TRADITIONAL6    10  // Traditional6: 6 axis machine, no restrictions.
#define SCARA           11  // SCARA: two link, two joint, 2D motion
````

## Controllers aka Brains ##

Makelangelo-firmware can be recompiled to work with one of many different types of Controllers.

```
#define BOARD_RUMBA        1  // Reprap discount Rumba board
#define BOARD_RAMPS        2  // Mega2560 + Ramps 1.4
#define BOARD_SANGUINOLULU 3  // Sanguinolulu
#define BOARD_TEENSYLU     4  // Teensylu
#define BOARD_WEMOS        5  // Wemos D1 R2 + CNC Shield v3 (see board_wemos.h)
#define BOARD_SIXI_MEGA    6  // Arduino Mega + custom shield for Sixi 2 robot
#define BOARD_CNCV3        7  // Mega2560 + CNC Shield v3
#define BOARD_ESP32        8  // ESP32 + Marginally Clever Polargraph PCB.
````

Not all brains can support all kinematics.

## Version ##

Newer versions might be available at https://www.marginallyclever.com/product/makelangelo-firmware/

## Note ##

Unless otherwise stated in the code, the default units of measurement are millimeters (mm), kilograms (kg), and seconds (s).

## Dependencies ##

Makelangelo-firmware depends on a few other libraries.  SPI, EEPROM, LiquidCrystal, and SdFat.  The Adafruit SdFAT library will NOT work, use the Greiman edition.

## Instructions ##

https://mcr.dozuki.com/Guide/How+to+update+Makelangelo+firmware/4?lang=en

For developers, please see https://github.com/MarginallyClever/Makelangelo/wiki/Home/

## Get help ##

Please visit the forums
https://marginallyclever.com/forum

## Special thanks ##

Makelangelo is derived from the work of Paul Fisher.  It is largely inspired by "Hektor":http://hektor.ch/ by Jürg Lehni and Uli Franke.



This file was downloaded from https://github.com/MarginallyClever/Makelangelo/
