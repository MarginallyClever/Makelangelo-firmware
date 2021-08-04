# Makelangelo-firmware #

CNC firmware for many different control boards and kinematic systems.

CNCs are computer controlled machines like mills, lathes, robot arms, and more.

Firmware is the code that lives in the CPU of each robot.  The CPU is the brain, the firmware is the lesson taught to the machine.

Control boards are the PCB that surrounds each CPU.  Think of it as the nervous system.  This firmware has been written for several kinds of control boards from different manufacturers.  Please feel free to make a pull request with your favorite flavor.

Kinematic systems is more complicated.  All the different types of robots supported eventually boil down to a few motors moving in concert.  These moving motors can be rearranged physically to make different shapes.  The firmware only knows the state of each motor.  BUT there is some room in some brains to add more.  More powerful brains can run [Forward Kinematics](https://en.wikipedia.org/wiki/Forward_kinematics) and even [Inverse Kinematics](https://en.wikipedia.org/wiki/Inverse_kinematics) with [Gradient Descent](https://www.marginallyclever.com/2020/04/gradient-descent-inverse-kinematics-for-6dof-robot-arms/).

The two leading alternatives right now are [GRBL](https://github.com/gnea/grbl) and [Marlin](https://marlinfw.org/).  GRBL is made for three axis spingle type cutting CNC machines.  Marlin is made for multi-axis 3D printing.  Makelangelo-firmware was initially developped by Marginally Clever Robots, Ltd. to address the need for a multi-axis stepper driven system that was neither of these - Plotters, robot arms, stewart platforms, and more.  Things without a spindle or hot bits.

This firmware fully or partially supports many kinematic models.  It was originally written for the http://www.makelangelo.com/ Makelangelo art robot, a 3 motor polargraph art machine.  It has since expanded to suport more types.  Please feel free to make a pull request with your favorite flavor.

It pairs really well with Makelangelo Software, a project to give humans a pleasant GUI.

## Installation and Usage ##

Please see the pictoral guide at https://mcr.dozuki.com/Guide/How+to+update+Makelangelo+firmware/4?lang=en

## Kinematics aka Shapes ##

Makelangelo-firmware can be recompiled to work as one of many different types of CNC:

```
#define POLARGRAPH       1  // Polargraph: wall hanging V shape like Makelangelo
#define TRADITIONALXY    3  // Traditional: classic XYZ gantry
#define COREXY           2  // CoreXY: gantry with cross-belt tensioning
#define ZARPLOTTER       4  // Zarplotter:  4 motor, x-shaped 2D motion
#define SKYCAM           5  // Skycam: 4 motor, x-shaped 3D motion
#define DELTA            6  // Delta: 3 arm delta robot, rotary action.  untested.
#define STEWART_ROTARY   7  // Stewart platform: 6 arm stewart platform, rotary action.  untested.
#define ARM3             8  // Arm3: 3DOF palletizing robot arm.
#define SIXI             9  // Sixi: 6DOF robot arm.
#define TRADITIONAL6    10  // Traditional6: 6 axis machine, no restrictions.
#define SCARA           11  // SCARA: two link, two joint, 2D motion
#define SIXI3           12  // Sixi 3 robot arm.  5-6 DOF + Servo
#define STEWART_LINEAR  13  // Stewart platform: 6 arm stewart platform, linear action.
```

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
```

Not all brains can support all kinematics.

## Version ##

Newer versions might be available at https://www.marginallyclever.com/product/makelangelo-firmware/

## Note ##

Unless otherwise stated in the code, the default units of measurement are millimeters (mm), kilograms (kg), and seconds (s).

## Dependencies ##

Makelangelo-firmware depends on a few other libraries.  SPI, EEPROM, LiquidCrystal, and SdFat.  The Adafruit SdFAT library will NOT work, use the Greiman edition.

## Instructions ##

https://mcr.dozuki.com/Guide/How+to+update+Makelangelo+firmware/4?lang=en

For developers, please see https://github.com/MarginallyClever/Makelangelo/wiki/Home/ and https://github.com/MarginallyClever/Makelangelo-firmware/wiki

## Get help ##

Please visit the forums
https://marginallyclever.com/forum

## Special thanks ##

Makelangelo is derived from the work of Paul Fisher.  It is largely inspired by "Hektor":http://hektor.ch/ by Jürg Lehni and Uli Franke.

This file was downloaded from https://github.com/MarginallyClever/Makelangelo/

[![PlatformIO CI](https://github.com/MarginallyClever/Makelangelo-firmware/actions/workflows/main.yml/badge.svg)](https://github.com/MarginallyClever/Makelangelo-firmware/actions/workflows/main.yml)
