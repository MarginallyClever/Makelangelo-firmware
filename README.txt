##### Makelangelo-firmware #####

http://www.makelangelo.com/

A mural drawing robot and a gentle introduction to 3D printing

Makelangelo is a polargraph mural drawing robot.  It is intended to teach students about programming, physics, trigonometry, and electronics in a fun, goal-oriented way.  Makelangelo is scalable: our test models have ranged from 30cm^2 to 300cm^2.

This project specifically deals with the firmware: the code in the brain of the robot that receives instructions and moves the motors.  It pairs really well with Makelangelo, a project to give humans a pleasant GUI.

Makelangelo-firmware can also support traditional XY gantries, CoreXY gantries, Zarplotters, Skycams, Delta robots, Stewart platforms, 3 axis arms, and 6 axis arms.
 	
## Version ##

Newer versions might be available at https://www.marginallyclever.com/product/makelangelo-firmware/

## Note ##

Unless otherwise stated in the code, the default units of measurement are millimeters (mm), kilograms (kg), and seconds (s).
## Instructions ##

- Make sure the parent folder is called Makelangelo-firmware.
- Open Makelangelo-firmware/Makelangelo-firmware.ino in arduino
- in Makelangelo-firmware/configure.h make sure BOARD_TYPE and MACHINE_STYLE are set for your board and machine style
  - For Makelangelo 3 or Makelangelo 5, choose POLARGRAPH
- For Makelangelo robots, in Makelangelo-firmware/polargraph.h, set
  for Makelangelo 3 #define MACHINE_HARDWARE_VERSION 3 
  for Makelangelo 5 #define MACHINE_HARDWARE_VERSION 5
- Tools > board > set type for your flavor of arduino 
- Tools > port > set the connection for your arudio
- upload

For developers, please see
https://github.com/MarginallyClever/Makelangelo/wiki/Home/

## Get help ##

Please visit the forums
https://marginallyclever.com/forum

## Special thanks ##

Makelangelo is derived from the work of Paul Fisher.  It is largely inspired by "Hektor":http://hektor.ch/ by Jürg Lehni and Uli Franke.



This file was downloaded from https://github.com/MarginallyClever/Makelangelo/
