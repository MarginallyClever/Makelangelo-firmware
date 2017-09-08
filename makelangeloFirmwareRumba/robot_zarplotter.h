#ifndef ROBOT_MAKELANGELO_H
#define ROBOT_MAKELANGELO_H
//------------------------------------------------------------------------------
// Makelangelo - a mural drawing robot
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/Makelangelo for more information.
//------------------------------------------------------------------------------


#ifdef ZARPLOTTER


#define MAKELANGELO_HARDWARE_VERSION 6
//#define MOTHERBOARD BOARD_RUMBA
#define MOTHERBOARD BOARD_RAMPS

#define NUM_MOTORS           (4)
#define NUM_SERVOS           (1)
#define NUM_TOOLS            (1)

#define MAX_FEEDRATE         (15000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (100)
#define MAX_JERK             (15.0)
#define DEFAULT_FEEDRATE     (10000.0)
#define DEFAULT_ACCELERATION (3500)

#define ZARPLOTTER_MOTOR_SIZE   (4.5f)
#define ZARPLOTTER_PLOTTER_SIZE (6.0f)
#define ZARPLOTTER_COMPENSATION (ZARPLOTTER_PLOTTER_SIZE/2.0f + ZARPLOTTER_MOTOR_SIZE)


#endif  // ZARPLOTTER


#endif  // #ifndef ROBOT_MAKELANGELO_H

