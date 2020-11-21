// ---------------------------------------------------------------------------
// Created by Francisco Malpartida on 20/08/11.
// Copyright (C) - 2018
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License v3.0
//    along with this program.
//    If not, see <https://www.gnu.org/licenses/gpl-3.0.en.html>.
// 
// ---------------------------------------------------------------------------
//
// Thread Safe: No
// Extendable: Yes
//
// @file LiquidCrystal.h
// This file implements a basic liquid crystal library that comes as standard
// in the Arduino SDK.
// 
// @brief 
// This is a basic implementation of the LiquidCrystal library of the
// Arduino SDK. The original library has been reworked in such a way that 
// this class implements the all methods to command an LCD based
// on the Hitachi HD44780 and compatible chipsets using the parallel port of
// the LCD (4 bit and 8 bit).
//
//
//
// @author F. Malpartida - fmalpartida@gmail.com
// ---------------------------------------------------------------------------
#ifndef LiquidCrystal_4bit_h
#define LiquidCrystal_4bit_h

#include <inttypes.h>

#include "LCD.h"
#include "FastIO.h"


/*!
 @defined 
 @abstract   Command execution time on the LCD.
 @discussion This defines how long a command takes to execute by the LCD.
 The time is expressed in micro-seconds.
 */
#define EXEC_TIME 37

class LiquidCrystal : public LCD
{
public:
   /*!
    @method     
    @abstract   8 bit LCD constructors.
    @discussion Defines the pin assignment that the LCD will have.
    The constructor does not initialize the LCD.
    */
   LiquidCrystal(uint8_t rs, uint8_t enable,
                 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                 uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
   LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable,
                 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                 uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
   
   // Constructors with backlight control
   LiquidCrystal(uint8_t rs, uint8_t enable,
                 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                 uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7,
                 uint8_t backlightPin, t_backlightPol pol);
   LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable,
                 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                 uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7,
                 uint8_t backlightPin, t_backlightPol pol);   
   /*!
    @method     
    @abstract   4 bit LCD constructors.
    @discussion Defines the pin assignment that the LCD will have.
    The constructor does not initialize the LCD.
    */
   LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable,
                 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
   LiquidCrystal(uint8_t rs, uint8_t enable,
                 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
   
   // Constructors with backlight control
   LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable,
                 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                 uint8_t backlightPin, t_backlightPol pol);
   LiquidCrystal(uint8_t rs, uint8_t enable,
                 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
                 uint8_t backlightPin, t_backlightPol pol);
   /*!
    @function
    @abstract   Send a particular value to the LCD.
    @discussion Sends a particular value to the LCD for writing to the LCD or
    as an LCD command.
    
    Users should never call this method.
    
    @param      value Value to send to the LCD.
    @result     mode LOW - write to the LCD CGRAM, HIGH - write a command to
    the LCD.
    */
   virtual void send(uint8_t value, uint8_t mode);
   
   /*!
    @function
    @abstract   Sets the pin to control the backlight.
    @discussion Sets the pin in the device to control the backlight.
    
    @param      pin: pin assigned to the backlight
    @param      pol: backlight pin control polarity (POSITIVE, NEGATIVE).
    */
   void setBacklightPin ( uint8_t pin, t_backlightPol pol );
   
#if defined(ARDUINO_ARCH_ESP32)
   /*!
    @function
    @abstract   Wrapper around ESP32-hal-ledcWrite.
    @discussion The ESP32 MCU does not have the analogWrite() function.
    This wrapper converts the analogWrite() function to the ledcWrite()
    on an ESP32 MCU.

    @param      channel: analog pin
    @param      value: from 0 to valueMax
    @param      valueMax: default: 255, Max: 1023
   */
   void analogWrite( uint8_t channel, uint32_t value, uint32_t valueMax );
#endif

   /*!
    @function
    @abstract   Switch-on/off the LCD backlight.
    @discussion Switch-on/off the LCD backlight.
    The setBacklightPin has to be called before setting the backlight for
    this method to work. @see setBacklightPin. For dimming control of the
    backlight, the configuration pin must be a PWM output pin. Dim control
    is achieved by passing a value from 1 to 255 as a parameter. If the
    pin configured when calling the setBacklightPin does not support PWM,
    then: (0) backlight off, (1..255) backlight on.
    
    @param      value: backlight value. 0: off, 1..255: dim control of the 
    backlight. For negative logic 255: off, 254..0: dim control.
    */
   void setBacklight ( uint8_t value );
   
private:
   
   /*!
    @method     
    @abstract   Initializes the LCD pin allocation and associated HW
    @discussion Initializes the LCD pin allocation and configuration.
    */
   void init(uint8_t fourbitmode, uint8_t rs, uint8_t rw, uint8_t enable,
             uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
             uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
   
   /*!
    @method     
    @abstract   Writes numBits bits from value value to the LCD.
    @discussion Writes numBists bits (the least significant) to the LCD control 
    data lines.
    */   
   void writeNbits(uint8_t value, uint8_t numBits);
   
   /*!
    @method     
    @abstract   Pulse the LCD enable line (En).
    @discussion Sends a pulse of 1 uS to the Enable pin to execute an command
    or write operation.
    */ 
   void pulseEnable();
   
   uint8_t _rs_pin;       // LOW: command.  HIGH: character.
   uint8_t _rw_pin;       // LOW: write to LCD.  HIGH: read from LCD.
   uint8_t _enable_pin;   // activated by a HIGH pulse.
   uint8_t _data_pins[8]; // Data pins.
   uint8_t _backlightPin; // Pin associated to control the LCD backlight
};

#endif
