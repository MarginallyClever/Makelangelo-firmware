#ifndef LCD_H
#define LCD_H
//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Please see http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#ifdef HAS_LCD

#include <Arduino.h>

//----------------------------------------------------

#ifdef LCD_IS_128X64

#define LCD_PIXEL_HEIGHT   64
#define LCD_PIXEL_WIDTH    128

// depends on the font selected
#define FONT_HEIGHT        9
#define FONT_WIDTH         6

// # of characters
#define LCD_HEIGHT         (LCD_PIXEL_HEIGHT/FONT_HEIGHT)  // 64/9=7
#define LCD_WIDTH          (LCD_PIXEL_WIDTH/FONT_WIDTH)    // 128/6=21

#include "dogm_font_data_6x9.h"

#endif

//----------------------------------------------------

#ifdef LCD_IS_SMART

// there is no practical per-pixel control.
//#define LCD_PIXEL_HEIGHT   ?
//#define LCD_PIXEL_WIDTH    ?

// there is no font selection.
//#define FONT_HEIGHT        ?
//#define FONT_WIDTH         ?

// # of characters
#define LCD_HEIGHT         4
#define LCD_WIDTH          20

#endif

//------------------------------------------------------------------------------
// Stuff that's calculated automatically OR the same for all models.
//------------------------------------------------------------------------------


#define BLEN_C             2
#define BLEN_B             1
#define BLEN_A             0
  
#define ENCROT0            0
#define ENCROT1            2
#define ENCROT2            3
#define ENCROT3            1

#define LCD_MESSAGE_LENGTH (LCD_HEIGHT * LCD_WIDTH + 1)  // we have two lines of 20 characters avialable in 7.16
#define LCD_DRAW_DELAY     (100)
#define LCD_TURN_PER_MENU  (3)  // was 5

extern char lcd_message[LCD_MESSAGE_LENGTH+1];
extern uint8_t speed_adjust;

#endif // HAS_LCD

extern void LCD_update();
extern void LCD_setup();

#endif // LCD_H
