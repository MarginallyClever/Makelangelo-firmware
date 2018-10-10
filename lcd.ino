//------------------------------------------------------------------------------
// Makelangelo - firmware for various robot kinematic models
// dan@marginallycelver.com 2013-12-26
// Copyright at end of file.  Please see
// http://www.github.com/MarginallyClever/makelangeloFirmware for more information.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#ifdef HAS_LCD
#include "lcd.h"

#include "sdcard.h"

#ifdef LCD_IS_SMART
#include <LiquidCrystal.h>
#endif
#ifdef LCD_IS_128X64
#include <Arduino.h>
#include <U8glib.h>
#include <SPI.h>
#include <Wire.h>
#include "dogm_font_data_6x9.h"
#endif

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
#ifdef LCD_IS_SMART
LiquidCrystal lcd(LCD_PINS_RS, LCD_PINS_ENABLE, LCD_PINS_D4, LCD_PINS_D5, LCD_PINS_D6, LCD_PINS_D7);
#endif
#ifdef LCD_IS_128X64
// This is not ideal - will not work when board models change.
U8GLIB_ST7920_128X64_1X u8g(LCD_PINS_D4,LCD_PINS_ENABLE,LCD_PINS_RS);


/**
 * Made with Marlin Bitmap Converter
 * http://marlinfw.org/tools/u8glib/converter.html
 *
 * This bitmap from the file 'icon_mono.png'
 * Filesize: 906 Bytes
 * Size bitmap: 108 bytes
 */
#define logoImageWidth 32
#define logoImageHeight 32

const unsigned char logoImage [] PROGMEM= {
  0x00,0x60,0x00,0x00, // .........##.....................
  0x00,0xF0,0x00,0x00, // ........####....................
  0x03,0x10,0x00,0x00, // ......##...#....................
  0x01,0x60,0x00,0x00, // .......#.##.....................
  0x01,0xC0,0x00,0x00, // .......###......................
  0x00,0x9F,0xF0,0x00, // ........#..#########............
  0x00,0xF0,0x3C,0x00, // ........####......####..........
  0x01,0xC0,0x3F,0x00, // .......###........######........
  0x03,0x00,0x1F,0x80, // ......##...........######.......
  0x06,0x00,0x1F,0xC0, // .....##............#######......
  0x0C,0x02,0x0F,0xE0, // ....##........#.....#######.....
  0x08,0x06,0x0F,0xE0, // ....#........##.....#######.....
  0x18,0x02,0x0F,0xF0, // ...##.........#.....########....
  0x10,0x02,0x0F,0xF0, // ...#..........#.....########....
  0x31,0x80,0x07,0xF0, // ..##...##............#######....
  0x23,0x80,0x07,0xF8, // ..#...###............########...
  0x23,0x00,0x07,0xF8, // ..#...##.............########...
  0x23,0x00,0x07,0xF8, // ..#...##.............########...
  0x23,0x00,0x07,0xF8, // ..#...##.............########...
  0x21,0x80,0x07,0xF8, // ..#....##............########...
  0x20,0x00,0x07,0xF8, // ..#..................########...
  0x30,0x00,0x37,0xF8, // ..##..............##.########...
  0x30,0x00,0x63,0xF0, // ..##.............##...######....
  0x10,0x00,0x63,0xF0, // ...#.............##...######....
  0x18,0x00,0x63,0xF0, // ...##............##...######....
  0x08,0x60,0x37,0xE0, // ....#....##.......##.######.....
  0x04,0x20,0x0F,0xC0, // .....#....#.........######......
  0x06,0x00,0x1F,0xC0, // .....##............#######......
  0x03,0x00,0x1F,0x80, // ......##...........######.......
  0x00,0xC0,0x3E,0x00, // ........##........#####.........
  0x00,0x70,0x3C,0x00, // .........###......####..........
  0x00,0x1F,0xE0,0x00  // ...........########.............
};
#endif

//------------------------------------------------------------------------------
// MACROS
//------------------------------------------------------------------------------

/**
 * Clear the screen
 */
#ifdef LCD_IS_SMART
#define LCD_clear()      lcd.clear()
#endif
#ifdef LCD_IS_128X64
void LCD_clear() {
  //u8g.firstPage(); 
  //while( u8g.nextPage() );
  u8g.setColorIndex(0);
  u8g.drawBox(0,0,LCD_PIXEL_WIDTH,LCD_PIXEL_HEIGHT);
  u8g.setColorIndex(1);
}
#endif

/**
 * print text to the LCD
 */
#ifdef LCD_IS_SMART
#define LCD_print      lcd.print
#endif
#ifdef LCD_IS_128X64
#define LCD_print      u8g.print
#endif


/**
 * Set the row/column of text at which to begin printing
 */
#ifdef LCD_IS_SMART
#define LCD_setCursor(x,y)   lcd.setCursor(x,y)
#endif
#ifdef LCD_IS_128X64
#define LCD_setCursor(x,y)   u8g.setPrintPos(((x)+1)*FONT_WIDTH,((y)+1)*FONT_HEIGHT)
#endif



// Convenience macros that make it easier to generate menus

#define MENU_START \
  ty=0;

#define MENU_END \
  num_menu_items=ty; \

#define MENU_ITEM_START(key) \
  if(ty>=screen_position && ty<screen_end) { \
    LCD_setCursor(0,ty-screen_position); \
    LCD_print((menu_position==ty)?'>':' '); \
    LCD_print(key); \

#define MENU_ITEM_END() \
  } \
  ++ty;

#define MENU_GOTO(new_menu) {  \
    lcd_click_now=false;  \
    LCD_clear();  \
    num_menu_items=screen_position=menu_position=menu_position_sum=0;  \
    screen_end = screen_position + LCD_HEIGHT;  \
    current_menu=new_menu;  \
    return;  \
  }

#define MENU_LABEL(menu_label) \
  MENU_ITEM_START(menu_label) \
  MENU_ITEM_END()
  
#define MENU_SUBMENU(menu_label,menu_method) \
  MENU_ITEM_START(menu_label) \
  if(menu_position==ty && lcd_click_now) MENU_GOTO(menu_method); \
  MENU_ITEM_END()

#define MENU_ACTION(menu_label,menu_method) MENU_SUBMENU(menu_label,menu_method)

#define MENU_LONG(key,value) \
  MENU_ITEM_START(key) \
  LCD_print_long(value); \
  if(menu_position==ty && lcd_click_now) LCD_update_long(key,value); \
  MENU_ITEM_END()

#define MENU_FLOAT(key,value) \
  MENU_ITEM_START(key) \
  LCD_print_float(value); \
  if(menu_position==ty && lcd_click_now) LCD_update_float(key,value); \
  MENU_ITEM_END()


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

long lcd_draw_delay = 0;

int lcd_rot_old  = 0;
int lcd_turn     = 0;
char lcd_click_old = HIGH;
char lcd_click_now = false;
uint8_t speed_adjust = 100;
char lcd_message[LCD_MESSAGE_LENGTH+1];

int menu_position_sum = 0, menu_position = 0, screen_position = 0, num_menu_items = 0, ty, screen_end;


void (*current_menu)();


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


void LCD_read() {
  // detect potentiometer changes
  int rot = ((digitalRead(BTN_EN1) == LOW) << BLEN_A)
          | ((digitalRead(BTN_EN2) == LOW) << BLEN_B);
  // potentiometer uses grey code.  Pattern is 0 3 
  switch (rot) {
    case ENCROT0:
      if ( lcd_rot_old == ENCROT3 ) lcd_turn++;
      if ( lcd_rot_old == ENCROT1 ) lcd_turn--;
      break;
    case ENCROT1:
      if ( lcd_rot_old == ENCROT0 ) lcd_turn++;
      if ( lcd_rot_old == ENCROT2 ) lcd_turn--;
      break;
    case ENCROT2:
      if ( lcd_rot_old == ENCROT1 ) lcd_turn++;
      if ( lcd_rot_old == ENCROT3 ) lcd_turn--;
      break;
    case ENCROT3:
      if ( lcd_rot_old == ENCROT2 ) lcd_turn++;
      if ( lcd_rot_old == ENCROT0 ) lcd_turn--;
      break;
  }
  if(lcd_rot_old != rot) {
    //if(lcd_turn !=0) Serial.print(lcd_turn>0?'+':'-');  // for debugging potentiometer
    //Serial.println(rot);  // for debugging potentiometer
  }
  
  lcd_rot_old = rot;

  
  // find click state
  int btn = digitalRead(BTN_ENC);
  if ( btn != lcd_click_old && btn == HIGH ) {
    lcd_click_now = true;
  }
  lcd_click_old = btn;
}


// display the current machine position and feed rate on the LCD.
void LCD_status_menu() {
  MENU_START

  // on click go to the main menu
  if (lcd_click_now) MENU_GOTO(LCD_main_menu);

  if (lcd_turn) {
    speed_adjust += lcd_turn;
  }

  // update the current status
  float offset[NUM_AXIES];
  get_end_plus_offset(offset);
  LCD_setCursor( 0, 0);  LCD_print('X');  LCD_print_float(offset[0]);
  LCD_setCursor(10, 0);  LCD_print('Z');  LCD_print_float(offset[2]);
#if MACHINE_STYLE == POLARGRAPH && defined(USE_LIMIT_SWITCH)
  LCD_setCursor(19, 0);  LCD_print(( digitalRead(LIMIT_SWITCH_PIN_LEFT) == LOW ) ? '*':' ');
#endif

  LCD_setCursor( 0, 1);  LCD_print('Y');  LCD_print_float(offset[1]);
  LCD_setCursor(10, 1);  LCD_print('F');  LCD_print_long(speed_adjust);  LCD_print(F("% "));
#if MACHINE_STYLE == POLARGRAPH && defined(USE_LIMIT_SWITCH)
  LCD_setCursor(19, 1);  LCD_print(( digitalRead(LIMIT_SWITCH_PIN_RIGHT) == LOW ) ? '*':' ');
#endif
  
  
  //LCD_setCursor(10, 1);  LCD_print('F');  LCD_print_float(feed_rate);
  //LCD_setCursor( 0, 1);  LCD_print(F("Makelangelo #"));  LCD_print(robot_uid);
  LCD_setCursor( 0, 2);
  if (sd_printing_now == true/* && sd_printing_paused==false*/) {
    LCD_print_float(sd_percent_complete);
    LCD_print('%');
  } else {
    LCD_print(F("          "));
  }
  LCD_print_message();
  MENU_END
}


void LCD_main_menu() {
  MENU_START

  MENU_SUBMENU("Back", LCD_status_menu);
  if (!sd_printing_now) {
#if MACHINE_HARDWARE_VERSION  == 5
    MENU_ACTION("Find home", LCD_find_home);
#else
    MENU_ACTION("This is home", LCD_this_is_home);
    MENU_ACTION("Go home", LCD_go_home);
#endif
    if (sd_inserted) {
      MENU_SUBMENU("Print from SD card...", LCD_start_menu);
    } else {
      MENU_LABEL("No SD card");
    }
    MENU_SUBMENU("Draw border", LCD_draw_border);
    MENU_SUBMENU("Drive", LCD_drive_menu);
  } else {
    if (sd_printing_paused) {
      MENU_ACTION("Unpause", LCD_pause);
    } else {
      MENU_ACTION("Pause", LCD_pause);
    }
    MENU_ACTION("Stop", LCD_stop);
  }
  MENU_END
}


void LCD_print_message() {
  lcd_message[LCD_MESSAGE_LENGTH - 1] = 0;
  LCD_setCursor( 0, 2);  LCD_print(lcd_message);
  LCD_setCursor( 0, 3);  LCD_print(lcd_message + LCD_WIDTH + 1);
}


void LCD_pause() {
  sd_printing_paused = (sd_printing_paused == true ? false : true);
  MENU_GOTO(LCD_main_menu);
}


void LCD_stop() {
  sd_printing_now = false;
  MENU_GOTO(LCD_main_menu);
}

void LCD_disable_motors() {
  motor_disengage();
  MENU_GOTO(LCD_main_menu);
}

void LCD_enable_motors() {
  motor_engage();
  MENU_GOTO(LCD_main_menu);
}


void LCD_find_home() {
  robot_findHome();
  MENU_GOTO(LCD_main_menu);
}


void LCD_this_is_home() {
  float offset[NUM_AXIES];
  for (int i = 0; i < NUM_AXIES; ++i) offset[i] = axies[i].homePos;
  teleport(offset);
  MENU_GOTO(LCD_main_menu);
}


void LCD_go_home() {
  float homes[NUM_AXIES];
  for (int i = 0; i < NUM_AXIES; ++i) homes[i] = axies[i].homePos;
  lineSafe( homes, DEFAULT_FEEDRATE );
  MENU_GOTO(LCD_main_menu);
}


void LCD_drive_menu() {
  MENU_START
  MENU_SUBMENU("Back", LCD_main_menu);
  MENU_ACTION("Disable motors", LCD_disable_motors);
  MENU_ACTION("Enable motors", LCD_enable_motors);
  MENU_SUBMENU("X", LCD_driveX);
  MENU_SUBMENU("Y", LCD_driveY);
  MENU_SUBMENU("Z", LCD_driveZ);
  MENU_SUBMENU("Feedrate", LCD_driveF);
  MENU_END
}


void LCD_driveX() {
  if (lcd_click_now) MENU_GOTO(LCD_drive_menu);

  float offset[NUM_AXIES];
  get_end_plus_offset(offset);
  
  if (lcd_turn) {
    offset[0] += lcd_turn > 0 ? 1 : -1;
    lineSafe(offset, feed_rate);
  }

  LCD_setCursor( 0, 0);
  LCD_print('X');
  LCD_print_float(offset[0]);
}


void LCD_driveY() {
  if (lcd_click_now) MENU_GOTO(LCD_drive_menu);

  float offset[NUM_AXIES];
  get_end_plus_offset(offset);

  if (lcd_turn) {
    offset[1] += lcd_turn > 0 ? 1 : -1;
    lineSafe(offset, feed_rate);
  }

  LCD_setCursor( 0, 0);
  LCD_print('Y');
  LCD_print_float(offset[1]);
}


void LCD_driveZ() {
  if (lcd_click_now) MENU_GOTO(LCD_drive_menu);

  float offset[NUM_AXIES];
  get_end_plus_offset(offset);

  if (lcd_turn) {
    // protect servo, don't drive beyond physical limits
    offset[2] += lcd_turn > 0 ? 1 : -1;
    lineSafe(offset, feed_rate);
  }

  LCD_setCursor( 0, 0);
  LCD_print('Z');
  LCD_print_float(offset[2]);
}


void LCD_driveF() {
  if (lcd_click_now) MENU_GOTO(LCD_drive_menu);

  if (lcd_turn) {
    // protect servo, don't drive beyond physical limits
    float newF = feed_rate + lcd_turn > 0 ? 1 : -1;
    if (newF < MIN_FEEDRATE) newF = MIN_FEEDRATE;
    if (newF > MAX_FEEDRATE) newF = MAX_FEEDRATE;
    // move
    feed_rate = newF;
  }

  LCD_setCursor( 0, 0);
  LCD_print('F');
  LCD_print_float(feed_rate);
}


void LCD_start_menu() {
  if (!sd_inserted) MENU_GOTO(LCD_main_menu);

  MENU_START
  MENU_SUBMENU("Back", LCD_main_menu);
  /*
    Serial.print(menu_position    );  Serial.print("\t");  // 0
    Serial.print(menu_position_sum);  Serial.print("\t");  // 1
    Serial.print(screen_position  );  Serial.print("\t");  // 0
    Serial.print(num_menu_items   );  Serial.print("\n");  // 8
  */

  root.rewindDirectory();
  while ( true ) {
    //long tStart = millis();
    File entry = root.openNextFile();
    //long tEnd = millis();
    //Serial.print(tEnd-tStart);
    //Serial.print('\t');
    if (!entry) {
      // no more files, return to the first file in the directory
      break;
    }
    char *filename = entry.name();
    //Serial.print( entry.isDirectory()?">":" " );
    //Serial.println(filename);/*
    if (!entry.isDirectory() && filename[0] != '_') {
      MENU_ITEM_START(filename)
      if (menu_position == ty && lcd_click_now) {
        lcd_click_now = false;
        SD_StartPrintingFile(filename);
        MENU_GOTO(LCD_status_menu);
      }
      MENU_ITEM_END()
    }
    entry.close();
  }

  //Serial.println();


  MENU_END
}


void LCD_draw_border() {
  MENU_START
    MENU_ACTION("A2 portrait",draw_A2_p);
    MENU_ACTION("A3 portrait",draw_A3_p);
    MENU_ACTION("A4 portrait",draw_A4_p);
    MENU_ACTION("A5 portrait",draw_A5_p);
    MENU_ACTION("US legal portrait",draw_USlegal_p);
    MENU_ACTION("US letter portrait",draw_USletter_p);
  
    MENU_ACTION("A2 landscape",draw_A2_l);
    MENU_ACTION("A3 landscape",draw_A3_l);
    MENU_ACTION("A4 landscape",draw_A4_l);
    MENU_ACTION("A5 landscape",draw_A5_l);
    MENU_ACTION("US legal landscape",draw_USlegal_l);
    MENU_ACTION("US letter landscape",draw_USletter_l);
  MENU_END
}

void draw_A2_p() {  draw_border(420,594,0);  }
void draw_A3_p() {  draw_border(297,420,0);  }
void draw_A4_p() {  draw_border(210,297,0);  }
void draw_A5_p() {  draw_border(148,210,0);  }
void draw_USletter_p() {  draw_border(216,279,0);  }
void draw_USlegal_p() {  draw_border(216,356,0);  }
  
void draw_A2_l() {  draw_border(420,594,1);  }
void draw_A3_l() {  draw_border(297,420,1);  }
void draw_A4_l() {  draw_border(210,297,1);  }
void draw_A5_l() {  draw_border(148,210,1);  }
void draw_USletter_l() {  draw_border(216,279,1);  }
void draw_USlegal_l() {  draw_border(216,356,1);  }


void draw_border(int width,int height,int landscape) {
  LCD_clear();
  LCD_setCursor(0, 0);
  LCD_print("Drawing border...");
    
  width /= 2;
  height /= 2;

  // get start position
  float start[NUM_AXIES];
  get_end_plus_offset(start);
  
  float pos[NUM_AXIES];
  
  // lift pen
  pos[2]=PEN_UP_ANGLE;  lineSafe( pos, feed_rate );
  // move to first corner
  pos[0] = -width;  pos[1] =  height;  lineSafe( pos, feed_rate );
  // lower pen
  pos[2]=PEN_DOWN_ANGLE;  lineSafe( pos, feed_rate );
  // move around border
  pos[0] =  width;  pos[1] =  height;  lineSafe( pos, feed_rate );
  pos[0] =  width;  pos[1] = -height;  lineSafe( pos, feed_rate );
  pos[0] = -width;  pos[1] = -height;  lineSafe( pos, feed_rate );
  pos[0] = -width;  pos[1] =  height;  lineSafe( pos, feed_rate );
  // lift pen
  pos[2]=PEN_UP_ANGLE;  lineSafe( pos, feed_rate );
  
  // return to start position
  lineSafe( start, feed_rate );

  MENU_GOTO(LCD_draw_border);
}


void LCD_update_long(char *label, long &value) {
  LCD_clear();
  do {
    LCD_read();
    if ( lcd_turn ) {
      value += lcd_turn > 0 ? 1 : -1;
      lcd_turn = 0;
    }
    LCD_setCursor(0, 0);
    LCD_print(label);
    LCD_setCursor(0, 1);
    LCD_print_long(value);
  } while ( !lcd_click_now );
}


void LCD_update_float(char *label, float &value,float stepSize=0.01) {
  LCD_clear();
  do {
    LCD_read();
    if ( lcd_turn ) {
      value += lcd_turn > 0 ? stepSize : -stepSize;
      lcd_turn = 0;
    }
    LCD_setCursor(0, 0);
    LCD_print(label);
    LCD_setCursor(0, 1);
    LCD_print_float(value);
  } while ( !lcd_click_now );
}


void LCD_print_long(long v) {
  long av = abs(v);
  int x = 1000;
  while (x > av && x > 1) {
    LCD_print(' ');
    x /= 10;
  };
  if (v > 0) LCD_print(' ');
  LCD_print(v);
}



void LCD_print_float(float v) {
  int left = abs(v);
  int right = abs((int)(v * 100) % 100);

  if (left < 1000) LCD_print(' ');
  if (left < 100) LCD_print(' ');
  if (left < 10) LCD_print(' ');
  if (v < 0) LCD_print('-');
  else LCD_print(' ');
  LCD_print(left);
  LCD_print('.');
  if (right < 10) LCD_print('0');
  LCD_print(right);
}
#endif  // HAS_LCD




void LCD_update() {
#ifdef HAS_LCD
  LCD_read();

  if (millis() >= lcd_draw_delay ) {
    lcd_draw_delay = millis() + LCD_DRAW_DELAY;

    //Serial.print(lcd_turn,DEC);
    //Serial.print('\t');  Serial.print(menu_position,DEC);
    //Serial.print('\t');  Serial.print(menu_position_sum,DEC);
    //Serial.print('\t');  Serial.print(screen_position,DEC);
    //Serial.print('\t');  Serial.print(screen_end,DEC);
    //Serial.print('\t');  Serial.print(num_menu_items,DEC);
    //Serial.print('\n');

    // update the menu position
    if ( lcd_turn && num_menu_items>1 ) {
      int originalPosition = menu_position_sum / LCD_TURN_PER_MENU;
      menu_position_sum += lcd_turn;
  
      menu_position = menu_position_sum / LCD_TURN_PER_MENU;
      if (menu_position > num_menu_items - 1) menu_position = num_menu_items - 1;
      if (menu_position < 0) menu_position = 0;
  
      if (originalPosition != menu_position) {
        LCD_clear();
      }

      Serial.println(menu_position);
  
      if (screen_position > menu_position) screen_position = menu_position;
      if (screen_position < menu_position - (LCD_HEIGHT - 1)) screen_position = menu_position - (LCD_HEIGHT - 1);
      screen_end = screen_position + LCD_HEIGHT;
    }

    // draw the new screen contents
#ifdef LCD_IS_128X64
  u8g.firstPage();
  do {
#endif
    (*current_menu)();
#ifdef LCD_IS_128X64
  } while(u8g.nextPage());
#endif

    lcd_turn = 0;  // reset.  must come after (*current_menu)() because it might be used there.  (damn globals...)
  }
#endif  // HAS_LCD
}

// initialize the Smart controller LCD panel
void LCD_init() {
#ifdef HAS_LCD

#ifdef LCD_IS_SMART
  lcd.begin(LCD_WIDTH, LCD_HEIGHT);
#endif
#ifdef LCD_IS_128X64
  u8g.begin();
  u8g.disableCursor();
  u8g.setFont(u8g_font_6x9);
//  u8g.setFont(u8g_font_helvR08);
#endif

  pinMode(BEEPER,OUTPUT);
  digitalWrite(BEEPER,LOW);

  pinMode(BTN_EN1, INPUT);
  pinMode(BTN_EN2, INPUT);
  pinMode(BTN_ENC, INPUT);
  digitalWrite(BTN_EN1, HIGH);
  digitalWrite(BTN_EN2, HIGH);
  digitalWrite(BTN_ENC, HIGH);
  current_menu = LCD_status_menu;
  menu_position_sum = 1;  /* 20160313-NM-Added so the clicking without any movement will display a menu */

  lcd_message[0]=0;

  LCD_drawSplash();
  //LCD_drawSplash();

#endif  // HAS_LCD
}


#ifdef HAS_LCD
void LCD_drawSplash() {
  // splash screen
  char message[LCD_WIDTH];
  char mhv[10];
  itoa(MACHINE_HARDWARE_VERSION,mhv,10);
  char *ptr = message;
  strcpy(ptr, MACHINE_STYLE_NAME );  ptr += strlen(MACHINE_STYLE_NAME);
  strcpy(ptr, " v" );                ptr += strlen(" v");
  strcpy(ptr, mhv );                 ptr += strlen(mhv);

  int x = (LCD_WIDTH - strlen(message)) / 2;
  int y = LCD_HEIGHT-2;
  int x2 = (LCD_WIDTH - strlen("marginallyclever.com")) / 2;
  int y2 = y+1;

#ifdef LCD_IS_128X64
  u8g.firstPage();
  do {
#endif
    LCD_clear();
#ifdef LCD_IS_128X64
    u8g.drawBitmapP((LCD_PIXEL_WIDTH-logoImageWidth)/2,0,logoImageWidth/8,logoImageHeight,logoImage);
#endif
    LCD_setCursor(x,y);
    LCD_print(message);
    LCD_setCursor(x2,y2);
    LCD_print("marginallyclever.com");
#ifdef LCD_IS_128X64
  } while(u8g.nextPage());
#endif

  delay(2500);
  LCD_clear();
}
#endif


/**
   This file is part of makelangelo-firmware.

   makelangelo-firmware is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   makelangelo-firmware is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with makelangelo-firmware.  If not, see <http://www.gnu.org/licenses/>.
*/
