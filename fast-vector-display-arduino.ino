/*
Fast Vector Display for Arduino
Copyright (C) 2018 Marcio Teixeira

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <float.h>
#include <math.h>
#include <limits.h>
#include <EEPROM.h>

/* Please select a pre-defined pattern or animation:
 *    1 - Merry Christmas
 *    2 - Flag
 *    3 - Triangle
 *    4 - Spiral
 *    5 - Happy New Year
 *    6 - Drunken Octopus TV Intro
 */
#define PATTERN 5

/* Or, to generate your own, uncomment the following and paste in coordinates from
 * an Inkscape SVG path here:
 */
//#define USER_PATTERN {0,0,-9.31445,70.46485,0,-58.36914,-30.15625,0,-5.27344,41.52929,-3.1875,-22.63281,-2.80078,-18.89648,-29.9375,0,0,88.93359,20.21485,0,0,-58.66601,8.51367,58.66601,14.33789,0,8.07422,-60.03906,0,60.03906,20.21484,0,0,-12.0957,21.50977,0,1.37304,-15.98438,2.54102,0,0,-15.76562,-2.81641,0,4.39453,-37.29883,3.51563,37.29883,-2.67578,0,0,15.76562,3.33594,0,1.15234,15.98438,21.98633,0,0,27.41992,23.125,0,0,-38.12305,9.33984,38.12305,23.89453,0,-14.50195,-48.7793,10.50195,-31.85547,0,54.02149,40.10156,0,0,-17.79883,-16.9746,0,0,-19.55469,14.44726,0,0,-16.91992,-14.44726,0,0,-16.86328,15.43554,0,0,-17.79883,-38.5625,0,0,26.61328,-18.96094,0,-10.27343,34.55274,0,-34.55274,-23.125,0,0,50.16602,-11.54883,-77.58399}

/* If your custom pattern uses relative coordinates, such as those obtained
 * from an Inkscape SVG, then leave the following defined. Otherwise, if you
 * wish to use absolute coordinates, comment out the following line: 
 */
#define USER_PATTERN_USES_RELATIVE_POINTS

/* The RAPID_SWEEP_MODE is extremely sensitive to the RC constant,
 * so configure these values here:
 */
#define R     150   // Resistance in ohms
#define C     10e-6 // Capacitance in farads

/* Pins 11 and 3 will provide the best performance. The other possibility
 * is pins 5 and 6, but these will increase the CPU load when servicing
 * the millis() and micros() interrupt.
 */
#define X_PIN  11    // Recommended: Pin 11   Alternate: Pin 5
#define Y_PIN   3    // Recommended: Pin 3    Alternate: Pin 6

/* Setting DOMAIN_RESTRICTION to 1 restricts the analog voltage
 * range to 1-254, which a significant decrease in flicker.
 */
#define DOMAIN_RESTRICTION 2

/* The FIXED_RC_MODE is an old method which is much slower
 * and uses more memory You shouldn't use it.
 */
//#define FIXED_RC_MODE

#define FRAMEBUFFER_SIZE 340 // Number of points in framebuffer

#define SHOW_FRAMERATE

#include "fast-vector-display-arduino.h"

/****************************** TIMESCALE CONFIGURATION ******************************/

#if X_PIN == 5 && Y_PIN == 6
  /* When using Timer 0, this will disrupt the millis()
  /* and micros() timescale, thus it is recommended to
  /* use pins 11 and 3 instead. */
  #define TIMER0   0x01
#endif

#if TIMER0 == 0x01
    // Pins 5 & 6: 62500 Hz
    #define DELAY_1S 64000
    #define MICRO_1S (DELAY_1S*1000)
#elif TIMER0 == 0x02
    // Pins 5 & 6: 7812.5 Hz
    #define DELAY_1S 8000
    #define MICRO_1S (DELAY_1S*1000)
#else
    // Pins 5 & 6: 976.5625 Hz (Default)
    #define DELAY_1S 1000
    #define MICRO_1S (DELAY_1S*1000)
#endif

#define ARDUINO_CLOCK_SPEED 16000000
#define TIMER1_PRESCALER        8
#define _TIMER1_SECS_TO_TICKS(secs, prescaler) (float(secs)*(ARDUINO_CLOCK_SPEED/prescaler)) - 1
#define _TIMER1_TICKS_TO_SECS(ticks, prescaler) (float(ticks+1)/(ARDUINO_CLOCK_SPEED/prescaler))
#define TIMER1_SECS_TO_TICKS(secs)  _TIMER1_SECS_TO_TICKS(secs,  TIMER1_PRESCALER)
#define TIMER1_TICKS_TO_SECS(ticks) _TIMER1_TICKS_TO_SECS(ticks, TIMER1_PRESCALER)

#define TICKS_TO_SECS(ticks) float(ticks)/MICRO_1S
#define SECS_TO_TICKS(secs)  (unsigned long)((secs) * MICRO_1S)
#define BENCH_SECS_TO_TICKS(secs)  (float(secs)*(ARDUINO_CLOCK_SPEED/TIMER1_PRESCALER)) - 1

/********************************** UTILITY ROUTINES *********************************/

#define N_PTS(a) (sizeof(a)/sizeof(a[0])/2)

// Graphical buffer types
#define DATA_FLOATS        0b00
#define DATA_PROG_MEM      0b00
#define DATA_SYSTEM_MEM    0b01
#define DATA_ABSOLUTE      0b00
#define DATA_RELATIVE      0b10

bool unpack_points(const void *pts, uint16_t src_n, float &fx, float &fy,
  uint8_t options = DATA_PROG_MEM | DATA_ABSOLUTE | DATA_FLOATS
) {
  static float x = 0, y = 0;
  static uint16_t i = 0;
  static const float *f;
  if(i == 0) {
    f = (const float*) pts;
    x = 0;
    y = 0;
  }
  switch(options & (DATA_SYSTEM_MEM | DATA_RELATIVE)) {
    case 0:
      x = pgm_read_float_near(f++);
      y = pgm_read_float_near(f++);
      break;
    case DATA_SYSTEM_MEM:
      x = *f++;
      y = *f++;
      break;
    case DATA_RELATIVE:
      x += pgm_read_float_near(f++);
      y += pgm_read_float_near(f++);
      break;
    case DATA_RELATIVE | DATA_SYSTEM_MEM:
      x += *f++;
      y += *f++;
      break;
  }
  fx = x;
  fy = y;
  if(i++ == src_n) {
    i = 0;
    return false;
  } else {
    return true;
  }
}

void compute_rect(const void *src, uint16_t src_n, rect_t &bounds, uint8_t options = 0) {
  float max_x = FLT_MIN;
  float max_y = FLT_MIN;
  float min_x = FLT_MAX;
  float min_y = FLT_MAX;
  float x, y;
  while(unpack_points(src, src_n, x, y, options)) {
    max_x = max(x, max_x);
    max_y = max(y, max_y);
    min_x = min(x, min_x);
    min_y = min(y, min_y);
  }
  bounds.x = min_x;
  bounds.y = min_y;
  bounds.w = max_x - min_x;
  bounds.h = max_y - min_y;
}

void benchmark_start() {
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0b001; // No prescaler
  TIMSK1 = 0;
  TCNT1  = 0;     //initialize counter value to 0
}

uint16_t benchmark_end(const __FlashStringHelper* str) {
  uint16_t timer = TCNT1;
  sei(); // enable all interrupts
  Serial.print(str);
  Serial.println(timer);
  return timer;
}

void print_secs(float secs) {
  Serial.print(_TIMER1_SECS_TO_TICKS(secs, 1),0);
  Serial.print(F(" ticks ("));
  Serial.print(secs, 7);
  Serial.println(F(" secs)."));
}

void show_time_constants() {
  float L, f, t;
  
  Serial.print(F("RC Time Constant: "));
  print_secs(R*C);
  
  Serial.print(F("RC Time Constant times five: "));
  print_secs(5*R*C);

  t = rc_rise_time_ab(DOMAIN_RESTRICTION, 255 - DOMAIN_RESTRICTION);
  Serial.print(F("Max rise time: "));
  print_secs(t);
  if(t >= TIMER1_TICKS_TO_SECS(USHRT_MAX)) {
    Serial.println(F("  Warning: The maximum rise time is larger than the timer limit.\n\n"));
  }

  t = rc_fall_time_ab(255 - DOMAIN_RESTRICTION, DOMAIN_RESTRICTION);
  Serial.print(F("Max fall time: "));
  print_secs(t);
  if(t >= TIMER1_TICKS_TO_SECS(USHRT_MAX)) {
    Serial.println(F("  Warning: The maximum fall time is larger than the timer limit.\n\n"));
  }

  Serial.print(F("Min rise time: "));
  print_secs(rc_rise_time_ab(1, 2));

  Serial.print(F("Min fall time: "));
  print_secs(rc_fall_time_ab(255, 254));

  Serial.print(F("Low pass filter cut-off frequency: "));
  Serial.println(1/(2*PI*R*C));

  Serial.println();

  /* Note: The volatile arguments here are necessary since I found
   *       the compilre optimizes away rc_rise_time_ab completely!
   */
  volatile uint8_t a  = 0;
  volatile uint8_t b  = 20;
  volatile uint16_t c = 20;
  volatile uint16_t d = 0;
  volatile uint16_t uL = 0;
  volatile float vf1, vf2;
  volatile mat_t vM;
  volatile float fA, fB;
  mat_t m;
  
  benchmark_start();
  f = rc_rise_time_ab(a, b);
  benchmark_end(F("Ticks to compute rc_rise_time_ab: "));

  benchmark_start();
  uL = (c)/(b);
  benchmark_end(F("Ticks to compute 16-bit division: "));

  float f1 = vf1, f2 = vf2;
  benchmark_start();
  vf1 = f1 * f2;
  benchmark_end(F("Ticks to compute float multiplication: "));

  f1 = vf1, f2 = vf2;
  benchmark_start();
  vf1 = f1 / f2;
  benchmark_end(F("Ticks to compute float division: "));

  vf1 = 255;
  f1 = vf1;
  benchmark_start();
  vf2 = log(f1);
  benchmark_end(F("Ticks to compute float log: "));

  f1 = vf1, f2 = vf2;
  MAKE_MATRIX(m, vf1, vf2, vf1, vf2, vf1, vf2, vf1, vf2, vf1);
  benchmark_start();
  mat_apply(m,f1,f2);
  benchmark_end(F("Ticks to apply transformation matrix: "));
  vf1 = f1;
  vf2 = f2;
}

/**************************** DRAWING ROUTINES ***************************/

// Transforms all the points in the buffer
void draw_buffer(const void *src, uint16_t src_n, const mat_t &transform, uint8_t options = 0) {
  float x = 0, y = 0;
  while(unpack_points(src, src_n, x, y, options)) {
    mat_apply(transform, x, y);
    fb_scissor(x, y);
  }
}

/************************ TRANSFORMATION ROUTINES *************************/

void mat_premultiply(mat_t &m1, mat_t &m2) {
  mat_t r;
  r.a = (m1.a * m2.a) +  (m1.b * m2.d) +  m1.c * IF_FULL_MATRIX(m2.g, 0);
  r.b = (m1.a * m2.b) +  (m1.b * m2.e) +  m1.c * IF_FULL_MATRIX(m2.h, 0);
  r.c = (m1.a * m2.c) +  (m1.b * m2.f) +  m1.c * IF_FULL_MATRIX(m2.i, 1);
  
  r.d = (m1.d * m2.a) +  (m1.e * m2.d) +  m1.f * IF_FULL_MATRIX(m2.g, 0);
  r.e = (m1.d * m2.b) +  (m1.e * m2.e) +  m1.f * IF_FULL_MATRIX(m2.h, 0);
  r.f = (m1.d * m2.c) +  (m1.e * m2.f) +  m1.f * IF_FULL_MATRIX(m2.i, 1);

  IF_FULL_MATRIX(r.g = (m1.g * m2.a) +  (m1.h * m2.d) +  (m1.i * m2.g),0);
  IF_FULL_MATRIX(r.h = (m1.g * m2.b) +  (m1.h * m2.e) +  (m1.i * m2.h),0);
  IF_FULL_MATRIX(r.i = (m1.g * m2.c) +  (m1.h * m2.f) +  (m1.i * m2.i),0);
  m2 = r;
}

void mat_identity(mat_t &m) {
  MAKE_MATRIX(m,
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  );
}

void mat_translate(mat_t &m, float dx, float dy) {
  mat_t transform;
  MAKE_MATRIX(transform,
    1, 0, dx,
    0, 1, dy,
    0, 0, 1
  );
  mat_premultiply(transform,m);
}

void mat_scale(mat_t &m, float sx, float sy) {
  mat_t transform;
  MAKE_MATRIX(transform,
    sx,  0,  0,
    0,  sy,  0,
    0,   0,  1
  );
  mat_premultiply(transform,m);
}

void mat_shear(mat_t &m, float a) {
  mat_t transform;
  MAKE_MATRIX(transform,
    1, a, 0,
    0, 1, 0,
    0, 0, 1
  );
  mat_premultiply(transform,m);
}

void mat_rotate(mat_t &m, float a) {
  mat_t transform;
  MAKE_MATRIX(transform,
    cos(a), -sin(a),       0,
    sin(a),  cos(a),       0,
         0,       0,       1
  );
  mat_premultiply(transform,m);
}

void mat_apply(const mat_t &m, float &x, float &y) {
  float nx = (m.a * x) + (m.b * y) + (m.c);
  float ny = (m.d * x) + (m.e * y) + (m.f);
  #if defined(FULL_MATRICES)
  float nz = (m.g * x) + (m.h * y) + (m.i);
  x = nx/nz;
  y = ny/nz;
  #else
  x = nx; y = ny;
  #endif
}

void mat_print(const mat_t &m) {
  Serial.print(m.a);
  Serial.print(F(", "));
  Serial.print(m.b);
  Serial.print(F(", "));
  Serial.println(m.c);
  Serial.print(m.d);
  Serial.print(F(", "));
  Serial.print(m.e);
  Serial.print(F(", "));
  Serial.println(m.f);
  #if defined(FULL_MATRICES)
  Serial.print(m.g);
  Serial.print(F(", "));
  Serial.print(m.h);
  Serial.print(F(", "));
  Serial.println(m.i);
  #else
  Serial.print(F("0.00"));
  Serial.print(F(", "));
  Serial.print(F("0.00"));
  Serial.print(F(", "));
  Serial.println(F("1.00"));
  #endif
}

/************************** SCISSORING ROUTINES ***************************/

#define TRIM_X(xt) x = xt; y = (dx != 0) ? y0 + (xt - x0) * dy/dx : y;
#define TRIM_Y(yt) y = yt; x = (dy != 0) ? x0 + (yt - y0) * dx/dy : x;

/* When x0 and y0 is inside the boundary, crop x1 and y2 if
/* it falls outside the boundary */
void trim(float x0, float y0, float &x1, float &y1) {
  float dx = x1 - x0;
  float dy = y1 - y0;
  float x = x1, y = y1;
  if(x > 1) {
    TRIM_X(1);
  }
  else if(x < 0) {
    TRIM_X(0);
  }
  if(y > 1) {
    TRIM_Y(1);
  }
  else if(y < 0) {
    TRIM_Y(0);
  }
  x1 = x;
  y1 = y;
}

void fb_scissor(float x, float y) {
  static bool  last_out;
  static float last_x;
  static float last_y;
  static float out_pt_x;
  static float out_pt_y;

  float ox = x;
  float oy = y;
  if(!last_out) {
    if(x < 0 || x > 1 || y < 0 || y > 1) {
      trim(last_x, last_y, x, y);
      out_pt_x = x;
      out_pt_y = y;
      last_out = true;
    }
    fb_append(x,y);
  } else {
    if(x > 0 && x < 1 && y > 0 && y < 1) {
      trim(x, y, last_x, last_y);
      if((out_pt_y == 1.0 || out_pt_y == 0.0) && (out_pt_y != last_y)) {
        fb_append(out_pt_x, last_y);
      }
      if((out_pt_x == 1.0 || out_pt_x == 0.0) && (out_pt_x != last_x)) {
        fb_append(last_x, out_pt_y);
      }
      fb_append(last_x, last_y);
      fb_append(x, y);
      last_out = false;
    }
  }
  last_x = ox;
  last_y = oy;
}

/************************** FRAMEBUFFER ROUTINES ***************************/

/* The frame buffer is a circular buffer that can hold an
 * active and an inactive frame. Since each display point
 * requires two bytes, this means that the active and inactive 
 * frames share a total of 128 points among themselves.
 * 
 *    +---------+  
 *    |    0    |  F
 *    |    1    |  A   <-- fb_head 
 *    |    2    |  A              The active frame is between
 *    |    .    |  A              fb_head and fb_tail
 *    |    .    |  I   <-- fb_tail 
 *    |    .    |  I              The inactive frame is written
 *    |    .    |  I              starting at fb_tail at locations 
 *    |    .    |  I              indexed by fb_free
 *    |    .    |  F   <-- fb_free
 *    |    .    |  F              If fb_free reaches fb_head,
 *    |    .    |  F              then it will be set to fb_tail
 *    |   254   |  F              causing points in the inactive
 *    |   255   |  F              frame to be overwritten
 *    |   255   |  F   <-- FRAMEBUFFER_SIZE
 *    +---------+                 When fb_free reaches the end,
 *                                it wraps around to zero
 *
 * Once all points have been written to the inactive frame,
 * fb_swap makes the inactive frame active, and the active
 * frame inactive. This is done by updating fb_head and
 * fb_tail as such:
 *  
 *     fb_head   <-(copy)--  fb_tail  <-(copy)-- fb_free
 *     
 * After the swap, fb_tail == fb_free, meaning the inactive
 * frame is empty and ready to accept new points.
 * 
 * An interrupt service routine based on timer 1 is used to
 * update the display. The ISR uses the index fb_disp which
 * is cycled from fb_head to fb_tail.
 */
fbdat_t fb_data[FRAMEBUFFER_SIZE];
fb_idx_t fb_head = 0; // Start of active buffer
fb_idx_t fb_tail = 0; // End of active buffer; start of inactive buffer
fb_idx_t fb_free = 0; // Start of free space in inactive buffer
fb_idx_t fb_disp = 0; // ISR readout index
volatile bool fb_wait_for_end_of_frame = false;
 
/* Writes a point to the inactive frame */
void fb_append(float fx, float fy) {
  if(pointToAnalogVoltageAndDelay(fx, fy, fb_data[fb_free])) {
    // Increment to the next free slot.
    if(++fb_free == FRAMEBUFFER_SIZE) {
      fb_free = 0;
    }
    if(fb_free == fb_head) {
      Serial.println(F("Insufficient space in frame buffer. Truncating."));
      fb_free = fb_tail;
    }
  }
}

/* Makes the inactive frame active and the active frame inactive. This
 * is syncronized with the end of a frame readout to prevent tearing.
 */
void fb_swap() {
  // Wait until the ISR completes the frame
  fb_wait_for_end_of_frame = true;
  while(fb_wait_for_end_of_frame);
  
  // Update the frame buffer indices
  noInterrupts();
  fb_head = fb_tail;
  fb_tail = fb_free;
  fb_disp = fb_head;
  interrupts();

#if defined(SHOW_FRAMERATE)
  static uint8_t frames = 0;
  static long    last_t = millis();
  frames++;
  if(frames == 255) {
    Serial.print(float(frames)/(millis()-last_t)*DELAY_1S);
    Serial.println(F(" frames per second"));
    frames = 0;
    last_t = millis();
  }
#endif
}

/* Set up the frame buffer routine as an interrupt on timer 1 */

void fb_interrupt_setup() {
  /* Reference: http://www.avrbeginners.net/architecture/timers/timers.html */

  // Begin configuring the pin for PWM, we will adjust rates later.
  analogWrite(X_PIN, 128);
  analogWrite(Y_PIN, 128);
    
  cli();         // disable global interrupts

  /* **********************Configure Timer 0 *************************/

  #if defined(TIMER0)
    TCCR0B = (TCCR0B & 0b11111000) | TIMER0;
  #endif

  /* **********************Configure Timer 1 *************************/

  // clock_select: bits CS12, CS11 and CS10 configure the prescaler
  uint8_t clock_select = 0b000; // Values for CS12, CS11 and CS10 
  switch(TIMER1_PRESCALER) {
    case    0: clock_select = 0b000; break; // Stop timer
    case    1: clock_select = 0b001; break; // No prescaler
    case    8: clock_select = 0b010; break; // divide clock by    8
    case   64: clock_select = 0b011; break; // divide clock by   64
    case  256: clock_select = 0b100; break; // divide clock by  256
    case 1024: clock_select = 0b101; break; // divide clock by 1024
  }
  
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  TCCR1B = (TCCR1B & 0b11111000) | clock_select;
  
  // enable timer overflow interrupt
  TIMSK1 |= (1 << TOIE1);

  /* **********************Configure Timer 2 *************************/
  
  #if X_PIN == 11 && Y_PIN == 3
    // Configure Timer 2 for Fast PWM mode at 62500 Hz
    TCCR2A = 0;// set entire TCCR2A register to 0
    TCCR2B = 0;// same for TCCR2B
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS20);
    OCR2A = 0;
    OCR2B = 0;
  #endif
  
  sei(); // enable all interrupts
}

bool pointToAnalogVoltageAndDelay(float fx, float fy, fbdat_t &data) {
  static uint8_t last_x = 0;
  static uint8_t last_y = 0;
#if defined(DOMAIN_RESTRICTION)
  const uint8_t x = DOMAIN_RESTRICTION+fx*(255-DOMAIN_RESTRICTION*2);
  const uint8_t y = DOMAIN_RESTRICTION+(1.0-fy)*(255-DOMAIN_RESTRICTION*2);
#else
  const uint8_t x = fx*255;
  const uint8_t y = (1.0-fy)*255;
#endif

  // Compute required analog voltages in accordance to display mode
  #if defined(FIXED_RC_MODE)
    data.x = x;
    data.y = y;
    data.dt = TIMER1_SECS_TO_TICKS(R*C*5);
  #else
    uint8_t  vx, vy;
    uint16_t dt;
    if(last_x == x && last_y == y) {
      return false;
    }
    dt = compute_rapid_sweep_voltages(last_x, x, last_y, y, vx, vy);
    last_x = x;
    last_y = y;
    // Encode four bytes into three.
    if(vx == 0) {
      data.fast_bit = 0b00;
      data.slow_val = vy;
    } else if(vx == 255) {
      data.fast_bit = 0b01;
      data.slow_val = vy;
    } else if(vy == 0) {
      data.fast_bit = 0b10;
      data.slow_val = vx;
    } else if(vy == 255) {
      data.fast_bit = 0b11;
      data.slow_val = vx;
    } else {
      Serial.print(F("Invalid analog voltages: "));
      Serial.print(vx);
      Serial.print(F(","));
      Serial.println(vy);
    }
    if(dt & 0xC0000000) {
      Serial.print(F("Unable to encode voltages, overflow detected!"));
    }
    data.dt = dt & 0x3FFFFFFF;
  #endif
  return true;
}

#if X_PIN == 11 && Y_PIN == 3
  #define ANALOG_WRITE_X(x) OCR2B = x
  #define ANALOG_WRITE_Y(y) OCR2A = y
#elif X_PIN == 5 && Y_PIN == 6
  #define ANALOG_WRITE_X(x) OCR0B = x
  #define ANALOG_WRITE_Y(y) OCR0A = y
#else
  #warning Using slower analogWrite to update pins.
  #define ANALOG_WRITE_X(x) analogWrite(X_PIN, x)
  #define ANALOG_WRITE_Y(y) analogWrite(Y_PIN, y)
#endif

inline void fb_increment_ptr(fb_idx_t &fb_idx) {
  if(++fb_idx == FRAMEBUFFER_SIZE) {
    // If we reached the end of the circular buffer, loop around
    fb_idx = 0;
  }
  if(fb_idx == fb_tail) {
    // If we reached the end of the frame buffer, loop around
    fb_idx = fb_head;
    fb_wait_for_end_of_frame = false;
  }
}

/* Updates the vector display. After each change in the analog voltage,
 * there is a wait time for the voltage to settle, as dictated by the
 * RC constant.
 */
ISR(TIMER1_OVF_vect){
  // Retreive next point in the frame buffer
  if(fb_tail != fb_head) {
    #if defined(FIXED_RC_MODE)
      ANALOG_WRITE_X(fb_data[fb_disp].x);
      ANALOG_WRITE_Y(fb_data[fb_disp].y);
    #else
      switch(fb_data[fb_disp].fast_bit) {
        case 0b00:
          ANALOG_WRITE_X(0);
          ANALOG_WRITE_Y(fb_data[fb_disp].slow_val);
          break;
        case 0b01:
          ANALOG_WRITE_X(255);
          ANALOG_WRITE_Y(fb_data[fb_disp].slow_val);
          break;
        case 0b10:
          ANALOG_WRITE_X(fb_data[fb_disp].slow_val);
          ANALOG_WRITE_Y(0);
          break;
        case 0b11:
          ANALOG_WRITE_X(fb_data[fb_disp].slow_val);
          ANALOG_WRITE_Y(255);
          break;
      }
    #endif
    TCNT1 = 65536 - fb_data[fb_disp].dt;
    fb_increment_ptr(fb_disp);
  } else {
    fb_wait_for_end_of_frame = false;
  }
}

/*********************** GROUND TRUTH RC EQUATIONS *********************/

/* Reference: https://en.wikipedia.org/wiki/RC_time_constant */

/* Voltage of a capacitor after t when charging to Vcc via R from zero. */
float rc_voltage_rise0(float vcc, float t) {
  return vcc * (1 - exp(-t/(R*C)));
}

/* Voltage of a capacitor after t when discharging to ground via R
   from an initial voltage v0 */
float rc_voltage_fall0(float v0, float t) {
  return v0 * exp(-t/(R*C));
}

/*************************** INVERSE RC EQUATIONS ***********************/

/* Time to charge a capacitor from 0 volts to v when connected via R to Vcc */
float rc_rise_time_from_zero(float v, float vcc) {
  if(v == vcc) v = vcc*0.9935;
  return -1.0*R*C*log(1.0-v/vcc);
}

/* Time to discharge a capacitor from v0 to v when connected via R to GND */
float rc_fall_time_to_zero(float v0, float v) {
  return -1.0*R*C*log(v/v0);
}

/**************** PRE-COMPUTED LOGARITHM TABLE ****************/

/* In order to speed up the computation of RC time constants,
 * a logarithm table is written to EEPROM. Since the EEPROM has
 * limited writes, we only write values when they are incorrect.
 * This means that repeated execution of the program will reuse
 * the previously stored values.
 */

#define FORCE_INLINE __attribute__((always_inline)) inline
//#define OPTIMIZE    __attribute__ ((optimize(1)))

#define RC_DELAY_TABLE_OFFSET 0
    
bool update_rc_delay(uint8_t v, uint16_t d) {
  if(read_rc_delay(v) != d) {
    const uint16_t addr = RC_DELAY_TABLE_OFFSET + v;
    byte_and_word data;
    data._word = d;
    EEPROM.write(addr,       data._byte[0]);
    EEPROM.write(addr + 256, data._byte[1]);
    return true;
  }
  return false;
}

FORCE_INLINE uint16_t read_rc_delay(uint8_t v) {
  const uint16_t addr = v;
  byte_and_word data;
  data._byte[0] = EEPROM.read(addr);
  data._byte[1] = EEPROM.read(addr + 256);
  return data._word;
}

#define EEPROM_FINISH_WRITE         while(EECR & (1<<EEPE));
#define EEPROM_HIGH_ADDR(addr_high) EEARH=addr_high;
#define EEPROM_READ(addr_low, into) EEARL=addr_low; EECR |= (1<<EERE); into = EEDR;

FORCE_INLINE uint16_t read_rc_delay(uint8_t addr_a, uint8_t addr_b) {
  byte_and_word data_a;
  byte_and_word data_b;
  
  EEPROM_HIGH_ADDR(0b00);
  EEPROM_READ(addr_a, data_a._byte[0]);
  EEPROM_READ(addr_b, data_b._byte[0]);

  EEPROM_HIGH_ADDR(0b01);
  EEPROM_READ(addr_a, data_a._byte[1]);
  EEPROM_READ(addr_b, data_b._byte[1]);
  return data_b._word - data_a._word;
}

void compute_rc_delay_table() {
  uint8_t bytes_written = 0;
  Serial.print(F("Checking EEPROM values... "));
  for(uint16_t v = 0; v < 256; v++) {
    if(update_rc_delay(v, TIMER1_SECS_TO_TICKS(-1.0*R*C*log(float(v))))) {
      bytes_written++;
    }
  }
  Serial.print(bytes_written);
  Serial.println(F(" values written"));
}

/*************************** DERIVED RC EQUATIONS ***********************/

/* Time to charge a capacitor from v0 to v when connected via R to Vcc */
float rc_rise_time_from_v0(float v0, float v, float vcc) {
  return -1.0*R*C*log((vcc-v)/(vcc-v0));
}

/* Time to discharge a capacitor from v0 to v when connected via R to vlow */
float rc_fall_time_to_vlow(float v0, float v, float vlow) {
  return -1.0*R*C*log((v-vlow)/(v0-vlow));
}

/* Time to charge a capacitor from (a/255)*vcc to (b/255)*vcc when connected
   via R to vcc */
inline uint16_t rc_rise_time_ab(uint8_t a, uint8_t b) {
  return read_rc_delay(255-b)-read_rc_delay(255-a);
}

/* Compute the number of seconds for the capacitor voltage to fall
   from (a/255)*vcc to (b/255)*vcc when using a supply voltage of
   vcc */
inline uint16_t rc_fall_time_ab(uint8_t a, uint8_t b) {
  return read_rc_delay(b)-read_rc_delay(a);
}

/************************* RAPID SWEEP MODE ***********************/

/* In rapid sweep mode, we charge/discharge one of the capacitors through R to
 * VCC or GND, for the maximum speed transition between two voltage levels. The
 * other capacitor must be driven using a PWM voltage chosen such that the
 * transition time from a to b is identical. If we pass L, a and b into this
 * function, it will return the needed PWM voltage to acheive this.
 */
inline uint8_t pwm_voltage_for_same_switching_time(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  /* Derivation information:
   *   We set L equal to the part inside the log() in either rc_rise_time_from_v0 
   *   or rc_fall_time_to_vlow, then let v0 = c, let v = d:
   *    For (d > c), solve L = (vcc-v)/(vcc-v0)   for vcc 
   *    For (d < c), solve L = (v-vlow)/(v0-vlow) for vlow
   * In both cases, the result is (L*c-d)/(L-1)
   */
  //float L = float(b)/float(a);
  //return (L*c-d)/(L-1);
  return (long(b)*c-long(a)*d)/(b-a);
}

inline uint16_t compute_rapid_sweep_voltages(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, uint8_t &vx, uint8_t &vy) {
  uint16_t dt_x, dt_y;

  const bool x_rising  = x2 > x1;
  const bool y_rising  = y2 > y1;
  const bool x_falling = x2 < x1;
  const bool y_falling = y2 < y1;

  if(x_rising) {
    vx = 255;
    x1 = 255-x1;
    x2 = 255-x2;
    dt_x = read_rc_delay(x1,x2);
  } else if(x_falling) {
    vx = 0;
    dt_x = read_rc_delay(x1,x2);
  } else {
    vx = x1;
    dt_x = 0;
  }

  if(y_rising) {
    vy = 255;
    y1 = 255-y1;
    y2 = 255-y2;
    dt_y = read_rc_delay(y1,y2);
  } else if(y_falling) {
    vy = 0;
    dt_y = read_rc_delay(y1,y2);
  } else {
    vy = y1;
    dt_y = 0;
  }

  /* The limiting factor will be the slowest of the X or Y transitions. */
  /* The other, being faster, must be slowed via PWM to match the slowest. */
  if(dt_y < dt_x) {
    if(y_rising) {
      y1 = 255-y1;
      y2 = 255-y2;
    }
    vy = pwm_voltage_for_same_switching_time(x1, x2, y1, y2);
    return dt_x;
  } else if(dt_y > dt_x) {
    if(x_rising) {
      x1 = 255-x1;
      x2 = 255-x2;
    }
    vx = pwm_voltage_for_same_switching_time(y1, y2, x1, x2);
    return dt_y;
  } else {
    return dt_x;
  }
}

/******************************* DATA BUFFERS *******************************/

#define S 0.5

#if defined(USER_PATTERN)
const float PROGMEM user_pattern[] = USER_PATTERN;
rect_t user_pattern_rect;

#elif PATTERN == 1
const float PROGMEM merry_christmas[] = {
  0,0,0.0123,-0.1378, 0,-0.1205, -0.02706,0, -0.01968,0.025, 0.0049,0.025, 0.05903,-0.039, 0.03198,-0.037, 0.04427,0.02, 0.01476,0.047, -0.0098,0.1919, 0.0098,-0.2116, 0.03444,-0.02, 0.04919,0.02, 0.02214,0.049, -0.0074,0.1083, 0,0.071, 0.02952,0.01, 0.02706,-0.059, 0.06149,-0.052, -0.04673,-0.01, -0.01476,0.064, 0.05657,0.039, 0.02952,-0.032, 0.0098,-0.069, 0.0246,0.015, 0.01476,-0.01, 0.0025,0.064, 0.0246,0.032, 0.02214,-0.049, 0.0049,-0.052, 0.01968,0.022, 0.01722,-0.015, 0,0.071, 0.02214,0.015, 0.02952,-0.044, 0.01722,-0.049, 0.01722,0.02, 0.0049,0.054, 0.03936,0.02, 0.02952,-0.054, -0.0074,-0.044, 0.0025,0.1205, -0.01968,0.1107, -0.0123,0.034, -0.04181,-0.012, -0.0098,-0.049, 0.04673,-0.066, 0.07625,-0.052, 0.0123,-0.039, 0.32959497,-0.155, 0.0025,-0.03, -0.05903,0.017, 0,0.044, 0.02952,-0.044, -0.08117,-0.039, -0.08117,0.042, -0.03198,0.1132, 0.03198,0.084, 0.06887,0.049, 0.09593,-0.02, 0.07871,-0.1156, 0.0074,-0.096, -0.01968,-0.069, -0.01722,0.096, -0.0074,0.1279, -0.0025,0.054, 0.0098,-0.079, 0.03444,-0.027, 0.0246,0.017, 0.0049,0.052, 0.0074,0.032, 0.03936,-0.034, 0.0049,-0.071, 0.01722,0.022, 0.01722,-0.015, 0,0.069, 0.01476,0.03, 0.03198,-0.03, 0.0123,-0.071, 0,0.064, 0.01968,0.034, 0.03935,-0.074, 0.02214,-0.025, 0.02214,0.049, -0.0098,0.037, -0.04919,0, 0,-0.029, 0.01968,0.042, 0.05903,-0.01, 0.02952,-0.042, 0.0074,-0.096, 0.0025,-0.062, 0,0.059, -0.04427,0.022, 0.08855,-0.037, -0.04673,0.022, -0.0123,0.1057, 0.0246,0.034, 0.04673,-0.02, 0.0123,-0.042, 0.0246,-0.027, 0,0.096, 0.0025,-0.076, 0.03444,-0.029, 0.03444,0.032, -0.0098,0.066, 0.0098,-0.071, 0.04427,-0.027, 0.01968,0.029, 0.0123,0.069, 0.01968,0, 0.02952,-0.071, 0.0369,-0.025, 0.02706,0.015, -0.03444,-0.01, -0.0369,0.022, 0.01476,0.059, 0.03198,0.01, 0.02214,-0.032, 0,-0.054, 0.0098,0.079, 0.02706,-0.01, 0.02214,-0.064, 0.02952,-0.02, 0,0.03, 0.02214,0.039, -0.02952,0.03, -0.0369,-0.027, 0.06149,0.03, 0.03198,-0.017,0,0
};
rect_t merry_christmas_rect;

#elif PATTERN == 2
const float square_pts[] PROGMEM = {
  -S, -S,
   S, -S,
   S,  S,
  -S,  S,
   0,  0,
  -S, -S,
};

#elif PATTERN == 3
const float triangle_pts[] PROGMEM = {
   0,   S,
   S,  -S,
   -S,   -S,
   0,   S,
};

#elif PATTERN == 4
const float spiral_pts[] PROGMEM = {
   -1.0,  -1.0,
    1.0,  -1.0,
    1.0,   1.0,
   -1.0,   1.0,
   -1.0,  -0.8,
    0.8,  -0.8,
    0.8,   0.8,
   -0.8,   0.8,
   -0.8,  -0.6,
    0.6,  -0.6,
    0.6,   0.6,
   -0.6,   0.6,
   -0.6,  -0.4,
    0.4,  -0.4,
    0.4,   0.4,
   -0.4,   0.4,
   -0.4,  -0.2,
    0.2,  -0.2,
    0.2,   0.2,
   -0.2,   0.2,
   -0.2,   0.0,
   -1.0,  -1.0
};

#elif PATTERN == 5
const float PROGMEM happy_new_year[] = {
0,0, -4.20712,40.7767, -3.91386,-8.5664, -15.49983,-7.62242, 3.571,-22.88564, -6.80724,42.63053, 3.12963,-19.96191, -6.36967,1.84273, 0,3.88349, 14.84118,6.78094, 13.6264,-0.86047, 4.32132,-6.87237, 2.90124,-5.395, 6.59048,0.9519, -6.9217,-0.62069, -2.78324,8.38009, 3.03441,4.32511, 4.42793,-1.32106, 3.1983,-11.09087, -0.51788,11.87129, 4.20712,-0.32362, 3.23625,-11.32686, -3.23625,36.56958, 4.53075,-36.89321, 5.50162,-0.64725, 2.58899,6.6343, -2.42718,5.82525, -5.82524,0.48544, 10.51779,-0.32362, 4.53075,-11.65049, -2.91262,35.27508, 4.53074,-35.92233, 5.98706,0.16181, 1.94174,6.79611, -3.55987,5.98706, -4.20711,-0.48544, 10.35598,-0.1618, 6.95793,-12.62137, 3.39806,12.45955, 5.66343,-0.32362, 1.94175,-12.13593, -2.589,33.65696, -4.20712,3.23625, -5.50162,-5.178, 2.26538,-7.76699, 16.82847,-17.1521, 0,0, 2.46453,-1.3614, 15.27685,-8.73013, 12.78467,-0.98406, 14.03088,16.94826, 7.38715,22.81376, 0.27034,9.31535, -200.14336,-2.28967, 9.41552,17.28684, 7.03386,-6.17758, -2.66525,37.63326, 3.62474,-33.6887, 6.18337,-5.33049, 8.31556,0.63966, 3.09169,12.47335, -1.27932,13.96588, -0.31984,11.08742, 2.55864,1.17271, 3.62474,-5.11727, 1.91897,-5.01066, 8.74201,-5.22388, -1.59915,-1.91898, -4.90405,1.59915, -2.45203,4.69083, 1.17271,4.47761, 4.58422,3.1983, 5.97015,-1.91898, 3.1983,-5.11727, 0.31983,-6.82303, 0.31983,11.40725, 4.15778,2.66525, 3.73134,-3.83796, 0.95949,-9.70149, 1.38593,11.72708, 4.371,1.59915, 3.94456,-3.30491, -0.21322,-8.42218, -3.62473,-1.49253, 0.10661,3.30491, 7.21067,4.42765, 9.65323,-2.23895, 5.88915,-4.78207, 5.52471,-22.36198, -2.15836,15.57368, -1.36198,14.53179, 3.4964,6.24627, 5.43488,1.82322, 5.75693,-1.49253, 5.86354,-5.75693, 4.79744,-28.57143, -6.71642,52.87846, -6.07675,8.95522, -7.67591,0.10661, -4.15779,-7.03624, 3.41151,-8.84861, 7.67591,-7.03625, 13.0064,-5.86354, 3.3049,-6.60981, 4.47761,-3.09168, 4.2644,-3.41152, -2.45203,-1.49254, -5.4371,2.98508, -0.10661,4.47761, 1.27932,4.371, 4.15778,2.02559, 4.2644,-1.17271, 3.51812,-4.371, 1.0661,-5.11727, 4.05117,-2.55864, 4.26439,1.0661, -4.26439,-1.49254, -3.94456,3.41152, -0.21322,8.10234, 3.73134,2.02559, 4.58422,-3.73134, 0.63966,-8.95523, 1.59915,11.9403, 2.2388,0.31983, 2.02559,-3.09168, 1.70576,-6.60981, 0.10661,-3.51813, 1.91897,2.45203, 2.34542,-0.31983, 0.74627,-0.85288, -0.31983,9.59488, 2.45203,2.87847, 2.66524,-2.34542, 4.83658,-11.92377, 17.57438,8.46897, -20.25241,18.63366, -39.98239,19.79355, -46.24774,3.76328, -50.20818,-10.19709, -30.39125,-22.41482, -19.87602,-33.6063, 3.21332,-40.17675, 24.34464,-28.54715, 34.94102,-18.88773, 34.61948,-7.31752};
rect_t happy_new_year_rect;
rect_t happy_new_year_2018_rect;

#define ST 0.5, 0.5
#define PT(x,y) x/16.0,y/16.0
const float PROGMEM happy_new_year_line[]    = {0, 0, 0.25, 0.25, 0, 0};
const float PROGMEM happy_new_year_digit_1[] = {ST ,PT(0,6),PT(-2,0),PT(4,0),PT(-2,0),PT(0,-13),PT(-2,2),PT(2,-2),PT(0,7)};
//const float PROGMEM happy_new_year_digit_2[] = {ST, PT(-4,3),PT(-1,3),PT(11,0),PT(-11,0),PT(1,-3),PT(4,-3),PT(5,-2),PT(0,-4),PT(-5,-1),PT(-5,2),PT(5,-2),PT(5,1),PT(0,4),PT(-5,2)};
const float PROGMEM happy_new_year_digit_2[] = {ST, PT(-2,3),PT(-1,3),PT(7,0),PT(-7,0),PT(1,-3),PT(2,-3),PT(2,-1),PT(2,-1),PT(0,-3),PT(-2,-1),PT(-3,0),PT(-2,1),PT(2,-1),PT(3,0),PT(2,1),PT(0,3),PT(-2,1),PT(-2,1)};
const float PROGMEM happy_new_year_digit_3[] = {ST, PT(3,0),PT(1,1),PT(0,4),PT(-2.,1),PT(-2,0),PT(-2,-1),PT(2,1),PT(2,0),PT(2,-1),PT(0,-4),PT(-1,-1),PT(-3,0),PT(3,0),PT(1,-1),PT(0,-4),PT(-2,-1),PT(-2,0),PT(-2,1),PT(2,-1),PT(2,0),PT(2,1),PT(0,4),PT(-1,1),PT(-3,0)};
const float PROGMEM happy_new_year_digit_4[] = {ST, PT(-3,1),PT(3,-7),PT(-3,7),PT(3,-1),PT(3,-1),PT(-1,6),PT(2,-12),PT(-1,6),PT(3,-1),PT(-6,2)};
const float PROGMEM happy_new_year_digit_5[] = {ST, PT(2,0),PT(2,1),PT(0,4),PT(-2,1),PT(-2,0),PT(-2,-1),PT(2,1),PT(2,0),PT(2,-1),PT(0,-4),PT(-2,-1),PT(-2,0),PT(-2,1),PT(1,-6),PT(5,0),PT(0,0),PT(-5,0),PT(-1,6),PT(2,-1)};
const float PROGMEM happy_new_year_digit_6[] = {ST, PT(2,0),PT(2,1),PT(0,4),PT(-2,1),PT(-2,0),PT(-2,-1),PT(2,1),PT(2,0),PT(2,-1),PT(0,-4),PT(-2,-1),PT(-2,0),PT(-2,1),PT(0,4),PT(0,-9),PT(2,-1),PT(2,0),PT(2,1),PT(0,0),PT(-2,-1),PT(-2,0),PT(-2,1),PT(0,9),PT(0,-4),PT(2,-1)};
const float PROGMEM happy_new_year_digit_7[] = {ST, PT(-2,6),PT(2,-6),PT(6,-6),PT(-9,0),PT(9,0),PT(-6,6)};
const float PROGMEM happy_new_year_digit_8[] = {ST, PT(-2,1),PT(0,4),PT(2,1),PT(2,0),PT(2,-1),PT(0,-4),PT(-2,-1),PT(-2,0),PT(-2,-1),PT(0,-4),PT(2,-1),PT(2,0),PT(2,1),PT(0,4),PT(-2,1)};
//const float PROGMEM happy_new_year_digit_9[] = {ST, PT(-2,0),PT(-2,-1),PT(0,-4),PT(2,-1),PT(2,0),PT(2,1),PT(-2,-1),PT(-2,0),PT(-2,1),PT(0,4),PT(2,1),PT(2,0),PT(2,-1),PT(0,-4),PT(0,9),PT(-2,1),PT(-2,0),PT(-2,-1),PT(0,0),PT(2,1),PT(2,0),PT(2,-1),PT(0,-9),PT(0,4),PT(-2,1)};
const float PROGMEM happy_new_year_digit_9[] = {ST, PT(-2,-1),PT(0,-4),PT(2,-1),PT(2,0),PT(2,1),PT(-2,-1),PT(-2,0),PT(-2,1),PT(0,4),PT(2,1),PT(2,0),PT(2,-1),PT(0,-4),PT(0,9),PT(-2,1),PT(-2,0),PT(-2,-1),PT(0,0),PT(2,1),PT(2,0),PT(2,-1),PT(0,-9),PT(0,4),PT(-2,1),PT(-2,0)};
const float PROGMEM happy_new_year_2018[] = {-33.88672,10.25391,-11.32812,30.5664,3.61328,19.53126,11.03516,11.13281,-10.54883,9.11523,0,-12.67969,-23.24219,0,-38.12305,23.98633,0,-5.57617,-1.85547,-26.17187,-6.83593,-14.7461,-15.33203,-11.03515,-23.24219,-4.58985,-27.14844,6.54297,-14.94141,17.38281,-3.20898,18.12305,-9.5332,-14.37305,-35.54688,-11.71875,-20.60547,3.8086,-13.96484,11.23047,-6.73828,15.13671,-1.36719,20.89844,0,5.76172,35.35156,0,0,-15.03906,2.14844,-13.96485,6.73828,-3.51562,6.64063,3.22266,2.24609,9.57031,-7.32422,23.4375,-45.80078,78.02734,0,22.5586,86.81641,0,0,-26.95313,-43.26172,0,39.9414,-64.25781,3.75782,-9.86524,0,35.70508,2.34375,24.41406,7.91015,14.55079,14.94141,10.2539,20.94531,3.18555,0,-24.29492,-5.41797,-2.81641,-1.26953,-16.60156,0,-77.14844,1.36719,-15.52734,5.85937,-3.41797,5.66407,3.22265,1.26953,15.72266,0,78.32031,-1.36719,14.94141,-5.25,3.29492,0,24.31641,1.73438,0.0254,19.43359,-3.125,14.84375,-10.25391,8.10547,-15.33203,1.95312,-26.17187,0,-27.38086,16.15039,1.93945,4.88282,5.56641,0.8789,22.07031,0,84.76562,39.45313,0,0,-80.1875,13.67383,12.75586,27.83203,5.56641,0.20312,-0.004,0,-24.32031,-5.86719,-2.92187,-1.85546,-13.1836,0,-20.50781,1.66015,-12.10938,5.66406,-2.92968,5.85938,3.02734,1.85547,12.01172,0,20.21484,-1.46485,13.47657,-4.99609,2.90039,0,24.32421,27.75,-5.74023,14.45313,-14.94141,3.71093,-28.22265,-4.6875,-33.20313,-13.3789,-10.54687,13.3789,-11.23047,3.32032,-18.45703,-9.96094,-30.95703,-35.01367,-10.82618,0,24.33399,4.64257,2.60547,1.46485,10.54687,0,10.83985,-1.36719,11.13281,-5.27344,2.63672,-5.27343,-2.73438,-1.5625,-9.86328,0,-12.01172,1.36718,-10.35156,5.14844,-2.82617,0,-24.32031};

#elif PATTERN == 6
#define ST 0.5, 0.5
#define PT(x,y) x/16.0,y/16.0
const float PROGMEM happy_new_year_line[]    = {0, 0, 0.25, 0.25, 0, 0};
const float PROGMEM happy_new_year_digit_1[] = {ST ,PT(0,6),PT(-2,0),PT(4,0),PT(-2,0),PT(0,-13),PT(-2,2),PT(2,-2),PT(0,7)};
//const float PROGMEM happy_new_year_digit_2[] = {ST, PT(-4,3),PT(-1,3),PT(11,0),PT(-11,0),PT(1,-3),PT(4,-3),PT(5,-2),PT(0,-4),PT(-5,-1),PT(-5,2),PT(5,-2),PT(5,1),PT(0,4),PT(-5,2)};
const float PROGMEM happy_new_year_digit_2[] = {ST, PT(-2,3),PT(-1,3),PT(7,0),PT(-7,0),PT(1,-3),PT(2,-3),PT(2,-1),PT(2,-1),PT(0,-3),PT(-2,-1),PT(-3,0),PT(-2,1),PT(2,-1),PT(3,0),PT(2,1),PT(0,3),PT(-2,1),PT(-2,1)};
const float PROGMEM happy_new_year_digit_3[] = {ST, PT(3,0),PT(1,1),PT(0,4),PT(-2.,1),PT(-2,0),PT(-2,-1),PT(2,1),PT(2,0),PT(2,-1),PT(0,-4),PT(-1,-1),PT(-3,0),PT(3,0),PT(1,-1),PT(0,-4),PT(-2,-1),PT(-2,0),PT(-2,1),PT(2,-1),PT(2,0),PT(2,1),PT(0,4),PT(-1,1),PT(-3,0)};

const float PROGMEM drunken_octopus_pts[] = {124.276,1046.298,-10.25786,-1.3557,-9.4377,-3.1656,-17.456955,-7.3376,-21.50378,-14.0959,-8.23045,-6.912,-21.09221,-23.40574,-6.75519,-8.40198,-11.60331,-21.38445,-12.1228895,-51.294,8.7126095,-21.82271,21.31511,-9.98019,34.72073,12.78728,9.77498,23.43743,1.9067,33.36707,2.37639,10.24722,2.7162,11.4923,15.555975,26.04,40.09636,16.52727,41.09006,-15.93782,11.52016,-18.78992,0.99472,-22.99109,-11.4153,-31.06065,-6.32647,-11.20921,-8.22282,-19.87794,-9.73435,-47.57028,1.62966,-17.69144,4.64894,-18.29076,4.99068,-10.98523,7.65334,-9.70766,11.54366,-10.80627,7.38291,-4.9633,8.58452,-3.95932,34.00962,-5.66152,32.10576,8.58049,13.90451,8.84169,17.67338,20.42742,4.83969,10.1354,4.74134,22.58912,-3.16738,30.76862,-11.76327,38.42287,-11.27242,25.01895,0.0146,51.66895,17.77108,21.78184,40.44151,9.77496,18.17521,-3.7225,13.01815,-6.42748,7.38516,-6.70815,7.01628,-7.61919,5.34455,-9.79029,3.92715,-8.78769,4.85657,-16.88841,2.00016,-17.51804,-0.81738,-11.34034,1.58629,-13.82577,19.19581,-23.03099,24.79683,-2.75356,16.69811,9.77252,5.90329,9.65231,1.59112,18.90579,-7.51937,33.59112,-28.86925,46.77007,-49.83026,36.0963,-7.6631,2.3717,0.57953,0.8608,-7.71387,1.861,-11.65502,2.1892,-45.69875,-2.8038,-10.44886,-4.3973,-25.88267,-13.3932,-8.61411,-6.9098,-5.79798,-6.3097,-7.20754,-7.2207,-8.33021,-6.11919,-6.61748,-2.9997,-11.90648,1.9516,-14.62195,13.50399,-6.39686,7.8123,-8.49136,7.8042,-15.08068,8.3936,-7.59008,4.1162,-18.95836,7.2004,-11.52966,2.1611};
rect_t drunken_octopus_rect;
#endif
/******************************* MAIN PROGRAM *******************************/

void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);

  #if defined(USER_PATTERN)
    #if defined(USER_PATTERN_USES_RELATIVE_POINTS)
      compute_rect(user_pattern, N_PTS(user_pattern), user_pattern_rect, DATA_RELATIVE);
    #else
      compute_rect(user_pattern, N_PTS(user_pattern), user_pattern_rect, DATA_ABSOLUTE);
    #endif
  Serial.print(F("Points in user pattern: "));
  Serial.println(N_PTS(user_pattern));
  
  #elif PATTERN == 1
    compute_rect(merry_christmas, N_PTS(merry_christmas), merry_christmas_rect, DATA_RELATIVE);
    Serial.print(F("Points in merry christmas: "));
    Serial.println(N_PTS(merry_christmas));
    
  #elif PATTERN == 5
    compute_rect(happy_new_year, N_PTS(happy_new_year), happy_new_year_rect, DATA_RELATIVE);
    compute_rect(happy_new_year_2018, N_PTS(happy_new_year_2018), happy_new_year_2018_rect, DATA_RELATIVE);
  
    Serial.print(F("Points in happy new year: "));
    Serial.println(N_PTS(happy_new_year));

    Serial.print(F("Points in 2018: "));
    Serial.println(N_PTS(happy_new_year_2018));
  #elif PATTERN == 6
    compute_rect(drunken_octopus_pts, N_PTS(drunken_octopus_pts), drunken_octopus_rect, DATA_RELATIVE);
  #endif

  compute_rc_delay_table();
  show_time_constants();
  fb_interrupt_setup();
}

void circle(float t, float &x, float &y) {
  const float pi = 3.1415;
  x = cos(t*2*pi);
  y = sin(t*2*pi);
}

float linstep(float t, float tmin, float tmax) {
    return max(0.0, min(1.0, (t - tmin) / (tmax - tmin)));
}

#if defined(USER_PATTERN)
void loop() {
  const float cx = -user_pattern_rect.x - user_pattern_rect.w/2;
  const float cy = -user_pattern_rect.y - user_pattern_rect.h/2;
  mat_t transform;
  mat_identity(transform);
  mat_translate(transform, cx, cy);
  mat_scale(transform, 1/user_pattern_rect.w, 1/user_pattern_rect.h);
  mat_translate(transform, 0.5, 0.5);
  mat_scale(transform, 0.9, 0.9);
  #if defined(USER_PATTERN_USES_RELATIVE_POINTS)
    draw_buffer(user_pattern, N_PTS(user_pattern), transform, DATA_RELATIVE);
  #else
    draw_buffer(user_pattern, N_PTS(user_pattern), transform, DATA_ABSOLUTE);
  #endif
  fb_swap();
}
#elif PATTERN == 1
void loop() {
  const float cx = -merry_christmas_rect.x - merry_christmas_rect.w/2;
  const float cy = -merry_christmas_rect.y - merry_christmas_rect.h/2;
  const float tx =  -fmod(float(millis())/DELAY_1S/2, 2.5);
  mat_t transform;
  mat_identity(transform);
  mat_translate(transform, cx, cy);
  mat_scale(transform, 1/merry_christmas_rect.h, 1/merry_christmas_rect.h);
  mat_translate(transform, 0.5, 0.5);
  mat_scale(transform, 0.9, 0.9);
  mat_translate(transform, tx, 0);
  draw_buffer(merry_christmas, N_PTS(merry_christmas), transform, DATA_RELATIVE);
  fb_swap();
}
#elif PATTERN == 2
void loop() {
  mat_t transform;
  mat_identity(transform);
  mat_scale(transform, 0.25, 0.25);
  mat_translate(transform, 0.5, 0.5);
  draw_buffer(square_pts, N_PTS(square_pts), transform);
  fb_swap();
}
#elif PATTERN == 3
void loop() {
  mat_t transform;
  mat_identity(transform);
  mat_scale(transform, 0.25, 0.25);
  mat_translate(transform, 0.5, 0.5);
  draw_buffer(triangle_pts, N_PTS(triangle_pts), transform);
  fb_swap();
}
#elif PATTERN == 4
void loop() {
  mat_t transform;
  mat_identity(transform);
  mat_rotate(transform, float(millis())/DELAY_1S);
  mat_scale(transform, 0.25, 0.25);
  mat_translate(transform, 0.5, 0.5);
  draw_buffer(spiral_pts, N_PTS(spiral_pts), transform);
  fb_swap();
}
#elif PATTERN == 5
void loop() {
  static uint32_t start_t = 0;
  const float cx = -happy_new_year_rect.x - happy_new_year_rect.w/2;
  const float cy = -happy_new_year_rect.y - happy_new_year_rect.h/2;
  if(start_t == 0) {
    start_t = millis();
  }
  const float t = float(millis()-start_t)/DELAY_1S;
  if(t < 9.0) {
    mat_t transform;
    mat_identity(transform);
    mat_rotate(transform, t*2*PI);
    mat_translate(transform, 0.5, 0.5);
    draw_buffer(happy_new_year_line, N_PTS(happy_new_year_line), transform);
    mat_identity(transform);
    switch(9-int(t)) {
      case 1: draw_buffer(happy_new_year_digit_1, N_PTS(happy_new_year_digit_1), transform, DATA_RELATIVE); break;
      case 2: draw_buffer(happy_new_year_digit_2, N_PTS(happy_new_year_digit_2), transform, DATA_RELATIVE); break;
      case 3: draw_buffer(happy_new_year_digit_3, N_PTS(happy_new_year_digit_3), transform, DATA_RELATIVE); break;
      case 4: draw_buffer(happy_new_year_digit_4, N_PTS(happy_new_year_digit_4), transform, DATA_RELATIVE); break;
      case 5: draw_buffer(happy_new_year_digit_5, N_PTS(happy_new_year_digit_5), transform, DATA_RELATIVE); break;
      case 6: draw_buffer(happy_new_year_digit_6, N_PTS(happy_new_year_digit_6), transform, DATA_RELATIVE); break;
      case 7: draw_buffer(happy_new_year_digit_7, N_PTS(happy_new_year_digit_7), transform, DATA_RELATIVE); break;
      case 8: draw_buffer(happy_new_year_digit_8, N_PTS(happy_new_year_digit_8), transform, DATA_RELATIVE); break;
      case 9: draw_buffer(happy_new_year_digit_9, N_PTS(happy_new_year_digit_9), transform, DATA_RELATIVE); break;
    }
  } else if(t < 15.0) {
    const float t1 = linstep(t, 9, 13);
    const float s = t1 * 0.005;
    const float r = t1 * 3;
    mat_t transform;
    mat_identity(transform);
    mat_translate(transform, cx + (1.0 - t1)*200, cy);
    mat_scale(transform, s, s);
    mat_rotate(transform, 2*PI*r);
    mat_translate(transform, 0.5, 0.5);
    draw_buffer(happy_new_year, N_PTS(happy_new_year), transform, DATA_RELATIVE);
  } else if(t < 20.0) {
    mat_t transform;
    mat_identity(transform);
    mat_translate(transform, cx, cy);
    mat_scale(transform, 0.005, cos(2*PI*(t-15))*0.005);
    mat_rotate(transform, 2*PI*(t-15)*.1);
    mat_translate(transform, 0.5+(t-15)*.15, 0.5);
    draw_buffer(happy_new_year, N_PTS(happy_new_year), transform, DATA_RELATIVE);
  } else if(t < 30.0) {
    const float t1 = linstep(t, 20, 30);
    const float t2 = linstep(t, 20, 22);
    const float s = 0.012 - t2 * 0.01;
    const float r = t2 * 3;
    mat_t transform;
    mat_identity(transform);
    mat_translate(transform, cx + (1.0 - t1)*200, cy);
    mat_scale(transform, s, s);
    mat_rotate(transform, 2*PI*r);
    mat_translate(transform, 0.5, 0.5);
    draw_buffer(happy_new_year_2018, N_PTS(happy_new_year_2018), transform, DATA_RELATIVE);
  } else {
    const float t1 = linstep(t, 30, 30.15)*0.995;
    const float t2 = linstep(t, 30.15, 30.3)*0.995;
    const float s = 0.002;
    mat_t transform;
    mat_identity(transform);
    mat_translate(transform, cx, cy);
    mat_scale(transform, s*(1-t2), s*(1.0-t1));
    mat_translate(transform, 0.5, 0.5);
    draw_buffer(happy_new_year_2018, N_PTS(happy_new_year_2018), transform, DATA_RELATIVE);
  }
  fb_swap();
}
#elif PATTERN == 6
void loop() {
  static uint32_t start_t = 0;
  const float cx = -drunken_octopus_rect.x - drunken_octopus_rect.w/2;
  const float cy = -drunken_octopus_rect.y - drunken_octopus_rect.h/2;
  if(start_t == 0) {
    start_t = millis();
  }
  const float t = float(millis()-start_t)/DELAY_1S;
  if(t < 3.0) {
    mat_t transform;
    mat_identity(transform);
    mat_rotate(transform, t*2*PI);
    mat_translate(transform, 0.5, 0.5);
    draw_buffer(happy_new_year_line, N_PTS(happy_new_year_line), transform);
    mat_identity(transform);
    switch(3-int(t)) {
      case 3: draw_buffer(happy_new_year_digit_3, N_PTS(happy_new_year_digit_3), transform, DATA_RELATIVE); break;
      case 2: draw_buffer(happy_new_year_digit_2, N_PTS(happy_new_year_digit_2), transform, DATA_RELATIVE); break;
      case 1: draw_buffer(happy_new_year_digit_1, N_PTS(happy_new_year_digit_1), transform, DATA_RELATIVE); break;
    }
  } else if(t < 9.0) {
    // Zoom and rotate
    const float t1 = linstep(t, 3, 7);
    const float s = t1 * 0.002;
    const float r = t1 * 3;
    mat_t transform;
    mat_identity(transform);
    mat_translate(transform, cx + (1.0 - t1)*200, cy);
    mat_scale(transform, s, s);
    mat_rotate(transform, 2*PI*r);
    mat_translate(transform, 0.5, 0.5);
    draw_buffer(drunken_octopus_pts, N_PTS(drunken_octopus_pts), transform, DATA_RELATIVE);
  } else if(t < 12.0) {
    // Fast zoom out
    const float t1 = linstep(t, 9, 9.1);
    const float s = (1.0 - t1 * 0.98) * 0.002;
    mat_t transform;
    mat_identity(transform);
    mat_translate(transform, cx, cy);
    mat_scale(transform, s, s);
    mat_translate(transform, 0.5, 0.5);
    draw_buffer(drunken_octopus_pts, N_PTS(drunken_octopus_pts), transform, DATA_RELATIVE);
  } else {
    // Rocking motion
    mat_t transform;
    mat_identity(transform);
    mat_translate(transform, cx, cy);
    const float s = sin(t - 8);
    const float s_heartbeat = sin(t - 8) * 0.00025;
    const float r_heartbeat = sin((t - 8) * 0.7) * 0.5;
    mat_scale(transform, 0.002 + s_heartbeat, 0.002 + s_heartbeat);
    mat_rotate(transform, 2*PI*3 + r_heartbeat);
    mat_translate(transform, 0.5, 0.5);
    draw_buffer(drunken_octopus_pts, N_PTS(drunken_octopus_pts), transform, DATA_RELATIVE);    
  }
  
  /*const float cx = -drunken_octopus_rect.x - drunken_octopus_rect.w/2;
  const float cy = -drunken_octopus_rect.y - drunken_octopus_rect.h/2;
  mat_t transform;
  mat_identity(transform);
  mat_translate(transform, cx, cy);
  mat_scale(transform, 1/drunken_octopus_rect.w, 1/drunken_octopus_rectern_rect.h);
  mat_translate(transform, 0.5, 0.5);
  mat_scale(transform, 0.9, 0.9);
  draw_buffer(user_pattern, N_PTS(user_pattern), transform, DATA_RELATIVE);*/
  fb_swap();
}
#endif

/***************************************************************************/

/* Useful documents:
 *   Running PWM up to 8MHz
 *    https://withinspecifications.30ohm.com/2014/02/20/Fast-PWM-on-AtMega328/
 *    http://playground.arduino.cc/Main/TimerPWMCheatsheet
 */
