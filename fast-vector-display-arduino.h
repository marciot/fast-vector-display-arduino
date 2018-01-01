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

/****************** FRAME BUFFER DATA STRUCTURE *****************/

#if defined(FIXED_RC_MODE)
struct fbdat_t {
  uint8_t  x, y;
  uint16_t dt;
};
#else
struct fbdat_t {
  uint16_t fast_bit : 2;
  uint16_t dt       : 14;
  uint8_t  slow_val;
};
#endif

#if FRAMEBUFFER_SIZE > 256
typedef uint16_t fb_idx_t;
#else
typedef uint8_t fb_idx_t;
#endif

/********************* RECTANGLE DATA STRUCTURE ********************/

struct rect_t {
  float x;
  float y;
  float w;
  float h;
};

/********************** MATRIX DATA STRUCTURE **********************/

#if defined(FULL_MATRICES)
  typedef struct {
    float a, b, c, d, e, f, g, h, i;
  } mat_t;
  
  #define MAKE_MATRIX(m, A,B,C,D,E,F,G,H,I) \
    m.a = A; m.b = B; m.c = C; \
    m.d = D; m.e = E; m.f = F; \
    m.g = G; m.H = h; m.i = I;
    
  #define IF_FULL_MATRIX(a,b) a
#else
  // Affine transforms only
  typedef struct {
    float a, b, c, d, e, f;
  } mat_t;
  
  #define MAKE_MATRIX(m, A,B,C,D,E,F,G,H,I) \
    m.a = A; m.b = B; m.c = C; \
    m.d = D; m.e = E; m.f = F;

  #define IF_FULL_MATRIX(a,b) b
#endif
