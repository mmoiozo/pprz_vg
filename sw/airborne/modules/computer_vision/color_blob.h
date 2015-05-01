/*
 * Copyright (C) 2012-2013
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */


#include <stdint.h>
#include "lib/vision/image.h"

inline int colorblob_uyvy(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M, uint16_t *pix_x, uint16_t *pix_y, uint16_t *cp_u, uint16_t *cp_v);
inline int colorblob_uyvy(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M, uint16_t *pix_x, uint16_t *pix_y, uint16_t *cp_u, uint16_t *cp_v)
{
  int cnt = 0;
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
    
  char match = 0;
  uint16_t hold = 0;
  uint16_t old = 0;
  uint16_t x_mid = 0;
  uint16_t y_mid = 0;
  
  uint16_t x_integral [162] = {};//82 162
  uint16_t y_integral [240] = {};//uint16_t y_integral [240] = {};
      

  for (int y=0;y<output->h;y++)//output->h
  {
    for (int x=1;x<161;x++)//161 81
    {
      
      //center color picker
      if(x == 80 && y == output->h/2)
      {
	*cp_u = dest[0];       // U
	//*cp_u = dest[1];       // Y
        *cp_v = dest[2];        // V
      }
      
      // Color Check:
      if (
          // Light
               (dest[1] >= y_m)
            && (dest[1] <= y_M)
            && (dest[0] >= u_m)
            && (dest[0] <= u_M)
            && (dest[2] >= v_m)
            && (dest[2] <= v_M)
         )// && (dest[2] > 128))
      {
        cnt ++;
        // UYVY
        dest[0] = 250;//64;        // U
        dest[1] = source[1];  // Y
        dest[2] = 60;//255;        // V
        dest[3] = source[3];  // Y
        
        //Binary image 1
        match = 1;
        
        
      }
      else
      {
        // UYVY
        char u = source[0]-127;
        u/=4;
        dest[0] = 127;        // U
        dest[1] = source[1];  // Y
        u = source[2]-127;
        u/=4;
        dest[2] = 127;        // V
        dest[3] = source[3];  // Y
        
        // Binary image 0
        match = 0;
      }

      //blob center cross
      if(x == *pix_x || y == *pix_y)
      {
	dest[0] = 64;        // U
        dest[2] = 255;        // V
      }
      
      //center pix cross
      if(x == 80 || y == output->h/2)
      {
	dest[0] = 250;        // U
        dest[2] = 60;        // V
      }
      
      
      hold = match + x_integral[x-1] + x_integral[x] - old;
      old = x_integral[x];
      x_integral[x] = hold;
      
      
      dest+=4;
      source+=4;
    } 
    old = 0;
    y_integral[y] = hold;
  }
  
  x_mid = x_integral[160] / 2;//80 160
  y_mid = y_integral[239] / 2;//119 239
  
  for (uint16_t i=1;i<161;i++)//81 161
  {
    if(x_integral[i] > x_mid)
    {
      *pix_x = i;
      break;
    }
  }
  
  for (uint16_t j=0;j<240;j++)//120 240
  {
    if(y_integral[j] > y_mid)
    {
      *pix_y = j;
      break;
    }
  }
  
  return cnt;
}

inline int multi_blob_uyvy(struct image_t *input, struct image_t *output, uint8_t filter_thresholds[30], uint16_t *pix_x[5], uint16_t *pix_y[5], uint16_t *cp_u, uint16_t *cp_v);
inline int multi_blob_uyvy(struct image_t *input, struct image_t *output, uint8_t filter_thresholds[30], uint16_t *pix_x[5], uint16_t *pix_y[5], uint16_t *cp_u, uint16_t *cp_v)
{
  int cnt[5] = {0,0,0,0,0};
  int any_match = 0;
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  
  char match[5] = {};
  uint16_t hold[5] = {};
  uint16_t old[5] = {};
  uint16_t x_mid[5] = {};
  uint16_t y_mid[5] = {};
  
  uint16_t x_integral [162][5] = {};//82 162
  uint16_t y_integral [240][5] = {};//uint16_t y_integral [240] = {};
      

  for (int y=0;y<output->h;y++)//output->h
  {
    for (int x=1;x<161;x++)//161 81
    {
      
      //center color picker
      if(x == 80 && y == output->h/2)
      {
	*cp_u = dest[0];       // U
	//*cp_u = dest[1];       // Y
        *cp_v = dest[2];        // V
      }
      
      for(int i=0;i<5;i++)
      {
      // Color Check:
      if (
          // Light
               (dest[1] >= filter_thresholds[i*5])
            && (dest[1] <= filter_thresholds[(i*5)+1])
            && (dest[0] >= filter_thresholds[(i*5)+2])
            && (dest[0] <= filter_thresholds[(i*5)+3])
            && (dest[2] >= filter_thresholds[(i*5)+4])
            && (dest[2] <= filter_thresholds[(i*5)+5])
         )// && (dest[2] > 128))
      {
        cnt[i] ++;
        //Binary image 1
        match[i] = 1;
	any_match = 1;
        
        
      }
      
      }
      if(any_match > 0)
      {
	// UYVY
        dest[0] = 250;//64;        // U
        dest[1] = source[1];  // Y
        dest[2] = 60;//255;        // V
        dest[3] = source[3];  // Y
      }
      else
      {
        // UYVY
        char u = source[0]-127;
        u/=4;
        dest[0] = 127;        // U
        dest[1] = source[1];  // Y
        u = source[2]-127;
        u/=4;
        dest[2] = 127;        // V
        dest[3] = source[3];  // Y
        
      }
      any_match = 0;
      
      //blob center cross
      if(x == *pix_x[0] || y == *pix_y[0])
      {
	dest[0] = 64;        // U
        dest[2] = 255;        // V
      }
      
      //center pix cross
      if(x == 80 || y == output->h/2)
      {
	dest[0] = 250;        // U
        dest[2] = 60;        // V
      }
      
      for(int i=0;i<5;i++)
      {
      hold[i] = match[i] + x_integral[x-1][i] + x_integral[x][i] - old[i];
      old[i] = x_integral[x][i];
      x_integral[x][i] = hold[i];
      }
      
      dest+=4;
      source+=4;
    } 
    for(int i=0;i<5;i++)
      {
    old[i] = 0;
    y_integral[y][i] = hold[i];
      }
  }
  
  for(int c=0;c<5;c++)
      {
  x_mid[c] = x_integral[160][c] / 2;//80 160
  y_mid[c] = y_integral[239][c] / 2;//119 239
  
  for (uint16_t i=1;i<161;i++)//81 161
  {
    if(x_integral[i][c] > x_mid[c])
    {
      *pix_x[c] = i;
      break;
    }
  }
  
  for (uint16_t j=0;j<240;j++)//120 240
  {
    if(y_integral[j][c] > y_mid[c])
    {
      *pix_y[c] = j;
      break;
    }
  }
      }
  return cnt[0];
}
