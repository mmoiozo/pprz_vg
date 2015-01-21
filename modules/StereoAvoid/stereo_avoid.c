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


// Own header
#include "stereo_avoid.h"


// Navigate Based On Vision
#include "avoid_navigation.h"

// Paparazzi State: Attitude -> Vision
#include "state.h" // for attitude

// Serial Port
#include "mcu_periph/uart.h"

#define STEREO_PORT    UART1


#define __StereoLink(dev, _x) dev##_x
#define _StereoLink(dev, _x)  __StereoLink(dev, _x)
#define StereoLink(_x) _StereoLink(STEREO_PORT, _x)
#define StereoBuffer() StereoLink(ChAvailable())


void stereo_avoid_init(void) {

  // Navigation Code
  init_avoid_navigation();

  // UART


}

static void stereo_parse(uint8_t c);
static void stereo_parse(uint8_t c)
{
  static int cnt = 0;
  if (c == 0xff)
  {
    cnt = 1;
  }
  else if (cnt == 1)
  {
    avoid_navigation_data.stereo_bin[0] = c;
    cnt = 2;
  }
  else if (cnt == 2)
  {
    avoid_navigation_data.stereo_bin[1] = c;
    run_avoid_navigation_onvision();
    cnt = 0;
  }
  else
  {
    cnt = 0;
  }
}

void stereo_avoid_run(void) {

  if (StereoBuffer())
  {
    while (StereoLink(ChAvailable()))
      stereo_parse(StereoLink(Getch()));
  }

  //run_avoid_navigation_onvision()

  // 0xff
  // 0 - 101
  // 0 - 101
}

void stereo_avoid_start(void) {
}

void stereo_avoid_stop(void) {
}



