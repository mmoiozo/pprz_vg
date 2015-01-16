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

// Own Header
#include "avoid_navigation.h"

// Paparazzi Data
#include "state.h"

// Interact with navigation
#include "navigation.h"

// Know waypoint numbers and blocks
#include "generated/flight_plan.h"

// Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "boards/ardrone/navdata.h"

#include "led.h"

struct AvoidNavigationStruct avoid_navigation_data;
bool_t obstacle_detected = FALSE;
int32_t counter[6] = {0,0,0,0,0,0};
int32_t free_frame_counter = 0;
uint8_t obstacle_in_frame = 0;

// Called once on paparazzi autopilot start
void init_avoid_navigation()
{
  // Do nothing
  avoid_navigation_data.mode = 0;
}

// Called on each vision analysis result after receiving the struct
void run_avoid_navigation_onvision(void)
{
  // Send ALL vision data to the ground
  DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 7, avoid_navigation_data.stereo_bin);

  switch (avoid_navigation_data.mode)
  {
  case 0:     // Go to Goal and stop at obstacles
    for(uint8_t i=0; i<6; i++) {
      //count 4 subsequent obstacles in any of the bins
      if(avoid_navigation_data.stereo_bin[i]>7) {
        counter[i] = counter[i] + 1;
        if(counter[i] > 3) {
          for(uint8_t j=0; j<6; j++) {
            counter[j] = 0;
          }
          //Obstacle detected, go to turn until clear mode
          obstacle_detected = TRUE;
          avoid_navigation_data.mode = 1;
        }
      }
      else
        counter[i] = 0;
    }
    break;
  case 1:     // Turn until clear
    //count 20 subsequent free frames
    obstacle_in_frame = 0;
    for(uint8_t i=0; i<6; i++) {
      obstacle_in_frame += avoid_navigation_data.stereo_bin[i]>7;
    }
    if(obstacle_in_frame == 0) {
      free_frame_counter = free_frame_counter + 1;
      if(free_frame_counter > 10) {
        free_frame_counter = 0;
        //Stop and put waypoint 2.5 m ahead
        struct EnuCoor_i new_coor;
        struct EnuCoor_i* pos = stateGetPositionEnu_i();
        float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
        float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
        new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading*2.0);
        new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading*2.0);
        new_coor.z = pos->z;
        nav_move_waypoint(WP_W1, &new_coor);
        obstacle_detected = FALSE;
        avoid_navigation_data.mode = 0;
      }
    }
    else
      free_frame_counter = 0;
    break;
  default:    // do nothing
    break;
  }
//   avoid_navigation_data.stereo_bin[2] = avoid_navigation_data.stereo_bin[0]>20;
//   avoid_navigation_data.stereo_bin[3] = avoid_navigation_data.mode;
//   avoid_navigation_data.stereo_bin[4] = counter;
   avoid_navigation_data.stereo_bin[6] = free_frame_counter;

  if(obstacle_detected) {
    LED_ON(3);
  }
  else {
    LED_OFF(3);
  }
}

void increase_nav_heading(int32_t *heading, int32_t increment) {
  *heading = *heading + increment;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//  HELPER FUNCTIONS

/*

static uint8_t average_bin(void)
{
  uint16_t avg = 0;
  for (int i=0; i < N_BINS; i++)
    avg += gst2ppz.obstacle_bins[i];
  avg /= N_BINS;
  return avg;
}

static uint8_t all_bins_less_than(uint8_t thres)
{
  for (int i=0; i < N_BINS; i++)
    if (gst2ppz.obstacle_bins[i] >= thres)
      return FALSE;
  return TRUE;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//  AVOID FUNCTIONS

void run_avoid_navigation_climb_until_clear(void)
{
  if (all_bins_less_than(avoid_navigation_data.setting_climb_bin_threshold))
  {
    // Stop climbing after some time.
    if (avoid_navigation_data.climb_extra_when_clear_timer > 0)
    {
      avoid_navigation_data.climb_extra_when_clear_timer--;
      navigation_SetFlightAltitude(flight_altitude + avoid_navigation_data.setting_climb_speed);
    }
  }
  else
  {
    // On each video frame: about 10Hz
    // Climb X meters
    navigation_SetFlightAltitude(flight_altitude + avoid_navigation_data.setting_climb_speed);
    avoid_navigation_data.climb_extra_when_clear_timer = avoid_navigation_data.setting_climb_extra_climb_timer;
  }
}

void run_avoid_navigation_move_target_waypoint(void)
{
  // TODO:

  // Use flightplan: LINE p1-p2
  // WP_p1 -> WP_p2

  // Align the nose with the direction of motion:
  int32_t dx = waypoints[WP_p2].x - waypoints[WP_p1].x;
  int32_t dy = waypoints[WP_p2].y - waypoints[WP_p1].y;
  // nav_heading = atan2() // INT32_ANGLE_FRAC

  //nav_set_heading_towards_waypoint(WP_p1);

  dx *= dx;
  dy *= dy;

  uint8_t avg = average_bin();
  if (avg > 10)
  {
    // There is danger!!
  }
}

*/


