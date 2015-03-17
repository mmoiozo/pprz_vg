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

/**
 * @file visualposition.h
 */

#ifndef VISUAL_POSITION_H
#define VISUAL_POSITION_H

#include <stdint.h>

// Module functions
extern void visualposition_run(void);
extern void visualposition_start(void);
extern void visualposition_stop(void);

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern int color_count;

extern int32_t ecef_x_optitrack;
extern int32_t ecef_y_optitrack;
extern int32_t ecef_z_optitrack;

#endif /*  VISUAL POSITION H */
