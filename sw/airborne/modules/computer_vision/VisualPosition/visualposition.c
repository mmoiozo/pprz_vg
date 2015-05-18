/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/viewvideo.c
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

// Own header
#include "modules/computer_vision/viewvideo.h"
#include "modules/computer_vision/color_blob.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

// Video
#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"

// Threaded computer vision
#include <pthread.h>

//Messages
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/pprz_transport.h"
#include "subsystems/datalink/telemetry.h"
//ABI
#include "subsystems/abi.h"

//state 
#include "state.h"
#include "subsystems/imu.h"

// Calculations
#include <math.h>
#include "subsystems/ins/ins_int.h" // used for ins.sonar_z
#include "boards/ardrone/navdata.h" // for ultrasound Height
//coordinates
#include "math/pprz_geodetic_double.h"
#include "math/pprz_algebra_double.h"
#include "subsystems/gps/gps_datalink.h"

//camera angles
#define Fx		171.5606 //due to chroma downsampling in x-direction 343.1211 // Camera focal length (px/rad)
#define Fy		348.5053 // Camera focal length (px/rad)
//TIMGING
#define USEC_PER_MS 1000
#define USEC_PER_SEC 1000000
//Math
#define PI 3.141592653589793238462643383


// The video device
#ifndef VIEWVIDEO_DEVICE
#define VIEWVIDEO_DEVICE /dev/video2
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DEVICE)

// The video device size (width, height)
#ifndef VIEWVIDEO_DEVICE_SIZE
#define VIEWVIDEO_DEVICE_SIZE 320,240
#endif
#define __SIZE_HELPER(x, y) #x", "#y
#define _SIZE_HELPER(x) __SIZE_HELPER(x)
PRINT_CONFIG_MSG("VIEWVIDEO_DEVICE_SIZE = " _SIZE_HELPER(VIEWVIDEO_DEVICE_SIZE))

// The video device buffers (the amount of V4L2 buffers)
#ifndef VIEWVIDEO_DEVICE_BUFFERS
#define VIEWVIDEO_DEVICE_BUFFERS 15
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DEVICE_BUFFERS)

// Downsize factor for video stream
#ifndef VIEWVIDEO_DOWNSIZE_FACTOR
#define VIEWVIDEO_DOWNSIZE_FACTOR 1
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_DOWNSIZE_FACTOR)

// From 0 to 99 (99=high)
#ifndef VIEWVIDEO_QUALITY_FACTOR
#define VIEWVIDEO_QUALITY_FACTOR 20
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_QUALITY_FACTOR)

// RTP time increment at 90kHz (default: 0 for automatic)
#ifndef VIEWVIDEO_RTP_TIME_INC
#define VIEWVIDEO_RTP_TIME_INC 0
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_RTP_TIME_INC)

// Frames Per Seconds
#ifndef VIEWVIDEO_FPS
#define VIEWVIDEO_FPS 10
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_FPS)

// The place where the shots are saved (without slash on the end)
#ifndef VIEWVIDEO_SHOT_PATH
#define VIEWVIDEO_SHOT_PATH "/data/video/images"
#endif
PRINT_CONFIG_VAR(VIEWVIDEO_SHOT_PATH)

// Check if we are using netcat instead of RTP/UDP
#ifndef VIEWVIDEO_USE_NETCAT
#define VIEWVIDEO_USE_NETCAT FALSE
#endif
#if VIEWVIDEO_USE_NETCAT
PRINT_CONFIG_MSG("[viewvideo] Using netcat.")
#else
PRINT_CONFIG_MSG("[viewvideo] Using RTP/UDP stream.")
#endif

/* These are defined with configure */
PRINT_CONFIG_VAR(VIEWVIDEO_HOST)
PRINT_CONFIG_VAR(VIEWVIDEO_PORT_OUT)

static pthread_mutex_t visualposition_mutex;            ///< Mutex lock fo thread safety

// Main thread
static void *visualposition_thread(void *data);
void visualposition_periodic(void) { }

// Initialize the viewvideo structure with the defaults
struct viewvideo_t viewvideo = {
  .is_streaming = FALSE,
  .downsize_factor = VIEWVIDEO_DOWNSIZE_FACTOR,
  .quality_factor = VIEWVIDEO_QUALITY_FACTOR,
  .fps = VIEWVIDEO_FPS,
  .take_shot = FALSE,
  .shot_number = 0
};



//DEBUG MESSAGES

  int32_t phi_temp = 0;
  int32_t theta_temp = 0;
  int32_t psi_temp = 0;
  int32_t psi_map_temp = 0;
  int32_t blob_debug_x = 0;
  int32_t blob_debug_y = 0;
  int32_t sonar_debug = 10;
  
  uint32_t color_debug_u = 0;
  uint32_t color_debug_v = 0;
  uint32_t blob_x_debug = 0;
  uint32_t blob_y_debug = 0;
  
// Vision module
uint8_t color_lum_min = 60;
uint8_t color_lum_max = 160;
uint8_t color_cb_min  = 170;//95; //oranje
uint8_t color_cb_max  = 195;//125;
uint8_t color_cr_min  = 65;//170;
uint8_t color_cr_max  = 95;//240;

int color_count = 0;

uint16_t blob_center_x = 0;
uint16_t blob_center_y = 0;

uint16_t blob_x[5] = {0,0,0,0,0};
uint16_t blob_y[5] = {0,0,0,0,0};
uint16_t marker_positions[15] = {NULL};
float marker_body_positions[10] = {};
float marker_map[5] = {0,0,//center blob 1 orange
			  50,-10,// blob 2 blue
			  0,0,// empty
			  0,0,// empty
			  0,0};// empty
float marker_pos_g[5] = {};
float map_body_angle = 0;
float marker_heading = 0;
float marker_heading_reverse = 0;
float marker_heading_temp = 0;
float heading_offset = 0;
float compensated_heading = 0;
int calc_angle = 0;

uint16_t cp_value_u = 0;
uint16_t cp_value_v = 0;

float px_angle_x = 0.0;
float px_angle_y = 0.0;

float h = 0.0;

float x_pos_b = 0.0;
float y_pos_b = 0.0;

int32_t alt_optitrack = 0;
int32_t lat_optitrack = 0;
int32_t lon_optitrack = 0;
int32_t ecef_x_optitrack = 0;
int32_t ecef_y_optitrack = 0;
int32_t ecef_z_optitrack = 0;

float x_pos_optitrack = 0;
float y_pos_optitrack = 0;
float z_pos_optitrack = 0;

float x_pos = 0.0;
float y_pos = 0.0;
struct FloatEulers* body_angle;

//Speed calculations
int samples = 0;
float av_time = 0;
float x_speed = 0;
float y_speed = 0;
float z_speed = 0;

//timing
long diffTime;
int32_t dt = 0;
struct timeval start_time;
struct timeval end_time;

//values in flight arena

uint8_t filter_values[30] = {60,160,95,125,170,240,//orange
			     60,160,150,165,85,105,//blue
			     60,160,0,0,0,0,//empty
			     60,160,0,0,0,0,//empty
			     60,160,0,0,0,0};//empty
			     
//values in mav lab
/*
uint8_t filter_values[30] = {60,160,95,125,170,240,//orange
			     60,160,165,185,75,93,//blue
			     60,160,0,0,0,0,//empty
			     60,160,0,0,0,0,//empty
			     60,160,0,0,0,0};//empty
*/
// Defines to make easy use of paparazzi math
struct EnuCoor_d pos, speed,enu;
struct NedCoor_d ned;
struct EcefCoor_d ecef_pos, new_pos;
struct LlaCoor_d lla_pos;
struct LtpDef_d tracking_ltp;       ///< The tracking system LTP definition
struct EcefCoor_d ecef_vel;       ///< Last valid ECEF velocity in meters

//ABI
abi_event ev;

//function prototypes
int get_pos_b(uint16_t x_pix, uint16_t y_pix, float *body_x, float *body_y);
int get_angle(float x_a, float y_a, float x_b, float y_b, float *angle);
int get_pos_g(float x_body, float y_body, float *x_marker_g, float *y_marker_g);

static void sonar_abi(uint8_t sender_id __attribute__((unused)), float distance)
{
  // Update the distance if we got a valid measurement
  if (distance > 0) {
    h = distance*100;//in cm
  }
}

static void send_blob_debug(struct transport_tx *trans, struct link_device *dev) //static void send_blob_debug(void) 
 {
    pthread_mutex_lock(&visualposition_mutex);
 pprz_msg_send_BLOB_DEBUG(trans, dev, AC_ID, &color_debug_u, &color_debug_v, &sonar_debug, &phi_temp, &theta_temp, &psi_temp, &psi_map_temp, &imu.gyro.p, &imu.gyro.q, &imu.gyro.r);
  pthread_mutex_unlock(&visualposition_mutex);
 }


//TIMING
volatile long time_elapsed (struct timeval *t1, struct timeval *t2);
volatile long time_elapsed (struct timeval *t1, struct timeval *t2)
{
	long sec, usec;
	sec = t2->tv_sec - t1->tv_sec;
	usec = t2->tv_usec - t1->tv_usec;
	if (usec < 0) {
	--sec;
	usec = usec + USEC_PER_SEC;
	}
	return sec*USEC_PER_SEC + usec;
}
void start_timer(void);
void start_timer(void) {
	gettimeofday(&start_time, NULL);
}
long end_timer(void);
long end_timer(void) {
	gettimeofday(&end_time, NULL);
	return time_elapsed(&start_time, &end_time);
}


/**
 * Handles all the video streaming and saving of the image shots
 * This is a sepereate thread, so it needs to be thread safe!
 */
static void *visualposition_thread(void *data __attribute__((unused)))
{
   printf("started\n");
  // Start the streaming of the V4L2 device
  if (!v4l2_start_capture(viewvideo.dev)) {
    printf("[viewvideo-thread] Could not start capture of %s.\n", viewvideo.dev->name);
    return 0;
  }

  // Resize image if needed
  struct image_t img_small;
  image_create(&img_small,
               viewvideo.dev->w / viewvideo.downsize_factor,
               viewvideo.dev->h / viewvideo.downsize_factor,
               IMAGE_YUV422);

  // Create the JPEG encoded image
  struct image_t img_jpeg;
  image_create(&img_jpeg, img_small.w, img_small.h, IMAGE_JPEG);

  // Initialize timing
  int millisleep = 1;
  struct timeval last_time;
  gettimeofday(&last_time, NULL);

  struct UdpSocket video_sock;
  udp_socket_create(&video_sock, STRINGIFY(VIEWVIDEO_HOST), VIEWVIDEO_PORT_OUT, -1, VIEWVIDEO_BROADCAST);


  // Start streaming
  viewvideo.is_streaming = TRUE;
  while (viewvideo.is_streaming) {
    
    usleep(1000* millisleep);
     diffTime = end_timer();  //
    start_timer();
    dt = (int32_t)(diffTime)/USEC_PER_MS;// check the loop rate
    
    // Wait for a new frame (blocking)
    struct image_t img;
    v4l2_image_get(viewvideo.dev, &img);
    /*
     color_count = colorblob_uyvy(&img,&img,
        filter_values[0],filter_values[1],
        filter_values[2],filter_values[3],
        filter_values[4],filter_values[5],
	&blob_center_x,
	&blob_center_y,
	&cp_value_u,
        &cp_value_v
        );*/
     
     color_count = multi_blob_uyvy(&img,&img,
        filter_values,
	&blob_x,
	&blob_y,
	&cp_value_u,
        &cp_value_v,
	&marker_positions
        );
     
      body_angle = stateGetNedToBodyEulers_f();//fetch the body angles
      
      if(color_count > 0)
      {
	get_pos_b(marker_positions[1],marker_positions[2],&marker_body_positions[0],&marker_body_positions[1]);
      }
      //Marker selection
      if(color_count > 1)
      {
	get_pos_b(marker_positions[4],marker_positions[5],&marker_body_positions[2],&marker_body_positions[3]);
	calc_angle = get_angle(marker_body_positions[0],marker_body_positions[1],marker_body_positions[2],marker_body_positions[3],&marker_heading);
      //marker heading 0 to 360 deg (in radians)
      
       get_angle(marker_map[0],marker_map[1],marker_map[2],marker_map[3],&map_body_angle);
       
       if(marker_heading > PI)
       {
	 marker_heading_temp = marker_heading - (2*PI);
       }
       else
       {
	 marker_heading_temp = marker_heading;
       }
       
       if(marker_heading_temp < stateGetNedToBodyEulers_f()->psi)
       {
       heading_offset = marker_heading_temp - stateGetNedToBodyEulers_f()->psi + (2*PI);
       }
       else
       {
	 heading_offset = marker_heading_temp - stateGetNedToBodyEulers_f()->psi;
       }
      }
      compensated_heading = stateGetNedToBodyEulers_f()->psi + heading_offset;
      marker_heading_reverse = (2*PI) - marker_heading;
   
      get_pos_g(marker_body_positions[0], marker_body_positions[1], &marker_pos_g[0], &marker_pos_g[1]);
      
      
      
    if(marker_positions[0] == 1)
    {
    px_angle_x = (((float)blob_x[0] - 80)/Fx)-body_angle->phi;//calculate the angle with respect to the blob//marker_positions[1]
    px_angle_y = (((float)blob_y[0] - 120)/Fy)-body_angle->theta;//marker_positions[2]
    }
    else
    {
      px_angle_x = 0;
      px_angle_y = 0;
    }
    
    x_pos_b = -(tanf(px_angle_x)*h); //x_pos in cm
    y_pos_b = (tanf(px_angle_y)*h); // y_pos in cm
    
    x_pos = cosf(-compensated_heading)*x_pos_b - sinf(-compensated_heading)*y_pos_b; //transform of body position to global position
    y_pos = cosf(-compensated_heading)*y_pos_b + sinf(-compensated_heading)*x_pos_b;//try (-x_pos_b) etc heading should be 0 to 360 form or -180 to 180 positive clockwise marker_heading_reverse
    /*
    x_pos = cosf(-body_angle->psi)*x_pos_b - sinf(-body_angle->psi)*y_pos_b; //transform of body position to global position
    y_pos = cosf(-body_angle->psi)*y_pos_b + sinf(-body_angle->psi)*x_pos_b;*/
    
    //printf("pos test %f %f\n",x_pos,y_pos);
    pos.x = x_pos/100; //pos.x in M
    pos.y = y_pos/100; //pos.y in M
    pos.z = h/100;// agl_dist_value_filtered; //pos.z in M
    //float sonar = (ins_impl.sonar_alt - ins_impl.sonar_offset) * INS_SONAR_SENS;
    
    //printf("pos test %d %d %d\n",(int)pos.x,(int)pos.y,(int)pos.z);
    ecef_of_enu_point_d(&ecef_pos ,&tracking_ltp ,&pos);
    //printf("ecef test %f %f %f\n",ecef_pos.x,ecef_pos.y,ecef_pos.z);
    lla_of_ecef_d(&lla_pos, &ecef_pos);
    //fprintf("ecef test %f\n",&lla_pos);
    
    // Check if we have enough samples to estimate the velocity
    samples ++;
    av_time += dt;
    if(samples >= 2) {
      
      speed.x = (pos.x*1000) / av_time;
      speed.y = (pos.y*1000) / av_time;
      speed.z = (pos.z*1000) / av_time;

      // Conver the speed to ecef based on the Optitrack LTP
      ecef_of_enu_vect_d(&ecef_vel ,&tracking_ltp ,&speed);
    samples = 0;
    av_time = 0;
    }
/*
      parse_gps_datalink(
      1,                //uint8 Number of markers (sv_num)
      (int)(ecef_pos.x*100.0),                //int32 ECEF X in CM
      (int)(ecef_pos.y*100.0),                //int32 ECEF Y in CM
      (int)(ecef_pos.z*100.0),                //int32 ECEF Z in CM
      (int)(DegOfRad(lla_pos.lat)*1e7),       //int32 LLA latitude in deg*1e7
      (int)(DegOfRad(lla_pos.lon)*1e7),       //int32 LLA longitude in deg*1e7
      (int)(pos.z*1000.0),         //int32 LLA altitude in mm above elipsoid
      (int)(pos.z*1000.0),         //int32 HMSL height above mean sea level in mm
      (int)(ecef_vel.x*100.0), //int32 ECEF velocity X in cm/s
      (int)(ecef_vel.y*100.0), //int32 ECEF velocity Y in cm/s
      (int)(ecef_vel.z*100.0), //int32 ECEF velocity Z in cm/s
      0,
      (int)(compensated_heading*10000000.0));             //int32 Course in rad*1e7 //body_angle->psi
      */
      //optitrack coordinates in ecef
    new_pos.x = DOUBLE_OF_BFP(ecef_x_optitrack,8);//POS_FLOAT_OF_BFP(ecef_x_optitrack);
    new_pos.y = DOUBLE_OF_BFP(ecef_y_optitrack,8);//POS_FLOAT_OF_BFP(ecef_y_optitrack);
    new_pos.z = DOUBLE_OF_BFP(ecef_z_optitrack,8);//POS_FLOAT_OF_BFP(ecef_z_optitrack);
    
     enu_of_ecef_point_d(&enu, &tracking_ltp, &new_pos);
    x_pos_optitrack = enu.x*100; 
    y_pos_optitrack = enu.y*100; 
    z_pos_optitrack = enu.z*100; 
    //ned_of_ecef_point_d(&ned, &tracking_ltp, &new_pos);
   // x_pos_optitrack = ned.x*100; 
   // y_pos_optitrack = ned.y*100; 
   // z_pos_optitrack = ned.z*100;
 
     //Debug values
     color_debug_u = (int32_t)ecef_x_optitrack;//marker_body_positions[0];//x_pos_optitrack;//x_pos;//marker_pos_g[0];//cp_value_u;
     color_debug_v = (int32_t)ecef_y_optitrack;//marker_body_positions[1];//y_pos_optitrack;//y_pos;//marker_pos_g[1];//calc_angle;//cp_value_v;
     sonar_debug = (int32_t)z_pos_optitrack;//marker_positions[0];//color_count;//h;
     blob_x_debug = (int32_t)(blob_x[0]-80);
     blob_y_debug = (int32_t)(blob_y[0]-120);
     phi_temp = ANGLE_BFP_OF_REAL(stateGetNedToBodyEulers_f()->phi);
     theta_temp = ANGLE_BFP_OF_REAL(heading_offset);//stateGetNedToBodyEulers_f()->theta);
     psi_temp = ANGLE_BFP_OF_REAL(stateGetNedToBodyEulers_f()->psi);//marker_heading_temp);
     psi_map_temp = ANGLE_BFP_OF_REAL(compensated_heading);//marker_heading);
     
    // Only resize when needed
    if (viewvideo.downsize_factor != 1) {
      image_yuv422_downsample(&img, &img_small, viewvideo.downsize_factor);
      jpeg_encode_image(&img_small, &img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);
    } else {
      jpeg_encode_image(&img, &img_jpeg, VIEWVIDEO_QUALITY_FACTOR, VIEWVIDEO_USE_NETCAT);
    }


    // Send image with RTP
    rtp_frame_send(
      &video_sock,              // UDP socket
      &img_jpeg,
      0,                        // Format 422
      VIEWVIDEO_QUALITY_FACTOR, // Jpeg-Quality
      0,                        // DRI Header
      VIEWVIDEO_RTP_TIME_INC    // 90kHz time increment
    );
    // Extra note: when the time increment is set to 0,
    // it is automaticaly calculated by the send_rtp_frame function
    // based on gettimeofday value. This seems to introduce some lag or jitter.
    // An other way is to compute the time increment and set the correct value.
    // It seems that a lower value is also working (when the frame is received
    // the timestamp is always "late" so the frame is displayed immediately).
    // Here, we set the time increment to the lowest possible value
    // (1 = 1/90000 s) which is probably stupid but is actually working.

    // Free the image
    v4l2_image_free(viewvideo.dev, &img);
  }

  // Free all buffers
  image_free(&img_jpeg);
  image_free(&img_small);
  return 0;
}

//get marker body marker_positions
int get_pos_b(uint16_t x_pix, uint16_t y_pix, float *body_x, float *body_y)
{
   float pix_angle_x = (((float)x_pix - 80)/Fx)-body_angle->phi;//calculate the angle with respect to the blob
   float pix_angle_y = (((float)y_pix - 120)/Fy)-body_angle->theta;
   *body_x = -(tanf(pix_angle_x)*h); //x_pos in cm
   *body_y = (tanf(pix_angle_y)*h); // y_pos in cm
}

//get position in global frame based on single marker
int get_pos_g(float x_body, float y_body, float *x_marker_g, float *y_marker_g)
{/*
  *x_marker_g = cosf(marker_heading_temp)*x_body - sinf(marker_heading_temp)*y_body;//try -marker_heading_temp
   *y_marker_g = sinf(marker_heading_temp)*x_body + cosf(marker_heading_temp)*y_body;
  */
   *x_marker_g = cosf(marker_heading_reverse)*(-x_body) - sinf(marker_heading_reverse)*(-y_body);
   *y_marker_g = sinf(marker_heading_reverse)*(-x_body) + cosf(marker_heading_reverse)*(-y_body);
   /*
   *x_marker_g = cosf(marker_heading)*(-x_body) + sinf(marker_heading)*(-y_body);
   *y_marker_g = -sinf(marker_heading)*(-x_body) + cosf(marker_heading)*(-y_body);
   */
}

/* calculate angle based on two points */
int get_angle(float x_a, float y_a, float x_b, float y_b, float *angle)
{
  float x_tresh = 20;//treshhold value for when displacement is assumed small
  float y_tresh = 20;
  float delta_x = x_b - x_a;
  float delta_y = y_b - y_a;
  float dy_dx = delta_y/delta_x;
  int valid_data = 0;
  
  if(fabs(delta_x) <= x_tresh && fabs(delta_y) <= y_tresh)
  {
     valid_data = 0;// no valid calculation possible
  }
  else if(isinf(dy_dx) == 1 && dy_dx > 0)
  {
    *angle = PI/2;
    valid_data = 1;
  }
  else if(isinf(dy_dx) == 1 && dy_dx < 0)
  {
    *angle = (3/2)*PI;
    valid_data = 1;
  }
  else if(delta_x > 0 && dy_dx >= 0)
  {
    *angle = atanf(dy_dx);
    valid_data = 1;
  }
  else if(delta_x < 0 && dy_dx <= 0)
  {
    *angle = PI + atanf(dy_dx);
    valid_data = 1;
  }
  else if(delta_x < 0 && dy_dx >= 0)
  {
    *angle = PI + atanf(dy_dx);
    valid_data = 1;
  }
  else if(delta_x > 0 && dy_dx <= 0)
  {
    *angle = 2*PI + atanf(dy_dx);
    valid_data = 1;
  }
  
  return valid_data;
}

/**
 * Initialize the view video
 */
void visualposition_init(void)
{
#ifdef VIEWVIDEO_SUBDEV
  PRINT_CONFIG_MSG("[viewvideo] Configuring a subdevice!")
  PRINT_CONFIG_VAR(VIEWVIDEO_SUBDEV)

  // Initialize the V4L2 subdevice (TODO: fix hardcoded path, which and code)
  if (!v4l2_init_subdev(STRINGIFY(VIEWVIDEO_SUBDEV), 0, 1, V4L2_MBUS_FMT_UYVY8_2X8, VIEWVIDEO_DEVICE_SIZE)) {
    printf("[viewvideo] Could not initialize the %s subdevice.\n", STRINGIFY(VIEWVIDEO_SUBDEV));
    return;
  }
#endif

  // Initialize the V4L2 device
  viewvideo.dev = v4l2_init(STRINGIFY(VIEWVIDEO_DEVICE), VIEWVIDEO_DEVICE_SIZE, VIEWVIDEO_DEVICE_BUFFERS);
  if (viewvideo.dev == NULL) {
    printf("[viewvideo] Could not initialize the %s V4L2 device.\n", STRINGIFY(VIEWVIDEO_DEVICE));
    return;
  }

  // Create the shot directory
  char save_name[128];
  sprintf(save_name, "mkdir -p %s", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  if (system(save_name) != 0) {
    printf("[viewvideo] Could not create shot directory %s.\n", STRINGIFY(VIEWVIDEO_SHOT_PATH));
    return;
  }

#if VIEWVIDEO_USE_NETCAT
  // Create an Netcat receiver file for the streaming
  sprintf(save_name, "%s/netcat-recv.sh", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "i=0\n");
    fprintf(fp, "while true\n");
    fprintf(fp, "do\n");
    fprintf(fp, "\tn=$(printf \"%%04d\" $i)\n");
    fprintf(fp, "\tnc -l 0.0.0.0 %d > img_${n}.jpg\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "\ti=$((i+1))\n");
    fprintf(fp, "done\n");
    fclose(fp);
  }
#else
  // Create an SDP file for the streaming
  sprintf(save_name, "%s/stream.sdp", STRINGIFY(VIEWVIDEO_SHOT_PATH));
  FILE *fp = fopen(save_name, "w");
  if (fp != NULL) {
    fprintf(fp, "v=0\n");
    fprintf(fp, "m=video %d RTP/AVP 26\n", (int)(VIEWVIDEO_PORT_OUT));
    fprintf(fp, "c=IN IP4 0.0.0.0\n");
    fclose(fp);
  }
#endif
//init debug
    register_periodic_telemetry(DefaultPeriodic, "BLOB_DEBUG", send_blob_debug);
// Subscribe to the sonar_meas ABI messages
  AbiBindMsgAGL(ABI_BROADCAST, &ev, sonar_abi);
}

/**
 * Start with streaming
 */
void visualposition_start(void)
{
  // Check if we are already running
  if (viewvideo.is_streaming) {
    return;
  }

  // Start the streaming thread
  pthread_t tid;
  if (pthread_create(&tid, NULL, visualposition_thread, NULL) != 0) {
    printf("[vievideo] Could not create streaming thread.\n");
    return;
  }
   // Set the default tracking system position and angle
  struct EcefCoor_d tracking_ecef,tracking_ecef_optitrack;
  tracking_ecef.x = 3924304;
  tracking_ecef.y = 300360;
  tracking_ecef.z = 5002162;
 // tracking_offset_angle = 123.0 / 57.6;
  ltp_def_from_ecef_d(&tracking_ltp, &tracking_ecef);
}

/**
 * Stops the streaming
 * This could take some time, because the thread is stopped asynchronous.
 */
void visualposition_stop(void)
{
  // Check if not already stopped streaming
  if (!viewvideo.is_streaming) {
    return;
  }

  // Stop the streaming thread
  viewvideo.is_streaming = FALSE;

  // Stop the capturing
  if (!v4l2_stop_capture(viewvideo.dev)) {
    printf("[viewvideo] Could not stop capture of %s.\n", viewvideo.dev->name);
    return;
  }

  // TODO: wait for the thread to finish to be able to start the thread again!
}

/**
 * Take a shot and save it
 * This will only work when the streaming is enabled
 */
void viewvideo_take_shot(bool_t take)
{
  viewvideo.take_shot = take;
}
