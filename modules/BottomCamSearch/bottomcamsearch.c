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
#include "bottomcamsearch.h"

// UDP RTP Images
#include "udp/socket.h"

// Threaded computer vision
#include <pthread.h>

//Messages
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/pprz_transport.h"
#include "subsystems/datalink/telemetry.h"

// Timing
#include <sys/time.h>


/*
uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;
*/

uint8_t color_lum_min = 60;
uint8_t color_lum_max = 160;
uint8_t color_cb_min  = 95;
uint8_t color_cb_max  = 125;
uint8_t color_cr_min  = 170;
uint8_t color_cr_max  = 240;

int color_count = 0;

uint16_t blob_center_x = 0;
uint16_t blob_center_y = 0;

uint16_t cp_value_u = 0;
uint16_t cp_value_v = 0;

uint16_t integral_max_x = 3;
uint16_t integral_max_y = 4;

int32_t blob_debug_x = 0;
int32_t blob_debug_y = 0;
int32_t sonar_debug = 0;

float px_angle_x = 0.0;
float px_angle_y = 0.0;

float h = 0.0;

float x_pos_b = 0.0;
float y_pos_b = 0.0;

int32_t ecef_x_optitrack = 0;
int32_t ecef_y_optitrack = 0;
int32_t ecef_z_optitrack = 0;

int32_t x_pos_optitrack = 0;
int32_t y_pos_optitrack = 0;
int32_t z_pos_optitrack = 0;

float x_pos = 0.0;
float y_pos = 0.0;

//compensation angles
int32_t phi_temp = 0;
int32_t theta_temp = 0;
int32_t psi_temp = 0;
struct FloatEulers* body_angle;
  
//timing
long diffTime;
int32_t dt = 0;

//Speed calculations
int samples = 0;
float av_time = 0;
float x_speed = 0;
float y_speed = 0;
float z_speed = 0;

void bottomcamsearch_run(void) {
}

/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

// Video
#include "v4l/video.h"
#include "resize.h"
#include "color.h"

#include "encoding/jpeg.h"
#include "encoding/rtp.h"

#include <stdio.h>
#include <string.h>

//attitude
#include "state.h" 
// Calculations
#include <math.h>
#include "subsystems/ins/ins_int.h" // used for ins.sonar_z
//coordinates
#include "math/pprz_geodetic_double.h"
#include "math/pprz_algebra_double.h"
#include "subsystems/gps/gps_datalink.h"
//optitrack
//#include "subsystems/datalink/downlink.h"
//#include "dl_protocol.h"

// Defines to make easy use of paparazzi math
struct EnuCoor_d pos, speed,enu;
struct EcefCoor_d ecef_pos, new_pos;
struct LlaCoor_d lla_pos;
struct LtpDef_d tracking_ltp;       ///< The tracking system LTP definition
struct EcefCoor_d ecef_vel;       ///< Last valid ECEF velocity in meters

// Timers
struct timeval start_time;
struct timeval end_time;

#define USEC_PER_MS 1000
#define USEC_PER_SEC 1000000

#define Fx		171.5606 //due to chroma downsampling in x-direction 343.1211 // Camera focal length (px/rad)
#define Fy		348.5053 // Camera focal length (px/rad)


 static void send_blob_debug(void) {
 DOWNLINK_SEND_BLOB_DEBUG(DefaultChannel, DefaultDevice, &x_pos_optitrack, &blob_debug_y, &psi_temp, &theta_temp);//&cp_value_u, &cp_value_v);
 }


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

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void* data);
void *computervision_thread_main(void* data)
{
  // Video Input
  struct vid_struct vid;
  vid.device = (char*)"/dev/video2";
  vid.w=320;
  vid.h=240;
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    computervision_thread_status = -1;
    return 0;
  }

  // Video Grabbing
  //struct img_struct* img_new = video_create_image(&vid);

  // Video Resizing
  #define DOWNSIZE_FACTOR   2
  uint8_t quality_factor = 20; // From 0 to 99 (99=high) 50
  uint8_t dri_jpeg_header = 0;
  int millisleep = 1;//25;//250

  struct img_struct small;
  small.w = vid.w; /// DOWNSIZE_FACTOR;
  small.h = vid.h;/// DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);


  // Video Compression
  uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);

  // Network Transmit
  struct UdpSocket* vsock;
  vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);
  

  while (computer_vision_thread_command > 0)
  {
    usleep(1000* millisleep);
    
    diffTime = end_timer();  //
    start_timer();
    dt = (int32_t)(diffTime)/USEC_PER_MS;// check the loop rate
    
    video_grab_image(&vid, &small);

    color_count = colorblob_uyvy(&small,&small,
        color_lum_min,color_lum_max,
        color_cb_min,color_cb_max,
        color_cr_min,color_cr_max,
	&blob_center_x,
	&blob_center_y,
	&cp_value_u,
        &cp_value_v
        );

    
    body_angle 	= stateGetNedToBodyEulers_f();//fetch the body angles
    
    if(color_count > 30)
    {
    px_angle_x = (((float)blob_center_x - 80)/Fx)-body_angle->phi;//calculate the angle with respect to the blob
    px_angle_y = (((float)blob_center_y - 120)/Fy)-body_angle->theta;
    }
    else
    {
      px_angle_x = 0;
      px_angle_y = 0;
    }
    h = (float)ins_impl.sonar_z*100;// h in cm
    
    x_pos_b = -(tanf(px_angle_x)*h); //x_pos in cm
    y_pos_b = (tanf(px_angle_y)*h); // y_pos in cm
    
    x_pos = cosf(-body_angle->psi)*x_pos_b - sinf(-body_angle->psi)*y_pos_b; //transform of body position to global position
    y_pos = cosf(-body_angle->psi)*y_pos_b + sinf(-body_angle->psi)*x_pos_b;
    
    pos.x = x_pos/100; //pos.x in M
    pos.y = y_pos/100; //pos.y in M
    pos.z = (float)ins_impl.sonar_z; //pos.z in M
    
    ecef_of_enu_point_d(&ecef_pos ,&tracking_ltp ,&pos);
    lla_of_ecef_d(&lla_pos, &ecef_pos);
    
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
    //optitrack coordinates in ecef
    new_pos.x = ecef_x_optitrack;
    new_pos.y = ecef_y_optitrack;
    new_pos.z = ecef_z_optitrack;
    
    enu_of_ecef_point_d(&enu, &tracking_ltp, &new_pos);
    x_pos_optitrack = (int32_t)enu.x; 
    y_pos_optitrack = (int32_t)enu.y; 
    z_pos_optitrack = (int32_t)enu.z; 
    */
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
      (int)(body_angle->psi*10000000.0));             //int32 Course in rad*1e7
    
    //prepare for debug send
    phi_temp 	= ANGLE_BFP_OF_REAL(body_angle->phi);
    theta_temp 	= ANGLE_BFP_OF_REAL(body_angle->theta);//body_angle->theta);
    psi_temp = ANGLE_BFP_OF_REAL(body_angle->psi);
    blob_debug_x = (int32_t)x_pos_b;// x_pos debug
    blob_debug_y = (int32_t)y_pos_b;// y_pos debug 
    sonar_debug = (int32_t)h;
    
    
    printf("ColorCount = %d \n", color_count);

    // JPEG encode the image:
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
    uint32_t size = end-(jpegbuf);

    printf("Sending an image ...%u\n",size);
/*
 *
 */
    send_rtp_frame(
        vsock,            // UDP
        jpegbuf,size,     // JPEG
        small.w, small.h, // Img Size
        0,                // Format 422
        quality_factor,               // Jpeg-Quality
        dri_jpeg_header,                // DRI Header
        0              // 90kHz time increment
     );
  }
  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;
}

void bottomcamsearch_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
  
  register_periodic_telemetry(DefaultPeriodic, "BLOB_DEBUG", send_blob_debug);
  
  // Set the default tracking system position and angle
  struct EcefCoor_d tracking_ecef;
  tracking_ecef.x = 3924304;
  tracking_ecef.y = 300360;
  tracking_ecef.z = 5002162;
 // tracking_offset_angle = 123.0 / 57.6;
  ltp_def_from_ecef_d(&tracking_ltp, &tracking_ecef);
  
  
}

void bottomcamsearch_stop(void)
{
  computer_vision_thread_command = 0;
}



