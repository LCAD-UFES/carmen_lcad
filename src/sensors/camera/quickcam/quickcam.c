 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include <linux/videodev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "pwc-ioctl.h"
#include "ccvt.h"
#include "../camera_hw_interface.h"

static int camera_fd;

static char *camera_dev;
static int image_width;
static int image_height;
static int brightness;
static int hue;
static int saturation;
static int contrast;
static int camera_gamma;
  
static int denoisestrength;
static int antiflicker;
static int backlightcompensation;
static int useautosharpen;
static char *awbmode;
static int awbblue;
static int awbred;
static int useautoshutter;
static int useagc;
static int fps;

static int sharpenstrength;
static int shutterlength;
static int gain;

static int dospecial = 1;

static void check_camera_type(void)
{
  struct pwc_probe pp;

  if(ioctl(camera_fd, VIDIOCPWCPROBE, &pp) == 0) 
    return;

  carmen_die("Camera is not a Philips camera! exiting...\n");
}

static void setup_camera(void)
{
  struct video_picture svp;
  struct pwc_whitebalance pw;  
  struct video_window svw;  
  int tmp = 1;

  if(ioctl(camera_fd, VIDIOCPWCSCQUAL, &tmp) != 0) 
    carmen_die_syserror("Could not set camera parameters");
	
  if(useagc) {
    tmp = -1;
    if(ioctl(camera_fd, VIDIOCPWCSAGC, &tmp) != 0)
      carmen_die_syserror("Could not set camera parameters");
  } else {
    if(ioctl(camera_fd, VIDIOCPWCSAGC, &gain) != 0)
      carmen_die_syserror("Could not set camera parameters");
  }
	
  if(useautoshutter) {
    tmp = -1;
    if(ioctl(camera_fd, VIDIOCPWCSSHUTTER, &tmp) != 0)
      carmen_die_syserror("Could not set camera parameters");
  } else {
    if(ioctl(camera_fd, VIDIOCPWCSSHUTTER, &shutterlength) != 0)
       carmen_die_syserror("Could not set camera parameters");
  }
	
  if(ioctl(camera_fd, VIDIOCPWCGAWB, &pw) != 0)
    carmen_die_syserror("Could not set camera parameters");
  if (carmen_strcasecmp(awbmode, "Auto") == 0) {
    pw.mode = PWC_WB_AUTO;
  } else if(carmen_strcasecmp(awbmode, "Custom") == 0) {
    pw.mode = PWC_WB_MANUAL;
    pw.manual_red = awbred;
    pw.manual_blue = awbblue;
  } else if(carmen_strcasecmp(awbmode, "Indoor") == 0) {
    pw.mode = PWC_WB_INDOOR;
  } else if(carmen_strcasecmp(awbmode, "Outdoor") == 0) {
    pw.mode = PWC_WB_OUTDOOR;
  } else if(carmen_strcasecmp(awbmode, "Fluorescent") == 0) {
    pw.mode = PWC_WB_FL;
  } else {
    carmen_warn("Unknown white balance mode %s!\n", awbmode);
    pw.mode = PWC_WB_AUTO;
  }
  if(ioctl(camera_fd, VIDIOCPWCSAWB, &pw) != 0)
    carmen_die_syserror("Could not set camera parameters");
  
  // [fix] do something with the led?
  
  if(useautosharpen) {
    tmp = -1;
    if(ioctl(camera_fd, VIDIOCPWCSCONTOUR, &tmp) != 0)
      carmen_die_syserror("Could not set camera parameters");
  } else {
    if(ioctl(camera_fd, VIDIOCPWCSCONTOUR, &sharpenstrength) != 0)
      carmen_die_syserror("Could not set camera parameters");
  }
	
  if(backlightcompensation) {
    tmp = 1;
    if(ioctl(camera_fd, VIDIOCPWCSBACKLIGHT, &tmp) != 0)
      carmen_die_syserror("Could not set camera parameters");
  } else {	
    tmp = 0;
    if(ioctl(camera_fd, VIDIOCPWCSBACKLIGHT, &tmp) != 0)
      carmen_die_syserror("Could not set camera parameters");
  }
  
  if(antiflicker) {
    tmp = 1;
    if(ioctl(camera_fd, VIDIOCPWCSFLICKER, &tmp) != 0)
      carmen_die_syserror("Could not set camera parameters");
  } else {
    tmp = 0;
    if(ioctl(camera_fd, VIDIOCPWCSFLICKER, &tmp) != 0)
      carmen_die_syserror("Could not set camera parameters");
  }
  
  if(ioctl(camera_fd, VIDIOCPWCSDYNNOISE, &denoisestrength) != 0)
    carmen_die_syserror("Could not set camera parameters");
  
  if(ioctl(camera_fd, VIDIOCGPICT, &svp) != 0)
    carmen_die_syserror("Could not set camera parameters");
  
  if(brightness != -1) svp.brightness = brightness;
  if(hue != -1)        svp.hue        = hue;
  if(saturation != -1) svp.colour     = saturation;
  if(contrast != -1)   svp.contrast   = contrast;
  if(camera_gamma != -1)      svp.whiteness  = camera_gamma;
  
  if(dospecial) svp.palette = VIDEO_PALETTE_YUV420P;
  
  if(ioctl(camera_fd, VIDIOCSPICT, &svp) != 0)
    carmen_die_syserror("Could not set camera parameters");
  
  if(ioctl(camera_fd, VIDIOCGWIN, &svw) != 0)
    carmen_die_syserror("Could not set camera parameters");
  
  if(dospecial) svw.width = image_width;
  if(dospecial) svw.height = image_height;
  
  svw.flags &= ~PWC_FPS_FRMASK;
  svw.flags |= (fps << PWC_FPS_SHIFT);
  
  if(ioctl(camera_fd, VIDIOCSWIN, &svw) != 0)
    carmen_die_syserror("Could not set camera parameters");
}

void read_parameters(int argc, char **argv)
{
  carmen_param_t camera_params[] = {
    {"camera", "dev", CARMEN_PARAM_STRING, &camera_dev, 0, NULL},
    {"camera", "image_width", CARMEN_PARAM_INT, &image_width, 0, NULL},
    {"camera", "image_height", CARMEN_PARAM_INT, &image_height, 0, NULL},
    {"camera", "brightness", CARMEN_PARAM_INT, &brightness, 0, NULL},
    {"camera", "hue", CARMEN_PARAM_INT, &hue, 0, NULL},
    {"camera", "saturation", CARMEN_PARAM_INT, &saturation, 0, NULL},
    {"camera", "contrast", CARMEN_PARAM_INT, &contrast, 0, NULL},
    {"camera", "gamma", CARMEN_PARAM_INT, &camera_gamma, 0, NULL},
    {"camera", "denoisestrength", CARMEN_PARAM_INT, &denoisestrength, 0, NULL},
    {"camera", "awbmode", CARMEN_PARAM_STRING, &awbmode, 0, NULL},
    {"camera", "antiflicker", CARMEN_PARAM_ONOFF, &antiflicker, 0, NULL},
    {"camera", "backlightcompensation", CARMEN_PARAM_ONOFF, &backlightcompensation, 0, NULL},
    {"camera", "useautosharpen", CARMEN_PARAM_ONOFF, &useautosharpen, 0, NULL},
    {"camera", "useautoshutter", CARMEN_PARAM_ONOFF, &useautoshutter, 0, NULL},
    {"camera", "useagc", CARMEN_PARAM_ONOFF, &useagc, 0, NULL},
    {"camera", "fps", CARMEN_PARAM_INT, &fps, 0, NULL}};

  carmen_param_install_params(argc, argv, camera_params, 
			      sizeof(camera_params) / 
			      sizeof(camera_params[0]));

  carmen_param_set_module("camera");
  if (!useautosharpen) {
    if (carmen_param_get_int("sharpenstrength", &sharpenstrength,NULL) < 0)
      carmen_die("camera_sharpenstrength must be specified or "
		 "carmen_useautosharpen\nmust be turned on\n");
  }
  if (!useautoshutter) {
    if (carmen_param_get_int("shutterlength", &shutterlength,NULL) < 0)
      carmen_die("camera_shutterlength must be specified or "
		 "carmen_useautoshutter\nmust be turned on\n");
  }
  if (!useagc) {
    if (carmen_param_get_int("gain", &gain,NULL) < 0)
      carmen_die("camera_gain must be specified or carmen_useagc\n"
		 "must be turned on\n");
  }
  if(carmen_strcasecmp(awbmode, "Custom") == 0) {
    if (carmen_param_get_int("awbred", &awbred,NULL) < 0)
      carmen_die("camera_awbred must be specified or carmen_awbmode\n"
		 "cannot be set to custom.\n");
    if (carmen_param_get_int("awbblue", &awbblue,NULL) < 0)
      carmen_die("camera_awbblue must be specified or carmen_awbmode\n"
		 "cannot be set to custom.\n");
  }
}

carmen_camera_image_t *carmen_camera_start(int argc, char **argv)
{
  carmen_camera_image_t *image;

  read_parameters(argc, argv);

  carmen_warn("Opening camera\n");

  camera_fd = open(camera_dev, O_RDONLY | O_NOCTTY);
  if (camera_fd < 0)
    carmen_die_syserror("Could not open %s as camera device", camera_dev);

  check_camera_type();
  setup_camera();

  image = (carmen_camera_image_t *)calloc(1, sizeof(carmen_camera_image_t));

  image->width = image_width;
  image->height = image_height;
  image->bytes_per_pixel = 3;

  image->image_size = image->width*image->height*image->bytes_per_pixel;
  image->is_new = 0;
  image->timestamp = 0;
  image->image = (char *)calloc(image->image_size, sizeof(char));
  carmen_test_alloc(image->image);
  memset(image->image, 0, image->image_size*sizeof(char));

  return image;
}

void carmen_camera_shutdown(void)
{
  close(camera_fd);
}

void carmen_camera_grab_image(carmen_camera_image_t *image)
{
  long num_read;

  static char *yuv_buffer = NULL;

  if (yuv_buffer == NULL) {
    yuv_buffer = (char *)calloc(image->width*image->height*4, sizeof(char));
    carmen_test_alloc(yuv_buffer);
  }

  num_read = read(camera_fd, yuv_buffer, image->width*image->height*4);
  ccvt_420p_rgb24(image->width, image->height, yuv_buffer, image->image);
  
  image->timestamp = carmen_get_time();
  image->is_new = 1;
}
