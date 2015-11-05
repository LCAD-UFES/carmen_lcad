#include <unistd.h>
#include <cstdio>
#include <cassert>
#include <ros/time.h>
#include "uvc_cam/uvc_cam.h"
#include <signal.h>
#include <stdexcept>
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
extern "C"
{
#include "avilib.h"
}

const unsigned WIDTH = 160, HEIGHT = 120, FPS = 10;
static bool done = false;

void sigint_handler(int sig)
{
  done = true;
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    fprintf(stderr, "usage: view DEVICE\n");
    return 1;
  }
  ros::init(argc, argv, "uvc_cam");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::Image>("image", 1);

  avi_t avi;
  ros::Time t_prev(ros::Time::now());
  char fname[200], log_fname[200];
  sprintf(fname, "%.3f.avi", t_prev.toSec());
  sprintf(log_fname, "%.3f.txt", t_prev.toSec());
  FILE *time_log = fopen(log_fname, "w");
  if (!time_log)
    throw std::runtime_error("couldn't open frame time log");
  if (AVI_open_output_file(&avi, fname) < 0)
    throw std::runtime_error("couldn't open AVI file");
  AVI_set_video(&avi, WIDTH, HEIGHT, FPS, "RGB");

  uvc_cam::Cam cam(argv[1], uvc_cam::Cam::MODE_RGB);
  int count = 0, keyframe = 1;
  signal(SIGINT, sigint_handler);
  while (n.ok() && !done)
  {
    unsigned char *frame = NULL;
    uint32_t bytes_used;
    int buf_idx = cam.grab(&frame, bytes_used);
    //printf("      %d byte image\n", bytes_used);
    if (frame)
    {
      sensor_msgs::Image image; 
      
      image.header.stamp = ros::Time::now();
//      image.label = "UVC Camera Image";
      image.encoding = sensor_msgs::image_encodings::RGB8;
//      image.depth = "uint8";

      image.height = HEIGHT;
      image.width = WIDTH;

//      image.is_bigendian = ;
      image.step = 3 * WIDTH;
            
//      std_msgs::UInt8MultiArray & multi_arr = image.uint8_data;
//      multi_arr.layout.dim.resize(3);
//      multi_arr.layout.dim[0].label  = "height";
//      multi_arr.layout.dim[0].size   = HEIGHT;
//      multi_arr.layout.dim[0].stride = 3 * HEIGHT * WIDTH;
//      multi_arr.layout.dim[1].label  = "width";
//      multi_arr.layout.dim[1].size   = WIDTH;
//      multi_arr.layout.dim[1].stride = 3 * WIDTH;
//      multi_arr.layout.dim[2].label  = "channel";
//      multi_arr.layout.dim[2].size   = 3;
//      multi_arr.layout.dim[2].stride = 3;
      
//      multi_arr.data.resize(3 * HEIGHT * WIDTH);
//      memcpy(&multi_arr.data[0], frame, WIDTH*HEIGHT*3);
      
//      uint8_t* bgr = &multi_arr.data[0];

      image.set_data_size( image.step * image.height );
      uint8_t* bgr = &(image.data[0]);
      
      for (uint32_t y = 0; y < HEIGHT; y++)
	for (uint32_t x = 0; x < WIDTH; x++)
	  {
	    uint8_t *p = frame + y * WIDTH * 3 + x * 3;
	    uint8_t *q = bgr   + y * WIDTH * 3 + x * 3;
	    q[0] = p[2]; q[1] = p[1]; q[2] = p[0];
	  }
      pub.publish(image);
     
      fprintf(time_log, "%.6f\n", ros::Time::now().toSec());
      //AVI_write_frame(&avi, frame, bytes_used, keyframe);
      AVI_write_frame(&avi, frame, 3*HEIGHT*WIDTH, keyframe);

      //fwrite(frame, bytes_used, 1, f);

      cam.release(buf_idx);
      if (keyframe) keyframe = 0;
      /*
      memcpy(surf->pixels, frame, WIDTH*HEIGHT*3);
      cam.release(buf_idx);
      */
    }
    if (count++ % FPS == 0)
    {
      ros::Time t(ros::Time::now());
      ros::Duration d(t - t_prev);
      printf("%.1f fps\n", (double)FPS / d.toSec());
      t_prev = t;
    }
  }
  fclose(time_log);
  AVI_close(&avi);
  return 0;
}
