#include <unistd.h>
#include <cstdio>
#include <cassert>
#include <ros/time.h>
#include "uvc_cam/uvc_cam.h"
#include <signal.h>
#include <stdexcept>

const unsigned WIDTH = 640, HEIGHT = 480, FPS = 30;
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
  ros::Time t_prev(ros::Time::now());
  char fname[200], log_fname[200];
  sprintf(fname, "%.3f.raw", t_prev.toSec());
  FILE *f = fopen(fname, "w");
  uvc_cam::Cam cam(argv[1], uvc_cam::Cam::MODE_YUYV, WIDTH, HEIGHT, FPS);
  int count = 0, keyframe = 1;
  signal(SIGINT, sigint_handler);
  while (!done)
  {
    unsigned char *frame = NULL;
    uint32_t bytes_used;
    int buf_idx = cam.grab(&frame, bytes_used);
    if (frame)
    {
      double d = ros::Time::now().toSec();
      fwrite(&d, sizeof(double), 1, f);
      fwrite(frame, WIDTH*HEIGHT*2, 1, f);
      cam.release(buf_idx);
    }
    if (count++ % FPS == 0)
    {
      ros::Time t(ros::Time::now());
      ros::Duration d(t - t_prev);
      printf("%10d %.1f fps\n", count, (double)FPS / d.toSec());
      t_prev = t;
    }
  }
  fclose(f);
  printf("goodbye\n");
  return 0;
}

