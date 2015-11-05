#include <unistd.h>
#include <cstdio>
#include <cassert>
#include "uvc_cam/uvc_cam.h"

int main(int argc, char **argv)
{
  uvc_cam::Cam::enumerate();
  return 0;
}
