#include <unistd.h>
#include <cstdio>
#include <cassert>
#include <ros/time.h>
#include "SDL/SDL.h"

const unsigned WIDTH = 640, HEIGHT = 480;

inline unsigned char saturate(float f)
{
  return (unsigned char)( f >= 255 ? 255 : (f < 0 ? 0 : f));
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    fprintf(stderr, "usage: play FILE\n");
    return 1;
  }

  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "sdl init error: %s\n", SDL_GetError());
    return 1;
  }
  atexit(SDL_Quit);
  SDL_Surface *surf = SDL_SetVideoMode(WIDTH, HEIGHT, 24, SDL_HWSURFACE);
  assert(surf);
  SDL_WM_SetCaption("hello world", "hello world");
  ros::Time t_prev(ros::Time::now());
  int count = 0;
  uint8_t *bgr_frame = new uint8_t[WIDTH*HEIGHT*3];
  uint8_t *yuv_frame = new uint8_t[WIDTH*HEIGHT*3];
  FILE *f = fopen(argv[1],"r");
  if (!f)
  {
    fprintf(stderr, "couldn't open dump file\n");
    return 1;
  }
  for (bool done = false; !done && !feof(f);)
  {
    double t;
    if (1 != fread(&t, sizeof(double), 1, f))
      break;
    printf("frame time: %15f\n", t);
    if (1 != fread(yuv_frame, WIDTH*HEIGHT*2, 1, f))
      break;
    uint8_t *prgb = bgr_frame;
    uint8_t *pyuv = yuv_frame;
    for (uint32_t i = 0; i < WIDTH*HEIGHT*2; i += 4)
    {
      *prgb++ = saturate(pyuv[i]+1.772f  *(pyuv[i+1]-128));
      *prgb++ = saturate(pyuv[i]-0.34414f*(pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
      *prgb++ = saturate(pyuv[i]+1.402f  *(pyuv[i+3]-128));

      *prgb++ = saturate(pyuv[i+2]+1.772f*(pyuv[i+1]-128));
      *prgb++ = saturate(pyuv[i+2]-0.34414f*(pyuv[i+1]-128)-0.71414f*(pyuv[i+3]-128));
      *prgb++ = saturate(pyuv[i+2]+1.402f*(pyuv[i+3]-128));
    }
    memcpy(surf->pixels, bgr_frame, WIDTH*HEIGHT*3);

    if (count++ % 30 == 0)
    {
      ros::Time t(ros::Time::now());
      ros::Duration d(t - t_prev);
      printf("%.1f fps\n", 30.0 / d.toSec());
      t_prev = t;
    }
    //usleep(100000);
    SDL_UpdateRect(surf, 0, 0, WIDTH, HEIGHT);
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      switch (event.type)
      {
        case SDL_KEYDOWN:
          switch(event.key.keysym.sym)
          {
            case SDLK_ESCAPE: done = true; break;
            default: break;
          }
          break;
        case SDL_QUIT:
          done = true;
          break;
      }
    }
  }
  delete[] bgr_frame;
  return 0;
}

