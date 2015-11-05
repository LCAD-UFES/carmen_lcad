#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <cstdio>
#include <csignal>
#include <ros/time.h>
#include "uvc_cam/uvc_cam.h"
#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <ogg/ogg.h>
#include <vector>
#include <string>

const unsigned WIDTH = 640, HEIGHT = 480, FPS = 30;
static bool done = false;

void sigint_handler(int)
{
  done = true;
}

void logfilename(char *buf, uint32_t buf_len)
{
  time_t t;
  time(&t);
  struct tm *lt = localtime(&t);
  strftime(buf, buf_len, "videos/%Y%m%d-%H%M%S.ogg", lt);
}

void start_encoder(th_enc_ctx **context, ogg_stream_state *oggss, FILE **f)
{
  th_info info;
  th_info_init(&info);
  info.frame_width  = info.pic_width  = WIDTH;
  info.frame_height = info.pic_height = HEIGHT;
  info.pic_x = info.pic_y = 0;
  info.colorspace = TH_CS_UNSPECIFIED;
  info.pixel_fmt = TH_PF_420;
  //info.target_bitrate = 800000;
  info.target_bitrate = 0;
  info.quality = 50;
  info.keyframe_granule_shift = 6; // max keyframe interval = 1 << 6
  info.fps_numerator = FPS;
  info.fps_denominator = 1;
  info.aspect_numerator = 1;
  info.aspect_denominator = 1;
  
  *context = th_encode_alloc(&info);
  if (!(*context))
  {
    fprintf(stderr, "couldn't initialize the encoder\n");
    exit(1);
  }
  else
    printf("encoder ok\n");

  th_comment comment;
  th_comment_init(&comment);
  th_comment_add(&comment, (char *)"created by the uvc_cam ROS package");
  comment.vendor = (char *)"uvc_cam ROS package";

  ogg_stream_init(oggss, rand());

  char fn[100];
  logfilename(fn, sizeof(fn));
  *f = fopen(fn, "w");
  if (!(*f))
  {
    fprintf(stderr, "couldn't open output file\n");
    exit(1);
  }

  ogg_packet oggpacket;
  ogg_page oggpage;
  while (th_encode_flushheader(*context, &comment, &oggpacket) > 0)
  {
    ogg_stream_packetin(oggss, &oggpacket);
    while (ogg_stream_pageout(oggss, &oggpage))
    {
      fwrite(oggpage.header, oggpage.header_len, 1, *f);
      fwrite(oggpage.body  , oggpage.body_len  , 1, *f);
    }
  }

  // finish the headers, to begin a new page
  while (ogg_stream_flush(oggss, &oggpage) > 0)
  {
    fwrite(oggpage.header, oggpage.header_len, 1, *f);
    fwrite(oggpage.body  , oggpage.body_len  , 1, *f);
  }
}

void shutdown_encoder(th_enc_ctx **context, ogg_stream_state *oggss, FILE **f)
{
  printf("flushing previous log...\n");
  ogg_page oggpage;
  while (ogg_stream_flush(oggss, &oggpage) > 0)
  {
    fwrite(oggpage.header, oggpage.header_len, 1, *f);
    fwrite(oggpage.body  , oggpage.body_len  , 1, *f);
  }
  fclose(*f);
  th_encode_free(*context);
  ogg_stream_clear(oggss);
  *f = NULL;
  *context = NULL;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    fprintf(stderr, "usage: record_theora DEVICE\n");
    return 1;
  }
  signal(SIGINT, sigint_handler);
  srand(time(NULL));

  uvc_cam::Cam cam(argv[1], uvc_cam::Cam::MODE_YUYV, WIDTH, HEIGHT, FPS);
  cam.set_control(0x9a0901, 3); // aperture priority exposure mode
  cam.set_control(0x9a0903, 1); // auto exposure
  ros::Time t_prev(ros::Time::now());
  int count = 0;
  ogg_stream_state oggss;
  ogg_page oggpage;
  ogg_packet oggpacket;

  th_enc_ctx *context;
  FILE *ogg_file;
  start_encoder(&context, &oggss, &ogg_file);

  th_img_plane planes[3];

  planes[0].width = WIDTH;
  planes[0].stride = WIDTH;
  planes[0].height = HEIGHT;

  planes[1].width  = planes[2].width  = WIDTH/2;
  planes[1].stride = planes[2].stride = WIDTH/2;
  planes[1].height = planes[2].height = HEIGHT/2;

  planes[0].data = new uint8_t[WIDTH*HEIGHT];
  planes[1].data = new uint8_t[WIDTH*HEIGHT/4];
  planes[2].data = new uint8_t[WIDTH*HEIGHT/4];

  ros::Time log_start(ros::Time::now());
  while (!done)
  {
    // see if it's time to start a new logfile
    ros::Time t(ros::Time::now());
    ros::Duration log_duration(t - log_start);
    if (log_duration.toSec() > 60*60) // new log every hour
    {
      shutdown_encoder(&context, &oggss, &ogg_file);
      log_start = t;
      start_encoder(&context, &oggss, &ogg_file);
    }

    uint8_t *frame = NULL;
    uint32_t bytes_used;
    int buf_idx = cam.grab(&frame, bytes_used);
    if (count++ % 30 == 0)
    {
      ros::Duration d(t - t_prev);
      printf("%.1f fps\n", 30.0 / d.toSec());
      t_prev = t;
    }
    if (frame)
    {
      // separate out the luma/chroma planes, as required by theora
      uint8_t *pframe = frame;
      uint8_t *py = planes[0].data;
      uint8_t *pcr = planes[1].data;
      uint8_t *pcb = planes[2].data;
      for (uint32_t row = 0; row < HEIGHT; row+=2)
      {
        // skip every other chrominance row, to go from V4L2 YUYV to YUV420
        for (uint32_t col = 0; col < WIDTH; col+=2)
        {
          *py++  = *pframe++;
          *pcr++ = *pframe++;
          *py++  = *pframe++;
          *pcb++ = *pframe++;
        }
        for (uint32_t col = 0; col < WIDTH; col+=2)
        {
          *py++  = *pframe++;
          pframe++; // skip cr
          *py++  = *pframe++;
          pframe++; // skip cb
        }
      }
      cam.release(buf_idx); // give the image back to the kernel
      int ret = th_encode_ycbcr_in(context, planes);
      if (ret == TH_EFAULT)
        printf("EFAULT while encoding\n");
      else if (ret == TH_EINVAL)
        printf("EINVAL while encoding\n");

      while (th_encode_packetout(context, 0, &oggpacket) > 0)
      {
        ogg_stream_packetin(&oggss, &oggpacket);
        while (ogg_stream_pageout(&oggss, &oggpage))
        {
          fwrite(oggpage.header, oggpage.header_len, 1, ogg_file);
          fwrite(oggpage.body  , oggpage.body_len  , 1, ogg_file);
        }
      }
    }
  }
  shutdown_encoder(&context, &oggss, &ogg_file);

  return 0;
}

