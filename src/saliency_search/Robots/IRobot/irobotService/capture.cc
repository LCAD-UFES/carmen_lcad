/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 */

#include <stdio.h>

#include "capture.h"

int RGB_Y_tab[256];
int B_U_tab[256];
int G_U_tab[256];
int G_V_tab[256];
int R_V_tab[256];
const int BITS_OUT = 16;
char*           dev_name        = "/dev/video0";
io_method       io              = IO_METHOD_MMAP;
//io_method     io              = IO_METHOD_READ;
int              fd              = -1;
buffer*         buffers         = NULL;
unsigned int     n_buffers       = 0;
//int width = 160, height = 120;
int width = 320, height = 240;
frame* currentFrame = NULL;

void errno_exit(const char *s)
{
  printf ("%s error %d, %s\n",
      s, errno, strerror (errno));
  exit (0);
}

int xioctl(int fd, int request, void *arg)
{
  int r;

  do r = ioctl (fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}

void colorspace_init()
{
  const int VIDEOYUV_Y_OFFSET  = 16;
  const int VIDEOYUV_Y_RANGE   = 219; // Y range = [16,235]
  const int VIDEOYUV_UV_OFFSET = 128;
  const int VIDEOYUV_UV_RANGE  = 224; // UV range = [16,240]

  const double JPEGYUV_RGB_Y =  1.0;
  const double JPEGYUV_R_V   =  1.402;
  const double JPEGYUV_G_U   = -0.34414;
  const double JPEGYUV_G_V   = -0.71414;
  const double JPEGYUV_B_U   =  1.772;

  const double VIDEOYUV_RGB_Y = JPEGYUV_RGB_Y * (255.0/VIDEOYUV_Y_RANGE);
  const double VIDEOYUV_R_V   = JPEGYUV_R_V   * (255.0/VIDEOYUV_UV_RANGE);
  const double VIDEOYUV_G_U   = JPEGYUV_G_U   * (255.0/VIDEOYUV_UV_RANGE);
  const double VIDEOYUV_G_V   = JPEGYUV_G_V   * (255.0/VIDEOYUV_UV_RANGE);
  const double VIDEOYUV_B_U   = JPEGYUV_B_U   * (255.0/VIDEOYUV_UV_RANGE);


  // so that we get proper rounding in fixed-point integer arithmetic:
  const int half =
    BITS_OUT > 0
    ? 1<<(BITS_OUT-1)
    : 0;

  const int scale = 1<<BITS_OUT;

  int i;
  for (i = 0; i < 256; i++)
  {
    const int y  = i-VIDEOYUV_Y_OFFSET;
    const int uv = i-VIDEOYUV_UV_OFFSET;

    RGB_Y_tab[i] = half+(int)(0.5 + y  * VIDEOYUV_RGB_Y * scale);
    R_V_tab[i]   =      (int)(0.5 + uv * VIDEOYUV_R_V   * scale);
    // FIXME should probably have -0.5 instead of +0.5 here and
    // flip the sign of G_U_tab and G_V_tab:
    G_U_tab[i]   =      (int)(0.5 - uv * VIDEOYUV_G_U   * scale);
    G_V_tab[i]   =      (int)(0.5 - uv * VIDEOYUV_G_V   * scale);
    B_U_tab[i]   =      (int)(0.5 + uv * VIDEOYUV_B_U   * scale);
  }
}


void yuv422_to_rgb24_c(unsigned char* dst,
                       const int w, const int h,
                       const unsigned char* yuv422ptr,
                       const int byteswap)
{
  int i, j;
  if (byteswap)
    for (j = 0; j < h; ++j)
      for (i = 0; i < w; i += 2)
        {
          // we have 2 luminance pixels per chroma pair

          const unsigned char y1 = yuv422ptr[0];
          const unsigned char u = yuv422ptr[1];
          const unsigned char y2 = yuv422ptr[2];
          const unsigned char v = yuv422ptr[3];

          yuv422ptr += 4;

          const int r_v = R_V_tab[v];
          const int g_uv = - G_U_tab[u] - G_V_tab[v];
          const int b_u = B_U_tab[u];

          // first luminance pixel:
          const int rgb_y1 = RGB_Y_tab[y1];

          *dst++ = (rgb_y1 + r_v) >> BITS_OUT;
          *dst++ = (rgb_y1 + g_uv) >> BITS_OUT;
          *dst++ = (rgb_y1 + b_u) >> BITS_OUT;

          // second luminance pixel:
          const int rgb_y2 = RGB_Y_tab[y2];

          *dst++ = (rgb_y2 + r_v) >> BITS_OUT;
          *dst++ = (rgb_y2 + g_uv) >> BITS_OUT;
          *dst++ = (rgb_y2 + b_u) >> BITS_OUT;
        }

  else // no unsigned charswap
    for ( j = 0; j < h; ++j)
      for ( i = 0; i < w; i += 2)
        {
          // we have 2 luminance pixels per chroma pair

          const unsigned char y1 = yuv422ptr[1];
          const unsigned char u = yuv422ptr[0];
          const unsigned char y2 = yuv422ptr[3];
          const unsigned char v = yuv422ptr[2];

          yuv422ptr += 4;

          const int r_v = R_V_tab[v];
          const int g_uv = - G_U_tab[u] - G_V_tab[v];
          const int b_u = B_U_tab[u];

          // first luminance pixel:
          const int rgb_y1 = RGB_Y_tab[y1];

          *dst++ = (rgb_y1 + r_v) >> BITS_OUT;
          *dst++ = (rgb_y1 + g_uv) >> BITS_OUT;
          *dst++ = (rgb_y1 + b_u) >> BITS_OUT;

          // second luminance pixel:
          const int rgb_y2 = RGB_Y_tab[y2];

          *dst++ = (rgb_y2 + r_v) >> BITS_OUT;
          *dst++ = (rgb_y2 + g_uv) >> BITS_OUT;
          *dst++ = (rgb_y2 + b_u) >> BITS_OUT;
        }
}

unsigned char clamp(int d)
{
  int r = d;
  if (r > 255) r = 255;
  if (r < 0) r = 0;

  return r;
}

void yv12_to_rgb24_c(unsigned char* dst,
                     int dst_stride,
                     const unsigned char* y_src,
                     const unsigned char* u_src,
                     const unsigned char* v_src,
                     int y_stride,
                     int uv_stride,
                     int width,
                     int height)
{

  if (width & 1)
    printf("width must be even\n");

  if (height & 1)
    printf("height must be even\n");

  const int dst_dif = 6 * dst_stride - 3 * width;
  int y_dif = 2 * y_stride - width;

  unsigned char* dst2 = dst + 3 * dst_stride;
  const unsigned char* y_src2 = y_src + y_stride;

  if (height < 0) {                     /* flip image? */
    height = -height;
    y_src += (height - 1) * y_stride;
    y_src2 = y_src - y_stride;
    u_src += (height / 2 - 1) * uv_stride;
    v_src += (height / 2 - 1) * uv_stride;
    y_dif = -width - 2 * y_stride;
    uv_stride = -uv_stride;
  }

  int x,y;
  for (y = height / 2; y; y--) {
    for (x = 0; x < width / 2; x++) {
      const int u = u_src[x];
      const int v = v_src[x];

      const int r_v = R_V_tab[v];
      const int g_uv = - G_U_tab[u] - G_V_tab[v];
      const int b_u = B_U_tab[u];

      {
        const int rgb_y = RGB_Y_tab[*y_src];
        const int r = (rgb_y + r_v) >> BITS_OUT;
        const int g = (rgb_y + g_uv) >> BITS_OUT;
        const int b = (rgb_y + b_u) >> BITS_OUT;
        dst[0] = clamp(r);
        dst[1] = clamp(g);
        dst[2] = clamp(b);
        y_src++;
      }
      {
        const int rgb_y = RGB_Y_tab[*y_src];
        const int r = (rgb_y + r_v) >> BITS_OUT;
        const int g = (rgb_y + g_uv) >> BITS_OUT;
        const int b = (rgb_y + b_u) >> BITS_OUT;
        dst[3] = clamp(r);
        dst[4] = clamp(g);
        dst[5] = clamp(b);
        y_src++;
      }
      {
        const int rgb_y = RGB_Y_tab[*y_src2];
        const int r = (rgb_y + r_v) >> BITS_OUT;
        const int g = (rgb_y + g_uv) >> BITS_OUT;
        const int b = (rgb_y + b_u) >> BITS_OUT;
        dst2[0] = clamp(r);
        dst2[1] = clamp(g);
        dst2[2] = clamp(b);
        y_src2++;
      }
      {
        const int rgb_y = RGB_Y_tab[*y_src2];
        const int r = (rgb_y + r_v) >> BITS_OUT;
        const int g = (rgb_y + g_uv) >> BITS_OUT;
        const int b = (rgb_y + b_u) >> BITS_OUT;
        dst2[3] = clamp(r);
        dst2[4] = clamp(g);
        dst2[5] = clamp(b);
        y_src2++;
      }

      dst += 6;
      dst2 += 6;
    }

    dst += dst_dif;
    dst2 += dst_dif;

    y_src += y_dif;
    y_src2 += y_dif;

    u_src += uv_stride;
    v_src += uv_stride;
  }
}

frame* process_image(const void *p, const int len, const bool color)
{

  const int w2 = (width+1)/2;
  const int h2 = (height+1)/2;

  yv12_to_rgb24_c(currentFrame->data,
                  width, // dst_stride ,
                  (const unsigned char*)p,
                  (const unsigned char*)p + width*height,
                  (const unsigned char*)p + width*height + w2*h2,
                  width, // y_stride ,
                  w2, // uv_stride ,
                  width, // image width ,
                  height); // image height

  if (!color)
  {
    const unsigned char* sPtr = currentFrame->data;
    for(int i=0; i<width*height; i++)
    {
      currentFrame->lumData[i] = (*sPtr + *(sPtr+1) + *(sPtr+2))/3;
      sPtr += 3;
    }
  }

  return currentFrame;
}

int read_frame(void)
{
  struct v4l2_buffer buf;
  unsigned int i;

  switch (io) {
    case IO_METHOD_READ:
      if (-1 == (i = read (fd, buffers[0].start, buffers[0].length))) {
        switch (errno) {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit ("read");
        }
      }

      process_image (buffers[0].start, buffers[0].length);

      break;

    case IO_METHOD_MMAP:
      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      // wait until buffer 'itsCurrentFrame' has been fully captured:
      if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit ("VIDIOC_DQBUF");
        }
      }

      assert (buf.index < n_buffers);

      process_image (buffers[buf.index].start,buffers[buf.index].length );

      // get ready for capture of that frame again (for later):
      if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
        errno_exit ("VIDIOC_QBUF");

      break;

    case IO_METHOD_USERPTR:
      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit ("VIDIOC_DQBUF");
        }
      }

      for (i = 0; i < n_buffers; ++i)
        if (buf.m.userptr == (unsigned long) buffers[i].start
            && buf.length == buffers[i].length)
          break;

      assert (i < n_buffers);

      process_image ((void *) buf.m.userptr, 0);

      if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
        errno_exit ("VIDIOC_QBUF");

      break;
  }

  return 1;
}

frame* get_frame(bool color)
{
  struct v4l2_buffer buf;
  unsigned int i;

  switch (io) {
    case IO_METHOD_READ:
      if (-1 == (i = read (fd, buffers[0].start, buffers[0].length))) {
        switch (errno) {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit ("read");
        }
      }

      return process_image (buffers[0].start, buffers[0].length, color);

      break;

    case IO_METHOD_MMAP:
      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      // wait until buffer 'itsCurrentFrame' has been fully captured:
      if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */
             return 0;

            /* fall through */

          default:
            errno_exit ("VIDIOC_DQBUF error");
        }
      }

      assert (buf.index < n_buffers);


      // get ready for capture of that frame again (for later):
      if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
        errno_exit ("VIDIOC_QBUF");

      return process_image (buffers[buf.index].start,buffers[buf.index].length, color );
      break;

    case IO_METHOD_USERPTR:
      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit ("VIDIOC_DQBUF");
        }
      }

      for (i = 0; i < n_buffers; ++i)
        if (buf.m.userptr == (unsigned long) buffers[i].start
            && buf.length == buffers[i].length)
          break;

      assert (i < n_buffers);

      return process_image ((void *) buf.m.userptr, 0, color);

      if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
        errno_exit ("VIDIOC_QBUF");

      break;
  }

  return NULL;
}

void mainloop(void)
{
  unsigned int count;

  count = 100;

  while (count-- > 0) {
    for (;;) {
      fd_set fds;
      struct timeval tv;
      int r;

      FD_ZERO (&fds);
      FD_SET (fd, &fds);

      /* Timeout. */
      tv.tv_sec = 2;
      tv.tv_usec = 0;

      r = select (fd + 1, &fds, NULL, NULL, &tv);

      if (-1 == r) {
        if (EINTR == errno)
          continue;

        errno_exit ("select");
      }

      if (0 == r) {
        fprintf (stderr, "select timeout\n");
        exit (EXIT_FAILURE);
      }

      if (read_frame ())
        break;

      /* EAGAIN - continue select loop. */
    }
  }
}

void stop_capturing(void)
{
  enum v4l2_buf_type type;

  switch (io) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
        errno_exit ("VIDIOC_STREAMOFF");

      break;
  }
}

void start_capturing(void)
{
  unsigned int i;
  enum v4l2_buf_type type;

  currentFrame = new frame;
  currentFrame->data = new unsigned char[width*height*3];
  currentFrame->lumData = new unsigned char[width*height];
  currentFrame->width = width;
  currentFrame->height = height;
  currentFrame->pixType=0;


  switch (io) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;

        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
          errno_exit ("VIDIOC_QBUF");
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
        errno_exit ("VIDIOC_STREAMON");

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_USERPTR;
        buf.index       = i;
        buf.m.userptr   = (unsigned long) buffers[i].start;
        buf.length      = buffers[i].length;

        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
          errno_exit ("VIDIOC_QBUF");
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
        errno_exit ("VIDIOC_STREAMON");

      break;
  }
}

void uninit_device(void)
{
  unsigned int i;

  switch (io) {
    case IO_METHOD_READ:
      free (buffers[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap (buffers[i].start, buffers[i].length))
          errno_exit ("munmap");
      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i)
        free (buffers[i].start);
      break;
  }

  if (currentFrame != NULL)
  {
    if (currentFrame->data != NULL)
      free(currentFrame->data);
    if (currentFrame->lumData != NULL)
      free(currentFrame->lumData);
    free(currentFrame);
  }

  free (buffers);
}

void init_read(unsigned int buffer_size)
{
  buffers = (struct buffer*)calloc (1, sizeof (*buffers));

  if (!buffers) {
    fprintf (stderr, "Out of memory\n");
    exit (EXIT_FAILURE);
  }

  buffers[0].length = buffer_size;
  buffers[0].start = malloc (buffer_size);

  if (!buffers[0].start) {
    fprintf (stderr, "Out of memory\n");
    exit (EXIT_FAILURE);
  }
}

void init_mmap(void)
{
  struct v4l2_requestbuffers req;

  CLEAR (req);

  req.count               = 1;
  req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory              = V4L2_MEMORY_MMAP;

  if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf (stderr, "%s does not support "
          "memory mapping\n", dev_name);
      exit (EXIT_FAILURE);
    } else {
      errno_exit ("VIDIOC_REQBUFS");
    }
  }

  if (req.count < 2) {
    fprintf (stderr, "Insufficient buffer memory on %s\n",
        dev_name);
    exit (EXIT_FAILURE);
  }

  buffers = (struct buffer*)calloc (req.count, sizeof (*buffers));

  if (!buffers) {
    fprintf (stderr, "Out of memory\n");
    exit (EXIT_FAILURE);
  }

  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = n_buffers;

    if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
      errno_exit ("VIDIOC_QUERYBUF");

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start =
      mmap (NULL /* start anywhere */,
          buf.length,
          PROT_READ | PROT_WRITE /* required */,
          MAP_SHARED /* recommended */,
          fd, buf.m.offset);

    if (MAP_FAILED == buffers[n_buffers].start)
      errno_exit ("mmap");
  }
}

void init_userp(unsigned int buffer_size)
{
  struct v4l2_requestbuffers req;

  CLEAR (req);

  req.count               = 4;
  req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory              = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf (stderr, "%s does not support "
          "user pointer i/o\n", dev_name);
      exit (EXIT_FAILURE);
    } else {
      errno_exit ("VIDIOC_REQBUFS");
    }
  }

  buffers = (struct buffer*)calloc (4, sizeof (*buffers));

  if (!buffers) {
    fprintf (stderr, "Out of memory\n");
    exit (EXIT_FAILURE);
  }

  for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
    buffers[n_buffers].length = buffer_size;
    buffers[n_buffers].start = malloc (buffer_size);

    if (!buffers[n_buffers].start) {
      fprintf (stderr, "Out of memory\n");
      exit (EXIT_FAILURE);
    }
  }
}

void init_device(int pix_fmt)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  colorspace_init();

  printf("Settting to pixformat %i\n", pix_fmt);
  if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf (stderr, "%s is no V4L2 device\n",
          dev_name);
      exit (EXIT_FAILURE);
    } else {
      errno_exit ("VIDIOC_QUERYCAP");
    }
  }

  printf("V4L2 kernel driver: %s\n", cap.driver);
  printf("FrameGrabber board name is: %s\n", cap.card);
  printf("FrameGrabber board bus info: %s\n", cap.bus_info);
  if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
    printf("    > Supports video capture\n");
  else printf("Not a video capture device.\n");
  if (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)
    printf("    > Supports video output\n");
  if (cap.capabilities & V4L2_CAP_VIDEO_OVERLAY)
    printf("    > Supports video overlay\n");
  if (cap.capabilities & V4L2_CAP_VBI_CAPTURE)
    printf("    > Supports raw VBI capture\n");
  if (cap.capabilities & V4L2_CAP_VBI_OUTPUT)
    printf("    > Supports raw VBI output\n");

  // NOTE: older versions of V4L2 don't know about sliced VBI:
#ifdef V4L2_CAP_SLICED_VBI_CAPTURE
  if (cap.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE)
    printf("    > Supports sliced VBI capture\n");
#endif
#ifdef V4L2_CAP_SLICED_VBI_OUTPUT
  if (cap.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT)
    printf("    > Supports sliced VBI_OUTPUT\n");
#endif
  if (cap.capabilities & V4L2_CAP_RDS_CAPTURE)
    printf("    > Supports RDS capture\n");
  if (cap.capabilities & V4L2_CAP_TUNER)
    printf("    > Has an RF tuner and/or modulator\n");
  if (cap.capabilities & V4L2_CAP_AUDIO)
    printf("    > Supports audio input and/or output\n");
  if (cap.capabilities & V4L2_CAP_READWRITE)
     printf("    > Supports read/write I/O method\n");
  if (cap.capabilities & V4L2_CAP_ASYNCIO)
    printf("    > Supports asynchronous I/O method\n");
  if (cap.capabilities & V4L2_CAP_STREAMING)
     printf("    > Supports streaming I/O (MMAP) method\n");

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // List available controls and their current settings:
  struct v4l2_queryctrl ctrl;
  int cid;
  for (cid = V4L2_CID_BASE; cid < V4L2_CID_LASTP1; cid ++)
    {
      memset(&ctrl, 0, sizeof(ctrl));
      ctrl.id = cid;

      // is that control supported?
      if (xioctl(fd, VIDIOC_QUERYCTRL, &ctrl) == 0 &&
          ctrl.type != 0)
        {
          // first of all, if we have a ModelParam for it, let's
          // attempt to set the current value:
          struct v4l2_control control; control.id = cid;
          switch(cid)
            {
            case V4L2_CID_BRIGHTNESS:
              control.value = 64;
              if (xioctl(fd, VIDIOC_S_CTRL, &control) == -1)
                printf("Failed to set brightness to %d (VIDIOC_S_CTRL)\n",
                        control.value);
              break;
            case V4L2_CID_CONTRAST:
              control.value = 0;
              if (xioctl(fd, VIDIOC_S_CTRL, &control) == -1)
                printf("Failed to set contrast to %d (VIDIOC_S_CTRL)\n",
                        control.value);
              break;
            case V4L2_CID_WHITENESS: // NOTE this is = V4L2_CID_GAMMA
              // NOTE: bttv driver supports saturation but not whiteness
              // NOTE: macbook supports gamma
              /*control.value = 50;
              if (xioctl(fd, VIDIOC_S_CTRL, &control) == -1)
                printf("Failed to set whiteness to %d (VIDIOC_S_CTRL)\n",
                        control.value);*/
              break;
            case V4L2_CID_SATURATION:
              control.value = 0;
              if (xioctl(fd, VIDIOC_S_CTRL, &control) == -1)
                printf("Failed to set saturation to %d (VIDIOC_S_CTRL)\n",
                        control.value);
              break;
            case V4L2_CID_HUE:
              control.value = 32768;
              if (xioctl(fd, VIDIOC_S_CTRL, &control) == -1)
                printf("Failed to set hue to %d (VIDIOC_S_CTRL)\n",
                        control.value);
              break;

            default: break; // Ignore other controls
            }

          // ok, now get the current value, silently ignoring errors:
          int val = -1;
          if (xioctl(fd, VIDIOC_G_CTRL, &control) == 0)
            val = control.value;

          switch(ctrl.type)
            {
            case V4L2_CTRL_TYPE_INTEGER:
              printf("CONTROL: %s=%d (Def=%d, Rng=[%d .. %d])%s\n",
                      ctrl.name, val, ctrl.default_value,
                      ctrl.minimum, ctrl.maximum,
                      ctrl.flags & V4L2_CTRL_FLAG_DISABLED ?
                      " (DISABLED)" : "");
              break;

            case V4L2_CTRL_TYPE_BOOLEAN:
              printf("CONTROL: %s=%s (Def=%s)%s\n",
                      ctrl.name, val ? "true" : "false",
                      ctrl.default_value ? "true" : "false",
                      ctrl.flags & V4L2_CTRL_FLAG_DISABLED ?
                      " (DISABLED)" : "");
              break;

            case V4L2_CTRL_TYPE_MENU:
              printf("CONTROL: %s=[menu type not yet implemented]%s\n",
                      ctrl.name,
                      ctrl.flags & V4L2_CTRL_FLAG_DISABLED ?
                      " (DISABLED)" : "");
              break;
            case V4L2_CTRL_TYPE_BUTTON:
              printf("CONTROL: %s=[button]%s\n",
                      ctrl.name,
                      ctrl.flags & V4L2_CTRL_FLAG_DISABLED ?
                      " (DISABLED)" : "");
              break;

            default: break;
            }
        }
    }

  // list available channels:
  int i;
  for (i = 0; ; i++)
    {
      struct v4l2_input inp;
      memset(&inp, 0, sizeof(inp));
      inp.index = i;
      if (xioctl(fd, VIDIOC_ENUMINPUT, &inp) != 0) break;
      printf("Video input %d is '%s'\n", i, inp.name);
    }



  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf (stderr, "%s is no video capture device\n",
        dev_name);
    exit (EXIT_FAILURE);
  }

  switch (io) {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        fprintf (stderr, "%s does not support read i/o\n",
            dev_name);
        exit (EXIT_FAILURE);
      }

      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        fprintf (stderr, "%s does not support streaming i/o\n",
            dev_name);
        exit (EXIT_FAILURE);
      }

      break;
  }


  /* Select video input, video standard and tune here. */
  int channel = 0;
  if (xioctl(fd, VIDIOC_S_INPUT, &channel) < 0)
    printf("Cannot select video input %d\n", channel);
  else
    printf("Selected video input %d\n", channel);



  // Reset cropping parameters and get resolution capabilities. NOTE:
  // just open()'ing the device does not reset it, according to the
  // unix toolchain philosophy. Hence, although here we do not provide
  // support for cropping, we still need to ensure that it is properly
  // reset:
  CLEAR (cropcap);
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
    printf("Video capture bounds: (%d, %d) -> (%d, %d)\n",
        cropcap.bounds.left, cropcap.bounds.top,
        cropcap.bounds.left + cropcap.bounds.width - 1,
        cropcap.bounds.top + cropcap.bounds.height - 1);
    printf("Video default capture rectangle: (%d, %d) -> (%d, %d)\n",
        cropcap.defrect.left, cropcap.defrect.top,
        cropcap.defrect.left + cropcap.defrect.width - 1,
        cropcap.defrect.top + cropcap.defrect.height - 1);

  } else {
    printf("Failed to get cropping capabilities -- IGNORED.\n");
  }

  memset (&crop, 0, sizeof(crop));
  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  crop.c = cropcap.defrect;
  if (xioctl(fd, VIDIOC_S_CROP, &crop) == -1)
    printf("Failed to reset cropping settings -- IGNORED.\n");



  // list available video formats and see which ones would work with
  // the requested resolution:
  CLEAR (fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  int fmt_to_use = -1; // in case user selected "auto" format

  for (i = 0; ; i ++)
  {
    struct v4l2_fmtdesc fdesc;
    memset(&fdesc, 0, sizeof(fdesc));
    fdesc.index = i;
    fdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_ENUM_FMT, &fdesc) != 0) break;
    unsigned int f;
    for (f = 0; f < (unsigned int)IROBOT_VIDFMT_AUTO; f ++)
      if (v4l2format[f] == fdesc.pixelformat) break;

    // try to see whether that would work with our given image dims:
    fmt.fmt.pix.pixelformat = fdesc.pixelformat;
    int worked = 0;
    if (xioctl(fd, VIDIOC_TRY_FMT, &fmt) == 0)
    {
      worked = 1;
      if (fmt_to_use == -1) fmt_to_use = fdesc.pixelformat;
    }
    printf("Video Format: Use '%i index %i' for '%s (Fourcc: %c%c%c%c)'%s%s\n",
        f == IROBOT_VIDFMT_AUTO ?  -1 : f,
        fdesc.index,
        fdesc.description,
        ((char*)(&fdesc.pixelformat))[0],
        ((char*)(&fdesc.pixelformat))[1],
        ((char*)(&fdesc.pixelformat))[2],
        ((char*)(&fdesc.pixelformat))[3],
        fdesc.flags & V4L2_FMT_FLAG_COMPRESSED ? " (COMPRESSED)" : "",
        worked ? " [OK]" : " [FAILED TO SET]");
  }
  printf("Using format %i\n", fmt_to_use);

  // Get the frame time, according to current video standard:
  struct v4l2_streamparm sparm;
  memset(&sparm, 0, sizeof(sparm));
  sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_G_PARM, &sparm) == -1)
    {
      printf("Cannot get video standard params - ASSUMING NTSC\n");
    }
  else
    printf("Capture at %.2f fps\n",
        (float)sparm.parm.capture.timeperframe.numerator/
        (float)sparm.parm.capture.timeperframe.denominator);

  //fmt.fmt.pix.pixelformat = fmt_to_use;
  fmt.fmt.pix.pixelformat = v4l2format[11];

  if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1)
    printf("Cannot set requested video mode/resolution %i [%dx%d], probably unsupported by hardware.",
              fmt_to_use, width, height);

  if (xioctl(fd, VIDIOC_G_FMT, &fmt) == 0)
    printf("Video mode/resolution set to %i [%dx%d]\n",
        fmt_to_use,
        fmt.fmt.pix.width,  fmt.fmt.pix.height);

  if ((int)fmt.fmt.pix.width != width ||
      (int)fmt.fmt.pix.height != height)
    printf("Hardware suggests changing input dims to %dx%d instead of "
        "requested %dx%d\n",
        fmt.fmt.pix.width, fmt.fmt.pix.height,
        width, height);

  width = fmt.fmt.pix.width;
  height = fmt.fmt.pix.height;


  /* Note VIDIOC_S_FMT may change width and height. */

  switch (io) {
    case IO_METHOD_READ:
      init_read (fmt.fmt.pix.sizeimage);
      break;

    case IO_METHOD_MMAP:
      printf("Using MMap\n");
      init_mmap ();
      break;

    case IO_METHOD_USERPTR:
      init_userp (fmt.fmt.pix.sizeimage);
      break;
  }
}

void close_device(void)
{
  if (-1 == close (fd))
    errno_exit ("close");

  fd = -1;
}

void open_device(void)
{
  struct stat st;

  if (-1 == stat (dev_name, &st)) {
    fprintf (stderr, "Cannot identify '%s': %d, %s\n",
        dev_name, errno, strerror (errno));
    exit (EXIT_FAILURE);
  }

  if (!S_ISCHR (st.st_mode)) {
    fprintf (stderr, "%s is no device\n", dev_name);
    exit (EXIT_FAILURE);
  }

  fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == fd) {
    fprintf (stderr, "Cannot open '%s': %d, %s\n",
        dev_name, errno, strerror (errno));
    exit (EXIT_FAILURE);
  }
}
