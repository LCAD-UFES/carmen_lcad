/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 */

#ifndef CAPTURE_H
#define CAPTURE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev.h>
#include <linux/videodev2.h>

#include <sys/resource.h>
#include <sys/time.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

typedef enum {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
} io_method;

struct buffer {
        void *                  start;
        size_t                  length;
};

struct frame {
        unsigned char*       data;
        int         width;
        int         height;
        int         pixType;
};


// Note, the canonical VideoFormat has diverged from this copy, so the
// enum values will no longer be in correspondence
enum IrobotVideoFormat
  {
    IROBOT_VIDFMT_GREY    = 0,  // format: [ grey(8) ]
    IROBOT_VIDFMT_RAW     = 1,
    IROBOT_VIDFMT_RGB555  = 2,  // format: [ (1) r(5) g(5) b(5) ]
    IROBOT_VIDFMT_RGB565  = 3,  // format: [ r(5) g(6) b(5) ]
    IROBOT_VIDFMT_RGB24   = 4,  // format: [ r(8) g(8) b(8) ]
    IROBOT_VIDFMT_RGB32   = 5,  // format: [ r(8) g(8) b(8) ]
    IROBOT_VIDFMT_YUYV    = 6,  // format: [ Y0(8) U0(8) Y1(8) V0(8) ]
    IROBOT_VIDFMT_UYVY    = 7,  // format: [ U0(8) Y0(8) V0(8) Y1(8) ]
    IROBOT_VIDFMT_YUV444  = 8,  // format: [ U0(8) Y0(8) V0(8) U1(8) Y1(8) V1(8) ]
    IROBOT_VIDFMT_YUV422  = 9,  // format: same as IROBOT_VIDFMT_UYVY
    IROBOT_VIDFMT_YUV411  = 10, // format: [ U(8) Y0(8) Y1(8) V(8) Y2(8) Y3(8) ]
    IROBOT_VIDFMT_YUV420  = 11, // does this exist?
    IROBOT_VIDFMT_YUV410  = 12, // does this exist?
    IROBOT_VIDFMT_YUV444P = 13,
    IROBOT_VIDFMT_YUV422P = 14,
    IROBOT_VIDFMT_YUV411P = 15,
    IROBOT_VIDFMT_YUV420P = 16,
    IROBOT_VIDFMT_YUV410P = 17,
    // KEEP THIS ITEM LAST:
    IROBOT_VIDFMT_AUTO    = 18  // auto selection of best mode
  };

//! Mapping between our IrobotVideoFormat and V4L2 pixfmt:
  static __u32 v4l2format[IROBOT_VIDFMT_AUTO] = {
    V4L2_PIX_FMT_GREY,
    0xffffffff,  // RAW unsupported by V4L2?
    V4L2_PIX_FMT_RGB555,
    V4L2_PIX_FMT_RGB565,
    V4L2_PIX_FMT_BGR24,
    V4L2_PIX_FMT_BGR32,
    V4L2_PIX_FMT_YUYV,
    V4L2_PIX_FMT_UYVY,
    0xffffffff,  // YUV444 ???
    V4L2_PIX_FMT_UYVY,
    V4L2_PIX_FMT_Y41P,
    V4L2_PIX_FMT_YUV420,
    V4L2_PIX_FMT_YUV410,
    0xffffffff,  // YUV444P?
    V4L2_PIX_FMT_YUV422P,
    V4L2_PIX_FMT_YUV411P,
    V4L2_PIX_FMT_YUV420,
    V4L2_PIX_FMT_YUV410
  };

extern int RGB_Y_tab[256];
extern int B_U_tab[256];
extern int G_U_tab[256];
extern int G_V_tab[256];
extern int R_V_tab[256];
extern const int BITS_OUT;
extern char*           dev_name;
extern io_method        io;
extern int              fd;
extern buffer*         buffers;
extern unsigned int     n_buffers;
extern int width, height;
extern frame*     currentFrame;

void errno_exit(const char *s);
int xioctl(int fd, int request, void *arg);
void colorspace_init();
void yuv422_to_rgb24_c(unsigned char* dst,
                       const int w, const int h,
                       const unsigned char* yuv422ptr,
                       const int byteswap);
unsigned char clamp(int d);
void yv12_to_rgb24_c(unsigned char* dst,
                     int dst_stride,
                     const unsigned char* y_src,
                     const unsigned char* u_src,
                     const unsigned char* v_src,
                     int y_stride,
                     int uv_stride,
                     int width,
                     int height);
frame* process_image(const void *p, const int len);
int read_frame(void);
frame* get_frame(void);
void mainloop(void);
void stop_capturing(void);
void start_capturing(void);
void uninit_device(void);
void init_read(unsigned int buffer_size);
void init_mmap(void);
void init_userp(unsigned int buffer_size);
void init_device(int pix_fmt);
void close_device(void);
void open_device(void);

#endif
