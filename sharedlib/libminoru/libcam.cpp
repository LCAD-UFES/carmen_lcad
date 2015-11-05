/*
 Copyright (C) 2009 Giacomo Spigler

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev2.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

#define BUFFER_SIZE 4

#include "libcam.h"

#include <opencv/cv.h>

static void errno_exit (const char *           s)
{
	fprintf (stderr, "%s error %d, %s\n",
			s, errno, strerror (errno));

	exit (EXIT_FAILURE);
}

static int xioctl(int fd, int request, void *arg)
{
	int r,itt=0;

	do {
		r = ioctl (fd, request, arg);
		itt++;
	}
	while ((-1 == r) && (EINTR == errno) && (itt<100));

	return r;
}

Camera::Camera(const char *n, int w, int h, int f) {
	name=n;
	width=w;
	height=h;
	fps=f;

	w2=w/2;


	io=IO_METHOD_MMAP;

	data=(unsigned char *)malloc(w*h*BUFFER_SIZE);

	this->Open();
	this->Init();
	this->Start();
	initialised = true;
}

Camera::~Camera() {
	this->StopCam();
}

void Camera::StopCam()
{
	if (initialised) {
		this->Stop();
		this->UnInit();
		this->Close();

		free(data);
		initialised = false;
	}
}

void Camera::Open() {
	struct stat st;
	if(-1==stat(name, &st)) {
		fprintf(stderr, "Cannot identify '%s' : %d, %s\n", name, errno, strerror(errno));
		exit(1);
	}

	if(!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", name);
		exit(1);
	}

	fd=open(name, O_RDWR | O_NONBLOCK, 0);

	if(-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n", name, errno, strerror(errno));
		exit(1);
	}

}

void Camera::Close() {
	if(-1==close(fd)) {
		errno_exit("close");
	}
	fd=-1;

}

void Camera::Init() {
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	unsigned int min;

	CLEAR (cap);
	if(-1 == xioctl (fd, (int)VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device\n",name);
			exit(1);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}

	if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device\n", name);
		exit(1);
	}

	switch(io) {
	case IO_METHOD_READ:
		if(!(cap.capabilities & V4L2_CAP_READWRITE)) {
			fprintf(stderr, "%s does not support read i/o\n", name);
			exit (1);
		}

		break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		if(!(cap.capabilities & V4L2_CAP_STREAMING)) {
			fprintf (stderr, "%s does not support streaming i/o\n", name);
			exit(1);
		}

		break;
	}


	CLEAR (cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if(0 == xioctl (fd, (int)VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if(-1 == xioctl (fd, (int)VIDIOC_S_CROP, &crop)) {
			switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	} else {
		/* Errors ignored. */
	}

	CLEAR (fmt);

	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = width;
	fmt.fmt.pix.height      = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;


	if(-1 == xioctl (fd, (int)VIDIOC_S_FMT, &fmt))
		errno_exit ("VIDIOC_S_FMT");


  /*
    struct v4l2_standard s;
    s.name[0]='A';
    s.frameperiod.numerator=1;
    s.frameperiod.denominator=fps;

    if(-1==xioctl(fd, VIDIOC_S_STD, &s))
    errno_exit("VIDIOC_S_STD");
  */

	struct v4l2_streamparm p;
	p.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
	//p.parm.capture.capability=V4L2_CAP_TIMEPERFRAME;
	//p.parm.capture.capturemode=V4L2_MODE_HIGHQUALITY;
	p.parm.capture.timeperframe.numerator=1;
	p.parm.capture.timeperframe.denominator=fps;
	p.parm.output.timeperframe.numerator=1;
	p.parm.output.timeperframe.denominator=fps;
	//p.parm.output.outputmode=V4L2_MODE_HIGHQUALITY;
	//p.parm.capture.extendedmode=0;
	//p.parm.capture.readbuffers=n_buffers;


	if(-1==xioctl(fd, (int)VIDIOC_S_PARM, &p))
		errno_exit("VIDIOC_S_PARM");

	//default values, mins and maxes
	struct v4l2_queryctrl queryctrl;

	memset(&queryctrl, 0, sizeof(queryctrl));
	queryctrl.id = V4L2_CID_BRIGHTNESS;
	if(-1 == xioctl (fd, (int)VIDIOC_QUERYCTRL, &queryctrl)) {
		if(errno != EINVAL) {
			//perror ("VIDIOC_QUERYCTRL");
			//exit(EXIT_FAILURE);
			printf("brightness error\n");
		} else {
			printf("brightness is not supported\n");
		}
	} else if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("brightness is not supported\n");
	}
	mb=queryctrl.minimum;
	Mb=queryctrl.maximum;
	db=queryctrl.default_value;


	memset(&queryctrl, 0, sizeof(queryctrl));
	queryctrl.id = V4L2_CID_CONTRAST;
	if(-1 == xioctl (fd, (int)VIDIOC_QUERYCTRL, &queryctrl)) {
		if(errno != EINVAL) {
			//perror ("VIDIOC_QUERYCTRL");
			//exit(EXIT_FAILURE);
			printf("contrast error\n");
		} else {
			printf("contrast is not supported\n");
		}
	} else if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("contrast is not supported\n");
	}
	mc=queryctrl.minimum;
	Mc=queryctrl.maximum;
	dc=queryctrl.default_value;


	memset(&queryctrl, 0, sizeof(queryctrl));
	queryctrl.id = V4L2_CID_SATURATION;
	if(-1 == xioctl (fd, (int)VIDIOC_QUERYCTRL, &queryctrl)) {
		if(errno != EINVAL) {
			//perror ("VIDIOC_QUERYCTRL");
			//exit(EXIT_FAILURE);
			printf("saturation error\n");
		} else {
			printf("saturation is not supported\n");
		}
	} else if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("saturation is not supported\n");
	}
	ms=queryctrl.minimum;
	Ms=queryctrl.maximum;
	ds=queryctrl.default_value;


	memset(&queryctrl, 0, sizeof(queryctrl));
	queryctrl.id = V4L2_CID_HUE;
	if(-1 == xioctl (fd, (int)VIDIOC_QUERYCTRL, &queryctrl)) {
		if(errno != EINVAL) {
			//perror ("VIDIOC_QUERYCTRL");
			//exit(EXIT_FAILURE);
			printf("hue error\n");
		} else {
			printf("hue is not supported\n");
		}
	} else if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("hue is not supported\n");
	}
	mh=queryctrl.minimum;
	Mh=queryctrl.maximum;
	dh=queryctrl.default_value;


	memset(&queryctrl, 0, sizeof(queryctrl));
	queryctrl.id = V4L2_CID_HUE_AUTO;
	if(-1 == xioctl (fd, (int)VIDIOC_QUERYCTRL, &queryctrl)) {
		if(errno != EINVAL) {
			//perror ("VIDIOC_QUERYCTRL");
			//exit(EXIT_FAILURE);
			printf("hueauto error\n");
		} else {
			printf("hueauto is not supported\n");
		}
	} else if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("hueauto is not supported\n");
	}
	ha=queryctrl.default_value;


	memset(&queryctrl, 0, sizeof(queryctrl));
	queryctrl.id = V4L2_CID_SHARPNESS;
	if(-1 == xioctl (fd, (int)VIDIOC_QUERYCTRL, &queryctrl)) {
		if(errno != EINVAL) {
			//perror ("VIDIOC_QUERYCTRL");
			//exit(EXIT_FAILURE);
			printf("sharpness error\n");
		} else {
			printf("sharpness is not supported\n");
		}
	} else if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("sharpness is not supported\n");
	}
	msh=queryctrl.minimum;
	Msh=queryctrl.maximum;
	dsh=queryctrl.default_value;

	/* Note VIDIOC_S_FMT may change width and height. */
	min = fmt.fmt.pix.width * 2;
	if(fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if(fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;

	switch(io) {
	case IO_METHOD_READ:
		break;

	case IO_METHOD_MMAP:
		init_mmap();
		break;

	case IO_METHOD_USERPTR:
		break;
	}

}

void Camera::init_mmap() {
	struct v4l2_requestbuffers req;

	CLEAR (req);

	req.count               = BUFFER_SIZE;
	req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory              = V4L2_MEMORY_MMAP;

	if(-1 == xioctl (fd, (int)VIDIOC_REQBUFS, &req)) {
		if(EINVAL == errno) {
			fprintf (stderr, "%s does not support memory mapping\n", name);
			exit (1);
		} else {
			errno_exit ("VIDIOC_REQBUFS");
		}
	}

	if(req.count < BUFFER_SIZE/2) {
		fprintf (stderr, "Insufficient buffer memory on %s\n", name);
		exit(1);
	}

	buffers = (buffer *)calloc(req.count, sizeof (*buffers));

	if(!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(1);
	}

	for(n_buffers = 0; n_buffers < (int)req.count; ++n_buffers) {
	    struct v4l2_buffer buf;

		CLEAR (buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = n_buffers;

		if(-1 == xioctl (fd, (int)VIDIOC_QUERYBUF, &buf))
			errno_exit ("VIDIOC_QUERYBUF");

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = mmap (NULL /* start anywhere */,
				buf.length,
				PROT_READ | PROT_WRITE /* required */,
				MAP_SHARED /* recommended */,
				fd, buf.m.offset);

		if(MAP_FAILED == buffers[n_buffers].start)
			errno_exit ("mmap");
	}

}

void Camera::UnInit() {
	unsigned int i;

	switch(io) {
	case IO_METHOD_READ:
		free (buffers[0].start);
		break;

	case IO_METHOD_MMAP:
		for(i = 0; i < (unsigned int)n_buffers; ++i)
			if(-1 == munmap (buffers[i].start, buffers[i].length))
				errno_exit ("munmap");
		break;

	case IO_METHOD_USERPTR:
		for (i = 0; i < (unsigned int)n_buffers; ++i)
			free (buffers[i].start);
		break;
	}

	free (buffers);
}

void Camera::Start() {
	unsigned int i;
	enum v4l2_buf_type type;

	switch(io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	case IO_METHOD_MMAP:
		for(i = 0; i < (unsigned int)n_buffers; ++i) {
		    struct v4l2_buffer buf;

			CLEAR (buf);

			buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory      = V4L2_MEMORY_MMAP;
			buf.index       = i;

			if(-1 == xioctl (fd, (int)VIDIOC_QBUF, &buf))
				errno_exit ("VIDIOC_QBUF");
		}

		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if(-1 == xioctl (fd, (int)VIDIOC_STREAMON, &type))
			errno_exit ("VIDIOC_STREAMON");

		break;

	case IO_METHOD_USERPTR:
		for(i = 0; i < (unsigned int)n_buffers; ++i) {
		    struct v4l2_buffer buf;

			CLEAR (buf);

			buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory      = V4L2_MEMORY_USERPTR;
			buf.index       = i;
			buf.m.userptr	= (unsigned long) buffers[i].start;
			buf.length      = buffers[i].length;

			if(-1 == xioctl (fd, (int)VIDIOC_QBUF, &buf))
				errno_exit ("VIDIOC_QBUF");
		}

		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if(-1 == xioctl (fd, (int)VIDIOC_STREAMON, &type))
			errno_exit ("VIDIOC_STREAMON");

		break;
	}

}

void Camera::Stop() {
	enum v4l2_buf_type type;

	switch(io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if(-1 == xioctl (fd, (int)VIDIOC_STREAMOFF, &type))
			errno_exit ("VIDIOC_STREAMOFF");

		break;
	}

}

unsigned char *Camera::QueryFrame()
{
	if (GrabFrame())
		return RetrieveFrame();
	return 0;
}

int Camera::ReadFrame()
{
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl (fd, (int)VIDIOC_DQBUF, &buf))
    	return 0;

    assert(buf.index < (unsigned int)n_buffers);

    bufferIndex = buf.index;

    if (-1 == xioctl(fd, (int)VIDIOC_QBUF, &buf))
    	return 0;

    return 1;
}

// Grab Frame should not wait for the end of the first frame, and should return quickly
int Camera::GrabFrame()
{
    unsigned int count = 1;

    while (count-- > 0)
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO (&fds);
            FD_SET (fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select (fd+1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
            }

            if (0 == r)
            {
                fprintf (stderr, "select timeout\n");

                /* end the infinite loop */
                break;
            }

            if (ReadFrame())
                break;
        }
    }

	return 1;
}

/*
 * Retrieve Frame should in turn wait for the end of frame capture, and should not
   trigger the capture of the next frame (the user chooses when to do it using GrabFrame)
 */
unsigned char *Camera::RetrieveFrame()
{
	if (buffers[bufferIndex].start)
		memcpy(data, (unsigned char *)buffers[bufferIndex].start, buffers[bufferIndex].length);
	return data;
}

void Camera::toIplImage(IplImage *l) {
	unsigned char *l_=(unsigned char *)l->imageData;


	for(int x=0; x<w2; x++) {
		for(int y=0; y<height; y++) {
			int y0, y1, u, v; //y0 u y1 v

			int i=(y*w2+x)*4;
			y0=data[i];
			u=data[i+1];
			y1=data[i+2];
			v=data[i+3];

			int r, g, b;
			r = y0 + (1.370705 * (v-128));
			g = y0 - (0.698001 * (v-128)) - (0.337633 * (u-128));
			b = y0 + (1.732446 * (u-128));

			if(r > 255) r = 255;
			if(g > 255) g = 255;
			if(b > 255) b = 255;
			if(r < 0) r = 0;
			if(g < 0) g = 0;
			if(b < 0) b = 0;

			i=(y*l->width+2*x)*3;
			l_[i] = (unsigned char)(b); //B
			l_[i+1] = (unsigned char)(g); //G
			l_[i+2] = (unsigned char)(r); //R


			r = y1 + (1.370705 * (v-128));
			g = y1 - (0.698001 * (v-128)) - (0.337633 * (u-128));
			b = y1 + (1.732446 * (u-128));

			if(r > 255) r = 255;
			if(g > 255) g = 255;
			if(b > 255) b = 255;
			if(r < 0) r = 0;
			if(g < 0) g = 0;
			if(b < 0) b = 0;

			l_[i+3] = (unsigned char)(b); //B
			l_[i+4] = (unsigned char)(g); //G
			l_[i+5] = (unsigned char)(r); //R

		}
	}

}

int Camera::minBrightness() {
	return mb;
}


int Camera::maxBrightness() {
	return Mb;
}


int Camera::defaultBrightness() {
	return db;
}


int Camera::minContrast() {
	return mc;
}


int Camera::maxContrast() {
	return Mc;
}


int Camera::defaultContrast() {
	return dc;
}


int Camera::minSaturation() {
	return ms;
}


int Camera::maxSaturation() {
	return Ms;
}


int Camera::defaultSaturation() {
	return ds;
}


int Camera::minHue() {
	return mh;
}


int Camera::maxHue() {
	return Mh;
}


int Camera::defaultHue() {
	return dh;
}


bool Camera::isHueAuto() {
	return ha;
}


int Camera::minSharpness() {
	return msh;
}


int Camera::maxSharpness() {
	return Msh;
}


int Camera::defaultSharpness() {
	return dsh;
}

int Camera::setBrightness(int v) {
	if(v<mb || v>Mb) return -1;

	struct v4l2_control control;
	control.id = V4L2_CID_BRIGHTNESS;
	control.value = v;

	if(-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
		perror("error setting brightness");
		return -1;
	}

	return 1;
}

int Camera::setContrast(int v) {
	if(v<mc || v>Mc) return -1;

	struct v4l2_control control;
	control.id = V4L2_CID_CONTRAST;
	control.value = v;

	if(-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
		perror("error setting contrast");
		return -1;
	}

	return 1;
}

int Camera::setSaturation(int v) {
	if(v<ms || v>Ms) return -1;

	struct v4l2_control control;
	control.id = V4L2_CID_SATURATION;
	control.value = v;

	if(-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
		perror("error setting saturation");
		return -1;
	}

	return 1;
}

int Camera::setHue(int v) {
	if(v<mh || v>Mh) return -1;

	struct v4l2_control control;
	control.id = V4L2_CID_HUE;
	control.value = v;

	if(-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
		perror("error setting hue");
		return -1;
	}

	return 1;
}

int Camera::setHueAuto(bool v) {
	if(v<mh || v>Mh) return -1;

	struct v4l2_control control;
	control.id = V4L2_CID_HUE_AUTO;
	control.value = v;

	if(-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
		perror("error setting hue auto");
		return -1;
	}

	return 1;
}

int Camera::setSharpness(int v) {
	if(v<mh || v>Mh) return -1;

	struct v4l2_control control;
	control.id = V4L2_CID_SHARPNESS;
	control.value = v;

	if(-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
		perror("error setting sharpness");
		return -1;
	}

	return 1;
}

StereoCamera::StereoCamera(const char *l, const char *r, int w, int h, int f)
{
	left = new Camera(l, w, h, f);
	right = new Camera(r, w, h, f);
	left_name=l;
	right_name=r;
	width=w;
	height=h;
	fps=f;
}

StereoCamera::~StereoCamera()
{
	this->StopCam();
}

void StereoCamera::StopCam()
{
	left->StopCam();
	right->StopCam();
}

bool StereoCamera::GrabFrames()
{
	return (left->GrabFrame() && right->GrabFrame());
}

void StereoCamera::RetrieveLeftImage(IplImage *im)
{
	left->RetrieveFrame();
	left->toIplImage(im);
}

void StereoCamera::RetrieveRightImage(IplImage *im)
{
	right->RetrieveFrame();
	right->toIplImage(im);
}
