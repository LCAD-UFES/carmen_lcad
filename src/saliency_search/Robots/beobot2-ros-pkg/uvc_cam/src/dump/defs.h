/*******************************************************************************#
#           guvcview              http://guvcview.berlios.de                    #
#                                                                               #
#           Paulo Assis <pj.assis@gmail.com>                                    #
#                                                                               #
# This program is free software; you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation; either version 2 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
#                                                                               #
# You should have received a copy of the GNU General Public License             #
# along with this program; if not, write to the Free Software                   #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA     #
#                                                                               #
********************************************************************************/

#ifndef DEFS_H
#define DEFS_H
#include <inttypes.h>
#include <sys/types.h>

typedef uint64_t QWORD;
typedef uint32_t DWORD;
typedef uint16_t WORD;
typedef uint8_t  BYTE;
typedef unsigned int LONG;
typedef unsigned int UINT;

typedef unsigned long long ULLONG;
typedef unsigned long      ULONG;

typedef char* pchar;

typedef char		   INT8;
typedef unsigned char	   UINT8;
typedef short		   INT16;
typedef unsigned short int UINT16;
typedef int		   INT32;
typedef unsigned int	   UINT32;

/* 0 is device default*/
static const int stdSampleRates[] = 
{ 
	0, 8000,  9600, 11025, 12000,
	16000, 22050, 24000,
	32000, 44100, 48000,
	88200, 96000,
	-1   /* Negative terminated list. */
};

/*----------- guvcview version ----------------*/
//#define VERSION ("") /*defined in config.h*/
#define DEBUG (0)
/*---------- Thread Stack Size (bytes) --------*/
#define TSTACK (128*64*1024) /* Debian Default 128 pages of 64k = 8388608 bytes*/

/*----------- AVI max file size ---------------*/
#define AVI_MAX_SIZE (1900*1024*1024)
/* extra size beyond MAX SIZE at wich we can still write data*/ 
#define AVI_EXTRA_SIZE (20*1024*1024)

#define INCPANTILT 64 // 1°

#define WINSIZEX 575
#define WINSIZEY 610

#define AUTO_EXP 8
#define MAN_EXP	1

#define DHT_SIZE 432

#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480

#define DEFAULT_IMAGE_FNAME	"Image.jpg"
#define DEFAULT_AVI_FNAME	"capture.avi"
#define DEFAULT_FPS	25
#define DEFAULT_FPS_NUM 1
#define SDL_WAIT_TIME 30 /*SDL - Thread loop sleep time */

/*clip value between 0 and 255*/
#define CLIP(value) (unsigned char)(((value)>0xFF)?0xff:(((value)<0)?0:(value)))

/*FILTER FLAGS*/
#define YUV_NOFILT 0x0000
#define YUV_MIRROR 0x0001
#define YUV_UPTURN 0x0002
#define YUV_NEGATE 0x0004
#define YUV_MONOCR 0x0008

#endif

