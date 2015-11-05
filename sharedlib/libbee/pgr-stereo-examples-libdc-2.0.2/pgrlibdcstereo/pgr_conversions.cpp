//=============================================================================
// Copyright � 2008 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research Inc.
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//
//=============================================================================

//=============================================================================
//
// pgr_conversions.cpp
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <errno.h>
#include <dc1394/log.h>
#include <dc1394/utils.h>
#include <dc1394/register.h>
#include <dc1394/control.h>

//=============================================================================
// PGR Includes
//=============================================================================


//=============================================================================
// Implementation
//=============================================================================

// taken from dc1394_deinterlace_stereo
// change a 24bit rgb image (8bit/channel) into three 8bit images on top
// of each other
void
dc1394_deinterlace_rgb( unsigned char* src, 
			unsigned char* dest, 
			unsigned int width, 
			unsigned int height)
{
  register int i = (width*height)-1;
  register int r = ((width*height)/3)-1;
  register int g = ((width*height)*2/3)-1;
  register int b = (width*height)-1;

  while (i >= 0) {
    dest[r--] = src[i--];
    dest[g--] = src[i--];
    dest[b--] = src[i--];
  }
}

void
dc1394_deinterlace_rgb_single( unsigned char* src, 
			unsigned char* destR,
			unsigned char* destG,
			unsigned char* destB,
			unsigned int width, 
			unsigned int height)
{
  register unsigned int i = 0;
  register unsigned int r = 0;
  register unsigned int g = 0;
  register unsigned int b = 0;

  while (i < (width * height)) {
    destR[r++] = src[i++];
    destG[g++] = src[i++];
    destB[b++] = src[i++];
  }
}

void
my_dc1394_deinterlace_rgb( unsigned char* src, 
			unsigned char* destR,
			unsigned char* destG,
			unsigned char* destB, 
			unsigned int width, 
			unsigned int height)
{
  register int i = (width*height)-1;
  register int r = (width*height)/3-1;
  register int g = (width*height)/3-1;
  register int b = (width*height)/3-1;

  while (i >= 0) {
    destR[r--] = src[i--];
    destG[g--] = src[i--];
    destB[b--] = src[i--];
  }
}

// taken from dc1394_deinterlace_stereo
// change a 24bit rgb image (8bit/channel) into three 8bit images on top
// of each other
void
dc1394_deinterlace_red( unsigned char* src, 
			  unsigned char* dest, 
			  unsigned int width, 
			  unsigned int height)
{
  register int i = (width*height)-1;
  register int r = ((width*height)/3)-1;

  while (i >= 0) {
    dest[r--] = src[i-=3];
  }
}

void
dc1394_deinterlace_green( unsigned char* src, 
			  unsigned char* dest, 
			  unsigned int width, 
			  unsigned int height)
{
  register int i = (width*height)-2;
  register int g = ((width*height)*2/3)-1;

  while (i >= 0) {
    dest[g--] = src[i-=3];
  }
}

void
dc1394_deinterlace_blue( unsigned char* src, 
			  unsigned char* dest, 
			  unsigned int width, 
			  unsigned int height)
{
  register int i = (width*height)-3;
  register int b = (width*height)-1;

  while (i >= 0) {
    dest[b--] = src[i-=3];
  }
}
