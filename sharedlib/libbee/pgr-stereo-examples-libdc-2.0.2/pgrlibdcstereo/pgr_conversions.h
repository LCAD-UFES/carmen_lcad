#ifndef PGR_CONVERSIONS_H
#define PGR_CONVERSIONS_H

//=============================================================================
// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
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
// pgr_conversions.h
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <dc1394/control.h>
#include <dc1394/conversions.h>

//=============================================================================
// PGR Includes
//=============================================================================


//=============================================================================
// Name: dc1394_deinterlace_rgb
//
// Input:
//  src    - The buffer to be de-interlaced
//  width  - The src buffer width (ncols)
//  height - The src buffer height (nrows)
//
// Output:
//  dest   - The output buffer 
//
// Description:
//  This function is a simple re-spin of dc1394_deinterlace_stereo(), which 
//  deinterlaces a 16 bit image into 2 8 bit images.  _rgb() makes 3 8bit images 
//  from a 24 bit image.
// 
//  The 2 8 bit images are stored in continguous memory at 'dest'.  
//
//=============================================================================
void
dc1394_deinterlace_rgb( unsigned char* src, 
			unsigned char* dest, 
			unsigned int width, 
			unsigned int height);

void
dc1394_deinterlace_rgb_single( unsigned char* src, 
			unsigned char* destR, 
			unsigned char* destG,
			unsigned char* destB,
			unsigned int width, 
			unsigned int height);

//=============================================================================
// Modified by Jorcy de Oliveira Neto for color stereo/rectification purposes
//
// Name:	dc1394_deinterlace_red
//		dc1394_deinterlace_green
//		dc1394_deinterlace_blue
//
// Input:
//  src    - The buffer to be de-interlaced
//  width  - The src buffer width (ncols)
//  height - The src buffer height (nrows)
//
// Output:
//  dest   - The output buffer 
//
// Description:
//  These function is a simple re-spin of dc1394_deinterlace_stereo(), which 
//  deinterlaces a 16 bit image into 2 8 bit images.  This function, however,
//  simply deinterlaces the green channel from a 24 bit image.   This can be
//  done as a cheap approximation of a monochrome signal from a 24 bit image,
//  since monochrome is 70% from the green signal
//
//=============================================================================
void
my_dc1394_deinterlace_rgb( unsigned char* src, 
			unsigned char* destR,
			unsigned char* destG,
			unsigned char* destB, 
			unsigned int width, 
			unsigned int height);

void
dc1394_deinterlace_red( unsigned char* src, 
			  unsigned char* dest, 
			  unsigned int width, 
			  unsigned int height);

void
dc1394_deinterlace_green( unsigned char* src, 
			  unsigned char* dest, 
			  unsigned int width, 
			  unsigned int height);

void
dc1394_deinterlace_blue( unsigned char* src, 
			  unsigned char* dest, 
			  unsigned int width, 
			  unsigned int height);

#endif
