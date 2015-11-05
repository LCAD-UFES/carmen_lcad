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
// pgr_registers.cpp
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
#include "pgr_registers.h"

//=============================================================================
// Implementation
//=============================================================================


//=============================================================================
// getBayerTile()
// Query the PGR specific register that indicates the Bayer tile pattern
// color coding for this camera.
//
// For more information check the PGR IEEE-1394 Digital Camera Register
// Reference
//
dc1394error_t
getBayerTile( dc1394camera_t* camera,
	      dc1394color_filter_t* bayerPattern )
{

   uint32_t value;
   dc1394error_t err;

   // query register 0x1040
   // This register is an advanced PGR register called BAYER_TILE_MAPPING
   // For more information check the PGR IEEE-1394 Digital Camera Register Reference
   err = dc1394_get_control_register( camera, BAYER_TILE_MAPPING_REGISTER, &value );
   if ( err != DC1394_SUCCESS )
   {
      return err;
   }

   // Ascii R = 52 G = 47 B = 42 Y = 59
   switch( value )
   {
      default:
      case 0x59595959:	// YYYY
	 // no bayer
	 *bayerPattern = (dc1394color_filter_t) 0;
	 break;
      case 0x52474742:	// RGGB
	 *bayerPattern = DC1394_COLOR_FILTER_RGGB;
	 break;
      case 0x47425247:	// GBRG
	 *bayerPattern = DC1394_COLOR_FILTER_GBRG;
	 break;
      case 0x47524247:	// GRBG
	 *bayerPattern = DC1394_COLOR_FILTER_GRBG;
	 break;
      case 0x42474752:	// BGGR
	 *bayerPattern = DC1394_COLOR_FILTER_BGGR;
	 break;
   }

   return err;
}

//=============================================================================
// setEndian()
// Set the endian-ness of 16-bit data coming from the camera.
//
// This is a PGR camera specific register
// For more information check the PGR IEEE-1394 Digital Camera Register
// Reference
//
dc1394error_t
setEndian( dc1394camera_t* camera,
	   bool bBigEndian )
{

   uint32_t value;
   dc1394error_t err;

   if ( bBigEndian )
   {
      // use 16-bit modes in big endian (default 1394) format
      value = 0x80000001;
   }
   else
   {
      // use 16-bit modes in little endian (pgr default) format
      value = 0x80000000;
   }


   err = dc1394_set_control_register( camera,
				   IMAGE_DATA_FORMAT_REGISTER,
				   value );
   return err;
}


//=============================================================================
// getSensorInfo()
// This function queries the SENSOR BOARD INFO register and extracts
// the sensor resolution and whether it is color or monochrome from the
// register
//
// This is a PGR camera specific register
// For more information check the PGR IEEE-1394 Digital Camera Register
// Reference
//
dc1394error_t
getSensorInfo( dc1394camera_t* 	camera,
	       bool*		pbColor,
	       unsigned int*	pnRows,
	       unsigned int*	pnCols )
{

   uint32_t value;
   dc1394error_t err;

   // This register is an advanced PGR register called SENSOR_BOARD_INFO
   err = dc1394_get_control_register( camera, SENSOR_BOARD_INFO_REGISTER, &value );
   if ( err != DC1394_SUCCESS )
   {
      return err;
   }

   unsigned char ucSensorInfo = 0xf & value;

   printf("value: %x\n", ucSensorInfo);

   switch( ucSensorInfo )
   {
      default:
	 // unknown sensor!
	 printf( "Illegal sensor board info detected!\n" );
	 return DC1394_FAILURE;
      case 0xA:	// color 640x480
	 *pbColor	= true;
	 *pnRows	= 480;
	 *pnCols	= 640;
	 break;
      case 0xB:	// mono 640x480
	 *pbColor	= false;
	 *pnRows	= 480;
	 *pnCols	= 640;
	 break;
      case 0xC:	// color 1024x768
	 *pbColor	= true;
	 *pnRows	= 768;
	 *pnCols	= 1024;
	 break;
      case 0xD:	// mono 1024x768
	 *pbColor	= false;
	 *pnRows	= 768;
	 *pnCols	= 1024;
	 break;
      case 0xE:	// color 1280x960
	 *pbColor	= true;
	 *pnRows	= 960;
	 *pnCols	= 1280;
	 break;
      case 0xF:	// mono 1280x960
	 *pbColor	= false;
	 *pnRows	= 960;
	 *pnCols	= 1280;
	 break;
   }

   return err;
}


