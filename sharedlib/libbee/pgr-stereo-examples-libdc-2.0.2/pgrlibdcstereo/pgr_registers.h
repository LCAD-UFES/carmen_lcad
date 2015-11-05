#ifndef PGR_REGISTERS_H
#define PGR_REGISTERS_H

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
// pgr_registers.h
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

// PGR specific register that contains the Bayer Tile mapping information
#define BAYER_TILE_MAPPING_REGISTER 	(0x1040)
#define SENSOR_BOARD_INFO_REGISTER  	(0x1f28)
#define IMAGE_DATA_FORMAT_REGISTER	(0x1048)

//=============================================================================
// Name: getSensorInfo
//
// Input:
//  camera      - The camera to be queried
//
// Output:
//  pbColor   	- Is the camera color?
//  pnRows	- Number of rows in image resolution
//  pnCols	- Number of columns in image resolution
//
// Description:
//  This function queries the PGR registers that identify the sensor info.
//  This is a PGR specific register query.
//
//=============================================================================
dc1394error_t
getSensorInfo( dc1394camera_t* 	camera,
	       bool*		pbColor,
	       unsigned int*	pnRows,
	       unsigned int*	pnCols );

//=============================================================================
// Name: setEndian
//
// Input:
//  camera      - The camera to be queried
//
// Output:
//  bBigEndian  - "true" means transmit 16-bit data in big endian 
//		  (DCAM-compliant) mode
//		- "false" means transmit 16-bit data in little endian mode
//		  which allows the buffers to be directly cast into unsigned
//		  short on Intel-based PC platforms
//
// Description:
//
//=============================================================================
dc1394error_t
setEndian( dc1394camera_t* camera,
	   bool bBigEndian );

//=============================================================================
// Name: getBayerTile
//
// Input:
//  camera      - The camera to be queried
//
// Output:
//  bayerTile   - Returns with the Bayer tile pattern 
//
// Description:
//  This function queries the PGR registers that identify the bayer tile
//  pattern for a bayer camera.
//
//=============================================================================
dc1394error_t
getBayerTile( dc1394camera_t* camera,
	      dc1394color_filter_t* bayerPattern );

#endif
