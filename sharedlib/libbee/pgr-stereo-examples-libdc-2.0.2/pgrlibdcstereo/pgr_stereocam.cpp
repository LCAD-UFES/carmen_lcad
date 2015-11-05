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
// pgr_stereocam.cpp
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <dc1394/log.h>
#include <dc1394/utils.h>
#include <dc1394/register.h>
#include <dc1394/control.h>

//=============================================================================
// PGR Includes
//=============================================================================
#include "pgr_conversions.h"
#include "pgr_stereocam.h"
#include "pgr_registers.h"


//=============================================================================
// Implementation
//=============================================================================

// used when querying product model strings to determine model types
const char* g_szBB3Base = "Bumblebee XB3";
const char* g_szBB2Base = "Bumblebee2";
const char* g_szBBBase 	= "BumbleBee";


bool
isBumblebeeXB3( dc1394camera_t* camera )
{
	if ( !strncmp( camera->model, g_szBB3Base, strlen(g_szBB3Base) ) )
		return true;

	return false;
}

bool
isBumblebee2( dc1394camera_t* camera )
{
	if ( !strncmp( camera->model, g_szBB2Base, strlen(g_szBB2Base) ) )
		return true;

	return false;
}

bool
isBumblebee( dc1394camera_t* camera )
{
	if ( !strncmp( camera->model, g_szBBBase, strlen(g_szBBBase) ) )
		return true;

	return false;
}

bool isStereoCamera( dc1394camera_t* camera )
{
	if ( isBumblebeeXB3( camera ) )
		return true;
	if ( isBumblebee2( camera ) )
		return true;
	if ( isBumblebee( camera ) )
		return true;

	return false;
}


PGRStereoCameraModel_t
getCameraModel( dc1394camera_t* camera )
{
	if ( isBumblebeeXB3( camera ) )
		return BUMBLEBEEXB3;
	else if ( isBumblebee2( camera ) )
		return BUMBLEBEE2;
	else if ( isBumblebee( camera ) )
		return BUMBLEBEE;

	return UNKNOWN_CAMERA;
}

//=============================================================================
// querySteroeCamera()
//
// Given that the camera handle is a stereo camera, query the camera for
// stereo specific information about this camera and populate the
// PGRStereoCamera_t handle structure
//
dc1394error_t
queryStereoCamera( dc1394camera_t* 	camera,
		PGRStereoCamera_t* 	stereoCamera )
{

	// set the camera handle
	stereoCamera->camera = camera;

	// find out what base model camera we have
	stereoCamera->model = getCameraModel( camera );
	if ( stereoCamera->model == UNKNOWN_CAMERA )
		return DC1394_FAILURE;


	dc1394error_t err;

	if ( stereoCamera->model != BUMBLEBEE )
	{
		err = getSensorInfo( camera,
				&stereoCamera->bColor,
				&stereoCamera->nRows,
				&stereoCamera->nCols );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Could not query the Sensor Info Register!\n" );
			return err;
		}
	}
	else // model == BUMBLEBEE
	{
		// This is a Bumblebee "1".  This camera does not support the
		// sensor board info register so we need to determine if it is color
		// and the resolution the hard way
		//
		// It will be nice when we don't need to support BB1 anymore as it is
		// not completely DC-compliant

		dc1394video_modes_t 	video_modes;
		err = dc1394_video_get_supported_modes( camera, &video_modes );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Can't get video modes\n" );
			return err;
		}

		// find the highest res mode that is greyscale (MONO16)
		printf( "Searching for the highest resolution MONO16 mode available...\n" );
		dc1394video_mode_t 	video_mode = DC1394_VIDEO_MODE_FORMAT7_MIN;
		dc1394color_coding_t 	coding;
		for ( int i = video_modes.num-1; i >= 0; i-- )
		{
			// don't consider FORMAT 7 modes (i.e. "scalable")
			if ( !dc1394_is_video_mode_scalable( video_modes.modes[i] ) )
			{
				dc1394_get_color_coding_from_video_mode( camera, video_modes.modes[i], &coding );
				if ( coding == DC1394_COLOR_CODING_MONO16 )
				{
					video_mode = video_modes.modes[i];
					break;
				}
			}
		}
		if ( video_mode == DC1394_VIDEO_MODE_640x480_MONO16 )
		{
			stereoCamera->nRows = 480;
			stereoCamera->nCols = 640;
		}
		else if ( video_mode == DC1394_VIDEO_MODE_1024x768_MONO16 )
		{
			stereoCamera->nRows = 768;
			stereoCamera->nCols = 1024;
		}
		else
		{
			fprintf( stderr, "Cannot find valid MONO16 video mode!\n" );
			return DC1394_FAILURE;
		}


		dc1394color_filter_t bayerPattern;
		err = getBayerTile( stereoCamera->camera, &bayerPattern );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Failed to read the Bayer Tile Pattern register\n" );
			return err;
		}
		// at this point all we need to know is "is it color or mono?"
		if ( bayerPattern == 0 )
			stereoCamera->bColor = false;
		else
			stereoCamera->bColor = true;

	}

	// a hack to figure out how many bytes per pixel are needed.
	// if the camera is a BB3, then it is 3, otherwise 2
	if ( stereoCamera->nRows == 960 )
	{
		//stereoCamera->nBytesPerPixel	= 3;
		// note: for performance reasons we have changed the default behavior
		// of the XB3 for these examples to only use the 2 wide-baseline pair.
		// This makes for faster image transmission.
		// If you change the code to transmit all 3 images, this value will
		// have to revert to 3.
		stereoCamera->nBytesPerPixel	= 2;
	}
	else
	{
		stereoCamera->nBytesPerPixel	= 2;
	}

	return DC1394_SUCCESS;
}


//=============================================================================
// setStereoVideoCapture()
//
// Given that the camera handle is a stereo camera, query the camera for
// stereo specific information about this camera and populate the
// PGRStereoCamera_t handle structure
//
// note: currently always allocated maximum framerate
dc1394error_t
setStereoVideoCapture( PGRStereoCamera_t* stereoCamera, bool is_legacy_firewire )
{
	dc1394error_t 	err;
	dc1394color_coding_t	coding;

	switch( stereoCamera->model )
	{
	case BUMBLEBEE:
		dc1394video_mode_t 	videoMode;
		dc1394framerate_t 	fps;

		// note: both B&W and Bayer Bumblebees transmit in mono mode
		if ( stereoCamera->nCols == 640 )
		{
			// lo res
			videoMode 	= DC1394_VIDEO_MODE_640x480_MONO16;
			fps		= DC1394_FRAMERATE_30;
		}
		else
		{
			// assume hi res
			videoMode 	= DC1394_VIDEO_MODE_1024x768_MONO16;
			fps		= DC1394_FRAMERATE_15;
		}

		// make the calls to set up the capture mode
		dc1394_video_set_iso_speed( stereoCamera->camera, DC1394_ISO_SPEED_400 );
		dc1394_video_set_mode(      stereoCamera->camera, videoMode );
		dc1394_video_set_framerate( stereoCamera->camera, fps );
		err = dc1394_capture_setup( stereoCamera->camera, 1, DC1394_CAPTURE_FLAGS_DEFAULT );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Can't setup Bumblebee capture\n" );
			return err;
		}
		break;

	case BUMBLEBEE2:
		// Bumblebee2 transmits stereo images in Format 7

		// load the factory defaults - this is auto-everything
		err = dc1394_memory_load( stereoCamera->camera, 0 );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Can't load default memory channel\n" );
			return err;
		}

		// set 16-bit transmission to be PGR-default little endian mode
		err = setEndian( stereoCamera->camera, false );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Can't set Bumblebee2 into little-endian mode\n" );
			return err;
		}

		// color cameras transmit in "RAW16", mono cameras in "MONO16"
		coding	= DC1394_COLOR_CODING_MONO16;
		if ( stereoCamera->bColor )
			coding = DC1394_COLOR_CODING_RAW16;

		dc1394_video_set_iso_speed( stereoCamera->camera, DC1394_ISO_SPEED_400 );
		dc1394_video_set_mode( stereoCamera->camera, DC1394_VIDEO_MODE_FORMAT7_3 );

		err = dc1394_format7_set_roi( stereoCamera->camera,
				DC1394_VIDEO_MODE_FORMAT7_3,
				coding,
				// bytes per packet - sets frame rate
				DC1394_USE_RECOMMENDED,
				0,
				0,
				stereoCamera->nCols,
				stereoCamera->nRows );



		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Can't setup Bumblebee2 capture\n" );
			return err;
		}

		err = dc1394_capture_setup( stereoCamera->camera, 2, DC1394_CAPTURE_FLAGS_DEFAULT );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Can't setup Bumblebee capture\n" );
			return err;
		}
		break;

	case BUMBLEBEEXB3:
		// Bumblebee3 transmits stereo images in Format 7

		// load the factory defaults - this is auto-everything
		err = dc1394_memory_load( stereoCamera->camera, 0 );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Can't load default memory channel\n" );
			return err;
		}


		if ( stereoCamera->nBytesPerPixel == 2 )
		{
			// run in 2 camera mode

			// NOTE: this code will always only set up the wide-baseline pair
			// To change to the inner baseline pair you need to set the PAN
			// register to 1.
			// PAN = 0 is the wide baseline pair which is set by default.
			// To change to all 3 images being transmitted you need to change
			// modes to "coding	= DC1394_COLOR_CODING_RGB8;"
			//

			// set 16-bit transmission to be PGR-default little endian mode
			err = setEndian( stereoCamera->camera, false );
			if ( err != DC1394_SUCCESS )
			{
				fprintf( stderr, "Can't set Bumblebee2 into little-endian mode\n" );
				return err;
			}

			// color cameras transmit in "RAW16", mono cameras in "MONO16"
			coding	= DC1394_COLOR_CODING_MONO16;
			if ( stereoCamera->bColor )
				coding = DC1394_COLOR_CODING_RAW16;
		}
		else
		{
			// 3 camera mode transmits in RGB8
			coding	= DC1394_COLOR_CODING_RGB8;
		}

		// assume the XB is plugged into a 1394B network
		// XB3 can work with a 1394A bus but code changes will be required
		if (is_legacy_firewire)
		{
			dc1394_video_set_operation_mode( stereoCamera->camera, DC1394_OPERATION_MODE_LEGACY );
			dc1394_video_set_iso_speed( stereoCamera->camera, DC1394_ISO_SPEED_400 );
		}
		else
		{
			dc1394_video_set_operation_mode( stereoCamera->camera, DC1394_OPERATION_MODE_1394B );
			dc1394_video_set_iso_speed( stereoCamera->camera, DC1394_ISO_SPEED_800 );
		}
		dc1394_video_set_mode( stereoCamera->camera, DC1394_VIDEO_MODE_FORMAT7_3 );
		err = dc1394_format7_set_roi( stereoCamera->camera,
				DC1394_VIDEO_MODE_FORMAT7_3,
				coding,
				// bytes per packet - sets frame rate
				DC1394_USE_MAX_AVAIL,
				0,
				0,
				stereoCamera->nCols,
				stereoCamera->nRows );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Can't setup Bumblebee XB3 capture\n" );
			return err;
		}

		err = dc1394_capture_setup( stereoCamera->camera, 1, DC1394_CAPTURE_FLAGS_DEFAULT );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Can't setup Bumblebee capture\n" );
			return err;
		}
		break;

	default:
		fprintf( stderr,
				"setStereoVideoCapture() - \n"
				"\tThis camera is not a supported stereo camera!\n" );
		return DC1394_FAILURE;
	}

	// get the bayer tile info so we will know how to color process
	// this mode
	err = getBayerTile( stereoCamera->camera,
			&stereoCamera->bayerTile );
	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "Could not query the Bayer Tile Register!\n" );
		return err;
	}

	return DC1394_SUCCESS;
}


//=============================================================================
// startTransmission()
//
// A helper function to wrap up the sleeps involved with starting transmission.
// This loop method was taken from the libdc1394-2 examples.
//
dc1394error_t
startTransmission( PGRStereoCamera_t* stereoCamera )
{
	dc1394error_t err;
	// have the camera start sending us data
	err = dc1394_video_set_transmission( stereoCamera->camera, DC1394_ON );
	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "Unable to start camera iso transmission\n" );
		return err;
	}

	// printf( "Waiting for transmission... \n" );
	//  Sleep untill the camera has a transmission
	dc1394switch_t status = DC1394_OFF;

	for ( int i = 0; i <= 5; i++ )
	{
		// By Jorcy de Oliveira Neto : No camera delay must be applied here !!!
		//usleep(50000);
		err = dc1394_video_get_transmission( stereoCamera->camera, &status );
		if ( err != DC1394_SUCCESS )
		{
			fprintf( stderr, "Unable to get transmision status\n" );
			return err;
		}
		if ( status != DC1394_OFF )
			break;

		if( i == 5 )
		{
			fprintf(stderr,"Camera doesn't seem to want to turn on!\n");
			return DC1394_FAILURE;
		}
	}

	return DC1394_SUCCESS;
}


//=============================================================================
// extractImagesColor()
//
// De-interleave the stereo images into single bayer patterns.
// De-bayer those images into color images.
// Construct a TriclopsInput for stereo processing from these images.
//
// Modified By Jorcy de Oliveira Neto, for color stereo processing purposes
//
void
extractImagesColor( PGRStereoCamera_t* 	 stereoCamera,
		dc1394bayer_method_t bayerMethod,
		unsigned char* 	 pucDeInterleaved,
		unsigned char* 	 pucRGB,
		unsigned char* 	 pucRed,
		unsigned char* 	 pucGreen,
		unsigned char* 	 pucBlue,
		unsigned char** 	 ppucRightRGB,
		unsigned char** 	 ppucLeftRGB,
		unsigned char** 	 ppucCenterRGB,
		TriclopsInput*  	 pTriclopsInput )
{
	struct timeval tv;

	dc1394error_t err;
	dc1394video_frame_t* frame;
	err = dc1394_capture_dequeue( stereoCamera->camera,
			DC1394_CAPTURE_POLICY_WAIT,
			&frame );

	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "extractImagesColor - cannot dequeue image!\n" );
		return;
	}

	if (gettimeofday(&tv, NULL) < 0)
		fprintf(stderr, "extractImagesColor() encountered error in gettimeofday : %s.\n File pgr_stereocam.cpp\n\n",
				strerror(errno));

	unsigned char* pucGrabBuffer = frame->image;

	if ( stereoCamera->nBytesPerPixel == 2 )
	{
		// de-interlace the 16 bit data into 2 bayer tile pattern images
		dc1394_deinterlace_stereo( pucGrabBuffer,
				pucDeInterleaved,
				stereoCamera->nCols,
				2*stereoCamera->nRows );

		// extract color from the bayer tile image
		// note: this will alias colors on the top and bottom rows
		dc1394_bayer_decoding_8bit( pucDeInterleaved,
				pucRGB,
				stereoCamera->nCols,
				2*stereoCamera->nRows,
				stereoCamera->bayerTile,
				bayerMethod );

		// RGB de-interlacing for color rectification/stereo processing
		dc1394_deinterlace_rgb_single(pucRGB, pucRed, pucGreen, pucBlue, stereoCamera->nCols, 6*stereoCamera->nRows);

		*ppucRightRGB 	= pucRGB;
		*ppucLeftRGB 	= pucRGB + 3 * stereoCamera->nRows * stereoCamera->nCols;
		*ppucCenterRGB	= *ppucLeftRGB;

	}
	else
	{
		dc1394_deinterlace_rgb( pucGrabBuffer,
				pucDeInterleaved,
				stereoCamera->nCols,
				3*stereoCamera->nRows );
		// extract color from the bayer tile image
		// note: this will alias colors on the top and bottom rows
		dc1394_bayer_decoding_8bit( pucDeInterleaved,
				pucRGB,
				stereoCamera->nCols,
				3*stereoCamera->nRows,
				stereoCamera->bayerTile,
				bayerMethod );
		// RGB de-interlacing for color rectification/stereo processing
		dc1394_deinterlace_rgb_single(pucRGB,pucRed,pucGreen,pucBlue,stereoCamera->nCols,9*stereoCamera->nRows);

		// NOTE: this code needs to be double checked.
		// Currently 3-bytes-per-pixel is not activatable in this example
		*ppucRightRGB 	= pucRGB;
		*ppucCenterRGB 	= pucRGB + 3 * stereoCamera->nRows * stereoCamera->nCols;
		*ppucLeftRGB 	= pucRGB + 6 * stereoCamera->nRows * stereoCamera->nCols;
	}

	pTriclopsInput->inputType 	= TriInp_RGB;
	pTriclopsInput->nrows	= stereoCamera->nRows;
	pTriclopsInput->ncols	= stereoCamera->nCols;
	pTriclopsInput->rowinc	= stereoCamera->nCols;

	pTriclopsInput->u.rgb.red   	= pucRed;
	pTriclopsInput->u.rgb.green 	= pucGreen;
	pTriclopsInput->u.rgb.blue  	= pucBlue;

	pTriclopsInput->timeStamp.sec = tv.tv_sec;
	pTriclopsInput->timeStamp.u_sec = tv.tv_usec;

	// return buffer for use
	dc1394_capture_enqueue( stereoCamera->camera, frame );
	return;
}

//=============================================================================
// extractImagesMono()
//
// De-interleave the stereo images into single images
// Construct a TriclopsInput for stereo processing from these images.
//
void
extractImagesMono( PGRStereoCamera_t* 	stereoCamera,
		unsigned char* 	pucDeInterleaved,
		unsigned char** 	ppucRightMono8,
		unsigned char** 	ppucLeftMono8,
		unsigned char** 	ppucCenterMono8,
		TriclopsInput*  	pTriclopsInput )
{

	dc1394error_t err;
	// RC7
	dc1394video_frame_t* frame;
	err = dc1394_capture_dequeue( stereoCamera->camera,
			DC1394_CAPTURE_POLICY_WAIT,
			&frame );
	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "extractImagesColor - cannot dequeue image!\n" );
		return;
	}

	unsigned char* pucGrabBuffer = frame->image;

	unsigned char* right;
	unsigned char* left;
	unsigned char* center;
	if ( stereoCamera->nBytesPerPixel == 2 )
	{
		// de-interlace the 16 bit data into 2 mono images
		dc1394_deinterlace_stereo( pucGrabBuffer,
				pucDeInterleaved,
				stereoCamera->nCols,
				2*stereoCamera->nRows );
		right = pucDeInterleaved;
		left  = pucDeInterleaved + stereoCamera->nRows * stereoCamera->nCols;
		center= left;
	}
	else
	{
		dc1394_deinterlace_rgb( pucGrabBuffer,
				pucDeInterleaved,
				stereoCamera->nCols,
				3*stereoCamera->nRows );

		// NOTE: this code needs to be double checked.
		// Currently 3-bytes-per-pixel is not activatable in this example
		right 	= pucDeInterleaved;
		center  	= pucDeInterleaved + stereoCamera->nRows * stereoCamera->nCols;
		left	= pucDeInterleaved + 2 * stereoCamera->nRows * stereoCamera->nCols;
	}

	*ppucRightMono8 	= right;
	*ppucLeftMono8 	= left;
	*ppucCenterMono8 	= center;
	pTriclopsInput->inputType 	= TriInp_RGB;
	pTriclopsInput->nrows	= stereoCamera->nRows;
	pTriclopsInput->ncols	= stereoCamera->nCols;
	pTriclopsInput->rowinc	= stereoCamera->nCols;
	pTriclopsInput->u.rgb.red   = right;
	pTriclopsInput->u.rgb.green = left;
	pTriclopsInput->u.rgb.blue  = left;

	// return buffer for use
	dc1394_capture_enqueue( stereoCamera->camera, frame );

	return;
}


#define REG_CONFIG_LENGTH         	0x1FFC
#define REG_CONFIG_DATA           	0x2000
#define REG_UNIT_DIRECTORY_OFFSET      	0x0424


dc1394error_t
writeTriclopsConfigFromCameraToFile( dc1394camera_t* camera,
		const char* outputFile)
{
	dc1394error_t err;
	uint32_t 	ulQuadlet;

	err = dc1394_get_control_register( camera, REG_CONFIG_LENGTH, &ulQuadlet );
	if ( err != DC1394_SUCCESS )
	{
		fprintf(stderr, "dc1394_get_control_register(REG_CONFIG_LENGTH) failed\n");
		return err;
	}

	// the length of the config file
	unsigned long ulFileSizeBytes = ulQuadlet;
	if( ulFileSizeBytes == 0 )
	{
		fprintf( stderr, "File size == 0!\n" );
		return DC1394_FAILURE;
	}

	FILE* pfile = fopen( outputFile, "w" );
	if ( !pfile )
	{
		fprintf( stderr, "Can't open temporary file\n" );
		return DC1394_FAILURE;
	}

	// Read the config file, and save it to the output file,
	// while fixing endianness.
	for( unsigned long offset = 0 ; offset < ulFileSizeBytes; offset += 4 )
	{
		err = dc1394_get_control_register( camera,
				REG_CONFIG_DATA + offset,
				&ulQuadlet );

		if( err != DC1394_SUCCESS )
		{
			fprintf( stderr,
					"Can't get control register 0x%x\n",
					(int) (REG_CONFIG_DATA+offset) );
			fclose( pfile );
			return err;
		}


		for( int i = 24; i >= 0; i -= 8 )
		{
			fputc( ( (ulQuadlet>>i) & 0xFF ), pfile );
		}
	}

	fclose( pfile );

	return DC1394_SUCCESS;
}


TriclopsError
getTriclopsContextFromCamera( PGRStereoCamera_t* stereoCamera,
		TriclopsContext* pTriclops )
{
	// note: if you have multi-users you should use a more robust temporary file name
	// creation method to avoid permission conflicts
	const char* tempFile = "/tmp/triclops.cal";
	dc1394error_t err;
	err = writeTriclopsConfigFromCameraToFile( stereoCamera->camera, tempFile );
	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "Couldn't write config data to file\n" );
		return TriclopsErrorSystemError;
	}

	TriclopsError  tErr;
	tErr = triclopsGetDefaultContextFromFile( pTriclops, (char*) tempFile );
	if ( tErr != TriclopsErrorOk )
	{
		fprintf( stderr, "triclopsGetDefaultContextFromFile failed!\n" );
	}

	return tErr;
}

//=============================================================================
// extractImagesColor()
//
// De-interleave the stereo images into single bayer patterns.
// De-bayer those images into color images.
// Construct a TriclopsInput for stereo processing from these images.
//
void
extractImagesColorXB3( PGRStereoCamera_t* 	 stereoCamera,
		dc1394bayer_method_t bayerMethod,
		unsigned char* 	pucDeInterleaved,
		unsigned char* 	pucRGB,
		unsigned char* 	pucGreen,
		unsigned char** 	ppucRightRGB,
		unsigned char** 	ppucLeftRGB,
		unsigned char** 	ppucCenterRGB,
		TriclopsInput*  	pShortInput,
		TriclopsInput* 	pWideInput  )
{

	dc1394error_t err;
	dc1394video_frame_t* frame;
	err = dc1394_capture_dequeue( stereoCamera->camera,
			DC1394_CAPTURE_POLICY_WAIT,
			&frame );
	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "extractImagesColor - cannot dequeue image!\n" );
		return;
	}

	unsigned char* pucGrabBuffer = frame->image;

	dc1394_deinterlace_rgb( pucGrabBuffer,
			pucDeInterleaved,
			stereoCamera->nCols,
			3*stereoCamera->nRows );
	// extract color from the bayer tile image
	// note: this will alias colors on the top and bottom rows
	dc1394_bayer_decoding_8bit( pucDeInterleaved,
			pucRGB,
			stereoCamera->nCols,
			3*stereoCamera->nRows,
			stereoCamera->bayerTile,
			bayerMethod );
	// now deinterlace the RGB Buffer
	dc1394_deinterlace_green( pucRGB,
			pucGreen,
			stereoCamera->nCols,
			9*stereoCamera->nRows );
	// NOTE: this code needs to be double checked.
	// Currently 3-bytes-per-pixel is not activatable in this example
	int iOneBufferPixels = stereoCamera->nRows * stereoCamera->nCols;
	*ppucLeftRGB 	= pucRGB;
	*ppucCenterRGB 	= pucRGB + 3 * iOneBufferPixels;
	*ppucRightRGB 	= pucRGB + 6 * iOneBufferPixels;

	pShortInput->inputType 	= TriInp_RGB;
	pShortInput->nrows		= stereoCamera->nRows;
	pShortInput->ncols		= stereoCamera->nCols;
	pShortInput->rowinc		= stereoCamera->nCols;
	pShortInput->u.rgb.red   	= pucGreen + 2*iOneBufferPixels;
	pShortInput->u.rgb.green 	= pucGreen + iOneBufferPixels;
	pShortInput->u.rgb.blue  	= pShortInput->u.rgb.green;

	pWideInput->inputType 	= TriInp_RGB;
	pWideInput->nrows		= stereoCamera->nRows;
	pWideInput->ncols		= stereoCamera->nCols;
	pWideInput->rowinc		= stereoCamera->nCols;
	pWideInput->u.rgb.red   	= pucGreen + 2*iOneBufferPixels;
	pWideInput->u.rgb.green 	= pucGreen;
	pWideInput->u.rgb.blue  	= pWideInput->u.rgb.green;

	// return buffer for use
	dc1394_capture_enqueue( stereoCamera->camera, frame );
	return;
}

//=============================================================================
// extractImagesMono()
//
// De-interleave the stereo images into single images
// Construct a TriclopsInput for stereo processing from these images.
//
void
extractImagesMonoXB3( PGRStereoCamera_t* 	stereoCamera,
		unsigned char* 	pucDeInterleaved,
		unsigned char** 	ppucRightMono8,
		unsigned char** 	ppucLeftMono8,
		unsigned char** 	ppucCenterMono8,
		TriclopsInput*  	pShortInput,
		TriclopsInput* 	pWideInput )
{

	dc1394error_t err;
	// RC7
	dc1394video_frame_t* frame;
	err = dc1394_capture_dequeue( stereoCamera->camera,
			DC1394_CAPTURE_POLICY_WAIT,
			&frame );
	if ( err != DC1394_SUCCESS )
	{
		fprintf( stderr, "extractImagesColor - cannot dequeue image!\n" );
		return;
	}

	unsigned char* pucGrabBuffer = frame->image;

	unsigned char* right;
	unsigned char* left;
	unsigned char* center;

	dc1394_deinterlace_rgb( pucGrabBuffer,
			pucDeInterleaved,
			stereoCamera->nCols,
			3*stereoCamera->nRows );

	// NOTE: this code needs to be double checked.
	// Currently 3-bytes-per-pixel is not activatable in this example
	left 	= pucDeInterleaved;
	center  	= pucDeInterleaved + stereoCamera->nRows * stereoCamera->nCols;
	right	= pucDeInterleaved + 2 * stereoCamera->nRows * stereoCamera->nCols;

	*ppucRightMono8 	= right;
	*ppucLeftMono8 	= left;
	*ppucCenterMono8 	= center;

	pShortInput->inputType 	= TriInp_RGB;
	pShortInput->nrows		= stereoCamera->nRows;
	pShortInput->ncols		= stereoCamera->nCols;
	pShortInput->rowinc		= stereoCamera->nCols;
	pShortInput->u.rgb.red   	= right;
	pShortInput->u.rgb.green 	= center;
	pShortInput->u.rgb.blue  	= center;

	pWideInput->inputType 	= TriInp_RGB;
	pWideInput->nrows		= stereoCamera->nRows;
	pWideInput->ncols		= stereoCamera->nCols;
	pWideInput->rowinc		= stereoCamera->nCols;
	pWideInput->u.rgb.red   	= right;
	pWideInput->u.rgb.green 	= left;
	pWideInput->u.rgb.blue  	= left;

	// return buffer for use
	dc1394_capture_enqueue( stereoCamera->camera, frame );

	return;
}
