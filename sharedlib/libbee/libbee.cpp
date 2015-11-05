/*! \file
 * \brief Bumblebee2 capture library
 */

/********************************** WARNING:***********************************************
 * For Bumblebee2 camera usage, one must have full permission to use the firewire devices
 * The following commands must be propted in the /dev/ as root :
 *
 * chmod 777 fw0
 * chmod 777 fw1
 * ln -s fw0 video1394
 *******************************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <sys/time.h>
#include "include/libbee.hpp"

double carmen_get_time();

//
// Macro to check, report on, and handle libdc1394 error codes.
//

#define _HANDLE_LIBDC1394_ERROR(function,error,camera) \
		{ \
	if ( error != DC1394_SUCCESS ) \
	{ \
		printf( " libdc1394 error in %s: %d\n", function, error ); \
		libbee_cleanup_and_exit(camera); \
	} \
		} \

//
// Macro to check, report on, and handle Triclops API error codes.
//
#define _HANDLE_TRICLOPS_ERROR(function,error,camera,triclops) \
		{ \
			if( error != TriclopsErrorOk ) \
			{ \
				printf( " Triclops error : Error %d reported in %s.\n",error,function); \
				triclopsDestroyContext(triclops); \
				libbee_cleanup_and_exit(camera); \
			} \
		} \

// Left or right positions
#define	LEFT 0
#define	RIGHT 1

// Global stereo camera context (PGRStereoCamera and dc1394camera)
PGRStereoCamera_t stereoCamera;
dc1394camera_t* 	camera;
// Global Triclops SDK context and error
TriclopsContext triclops;
TriclopsError 	e;

// allocate a buffer to hold the de-interleaved images
unsigned int   nBufferSize;
unsigned char* pucDeInterlacedBuffer;

// Color Image Capture Buffers
unsigned char* pucRGBBuffer;
unsigned char* pucRedBuffer;
unsigned char* pucGreenBuffer;
unsigned char* pucBlueBuffer;

//=============================================================================
// a simple function to write a .pgm file
int
libbee_writePgm( char* 	szFilename,
		unsigned char* pucBuffer,
		int		width,
		int		height )
{
	FILE* stream;
	stream = fopen( szFilename, "wb" );
	if( stream == NULL)
	{
		perror( "Can't open image file" );
		return 1;
	}

	fprintf( stream, "P5\n%u %u 255\n", width, height );
	fwrite( pucBuffer, width, height, stream );
	fclose( stream );
	return 0;
}

//=============================================================================
// a simple function to write a .ppm file
int
libbee_writePpm( char* 	szFilename,
		unsigned char* pucBuffer,
		int		width,
		int		height )
{
	FILE* stream;
	stream = fopen( szFilename, "wb" );
	if( stream == NULL)
	{
		perror( "Can't open image file" );
		return 1;
	}

	fprintf( stream, "P6\n%u %u 255\n", width, height );
	fwrite( pucBuffer, 3*width, height, stream );
	fclose( stream );
	return 0;
}


//=============================================================================
// libbee_cleanup_camera
// This is called for destroying the existing connections
// to the 1394 drivers
void
libbee_cleanup_camera( dc1394camera_t* camera )
{

	delete[] pucDeInterlacedBuffer;

	if ( pucRGBBuffer )
		delete[] pucRGBBuffer;
	if ( pucRedBuffer )
		delete[] pucRedBuffer;
	if ( pucGreenBuffer )
		delete[] pucGreenBuffer;
	if ( pucBlueBuffer )
		delete[] pucBlueBuffer;

	dc1394_capture_stop( camera );
	dc1394_video_set_transmission( camera, DC1394_OFF );
	dc1394_camera_free( camera );
}

//=============================================================================
// libbee_cleanup_camera
// This is called when the program exits and destroys the existing connections
// to the 1394 drivers
void
libbee_cleanup_and_exit( dc1394camera_t* camera )
{
	dc1394_capture_stop( camera );
	dc1394_video_set_transmission( camera, DC1394_OFF );
	dc1394_camera_free( camera );
	exit( 0 );
}

//======================================================================================================================
//Initialize global Bumblebee2 camera context both the Stereo Camera and Triclops context
//
void	libbbee_initialize_camera_context(long int guid, int rectified_width, int rectified_height, bool is_rectified, bool is_legacy_firewire)
{
	dc1394error_t 		err;
	dc1394_t* 		d;
	//unsigned int 		nThisCam;

	// Find cameras on the 1394 buses
	d = dc1394_new ();
	camera = dc1394_camera_new(d, guid);
	if(!camera)
	{
		printf("Failed to initialize camera\n");// with guid %lld", list->ids[nThisCam].guid);
		exit(0);
	}

	if (!isStereoCamera(camera))
		dc1394_camera_free(camera);

	//	uint32_t value;
	//	err = dc1394_get_control_register( camera, 0x1f28, &value );
	//
	//	printf("value:%x\n", value);
	//
	//	value-=2;
	//	err = dc1394_set_control_register(camera, 0x1f28, value);
	//
	//	printf("value:%x\n", value);

	// query information about this stereo camera
	err = queryStereoCamera(camera, &stereoCamera);
	_HANDLE_LIBDC1394_ERROR("queryStereoCamera",err,camera);

	if ( stereoCamera.nBytesPerPixel != 2 )
	{
		// can't handle XB3 3 bytes per pixel
		fprintf( stderr,"Example has not been updated to work with XB3 in 3 camera mode yet!\n" );
		libbee_cleanup_and_exit( stereoCamera.camera );
	}

	// set the capture mode
	err = setStereoVideoCapture( &stereoCamera, is_legacy_firewire );
	_HANDLE_LIBDC1394_ERROR("setStereoVideoCapture",err,stereoCamera.camera);

	printf("****** width:%d, height:%d bytesPerPixel: %d********\n", stereoCamera.nCols, stereoCamera.nRows, stereoCamera.nBytesPerPixel);

	// have the camera start sending us data
	err = startTransmission( &stereoCamera );
	_HANDLE_LIBDC1394_ERROR("startTransmission",err,stereoCamera.camera);

	e = getTriclopsContextFromCamera( &stereoCamera, &triclops );
	_HANDLE_TRICLOPS_ERROR("getTriclopsContextFromCamera",e,camera,triclops);

	if (is_rectified)
	{
		e = triclopsSetResolution( triclops, rectified_height, rectified_width);
		_HANDLE_TRICLOPS_ERROR("triclopsSetResolution",e,camera,triclops);
	}

	/*   TriRectQlty_FAST,
   	TriRectQlty_STANDARD,
   	TriRectQlty_ENHANCED_1,
   	TriRectQlty_ENHANCED_2
	 */
	e = triclopsSetRectImgQuality(triclops, TriRectQlty_STANDARD);
	_HANDLE_TRICLOPS_ERROR("triclopsSetRectImgQuality",e,camera,triclops);

	nBufferSize = stereoCamera.nRows * stereoCamera.nCols * stereoCamera.nBytesPerPixel;

	pucDeInterlacedBuffer = new unsigned char[ nBufferSize ];

	// Color Image Capture Buffers
	pucRGBBuffer 	= new unsigned char[ 3 * nBufferSize ];
	pucRedBuffer	= new unsigned char[ nBufferSize ];
	pucGreenBuffer	= new unsigned char[ nBufferSize ];
	pucBlueBuffer	= new unsigned char[ nBufferSize ];
}

//=====================================
// Terminates the libbee camera context
//
void	libbbee_terminate_camera_context(void)
{
	// Cleanup the camera context
	libbee_cleanup_camera( camera );
}

//======================================================================================================================
// Captures a rectified image from Bumblebee2 camera, it must be selected wheter it is the leftmost or the rightmost one
//

void libbee_get_rectified_images(TriclopsColorImage* left, TriclopsColorImage* right, double *timestamp)
{
	// a TriclopsInput image
	TriclopsInput input;

	// allocate a buffer to hold the de-interleaved images
	unsigned char* pucRightRGB	= NULL;
	unsigned char* pucLeftRGB	= NULL;
	unsigned char* pucCenterRGB	= NULL;

	// B&W Image Capture Buffers
	unsigned char* pucRightMono	= NULL;
	unsigned char* pucLeftMono	= NULL;
	unsigned char* pucCenterMono	= NULL;

	if ( stereoCamera.bColor )
	{
		// get the images from the capture buffer and do all required processing
		// note: produces a TriclopsInput that can be used for stereo processing


		extractImagesColor( &stereoCamera,
				DC1394_BAYER_METHOD_NEAREST,
				pucDeInterlacedBuffer,
				pucRGBBuffer,
				pucRedBuffer,
				pucGreenBuffer,
				pucBlueBuffer,
				&pucRightRGB,
				&pucLeftRGB,
				&pucCenterRGB,
				&input );
	}
	else
	{
		// get the images from the capture buffer and do all required processing
		// note: produces a TriclopsInput that can be used for stereo processing
		extractImagesMono( &stereoCamera,
				pucDeInterlacedBuffer,
				&pucRightMono,
				&pucLeftMono,
				&pucCenterMono,
				&input );
	}

	// make sure we are in subpixel mode
	triclopsSetSubpixelInterpolation( triclops, 1 );
	_HANDLE_TRICLOPS_ERROR("triclopsSetSubpixelInterpolation",e,camera,triclops);

	// Get timestamp
	*timestamp = input.timeStamp.sec + input.timeStamp.u_sec/1000000.0;

	e = triclopsRectifyColorImage( triclops, TriCam_RIGHT, &input, right );
	_HANDLE_TRICLOPS_ERROR("triclopsRectifyPackedColorImage",e,camera,triclops);

	unsigned char * aux;
	aux = (unsigned char*)input.u.rgb.red;
	input.u.rgb.red = &aux[stereoCamera.nCols * stereoCamera.nRows];
	aux = (unsigned char*)input.u.rgb.green;
	input.u.rgb.green = &aux[stereoCamera.nCols * stereoCamera.nRows];
	aux = (unsigned char*)input.u.rgb.blue;
	input.u.rgb.blue = &aux[stereoCamera.nCols * stereoCamera.nRows];

	e = triclopsRectifyColorImage( triclops, TriCam_LEFT, &input, left );
	_HANDLE_TRICLOPS_ERROR("triclopsRectifyPackedColorImage",e,camera,triclops);
}

//======================================================================================================================
// Convert the image from the rgb buffer to the Triclops format
//

void CreateTriclopsImageFromBufferRGB(
		TriclopsColorImage *left,
		TriclopsColorImage *right,
		unsigned char *leftBuffer,
		unsigned char *rightBuffer)
{
	uint i = 0;

	// initialize image variables
	left->nrows = right->nrows = stereoCamera.nRows;
	left->ncols = right->ncols = stereoCamera.nCols;
	left->rowinc= right->rowinc = stereoCamera.nCols;

	// alloc left image data
	left-> red = (unsigned char*) malloc (left->nrows * left->ncols * sizeof(unsigned char));
	left-> green = (unsigned char*) malloc (left->nrows * left->ncols * sizeof(unsigned char));
	left-> blue = (unsigned char*) malloc (left->nrows * left->ncols * sizeof(unsigned char));
	// alloc right image data
	right-> red = (unsigned char*) malloc (right->nrows * right->ncols * sizeof(unsigned char));
	right-> green = (unsigned char*) malloc (right->nrows * right->ncols * sizeof(unsigned char));
	right-> blue = (unsigned char*) malloc (right->nrows * right->ncols * sizeof(unsigned char));

	// main loop: convert images from Buffer to TriclopsColorImage
	for(i = 0; i < (stereoCamera.nRows * stereoCamera.nCols); i++)
	{
		int p = 3 * i;

		left->red[i]   = leftBuffer[p];
		left->green[i] = leftBuffer[p + 1];
		left->blue[i]  = leftBuffer[p + 2];

		right->red[i]   = rightBuffer[p];
		right->green[i] = rightBuffer[p + 1];
		right->blue[i]  = rightBuffer[p + 2];
	}
}

//======================================================================================================================
// Convert the image from the mono buffer to the Triclops format
//

void CreateTriclopsImageFromBufferMono(
		TriclopsColorImage *left,
		TriclopsColorImage *right,
		unsigned char *leftBuffer,
		unsigned char *rightBuffer)
{
	uint i = 0;

	// initialize image variables
	left->nrows = right->nrows = stereoCamera.nRows;
	left->ncols = right->ncols = stereoCamera.nCols;
	left->rowinc= right->rowinc = stereoCamera.nCols;

	// alloc left image data
	left-> red = (unsigned char*) malloc (left->nrows * left->ncols * sizeof(unsigned char));
	left-> green = (unsigned char*) malloc (left->nrows * left->ncols * sizeof(unsigned char));
	left-> blue = (unsigned char*) malloc (left->nrows * left->ncols * sizeof(unsigned char));
	// alloc right image data
	right-> red = (unsigned char*) malloc (right->nrows * right->ncols * sizeof(unsigned char));
	right-> green = (unsigned char*) malloc (right->nrows * right->ncols * sizeof(unsigned char));
	right-> blue = (unsigned char*) malloc (right->nrows * right->ncols * sizeof(unsigned char));

	// main loop: convert images from Buffer to TriclopsColorImage
	for(i = 0; i < (stereoCamera.nRows * stereoCamera.nCols); i++)
	{
		left->red[i]   = leftBuffer[i];
		left->green[i] = leftBuffer[i];
		left->blue[i]  = leftBuffer[i];

		right->red[i]   = rightBuffer[i];
		right->green[i] = rightBuffer[i];
		right->blue[i]  = rightBuffer[i];
	}
}

//======================================================================================================================
// Captures a image from Bumblebee2 camera without rectify, it must
// be selected wheter it is the leftmost or the rightmost one
//

void libbee_get_raw_images(TriclopsColorImage* left, TriclopsColorImage* right, double *timestamp)
{
	// a TriclopsInput image
	TriclopsInput input;

	// allocate a buffer to hold the de-interleaved images
	unsigned char* pucRightRGB	= NULL;
	unsigned char* pucLeftRGB	= NULL;
	unsigned char* pucCenterRGB	= NULL;

	// B&W Image Capture Buffers
	unsigned char* pucRightMono	= NULL;
	unsigned char* pucLeftMono	= NULL;
	unsigned char* pucCenterMono	= NULL;

	if (stereoCamera.bColor)
	{
		// get the images from the capture buffer and do all required processing
		// note: produces a TriclopsInput that can be used for stereo processing

		extractImagesColor( &stereoCamera,
				DC1394_BAYER_METHOD_NEAREST,
				pucDeInterlacedBuffer,
				pucRGBBuffer,
				pucRedBuffer,
				pucGreenBuffer,
				pucBlueBuffer,
				&pucRightRGB,
				&pucLeftRGB,
				&pucCenterRGB,
				&input);

		CreateTriclopsImageFromBufferRGB(left, right, pucLeftRGB, pucRightRGB);
	}
	else
	{
		// get the images from the capture buffer and do all required processing
		// note: produces a TriclopsInput that can be used for stereo processing
		extractImagesMono( &stereoCamera,
				pucDeInterlacedBuffer,
				&pucRightMono,
				&pucLeftMono,
				&pucCenterMono,
				&input);

		CreateTriclopsImageFromBufferMono(left, right, pucLeftRGB, pucRightRGB);
	}

	// Get timestamp
	*timestamp = input.timeStamp.sec + input.timeStamp.u_sec/1000000.0;
}

//=====================================================================
// frees the alloc'ed TriclopsImage in libbee_get_rectified_images
//
void	libbee_destroy_flycapture_image(TriclopsColorImage *image)
{
	if(image)
		delete(image);
}

//=====================================
// Get the focal lenght (in pixels)
//
void getFocalLength(float *focal_lenght)
{
	triclopsGetFocalLength(triclops, focal_lenght);

}

//=====================================
// Get the camera baseline (in meters)
//
void getBaseline(float *baseline)
{
	triclopsGetBaseline(triclops, baseline);
}

//=====================================
// Get the camera center (in pixels)
//
void getImageCenter(float *crow, float *ccol)
{
	triclopsGetImageCenter(triclops, crow, ccol);
}

//=====================================
// Get the camera center (in pixels)
//
void getCameraResolution(int *nrows, int *ncols)
{
	triclopsGetResolution(triclops, nrows, ncols);
}

//======================================================================================================================
// Get calibration information
//

void libbee_get_calibration_information(float *focal_length, float *baseline, float *center_row, float *center_col, int *ncols, int *nrows)
{
	getBaseline(baseline);
	getFocalLength(focal_length);
	getImageCenter(center_row, center_col);
	getCameraResolution(nrows, ncols);
}
