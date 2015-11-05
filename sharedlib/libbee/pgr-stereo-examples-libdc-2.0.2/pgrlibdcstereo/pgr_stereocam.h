#ifndef PGR_STEREOCAM_H
#define PGR_STEREOCAM_H

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
// pgr_stereocam.h
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
#include "triclops.h"

// An enum type that itemizes the possible PGR Stereo cameras 
typedef enum
{
   UNKNOWN_CAMERA,
   BUMBLEBEE,
   BUMBLEBEE2,
   BUMBLEBEEXB3
} PGRStereoCameraModel_t;

// A structure that contains all information you need to access PGR Stereo
// cameras
typedef struct
{
      dc1394camera_t*		camera;
      PGRStereoCameraModel_t	model;
      dc1394color_filter_t	bayerTile;
      bool			bColor;
      unsigned int		nRows;
      unsigned int		nCols;
      unsigned int		nBytesPerPixel;
} PGRStereoCamera_t;


//=============================================================================
// Name: isStereoCamera
//
// Input:
//  camera      - The camera to be queried
//
// Return value:
//  A boolean value that is "true" if the camera is recognized as a PGR Stereo
//  camera and "false" otherwise.
//
// Description:
//   This function determines via checking the model string of the camera 
//   whether it is a known PGR stereo camera model.
//
//=============================================================================
bool
isStereoCamera( dc1394camera_t* camera );

//=============================================================================
// Name: getCameraModel
//
// Input:
//  camera      - The camera to be queried
//
// Return value:
//  Returns an enum value that defines which camera model this camera is.
//
//=============================================================================
PGRStereoCameraModel_t
getCameraModel( dc1394camera_t* camera );

//=============================================================================
// Name: queryStereoCamera
//
// Input:
//  camera       - The camera to be queried
//
// Output:
//  stereoCamera - a structure that contains all the required information 
//                 about this stereo camera
//
// Description:
//   This function queries information from the stereo camera and determines
//   information that is required to populate the PGRStereoCamera_t structure.
//   This includes information like what model and resolution the camera is and
//   whether the camera is color or B&W.
//
//=============================================================================
dc1394error_t
queryStereoCamera( dc1394camera_t* 	camera,
		   PGRStereoCamera_t *	stereoCamera );

//=============================================================================
// Name: setStereoVideoCapture
//
// Input:
//  stereoaCamera - The camera to grab from
//  legacyFirewire - supports 1394A
//
// Description:
//   This function figures out based on the model of the stereo camera what
//   format and mode the camera should be set up to transfer data in.
//
//   Note: currently always sets the maximum framerate.
//
//=============================================================================
dc1394error_t
setStereoVideoCapture( PGRStereoCamera_t* stereoCamera, bool is_legacy_firewire );

//=============================================================================
// Name: startTransmission
//
// Input:
//  stereoCamera - The camera to grab from  
//
// Description:
//   In the libdc1394 examples, there is a wait loop that waits after transmission
//   is requested until the transmission has successfully started.  This seemed 
//   like a nice thing to hide from every program so it is bundled up in here.
//=============================================================================
dc1394error_t
startTransmission( PGRStereoCamera_t* stereoCamera );

//=============================================================================
// Name: extractImagesColor
//
// Modified by "Jorcy de Oliveira Neto" for support color stereo rectification
//
//
// Input:
//  stereoCamera - The camera to grab from  
//  bayerMethod  - The requested method for performing bayer->color translations
//
// Output:
//  pucDeInterleaved - a buffer to hold the de-interleaved bayer images 
//                     size is (nrows * ncols * nimages)
//                     allocated outside this function
//  pucRGB           - a buffer to hold the RGB images - interleaved images 
//                     size is (nrows * ncols * nimages * 3)
//                     allocated outside this function
//  pucRed           - a buffer to hold the red channels of the de-inteleaved images
//                     size is (nrows * ncols * nimages)
//                     allocated outside this function
//  pucGreen         - a buffer to hold the green channels of the de-inteleaved images
//                     size is (nrows * ncols * nimages)
//                     allocated outside this function
//  pucBlue          - a buffer to hold the green channels of the de-inteleaved images
//                     size is (nrows * ncols * nimages)
//                     allocated outside this function
//  ppucRightRGB     - a pointer to the right RGB buffer (into pucRGB)
//                     size is (nrows * ncols * 3)
//                     points into pucRGB
//  ppucLeftRGB      - a pointer to the left RGB buffer 
//                     size is (nrows * ncols * 3)
//                     points into pucRGB
//  ppucCenterRGB    - a pointer to the center RGB buffer 
//                     size is (nrows * ncols * 3)
//                     points into pucRGB
//		       This is only valid for a BB XB3
//  pTriclopsInput   - the Triclops input 
//                     points into pucGreen
//
// Description:
//  A function to extract the grab buffer from the libdc1394 camera, and to 
//  perform the required de-interleaving and bayer color processing on the 
//  images to generate a left and right color image and a TriclopsInput for
//  stereo processing.
//=============================================================================
void
extractImagesColor( PGRStereoCamera_t* 	stereoCamera, 
		    dc1394bayer_method_t bayerMethod,
		    unsigned char* 	pucDeInterleaved,
		    unsigned char* 	pucRGB,
                    unsigned char* 	pucRed,
		    unsigned char* 	pucGreen,
                    unsigned char* 	pucBlue,
		    unsigned char** 	ppucRightRGB,
		    unsigned char** 	ppucLeftRGB,
		    unsigned char** 	ppucCenterRGB,
		    TriclopsInput*  	pTriclopsInput );

//=============================================================================
// Name: extractImagesMono
//
// Input:
//  stereoCamera - The camera to grab from  
//
// Output:
//  pucDeInterleaved - a buffer to hold the de-interleaved bayer images 
//                     size is (nrows * ncols * nimages)
//                     allocated outside this function
//  ppucRightMono8   - a pointer to the right RGB buffer (into pucRGB)
//                     size is (nrows * ncols * 3)
//                     points into pucRGB
//  ppucLeftMono8    - a pointer to the left RGB buffer 
//                     size is (nrows * ncols * 3)
//                     points into pucRGB
//  ppucCenterMono8  - a pointer to the center RGB buffer 
//                     size is (nrows * ncols * 3)
//                     points into pucRGB
//		       This is only valid for a BB XB3
//  pTriclopsInput   - the Triclops input 
//                     points into pucGreen
//
// Description:
//  A function to extract the grab buffer from the libdc1394 camera, and to 
//  perform the required de-interleaving on the images to generate a left 
//  and right mono image and a TriclopsInput for stereo processing.
//=============================================================================
void
extractImagesMono( PGRStereoCamera_t* 	stereoCamera, 
		   unsigned char* 	pucDeInterleaved,
		   unsigned char** 	ppucRightMono8,
		   unsigned char** 	ppucLeftMono8,
		   unsigned char** 	ppucCenterMono8,
		   TriclopsInput*  	pTriclopsInput );


//=============================================================================
// Name: getTriclopsContextFromCamera
//
// Input:
//  stereoCamera - The camera to get the TriclopsContext from
//
// Output:
//   pTriclops   - The retrieved TriclopsContext
//
// Description:
//  This function extracts the .cal calibration file from the camera, and writes
//  it to a file in /tmp.  It then loads the file back into a TriclopsContext.
// 
//  At the moment it is slow due to inefficient retrieval of the file from the
//  camera.
//
//  Also, the name of the file is hardcoded and is always the same.  This may
//  cause permission problems on multi-user systems.  Since code is provided,
//  however, please feel free to alter.
//=============================================================================
TriclopsError 
getTriclopsContextFromCamera( PGRStereoCamera_t* camera,
			      TriclopsContext* pTriclops );



void
extractImagesColorXB3( PGRStereoCamera_t* 	stereoCamera, 
		       dc1394bayer_method_t bayerMethod,
		       unsigned char* 	pucDeInterleaved,
		       unsigned char* 	pucRGB,
		       unsigned char* 	pucGreen,
		       unsigned char** 	ppucRightRGB,
		       unsigned char** 	ppucLeftRGB,
		       unsigned char** 	ppucCenterRGB,
		       TriclopsInput*  	pShortInput,
		       TriclopsInput* pWideInput );

void
extractImagesMonoXB3( PGRStereoCamera_t* 	stereoCamera, 
		   unsigned char* 	pucDeInterleaved,
		   unsigned char** 	ppucRightMono8,
		   unsigned char** 	ppucLeftMono8,
		   unsigned char** 	ppucCenterMono8,
		   TriclopsInput*  	pShortInput,
		      TriclopsInput* pWideInput );

#endif
  

