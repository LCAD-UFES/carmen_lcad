//=============================================================================
// Copyright © 2004 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research, Inc. (PGR).
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: example1.cpp,v 1.9 2004/01/20 00:39:16 mwhite Exp $
//=============================================================================
//=============================================================================
// Example 1:
//
// Loads three stereo images from a PPM file and performs stereo processing 
// to create a disparity image which is then output to depth.pgm.
//
// This program assumes there is a camera calibration file in the same directory
// named 'config'.
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>

//=============================================================================
// PGR Includes
//=============================================================================
#include "triclops.h"
#include "pnmutils.h"

//=============================================================================
// Project Includes
//=============================================================================




#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \



int
main( int /* argc */, char** /* argv */ )
{
   TriclopsContext     context;
   TriclopsImage       depthImage;
   TriclopsInput       inputData;
   TriclopsError       error;
   
   // get the camera module configuration
   error = triclopsGetDefaultContextFromFile( &context, "config" );
   _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", error );
   if ( error != TriclopsErrorOk )
   {
      printf( "Can't open calibration file 'config'\n" );
      exit( 1 );
   }
   
   // Load images from file
   TriclopsBool bErr = ppmReadToTriclopsInput( "input.ppm", &inputData );
   if( !bErr )
   {
      printf( "ppmReadToTriclopsInput() failed. Can't find input.ppm?\n" );
      exit( 1 );
   }
   
   // set up some stereo parameters:
   // set to 320x240 output images
   error = triclopsSetResolution( context, 240, 320 );
   _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", error );
   // set disparity range
   error = triclopsSetDisparity( context, 5, 60 );
   _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", error );
   
   // set the display mapping
   // note: disparity mapping corrupts the disparity values so that making
   // distance measurements is more difficult and less accurate.
   // Do not use it when you intend to actually use disparity values for
   // purposes other than display
   error = triclopsSetDisparityMapping( context, 128, 255 );
   _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparityMapping()", error );
   error = triclopsSetDisparityMappingOn( context, 1 );
   _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparityMappingOn()", error );
   // set the validation mappings to 0 (black)
   error = triclopsSetUniquenessValidationMapping( context, 0 );
   _HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidationMapping()", error );
   error = triclopsSetTextureValidationMapping( context, 0 );
   _HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidationMapping()", error );
   
   // Preprocessing the images
   error = triclopsPreprocess( context, &inputData );
   _HANDLE_TRICLOPS_ERROR( "triclopsPreprocess()", error );
     
   // stereo processing
   error =  triclopsStereo( context );
   _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", error );
   
   // retrieve the depth image from the context
   error = triclopsGetImage( context, TriImg_DISPARITY, TriCam_REFERENCE, &depthImage );
   _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", error );
   
   // save the depth image
   error = triclopsSaveImage( &depthImage, "depth.pgm" );
   _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", error );
   
   
   // clean up memory allocated in context
   freeInput( &inputData );
   error = triclopsDestroyContext( context );
   
   return 0;
}
