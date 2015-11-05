//=============================================================================
// Copyright © 2000 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: pnmutilsTest.cpp,v 1.2 2007/07/10 16:27:27 demos Exp $
//=============================================================================
//=============================================================================
// 
//=============================================================================


//=============================================================================
// Project Includes
//=============================================================================
#include "pnmutils.h"




int main( int argc, char** argv )
{
   // Usage: pnmutilsTest <file>
   
   if ( argc != 3 )
   {
      printf( "Usage: pnmutilsTest [PGM|PGM16|PPM] <image file>\n" );
      return 0;
   }

   if ( !strcmp( argv[1], "PGM" ) )
   {
      // try to read it as a triclops image
      TriclopsImage image;
      if ( !pgmReadToTriclopsImage( argv[2], &image ) )
      {
	 printf( "Failed to read '%s' in as an 8 bit triclops image\n",
		 argv[2] );
      }
      else
      {
	 if ( !pgmWriteFromTriclopsImage( "triclopsimage.pgm", &image ) )
	    printf( "Failed to write 8 bit triclops image\n" );
	 else
	    printf( "Wrote 8 bit triclops image\n" );
      }
   }
   else if ( !strcmp( argv[1], "PGM16" ) )
   {
      // try to read it as a triclops image
      TriclopsImage16 image16;
      if ( !pgmReadToTriclopsImage16( argv[2], &image16 ) )
      {
	 printf( "Failed to read '%s' in as an 16 bit triclops image\n",
		 argv[2] );
      }
      else
      {
	 if ( !pgmWriteFromTriclopsImage16( "triclopsimage16.pgm", &image16 ) )
	    printf( "Failed to write 16 bit triclops image\n" );
	 else
	    printf( "Wrote 16 bit triclops image\n" );
      }
   }
   else if ( !strcmp( argv[1], "PPM" ) )
   {
      // try to read it as a triclops input
      TriclopsInput input;
      if ( !ppmReadToTriclopsInput( argv[2], &input ) )
      {
	 printf( "Failed to read '%s' in as triclops input\n",
		 argv[2] );
      }
      else
      {
	 if ( !ppmWriteFromTriclopsInput( "triclopsinput.ppm", &input ) )
	    printf( "Failed to write triclops input\n" );
	 else
	    printf( "Wrote triclops input\n" );
      }
   }

   return 0;
}


//=============================================================================
// $Log: pnmutilsTest.cpp,v $
// Revision 1.2  2007/07/10 16:27:27  demos
// [1] Missing "int" type before main().  This was failing the VS2005 release compilation.
//
// Revision 1.1  2001/01/03 20:20:41  donm
// moved to PGRImage library
//
// Revision 1.2  2000/08/22 19:12:30  mwhite
// [1]  Fixed compiler error
//
// Revision 1.1  2000/07/25 00:04:40  mwhite
// [1]  Merged changes made to the triclops/examples version of
// the pnmutils into this version, which should now be considered
// the working development version.  The triclops/examples version
// does not include the timestamped comments.
//
//=============================================================================
