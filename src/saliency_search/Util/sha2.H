/*!@file Util/sha2.H general sha2 (256-bit) message-digest implementation */

// This code is from http://www.cr0.net:8040/code/crypto/sha256/, also
// under the GPL (see original copyright notice below).

/*
 *  FIPS-180-2 compliant SHA-256 implementation
 *
 *  Copyright (C) 2001-2003  Christophe Devine
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

//
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/sha2.H $
// $Id: sha2.H 5367 2005-08-22 21:45:50Z rjpeters $
//

#ifndef UTIL_SHA2_H_DEFINED
#define UTIL_SHA2_H_DEFINED

#include "Util/Types.H"

struct sha256_context
{
  uint32 total[2];
  uint32 state[8];
  byte buffer[64];
};

void sha256_starts( sha256_context* ctx );
void sha256_update( sha256_context* ctx, const byte* input, uint32 length );
void sha256_finish( sha256_context* ctx, byte digest[32] );


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_SHA2_H_DEFINED
