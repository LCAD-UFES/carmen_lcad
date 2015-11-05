/*!@file Util/jenkinshash.H 32-bit hash functions (NOT CRYPTOGRAPHIC!) */

// This code in the corresponding .C file is almost verbatim from
// http://burtleburtle.net/bob/c/lookup2.c (see original public domain
// notice below) with only trivial modifications. The prototypes here
// represent wrappings of those original functions from lookup2.c, and
// the comments are from the lookup2.c.

//
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/jenkinshash.H $
// $Id: jenkinshash.H 5359 2005-08-20 01:16:05Z rjpeters $
//

#ifndef UTIL_JENKINSHASH_H_DEFINED
#define UTIL_JENKINSHASH_H_DEFINED

#include "Util/Types.H"

//! hash a variable-length key into a 32-bit value
/*!
  k     : the key (the unaligned variable-length array of bytes)
  len   : the length of the key, counting by bytes
  level : can be any 4-byte value
Returns a 32-bit value.  Every bit of the key affects every bit of
the return value.  Every 1-bit and 2-bit delta achieves avalanche.
About 36+6len instructions.

The best hash table sizes are powers of 2.  There is no need to do
mod a prime (mod is sooo slow!).  If you need less than 32 bits,
use a bitmask.  For example, if you need only 10 bits, do
  h = (h & hashmask(10));
In which case, the hash table should have hashsize(10) elements.

If you are hashing n strings (ub1 **)k, do it like this:
  for (i=0, h=0; i<n; ++i) h = hash( k[i], len[i], h);

By Bob Jenkins, 1996.  bob_jenkins@burtleburtle.net.  You may use this
code any way you wish, private, educational, or commercial.  It's free.

See http://burlteburtle.net/bob/hash/evahash.html
Use for hash table lookup, or anything where one collision in 2^32 is
acceptable.  Do NOT use for cryptographic purposes.
 */
uint32 jenkinshash(const byte* key, uint32 length, uint32 initval = 0);

//! Hash a variable-length key into a 32-bit value
/*!
 This works on all machines.  jenkinshash2() is identical to jenkinshash() on
 little-endian machines, except that the length has to be measured
 in ub4s instead of bytes.  It is much faster than hash().  It
 requires
 -- that the key be an array of ub4's, and
 -- that all your machines have the same endianness, and
 -- that the length be the number of ub4's in the key
*/
uint32 jenkinshash2(const uint32* key, uint32 length, uint32 initval = 0);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_JENKINSHASH_H_DEFINED
