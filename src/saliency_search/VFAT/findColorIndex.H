/*! @file VFAT/findColorIndex.H [put description here] */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/findColorIndex.H $
// $Id: findColorIndex.H 6182 2006-01-31 18:41:41Z rjpeters $

#include "Image/Pixels.H"

class findColorIndex
{
private:
public:
  findColorIndex();
  ~findColorIndex();
  //! returns one of 16 colors rainbow indexed
  void FACgetColor12(unsigned int *index, PixRGB<float> *pix);
};
