/*! @file VFAT/findColorIndex.C [put description here] */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/findColorIndex.C $
// $Id: findColorIndex.C 6182 2006-01-31 18:41:41Z rjpeters $

#include "VFAT/findColorIndex.H"
findColorIndex::findColorIndex()
{}

findColorIndex::~findColorIndex()
{}

void findColorIndex::FACgetColor12(unsigned int *index, PixRGB<float> *pix)
{
  pix->setRed(0.0F);
  pix->setGreen(0.0F);
  pix->setBlue(0.0F);

  switch(*index)
  {
  case 0:  pix->setRed(255.0F); break;
  case 1:  pix->setRed(255.0F); pix->setGreen(128.0F); break;
  case 2:  pix->setRed(255.0F); pix->setGreen(255.0F); break;
  case 3:  pix->setRed(128.0F); pix->setGreen(255.0F); break;
  case 4:  pix->setGreen(255.0F); break;
  case 5:  pix->setGreen(255.0F); pix->setBlue(128.0F); break;
  case 6:  pix->setGreen(255.0F); pix->setBlue(255.0F); break;
  case 7:  pix->setGreen(128.0F); pix->setBlue(255.0F); break;
  case 8:  pix->setBlue(255.0F); break;
  case 9: pix->setBlue(255.0F); pix->setRed(128.0F); break;
  case 10: pix->setBlue(255.0F); pix->setRed(255.0F); break;
  case 11: pix->setBlue(128.0F); pix->setRed(255.0F); break;
  default: pix->setRed(255.0F); pix->setGreen(255.0F); pix->setBlue(255.0F);
    break;
  }
};
