/*! @file VFAT/readMatrix.H [put description here] */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/readMatrix.H $
// $Id: readMatrix.H 6182 2006-01-31 18:41:41Z rjpeters $

/*! this class will read in the file fileName as a standard whitespace
  delimited file and put it into a matrix. The matrix can be retrieved
  in either a double nested std::vector<float> or an Image<float>. The input
  file must state the dimensions on the first line. Comments may be
  placed using # charactors
*/

#include "Image/Image.H"
#include <vector>

class readMatrix
{
private:
  int sizeX, sizeY;
  std::vector<std::vector<double> > vectorIn;
  Image<float> outImage;
public:
  readMatrix(const char* fileName);
  ~readMatrix();
  std::vector<std::vector<double> >  returnMatrix();
  Image<float> returnMatrixAsImage();
  void echoMatrix();
};


