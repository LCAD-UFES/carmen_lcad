/*!@file Image/AffineTransform.C */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file:Farhan Baluch
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/AffineTransform.C $
// $Id: AffineTransform.C 10794 2009-02-08 06:21:09Z itti $
//


#ifndef IMAGE_AFFINETRANSFORM_C_DEFINED
#define IMAGE_AFFINETRANSFORM_C_DEFINED

#include "Image/AffineTransform.H"
#include "Image/MatrixOps.H"
#include "Image/LinearAlgebra.H"
#include <fstream>
#include <iostream>
#include "Image/IO.H"

AffineTransform::AffineTransform() {}
// AffineTransform::~AffineTransform() {}

Image<double> AffineTransform::computeTransform(const CalibrationTransform::Data& d)
{
//loop through calib pts work out the transform M = pInv(e) x S
//set itsTransform= new transform image

    const std::vector<CalPt>& itsCals = d.getItsCalPts();
    int numCalPts = (int)itsCals.size();
    Image<double> scr(3,numCalPts,ZEROS);
    Image<double> eye(3,numCalPts,ZEROS);
    Image<double> eyeInv;
    Image<double> M(3,3,ZEROS);

    Image<double>::iterator scrAptr = scr.beginw();
    Image<double>::iterator eyeAptr = eye.beginw();

    for(int i=0; i < numCalPts; i++)
    {
        *scrAptr++ = itsCals[i].scr_x;
        *scrAptr++ = itsCals[i].scr_y;
        *scrAptr++ = 1;

        *eyeAptr++ = itsCals[i].raw_x;
        *eyeAptr++ = itsCals[i].raw_y;
        *eyeAptr++ = 1;
    }

    int rank =0;
    eyeInv = svdPseudoInv(eye,SVD_LAPACK,&rank,0.0F);

    M = matrixMult(eyeInv,scr);
    itsTransform = M;

    return itsTransform;

}



Point2D<double> AffineTransform::getCalibrated(const Point2D<double>& raw)
{
    Image<double> rawIm(3,1,ZEROS);
    Image<double> resultIm(3,1,ZEROS);
    Point2D<double> calibPt;

    Image<double>::iterator Aptr = rawIm.beginw();

    *Aptr++ = raw.i;   //x_raw
    *Aptr++ = raw.j;   //y_raw
    *Aptr++ = 1;

    resultIm = matrixMult(rawIm,itsTransform);

    Aptr = resultIm.beginw();

    calibPt.i = *Aptr++;
    calibPt.j = *Aptr;

    return calibPt;
}


Image<double> AffineTransform::getTransform()
{
    return itsTransform;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_AFFINETRANSFORM_C_DEFINED

