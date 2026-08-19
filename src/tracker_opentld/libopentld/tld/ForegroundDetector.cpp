/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/
/*
 * ForegroundDetector.cpp
 *
 *  Created on: Nov 16, 2011
 *      Author: Georg Nebehay
 */

#include "ForegroundDetector.h"

#include "BlobResult.h"
/* Migração Ubuntu 26.04: símbolos da API C do OpenCV (IplImage, cvScalar,
   CV_FONT_*, cvDestroyAllWindows, ...) continuam existindo no OpenCV 4, mas só nestes
   headers *_c.h — antes chegavam por inclusão transitiva do opencv/cv.h. */
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>

using namespace cv;

namespace tld
{

ForegroundDetector::ForegroundDetector()
{
    fgThreshold = 16;
    minBlobSize = 0;
}

ForegroundDetector::~ForegroundDetector()
{
}

void ForegroundDetector::release()
{
}

void ForegroundDetector::nextIteration(const Mat &img)
{
    if(bgImg.empty())
    {
        return;
    }

    Mat absImg = Mat(img.cols, img.rows, img.type());
    Mat threshImg = Mat(img.cols, img.rows, img.type());

    absdiff(bgImg, img, absImg);
    threshold(absImg, threshImg, fgThreshold, 255, CV_THRESH_BINARY);

#if CV_MAJOR_VERSION >= 4
    /* Migração Ubuntu 26.04: o OpenCV 4 removeu o operador de conversão implícito
       cv::Mat -> IplImage; o substituto oficial é cvIplImage() (mesma semântica: header
       IplImage apontando para o buffer da Mat, sem cópia). */
    IplImage im = cvIplImage(threshImg);
#else
    IplImage im = (IplImage)threshImg;
#endif
    CBlobResult blobs = CBlobResult(&im, NULL, 0);

    blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, minBlobSize);

    std::vector<Rect>* fgList = detectionResult->fgList;
    fgList->clear();

    for(int i = 0; i < blobs.GetNumBlobs(); i++)
    {
        CBlob *blob = blobs.GetBlob(i);
        CvRect rect = blob->GetBoundingBox();
        fgList->push_back(rect);
    }

}

bool ForegroundDetector::isActive()
{
    return !bgImg.empty();
}

} /* namespace tld */
