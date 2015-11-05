#ifndef BLOBTRACKER_H_DEFINED
#define BLOBTRACKER_H_DEFINED

#include "Image/OpenCVUtil.H"

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include <vector>
#include "Util/sformat.H"

#ifdef HAVE_OPENCVAUX
#define MAX(a,b)  ((a) < (b) ? (b) : (a))
#include <opencv/cvaux.h>
#include <opencv/cvvidsurv.hpp>
#endif
#if (CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >= 2) || CV_MAJOR_VERSION>=3
#include <opencv2/legacy/blobtrack.hpp>
#include <opencv2/video/background_segm.hpp>
#endif

class Blob {
public:
  Point2D<int>  center;
  Dims          dims;
  int           id;
  double        confidence;
  std::string toString() { return sformat("[Blob Object: Center<%s>, Dims<%s>, Id %d, Conf %f]",convertToString(center).c_str(),convertToString(dims).c_str(),id,confidence); }
};

class BlobTracker : public ModelComponent
{
  public:
    BlobTracker(OptionManager& mgr,
       const std::string& descrName = "BlobTracker",
       const std::string& tagName = "BlobTracker");

    ~BlobTracker();

    void update(Image<PixRGB<byte> > img);

    std::vector<Blob> getBlobs() { return itsBlobs; }



  private:
    CvBlobTrackerAuto*      itsTracker;
    CvBlobTrackerAutoParam1 itsTrackerParams;
    std::vector<Blob>       itsBlobs;
};





#endif

