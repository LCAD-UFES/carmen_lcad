#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameIstream.H"
#include "RCBot/Motion/MotionEnergy.H"
#include "Image/ColorOps.H"
#include "Gist/SuperPixel.H"
#include "Image/ShapeOps.H"
#include "Image/Kernels.H"
#include "Image/Convolver.H"

#include "Image/LowPass.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/LowPass.H"

int main(int argc, char **argv)
{

  // instantiate a model manager:
  ModelManager manager("Test Motion Segment");

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  manager.exportOptions(MC_RECURSE);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "" "", 0, 0) == false) return(1);

  // let's do it!
  manager.start();


  int magthresh = 150;
  int pyrlevel = 1;
  MotionEnergyPyrBuilder<byte> motionPyr(Gaussian5, 0.0f, 10.0f, 3,
                                         magthresh);
  

  ifs->updateNext();
  Image<PixRGB<byte> > ima = ifs->readRGB();
  while(ima.initialized())
  {
    // Calculate horizontal and vertical motion vectors using a temporal sobel filter
    Image<byte> lum = luminance(ima);
    motionPyr.updateMotion(lum, pyrlevel+1);
    Image<float> vMotion = motionPyr.buildVerticalMotionLevel(pyrlevel);
    Image<float> hMotion = motionPyr.buildHorizontalMotionLevel(pyrlevel);
    Image<float> motionDir(vMotion.getDims(), NO_INIT);
    Image<float> motionMag(vMotion.getDims(), NO_INIT);

    // Calculate the motion magnitude and direction from the vertical and
    // horizontal motion vectors
    for(size_t i=0; i<vMotion.size(); i++)
    {
      motionDir.setVal(i, atan2(vMotion.getVal(i), hMotion.getVal(i)));
      motionMag.setVal(i, sqrt(pow(vMotion.getVal(i),2)+ pow(hMotion.getVal(i),2)));
    }
    motionDir = rescaleBilinear(motionDir, ima.getDims());
    motionMag = rescaleBilinear(motionMag, ima.getDims());

    // Blur the motion magnitude and direction like crazy
    Image<float> motionGaussian = gaussian<float>(0.0F, 8.0F, 0, 1.0F);
    motionDir = sepFilter(motionDir, motionGaussian, motionGaussian, CONV_BOUNDARY_REPLICATE);
    motionMag = sepFilter(motionMag, motionGaussian, motionGaussian, CONV_BOUNDARY_REPLICATE);
    motionDir *= motionMag;

    motionDir *= 200.0;
    motionMag *= 200.0;

    Image<float> motionDirNorm = motionDir; 
    Image<float> motionMagNorm = motionMag; 

    // Create a fake RGB image using the motion direction and magnitude as channels
    Image<PixRGB<byte> > motionFeatures(motionDir.getDims(), NO_INIT);
    for(size_t i=0; i<motionFeatures.size(); i++)
    {
      PixRGB<byte> features(motionDirNorm.getVal(i), motionMagNorm.getVal(i), 0);
      motionFeatures.setVal(i, features);
    }

    // Segment the fake RGB image using SuperPixel
    std::vector<std::vector<Point2D<int> > > groups;
    int num_ccs = 0;
    float sigma = .4;
    int k=1500;
    int minSize=600;
    Image<int> groupImage = SuperPixelSegment(motionFeatures, sigma, k, minSize, num_ccs, &groups);

    
    // Find the groups with high motion energy, and draw a convex hull around them
    Image<PixRGB<byte> > outputImage = ima;
    for(size_t i=0; i<groups.size(); i++)
    {
      double totalMotion = 0;
      std::vector<Point2D<float> > floatPoly;
      for(size_t j=0; j<groups[i].size(); j++)
      {
        totalMotion += motionMag.getVal(groups[i][j]);
        Point2D<float> p;
        p.i = groups[i][j].i;
        p.j = groups[i][j].j;
        floatPoly.push_back(p);
      }

      if(totalMotion/double(groups[i].size()) < 5.0) continue;

      std::vector<Point2D<float> > poly = approximateHull(floatPoly, 30);
      std::vector<Point2D<int> > intPoly;
      for(size_t j=0; j<poly.size(); j++)
      {
        Point2D<int> p;
        p.i = poly[j].i;
        p.j = poly[j].j;
        intPoly.push_back(p);
      }
      drawOutlinedPolygon(outputImage, intPoly, PixRGB<byte>(255,0,0));
    }

    ofs->writeRGB(motionFeatures, "Features");
    ofs->writeRGB(SuperPixelDebugImage(groups, ima), "SuperPixel");
    ofs->writeRGB(motionDir, "Motion Direction");
    ofs->writeRGB(motionMag, "Motion Magnitude");
    ofs->writeRGB(outputImage, "SuperPixel Segmentation");
    ofs->updateNext();
    ima = ifs->readRGB();
  }

  return 0;
}
