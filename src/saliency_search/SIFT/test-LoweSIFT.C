#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Media/FrameSeries.H"
#include "SIFT/LoweSIFT.H"
#include "Raster/Raster.H"
#include "Raster/GenericFrame.H"

int main(const int argc, const char **argv)
{
  ModelManager mgr("Test LoweSIFT");


  nub::ref<LoweSIFT> sift(new LoweSIFT(mgr));
  mgr.addSubComponent(sift);

  if(mgr.parseCommandLine(argc, argv, "Filename", 1, 1) == false) return 0;

  mgr.start();

  Image<PixRGB<byte> > img = Raster::ReadFrame(mgr.getExtraArg(0)).asRgb();

  sift->computeVO(img);
}
