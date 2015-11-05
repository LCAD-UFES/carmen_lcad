#include "SIFT/LoweSIFT.H"
#include "rutz/pipe.h"
#include "Image/ColorOps.H"

const ModelOptionCateg MOC_LoweSIFT = {
  MOC_SORTPRI_3, "Official David Lowe SIFT related options" };

const ModelOptionDef OPT_ExeName = 
{ MODOPT_ARG(std::string), "ExeName", &MOC_LoweSIFT, OPTEXP_CORE,
  "Path to the official David Lowe SIFT binary",
  "lowesift-exe", '\0', "path/to/sift", "sift" };

// ######################################################################
LoweSIFT::LoweSIFT(OptionManager& mgr, 
    std::string const & descrName,
    std::string const & tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsExePath(&OPT_ExeName, this)
{ }

// ######################################################################
  std::vector<rutz::shared_ptr<Keypoint> >
LoweSIFT::computeKeypoints(Image<PixRGB<byte> > const & img)
{
  if(!img.initialized()) LFATAL("Uninitialized Image!");

  Image<byte> imgGray = luminance(img);

  rutz::bidir_pipe siftExe(itsExePath.getVal().c_str(), NULL);

  // Output the image to the sift exe in binary PGM format
  siftExe.out_stream() << "P5\n";
  siftExe.out_stream() << imgGray.getWidth() << " " << imgGray.getHeight() << "\n";
  siftExe.out_stream() << "255\n";
  for(Image<byte>::const_iterator imgIt = imgGray.begin(); imgIt != imgGray.end(); ++imgIt)
  {
    siftExe.out_stream() << *imgIt;
  }

  // Close the output stream so the binary knows the file is done
  siftExe.close_out();

  // Read the total number of keypoints
  int numKeypoints;
  siftExe.in_stream() >> numKeypoints;

  // Read the length of each keypoint vector
  int keypointLength;
  siftExe.in_stream() >> keypointLength;

  // Loop through all the keypoints and read them in
  LDEBUG("Loading %d keypoints of length %d", numKeypoints, keypointLength);
  std::vector<rutz::shared_ptr<Keypoint> > keypoints;
  for(int keyIdx=0; keyIdx < numKeypoints; ++keyIdx)
  {
    // Read in the kp meta data
    float row, column, scale, orientation;
    siftExe.in_stream() >> row >> column >> scale >> orientation;

    // Read in the actual feature vector
    std::vector<byte> features(keypointLength);
    for(int dimenIdx=0; dimenIdx<keypointLength; ++dimenIdx)
    {
      byte feature;
      siftExe.in_stream() >> feature;
      features[dimenIdx] = feature;
    }

    // Create the keypoint
    keypoints.push_back(rutz::shared_ptr<Keypoint>(
          new Keypoint(features, column, row, scale, orientation, 1)));
  }

  return keypoints;
}
