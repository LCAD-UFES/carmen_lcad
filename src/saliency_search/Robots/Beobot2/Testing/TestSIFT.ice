

module TestSIFT
{
  const int DEFAULTWORKERPORT = 10000;

  sequence<byte> byteSequence;
  struct keypoint
  {
    float x;
    float y;
    float s;
    float o;
    float m;
    byteSequence oriFV;
  };



  sequence<keypoint> keypointSequence;
  sequence<double> idSequence;

  interface SIFTMatcher
  {
    idSequence matchKeypoints(keypointSequence keypoints);
  };
};
