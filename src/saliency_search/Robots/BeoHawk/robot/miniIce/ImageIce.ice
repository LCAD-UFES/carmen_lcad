#ifndef IMAGE_ICE
#define IMAGE_ICE

module ImageIceMod{

  sequence<byte> ByteSeq;

  struct DimsIce {
    int w;
    int h;
  };

  struct SensorPose
  {
    int val;
    float weight;
    float decay;
  };

  enum SensorType { PATH, SALIENCY, PINGER, BARBWIRE };

  struct SensorVote
  {
    SensorType type; // sensor type
    SensorPose heading; // sensor's vote for absolute heading
    SensorPose depth; // sensor's vote for relative depth
  };

  struct Point2DIce
  {
    int i;
    int j;
  };

  struct Point3DIce
  {
    float x;
    float y;
    float z;
  };

  struct RectangleIce
  {
    Point2DIce tl; //the top left point
    Point2DIce br; //the bottom right point
  };

  struct QuadrilateralIce
  {
    Point2DIce tl; //top left
    Point2DIce tr; //top right
    Point2DIce bl; //bottom left
    Point2DIce br; //bottom right
    Point2DIce center;
    float ratio;   // aspect ratio
    float angle;   // angle of largest side    
  };

  struct LineIce
  {
    Point2DIce pt1;
    Point2DIce pt2;

    float angle;
  };

  struct ImageIce {
    ByteSeq data;
    int width;
    int height;
    int pixSize;
  };

  interface ImageShuttle {
    void transferImage(ImageIce i);
  };


  sequence<ImageIce> ImageIceSeq;
};

#endif
