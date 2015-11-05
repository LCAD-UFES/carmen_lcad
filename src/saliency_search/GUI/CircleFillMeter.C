
#include "GUI/CircleFillMeter.H"
#include "Image/PixelsTypes.H"

CircleFillMeter::CircleFillMeter(int width, int height, int max = 0):
  itsWidth(width),
  itsHeight(height),
  itsCurrentValue(0),
  itsMaxRadius(14),
  itsMaxValue(max)
{
}

Image<PixRGB<byte> > CircleFillMeter::render(short val)
{
  itsCurrentValue = val;
  //  float fillRad = (float)(val) / 20.0;

  Image<PixRGB<byte> > circleImage(itsWidth,itsHeight,ZEROS);

  drawCircle(circleImage,
             Point2D<int>(itsWidth/2,itsHeight/2),
             .9*std::min(itsWidth,itsHeight)/2,
             PixRGB<byte>(100,100,100));

  drawCircle(circleImage,
             Point2D<int>(itsWidth/2,itsHeight/2),
             .35*std::min(itsWidth,itsHeight)/2,
             PixRGB<byte>(100,100,100));

  float numLines = ((float)val / (float)itsMaxValue)*24.0;

  for(int i = 0; i < (int)numLines; i++)
    {
      int x1 = (int)(.9*std::min(itsWidth,itsHeight)/2*cos((15*i-90)*(M_PI/180))); //shift by 90 so that 0 is up
      int y1 = (int)(.9*std::min(itsWidth,itsHeight)/2*sin((15*i-90)*(M_PI/180)));

      int x2 = (int)(.35*std::min(itsWidth,itsHeight)/2*cos((15*i-90)*(M_PI/180))); //shift by 90 so that 0 is up
      int y2 = (int)(.35*std::min(itsWidth,itsHeight)/2*sin((15*i-90)*(M_PI/180)));

      drawLine(circleImage, Point2D<int>(itsWidth/2+x2,itsHeight/2+y2),
               Point2D<int>(itsWidth/2+x1,itsHeight/2+y1),
               PixRGB<byte>(0,255,0),1);
    }

  return circleImage;
}
