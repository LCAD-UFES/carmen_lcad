
#include "GUI/CompassMeter.H"
#include "Image/PixelsTypes.H"

CompassMeter::CompassMeter(int width, int height):
  itsWidth(width),
  itsHeight(height),
  itsCurrentHeading(0),
  itsDesiredHeading(0)
{
}

Image<PixRGB<byte> > CompassMeter::render(short h)
{
  itsCurrentHeading = h;

  itsHeadingHist.push_front(h);
  if(itsHeadingHist.size() > 10)
    itsHeadingHist.pop_back();

  Image<PixRGB<byte> > compassImage(itsWidth,itsHeight,ZEROS);

  drawCircle(compassImage,Point2D<int>(itsWidth/2,itsHeight/2), .9*std::min(itsWidth,itsHeight)/2, PixRGB<byte>(255,0,0));

  std::list<short>::reverse_iterator it = itsHeadingHist.rbegin();
  float opacity=0;
  float opacity_step =1/float(itsHeadingHist.size());
  for(;it != itsHeadingHist.rend(); ++it)
    {
      //LINFO("Opacity:%f, Step: %f", opacity, opacity_step);
      int x = (int)(.9*std::min(itsWidth,itsHeight)/2*cos((*it-90)*(M_PI/180))); //shift by 90 so that 0 is up
      int y = (int)(.9*std::min(itsWidth,itsHeight)/2*sin((*it-90)*(M_PI/180)));
      drawArrow(compassImage,
                Point2D<int>(itsWidth/2,itsHeight/2),
                Point2D<int>(itsWidth/2+x,itsHeight/2+y),
                PixRGB<byte>(0,255*opacity*.5,255*opacity*.5));
      opacity+=opacity_step;
    }

  int x = (int)(.9*std::min(itsWidth,itsHeight)/2*cos((*(itsHeadingHist.begin())-90)*(M_PI/180))); //shift by 90 so that 0 is up
  int y = (int)(.9*std::min(itsWidth,itsHeight)/2*sin((*(itsHeadingHist.begin())-90)*(M_PI/180)));
  drawArrow(compassImage, Point2D<int>(itsWidth/2,itsHeight/2), Point2D<int>(itsWidth/2+x,itsHeight/2+y), PixRGB<byte>(0,255,0));

  return compassImage;
}
