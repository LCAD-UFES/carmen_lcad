
#include "GUI/PressureMeter.H"
#include "Image/PixelsTypes.H"

PressureMeter::PressureMeter(int width, int height):
  itsWidth(width),
  itsHeight(height)
{
}

Image<PixRGB<byte> > PressureMeter::render(int p)
{
  itsPressureHist.push_front(p);
  if(itsPressureHist.size() > 10)
    itsPressureHist.pop_back();

  Image<PixRGB<byte> > pressureImage(itsWidth,itsHeight,ZEROS);

  drawCircle(pressureImage,Point2D<int>(itsWidth/2,itsHeight/2), .9*std::min(itsWidth,itsHeight)/2, PixRGB<byte>(0,0,255));
  int avgPressure = 0;

  std::list<int>::reverse_iterator it = itsPressureHist.rbegin();
  for(;it != itsPressureHist.rend(); ++it)
    {
      avgPressure += *it;
    }
  if(itsPressureHist.size() > 0)
    avgPressure /= itsPressureHist.size();

  float drawPercentage = (float)(avgPressure) / 5000.0;

  int x = (int)(.9*std::min(itsWidth,itsHeight)/2*cos((drawPercentage*360)*(M_PI/180))); //shift by 90 so that 0 is up
  int y = (int)(.9*std::min(itsWidth,itsHeight)/2*sin((drawPercentage*360)*(M_PI/180)));
  drawArrow(pressureImage, Point2D<int>(itsWidth/2,itsHeight/2), Point2D<int>(itsWidth/2+x,itsHeight/2+y), PixRGB<byte>(0,255,0));

  return pressureImage;
}
