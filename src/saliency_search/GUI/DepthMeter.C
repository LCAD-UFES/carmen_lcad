
#include "GUI/DepthMeter.H"
#include "Image/PixelsTypes.H"
#include "Util/StringConversions.H"

DepthMeter::DepthMeter(int width, int height):
  itsWidth(width),
  itsHeight(height),
  itsCurrentDepth(0)
{

}

Image<PixRGB<byte> > DepthMeter::render(int d)
{
  itsCurrentDepth = d;
  itsDepthHist.push_front(d);
  if(itsDepthHist.size() > 5)
    itsDepthHist.pop_back();

  int avgDepth = 0;

  std::list<int>::reverse_iterator it = itsDepthHist.rbegin();
  for(;it != itsDepthHist.rend(); ++it)
    {
      avgDepth += *it;
    }
  if(itsDepthHist.size() > 0)
    avgDepth /= itsDepthHist.size();

  int minDrawDepth = avgDepth - itsHeight/2;

  Image<PixRGB<byte> > depthImage(itsWidth,itsHeight,ZEROS);

  int drawHeight = 0;

  // draw tick marks
  while(drawHeight < itsHeight)
    {
      if((minDrawDepth %100 ) % 13 == 0)
        {
          drawLine(depthImage,
                   Point2D<int>((int)(.38*(float)itsWidth),drawHeight),
                   Point2D<int>((int)(.62*(float)itsWidth),drawHeight),
                   PixRGB<byte>(100,100,100));
        }

      if(minDrawDepth % 25 == 0)
        {
          if(minDrawDepth % 100 == 0 ||
             minDrawDepth % 100 == 50)
            {
              drawLine(depthImage, Point2D<int>((int)(.1*(float)itsWidth),drawHeight),
                       Point2D<int>((int)(.9*(float)itsWidth),drawHeight),
                       PixRGB<byte>(200,200,200));
            }
          else if(minDrawDepth % 100 == 25 ||
                  minDrawDepth % 100 == 75)
            {
              writeText(depthImage,
                        Point2D<int>(0,drawHeight-12),
                        toStr<int>(minDrawDepth / 100).c_str(),
                        PixRGB<byte>(20,253,0),
                        PixRGB<byte>(0,0,0),
                        SimpleFont::FIXED(14));

              writeText(depthImage,
                        Point2D<int>((int)(.75*(float)itsWidth + 8),drawHeight-6),
                        toStr<int>(minDrawDepth % 100).c_str(),
                        PixRGB<byte>(20,253,0),
                        PixRGB<byte>(0,0,0),
                        SimpleFont::FIXED(8));

              drawLine(depthImage,
                       Point2D<int>((int)(.25*(float)itsWidth),drawHeight),
                       Point2D<int>((int)(.75*(float)itsWidth),drawHeight),
                       PixRGB<byte>(100,100,100));
            }

          if(drawHeight + 13 < itsHeight)
            {
              drawLine(depthImage,
                       Point2D<int>((int)(.38*(float)itsWidth),drawHeight + 13),
                       Point2D<int>((int)(.62*(float)itsWidth),drawHeight+13),
                       PixRGB<byte>(100,100,100));
            }

          drawHeight += 25;
          minDrawDepth += 25;
        }
      else
        {
          drawHeight++;
          minDrawDepth++;
        }
    }

  // draw current depth line as horizontal over middle of image
  drawLine(depthImage, Point2D<int>(0,(itsHeight/2-2)), Point2D<int>(itsWidth,(itsHeight/2 - 2)), PixRGB<byte>(255,0,0));
  drawLine(depthImage, Point2D<int>(0,(itsHeight/2+2)), Point2D<int>(itsWidth,(itsHeight/2 + 2)), PixRGB<byte>(255,0,0));

  return depthImage;
}

