#include "Robots/SeaBeeIII/PixelizerI.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#define TILE_SIZE 17 // size, in pixels, for each square tile

#ifndef PIXELIZERI_C
#define PIXELIZERI_C

// ######################################################################
PixelizerI::PixelizerI(int id, OptionManager& mgr,
                       const std::string& descrName, const std::string& tagName) :
  VisionBrainComponentI(mgr, descrName, tagName)
{
}

PixelizerI::~PixelizerI()
{
}

void PixelizerI::registerTopics()
{

}

void PixelizerI::updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera)
{
  if(img.initialized())
    {
      int cols,rows = 0;
      rows = img.getHeight() / TILE_SIZE;
      cols = img.getWidth() / TILE_SIZE;

      for(int i = 0; i < rows; i++)
        {
          for(int j = 0; j < cols; j++)
            {
              PixRGB<byte> avg = getAvgTileColor(j*TILE_SIZE, i*TILE_SIZE, img);
              for(int k = 0; k < TILE_SIZE * TILE_SIZE; k++)
                {
                  img.setVal(j*TILE_SIZE+(k % TILE_SIZE), i*TILE_SIZE+(k / TILE_SIZE), avg);
                }
            }
        }

      itsOfs->writeRGB(img, "Pixelizer Image",
                       FrameInfo("Pixelizer", SRC_POS));
    }
  else
    LINFO("Received uninitialized image.");
}

// ######################################################################

PixRGB<byte> PixelizerI::getAvgTileColor(int colOffset,
                                         int rowOffset,
                                         const Image<PixRGB<byte> >& img)
{
  int avgR = 0;
  int avgG = 0;
  int avgB = 0;
  unsigned char r,g,b;

  for(int i = 0; i < TILE_SIZE; i++)
    {
      for(int j = 0; j < TILE_SIZE; j++)
        {
          PixRGB<byte> pixel = img.getVal(colOffset + j,rowOffset + i);
          pixel.getRGB(r,g,b);
          avgR += r;
          avgG += g;
          avgB += b;
        }
    }

  int totalTileSize = TILE_SIZE * TILE_SIZE;
  avgR /= totalTileSize;
  avgG /= totalTileSize;
  avgB /= totalTileSize;

  return PixRGB<byte>(8*(avgR/8), 8*(avgG/8), 8*(avgB/8));
}

#endif
