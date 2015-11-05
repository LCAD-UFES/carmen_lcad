//#include "mvog_model/cell.h"
#include "mvog_model/cell_array.h"

namespace MVOG
{

Cell::Cell()
{
  pVolumesCount = 0;
  nVolumesCount = 0;
}

Cell::~Cell()
{
  delete[] pVolumes;
  delete[] nVolumes;

  pVolumes = NULL;
  nVolumes = NULL;
}

void Cell::addPVolume(double bot, double top)
{
	addVolume(bot, top, pVolumes, pVolumesCount);
}

void Cell::addNVolume(double bot, double top)
{
	addVolume(bot, top, nVolumes, nVolumesCount);
}

void Cell::addVolume(double bot, double top, Volume*& volumes, int& volumesCount)
{
  Volume v(bot, top);
  float minGap = 0.0; // FIXME: gap only works for 0.0, - gap of 1.0 needs code for adj. mass

  // **** non-empty list: iterate over volumes
  for (int i = 0; i < volumesCount; i++)
  {
    // 1. below next volume, and does not intersect
    if (v.getTop() < volumes[i].getBot() - minGap)
    {
      // insert it at position i;
      Volume * newVolumes = new Volume[volumesCount+1];
      memcpy(newVolumes, volumes, i*VOLUME_BYTE_SIZE);
      newVolumes[i] = v;
      memcpy(newVolumes+i+1, volumes, (volumesCount-i)*VOLUME_BYTE_SIZE);
      delete[] volumes;
      volumes = newVolumes;
      volumesCount++;

      return;
    }
    // 2. intersects with next volume
    else if (v.getBot() <= volumes[i].getTop() + minGap)
    {
      // get all volumes that v intersects with
      int stopIndex  = i+1;

      int c = i+1;
      while(c < volumesCount )
      {
        if (v.getTop() >= volumes[c].getBot() - minGap)
        {
           c++;
           stopIndex = c;
        }
        else break;
      }
      // merge with new volume

      volumes[i].setBot(std::min(volumes[i].getBot(), v.getBot()));
      volumes[i].setTop(std::max(volumes[i].getTop(), v.getTop()));
      volumes[i].setMass(volumes[i].getMass() + v.getMass()); // + FIXME: gap mass??

      // merge with rest
      volumes[i].setTop(std::max(volumes[i].getTop(), volumes[stopIndex - 1].getTop()));

      for (size_t t = i+1; t < stopIndex ; t++)
         volumes[i].setMass(volumes[i].getMass() + volumes[t].getMass()); // + FIXME: gap mass??

      // erase old volumes that were merged
      //printf ("erasing from %d to %d\n", i+1, stopIndex);
      //volumes.erase(volumes.begin() + i+1, volumes.begin() + stopIndex);

      Volume * newVolumes = new Volume[volumesCount - (stopIndex - i) + 1];
      memcpy(newVolumes+0, volumes, (i+1)*VOLUME_BYTE_SIZE);
      memcpy(newVolumes+i+1, volumes + stopIndex, (volumesCount - stopIndex)*VOLUME_BYTE_SIZE);
      delete[] volumes;
      volumes = newVolumes;
      volumesCount = volumesCount - (stopIndex - i) + 1;

      return;
    }
  }

  // insert it at position i;

  Volume * newVolumes = new Volume[volumesCount+1];
  memcpy(newVolumes, volumes, volumesCount*VOLUME_BYTE_SIZE);
  newVolumes[volumesCount] = v;
  delete[] volumes;
  volumes = newVolumes;

  volumesCount++;
}

void Cell::printPVolumes()
{
  printf("*** CELL (+) %d ****\n", pVolumesCount);

  for (int i = 0; i < pVolumesCount; i++)
    printf("\t%d [%f, %f] [%f]\n", i, pVolumes[i].getBot(), pVolumes[i].getTop(), pVolumes[i].getMass());
}

}
