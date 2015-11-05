#include "mvog_model/cell_vector.h"

namespace MVOG
{

Cell::Cell()
{


}

Cell::~Cell()
{


}

void Cell::addPVolume(double bot, double top)
{
	addVolume(bot, top, pVolumes);
}

void Cell::addNVolume(double bot, double top)
{
	addVolume(bot, top, nVolumes);
}

void Cell::addVolume(double bot, double top, VolumeVector& volumes)
{
  Volume v(bot, top);
  float minGap = 0.0; // FIXME: gap only works for 0.0, - gap of 1.0 needs code for adj. mass

  // **** non-empty list: iterate over volumes
  for (size_t i = 0; i < volumes.size(); i++)
  {
    // 1. below next volume, and does not intersect
    if (v.getTop() < volumes[i].getBot() - minGap)
    {
      // insert it at position i;
      volumes.insert(volumes.begin() + i, v);
      return;
    }
    // 2. intersects with next volume
    else if (v.getBot() <= volumes[i].getTop() + minGap)
    {
      // get all volumes that v intersects with
      unsigned int stopIndex  = i+1;

      unsigned int c = i+1;
      while(c < volumes.size() )
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
      volumes.erase(volumes.begin() + i+1, volumes.begin() + stopIndex);

      return;
    }
  }

  volumes.push_back(v);
}

void Cell::printPVolumes()
{
  printf("-------------\n");

  for (size_t i = 0; i < pVolumes.size(); i++)
    printf("\t%d [%f, %f] [%f]\n", (int)i, pVolumes[i].getBot(), pVolumes[i].getTop(), pVolumes[i].getMass());
}

}
