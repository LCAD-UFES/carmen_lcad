#include "mvog_model/cell.h"

namespace MVOG
{

Cell::Cell()
{
  pVolumesCount = 0;
  nVolumesCount = 0;
  pVolumes = NULL;
  nVolumes = NULL;
}

Cell::~Cell()
{
  delete[] pVolumes;
  delete[] nVolumes;

  pVolumes = NULL;
  nVolumes = NULL;
}

void Cell::addPVolume(float bot, float top)
{
	addVolume(bot, top, pVolumes, pVolumesCount);
}

void Cell::addNVolume(float bot, float top)
{
	addVolume(bot, top, nVolumes, nVolumesCount);
}

void Cell::addVolume(float bot, float top, VolumeArray& volumes, int& volumesCount)
{
  float minGap = 1.0; // FIXME: gap only works for 0.0, - gap of 1.0 needs code for adj. mass

  // **** create the volume
  Volume v;
  createVolume(bot, top, v);

  // **** non-empty list: iterate over volumes
  for (int i = 0; i < volumesCount; i++)
  {
    // 1. below next volume, and does not intersect
    if (getTop(v) < getBot(volumes[i]) - minGap)
    {
      // insert it at position i;
      VolumeArray newVolumes = new Volume[volumesCount+1];

      memcpy(newVolumes, volumes, i*VOLUME_BYTE_SIZE);
      memcpy(newVolumes[i], v, VOLUME_BYTE_SIZE);
      memcpy(newVolumes+i+1, volumes+i, (volumesCount-i)*VOLUME_BYTE_SIZE);


      delete[] volumes;
      volumes = newVolumes;
      volumesCount++;

      return;
    }

    // 2. intersects with next volume
    else if (getBot(v) <= getTop(volumes[i]) + minGap)
    {
    	double gap_mass = 0.0;
      // get all volumes that v intersects with
      int stopIndex  = i+1;

      int c = i+1;
      while(c < volumesCount )
      {
        if (getTop(v) >= getBot(volumes[c]) - minGap)
        {
           c++;
           stopIndex = c;
        }
        else break;
      }

      // merge with new volume
      setBot (volumes[i], std::min(getBot(volumes[i]), getBot(v)));
      setTop (volumes[i], std::max(getTop(volumes[i]), getTop(v)));

      //added by lcad
      if(getBot(volumes[i]) > getTop(v))
      	gap_mass = getBot(volumes[i]) - getTop(v);
      else
      	gap_mass = getBot(v) - getTop(volumes[i]);

      setMass(volumes[i], getMass(volumes[i]) + getMass(v) + gap_mass); // + FIXME: gap mass?? - gap_mass added by lcad

      // merge with rest
      setTop(volumes[i], std::max(getTop(volumes[i]), getTop(volumes[stopIndex - 1])));

      for (int t = i+1; t < stopIndex ; t++)
         setMass(volumes[i], getMass(volumes[i]) + getMass(volumes[t])); // + FIXME: gap mass??

      // erase old volumes that were merged
      //printf ("erasing from %d to %d\n", i+1, stopIndex);

      VolumeArray newVolumes = new Volume[volumesCount - (stopIndex - i) + 1];
      memcpy(newVolumes+0, volumes, (i+1)*VOLUME_BYTE_SIZE);
      memcpy(newVolumes+i+1, volumes + stopIndex, (volumesCount - stopIndex)*VOLUME_BYTE_SIZE);
      delete[] volumes;
      volumes = newVolumes;
      volumesCount = volumesCount - (stopIndex - i) + 1;

      return;
    }
  }

  // insert it at position i;

  VolumeArray newVolumes = new Volume[volumesCount+1];
  memcpy(newVolumes, volumes, volumesCount*VOLUME_BYTE_SIZE);
  memcpy(newVolumes[volumesCount], v, VOLUME_BYTE_SIZE);
  delete[] volumes;
  volumes = newVolumes;

  volumesCount++;
}

void Cell::printPVolumes()
{
  printf("*** CELL (+) %d ****\n", pVolumesCount);

  for (int i = 0; i < pVolumesCount; i++)
    printf("\t%d [%f, %f] [%f]\n", i, getBot(pVolumes[i]), getTop(pVolumes[i]), getMass(pVolumes[i]));
}

void Cell::printNVolumes()
{
  printf("*** CELL (-) %d ****\n", nVolumesCount);

  for (int i = 0; i < nVolumesCount; i++)
    printf("\t%d [%f, %f] [%f]\n", i, getBot(nVolumes[i]), getTop(nVolumes[i]), getMass(nVolumes[i]));
}


bool Cell::getOccDensity(float z, float& density) const
{
  float pd = getPDensity(z);
  float nd = getNDensity(z);

  if (pd == 0 && nd == 0) return false;

  density = pd / (pd + nd);

  return true;
}

float Cell::getPDensity(float z) const
{
  for(int i = 0; i < pVolumesCount; ++i)
  {
    if (getBot(pVolumes[i]) <= z && z <= getTop(pVolumes[i]))
      return getDensity(pVolumes[i]);
    if (z < getBot(pVolumes[i]))
      return 0;
  }

	return 0;
}

float Cell::getNDensity(float z) const
{
  for(int i = 0; i < nVolumesCount; ++i)
  {
    if (getBot(nVolumes[i]) <= z && z <= getTop(nVolumes[i]))
      return getDensity(nVolumes[i]);
    if (z < getBot(nVolumes[i]))
      return 0;
  }

	return 0;
}

bool Cell::validate()
{
  for (int i = 0; i<nVolumesCount-1; i++)
  {
    //if (getTop(nVolumes[i]) >= getBot(nVolumes[i+1]))
      return false;
  }

  for (int i = 0; i<pVolumesCount-1; i++)
  {
    //if (getTop(pVolumes[i]) >= getBot(pVolumes[i+1]))
     return false;
  }

  return true;
}

void Cell::clear()
{
  pVolumesCount = 0;
  nVolumesCount = 0;

  delete[] pVolumes;
  delete[] nVolumes;

  pVolumes = NULL;
  nVolumes = NULL;
}

void Cell::createMLVolumes(MLVolumeVector& mlVolumes)
{
  if (pVolumesCount == 0 && nVolumesCount == 0) return;

  int state = 0; // 0 - not in Positive, not in Negative
                 // 1 - in P, not in N
                 // 2 - not in P, in N
                 // 3 - in P, in N

  int cp = 0;   // index of current positive volume
  int cn = 0;   // index of curentt negative volume

  MLVolume ml;

  //printf("Starting ML iteration\n");

  while(true)
  {
    // **** STATE == 0: NOT IN P, NOT IN N *********************
    if (state == 0)
    {
      //printf("\nstate == 0\n");

      if (cp >= pVolumesCount)
      {
        // no more positive volumes - exit loop
        //printf("\tReached end of P volumes, EXIT\n");
        break;
      }
      if (cn >= nVolumesCount)
      {
        // no more negative volumes - add all remaining positive and exit
        while (cp < pVolumesCount)
        {
          createMLVolume(pVolumes[cp], ml);
          mlVolumes.push_back(ml);
          cp++;
        }
        //printf("\tReached end of N volumes, added all P, EXIT\n");
        break;
      }

      if(getBot(pVolumes[cp]) <= getBot(nVolumes[cn]))
      {
        // entering a new P volume

        ml.bot = getBot(pVolumes[cp]);

        state = 1;
        continue;
      }
      else
      {
        // enterint a new N volume - do nothing
        state = 2;
        continue;
      }
    } // state == 0
    else if (state == 1)
    {
      //printf("\nstate == 1\n");

      if (cn >= nVolumesCount || getTop(pVolumes[cp]) < getBot(nVolumes[cn]))
      {
        // no more N volumes, or next N does not intersect

        ml.top = getTop(pVolumes[cp]);
        mlVolumes.push_back(ml);
        cp++;

        state = 0;
        continue;
      }
      else
      {
        // entering an N volume

        if(getCombinedDensity(cp, cn) < 0.5)
        {
          // N volume with high density
          ml.top = getBot(nVolumes[cn]);
          mlVolumes.push_back(ml);
        }

        state = 3;
        continue;
      }
    } // state == 1
    else if (state == 2)
    {
      //printf("\nstate == 2\n");

      if (cp >= pVolumesCount || getBot(pVolumes[cp]) >= getTop(nVolumes[cn]))
      {
        // leaving N - next P does not intersect
        cn++;

        state = 0;
        continue;
      }
      else
      {
        // entering a P volume

        if (getCombinedDensity(cp, cn) >= 0.5)
        {
          // entering P volume with high density
          ml.bot = getBot(pVolumes[cp]);
        }

        state = 3;
        continue;
      }
    } // state == 2
    else // state == 3
    {
      //printf("\nstate == 3\n");

      if (getTop(pVolumes[cp]) >= getTop(nVolumes[cn]))
      {
        // N volume ends

        if (getCombinedDensity(cp, cn) < 0.5)
        {
          ml.bot = getTop(nVolumes[cn]);
        }

        cn++;
        state = 1;
        continue;
      }
      else
      {
        // P Volume ends

        if (getCombinedDensity(cp, cn) >= 0.5)
        {
          ml.top = getTop(pVolumes[cp]);
          mlVolumes.push_back(ml);
        }

        cp++;
        state = 2;
        continue;
      } // state == 3
    }
  }

  //printf("\t Done ML iteration\n");

}

float Cell::getCombinedDensity(int cp, int cn)
{
  float pd = getDensity(pVolumes[cp]);
  float nd = getDensity(nVolumes[cn]);

  return pd / (pd + nd);
}

bool isObstacle(int pIndex, int nIndex)
{
  return false;
}


}
