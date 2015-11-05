#include "mvog_model/volume.h"

namespace MVOG
{

void createVolume(float bot, float top, Volume& volume)
{
  if (top < bot) // swap top and bottom if neccessary
  {
    float temp = bot;
    bot = top;
    top = temp;
  }

  if (top - bot < 1.0) // ensure minimum height of 1
  {
    double m = bot + (top - bot)/2.0; //FIXME: first bot realy exist?
    top = m + 0.5;
    bot = m - 0.5;
  }

  double mass = top - bot; // ensure initial density of 1

  setBot (volume, bot);
  setTop (volume, top);
  setMass(volume, mass);
}

float getBot (const Volume& volume) { return volume[0]; }
float getTop (const Volume& volume) { return volume[1]; }
float getMass(const Volume& volume) { return volume[2]; }

float getDensity(const Volume& volume)
{
  return getMass(volume)/(getTop(volume) - getBot(volume));
}

void setBot (Volume& volume, float bot)  { volume[0] = bot;  }
void setTop (Volume& volume, float top)  { volume[1] = top;  }
void setMass(Volume& volume, float mass) { volume[2] = mass; }

void createMLVolume(const Volume& volume, MLVolume& mlVolume)
{
  mlVolume.bot = getBot(volume);
  mlVolume.top = getTop(volume);
}

}
