#ifndef MVOG_MODEL_VOLUME_H
#define MVOG_MODEL_VOLUME_H

#include <algorithm>

namespace MVOG
{
  typedef float Volume[3];

  const size_t VOLUME_BYTE_SIZE = sizeof(Volume);

  void createVolume(float bot, float top, Volume& volume);

  float getBot (const Volume& volume);
  float getTop (const Volume& volume);
  float getMass(const Volume& volume);

  float getDensity (const Volume& volume);

  void setBot (Volume& volume, float bot);
  void setTop (Volume& volume, float top);
  void setMass(Volume& volume, float mass);

  struct MLVolume
  {
    float bot;
    float top;
  };

  void createMLVolume(const Volume& volume, MLVolume& mlVolume);

}; // namespace MVOG

#endif // MVOG_MODEL_VOLUME_H
