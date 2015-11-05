#ifndef MVOG_MODEL_CELL_H
#define MVOG_MODEL_CELL_H

#include <cstdio>
#include <string.h>
#include <algorithm>
#include <vector>
#include <queue>

#include <mvog_model/volume.h>

#include <boost/numeric/interval.hpp>

namespace MVOG
{

typedef Volume*               VolumeArray;
typedef std::vector<MLVolume> MLVolumeVector;

class Boundary
{
  public:
    float z_;
    bool positive_;
    int index_;

    Boundary() { }
    Boundary(float z, bool positive, int index):z_(z), positive_(positive), index_(index) {}

    bool operator<(const Boundary &other) const
    {
      return (z_ >= other.z_);
    }
};


class Cell
{
    friend class Map;

  private:

    VolumeArray pVolumes;
    VolumeArray nVolumes;

    int pVolumesCount;
    int nVolumesCount;

    void addVolume(float bot, float top, VolumeArray& volumes, int& volumesCount);

    float getCombinedDensity(int cp, int cn);

  public:

    Cell();
    virtual ~Cell();

    void addPVolume(float bot, float top);
    void addNVolume(float bot, float top);

    VolumeArray getPVolumes() { return pVolumes; }
    VolumeArray getNVolumes() { return nVolumes; }

    int getPVolumesCount() { return pVolumesCount; }
    int getNVolumesCount() { return nVolumesCount; }

    bool  getOccDensity(float z, float& density) const;
    float getPDensity(float z) const;
    float getNDensity(float z) const;

    void printPVolumes();
    void printNVolumes();

    bool validate();

    void clear();

    void createMLVolumes(MLVolumeVector& mlVolumes);
};



}; // namespace MVOG

#endif // MVOG_MODEL_CELL_H
