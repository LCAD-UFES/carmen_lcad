#ifndef MVOG_MODEL_CELL_H
#define MVOG_MODEL_CELL_H

#include <cstdio>
#include <string.h>
//#include <mvog_model/volume.h>
#include <mvog_model/volume_vector.h>

namespace MVOG
{

const size_t VOLUME_BYTE_SIZE = sizeof(Volume);

class Cell
{

  private:

    Volume * pVolumes;
    Volume * nVolumes;

    int pVolumesCount;
    int nVolumesCount;

    void addVolume(double bot, double top, Volume*& volumes, int& volumesCount);

  public:

    Cell();
    virtual ~Cell();

    void addPVolume(double bot, double top);
    void addNVolume(double bot, double top);

    Volume * getPVolumes() { return pVolumes; }
    Volume * getNVolumes() { return nVolumes; }

    int getPVolumesCount() { return pVolumesCount; }
    int getNVolumesCount() { return nVolumesCount; }

    void printPVolumes();

};

}; // namespace MVOG

#endif // MVOG_MODEL_CELL_H
