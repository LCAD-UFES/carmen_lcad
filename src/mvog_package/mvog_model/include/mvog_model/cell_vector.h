#ifndef MVOG_MODEL_CELL_H
#define MVOG_MODEL_CELL_H

#include <cstdio>
#include <vector>
#include <mvog_model/volume_vector.h>

namespace MVOG
{

typedef std::vector<Volume> VolumeVector;

class Cell
{

  private:

    VolumeVector pVolumes;
    VolumeVector nVolumes;

    void addVolume(double bot, double top, VolumeVector& volumes);

  public:

    Cell();
    virtual ~Cell();

    void addPVolume(double bot, double top);
    void addNVolume(double bot, double top);

    VolumeVector * getPVolumes() { return &pVolumes; }
    VolumeVector * getNVolumes() { return &nVolumes; }

    void printPVolumes();

};

}; // namespace MVOG

#endif // MVOG_MODEL_CELL_H
