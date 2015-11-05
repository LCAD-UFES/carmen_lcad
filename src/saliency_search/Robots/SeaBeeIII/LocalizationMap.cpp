#include "LocalizationMap.h"

LocalizationMap::LocalizationMap()
{

}

LocalizationMap::LocalizationMap(LocalizationMapEntity e)
{
        addMapEntity(e);

        //mSensorReaders.resize(scales.size());

        /*for(unsigned int i = 0; i < scales.size(); i ++)
        {
                mSensorReaders[i] = LocalizationSensorReader(scales[i]);
        }*/

        //normalizeSensorReaderScales();
}

LocalizationMap::LocalizationMap(vector<LocalizationMapEntity> e)
{
        for(unsigned int i = 0; i < e.size(); i ++)
        {
                addMapEntity(e[i]);
        }
}

void LocalizationMap::addMapEntity(LocalizationMapEntity e)
{
        mMapEntities.resize(mMapEntities.size() + 1);
        mMapEntities[mMapEntities.size() - 1] = e;
}

void LocalizationMap::drawMe(Image<PixRGB<byte> > &img, const Camera cam)
{
        for(unsigned int i = 0; i < mMapEntities.size(); i ++)
        {
                mMapEntities[i].drawMe(img, cam);
        }
}
