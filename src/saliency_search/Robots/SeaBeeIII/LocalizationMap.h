#include <vector>
#include "Image/Image.H"
#include "Image/PixelsTypes.H"
#include "LocalizationUtil.h"
#include "LocalizationMapEntity.h"

#ifndef LOCALIZATIONMAP_H_
#define LOCALIZATIONMAP_H_

class LocalizationMap
{
public:
        LocalizationMap();
        LocalizationMap(LocalizationMapEntity e);
        LocalizationMap(vector<LocalizationMapEntity> e);

        void addMapEntity(LocalizationMapEntity e);
        void drawMe(Image<PixRGB<byte> > &img, Camera cam);

        vector<LocalizationMapEntity> mMapEntities;
        //LocalizationMapEntity mPinger;
        //LocalizationMapEntity mPinger2;
        //LocalizationMapEntity mBoundary;
};

#endif /* LOCALIZATIONMAP_H_ */
