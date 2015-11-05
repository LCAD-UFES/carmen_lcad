#include "Image/PixelsTypes.H"
#include "Image/Point2D.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "LocalizationUtil.h"

#ifndef LOCALIZATIONMAPENTITY_H_
#define LOCALIZATIONMAPENTITY_H_

class LocalizationMapEntity
{
public:
        struct FillType
        {
			const static int none = 0;
			const static int fill = 1;
			const static int grid = 2;
        };

        struct InteractionType
        {
			const static int internal = 0; //use this for normal objects
			const static int external = 1; //use this if the robot will be inside and bounded by the object
			const static int passive =  2;
        };

        struct ShapeType
        {
			const static int rect =   0;
			const static int line =   1;
			const static int square = 2;
			const static int circle = 3;
        };

        struct ObjectType
        {
        	const static int none = 0;
        	const static int boundary = 1;
        	const static int pipe = 2;
        	const static int bin = 3;
        	const static int buoy = 4;
        };

        LocalizationMapEntity();
        LocalizationMapEntity(Point2D<float> center, float orientation, Point2D<float> dim, int shapeType, PixRGB<byte> color = PixRGB<byte>(255, 255, 255), int mObjectType = ObjectType::none, int interactionType = InteractionType::internal, int fillType = FillType::none);

        void drawMe(Image<PixRGB<byte> > &img, const Camera cam);
        void drawShape(Image<PixRGB<byte> > &img, const Camera cam);
        void drawBackground(Image<PixRGB<byte> > &img, const Camera cam, Rectangle theRect);

        Point2D<float> mCenter;
		float mOrientation;
		Point2D<float> mDim;
		PixRGB<byte> mColor;

        int mShapeType; //how to draw this object
        int mFillType; //how to fill in this object
        int mInteractionType; //determines how the object should interact with sensors
        int mObjectType; //physical object description, of any
};

#endif /* LOCALIZATIONMAPENTITY_H_ */
