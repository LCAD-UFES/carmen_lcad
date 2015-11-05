#include "LocalizationMapEntity.h"

LocalizationMapEntity::LocalizationMapEntity()
{

}

LocalizationMapEntity::LocalizationMapEntity(Point2D<float> center, float orientation, Point2D<float> dim, int shapeType, PixRGB<byte> color, int objectType, int interactionType, int fillType)
{
        mCenter = center;
        mOrientation = orientation;
        mDim = dim;
        mShapeType = shapeType;
        mFillType = fillType;
        mObjectType = objectType;
        mColor = color;
        mInteractionType = interactionType;
}

void LocalizationMapEntity::drawMe(Image<PixRGB<byte> > &img, const Camera cam)
{
        drawShape(img, cam);
        //drawBackground(img, cam);
}

void LocalizationMapEntity::drawShape(Image<PixRGB<byte> > &img, const Camera cam)
{
        Point2D<int> thePoint = Point2D<int> (
                        round((mCenter.i - cam.mCenter.i) * cam.mScale),
                        -round((mCenter.j + cam.mCenter.j) * cam.mScale)
                        );
        thePoint += Point2D<int> (img.getWidth() / 2, img.getHeight() / 2);
        Dims theDim = Dims(cam.mScale * mDim.i, cam.mScale * mDim.j);
        Rectangle theRect = Rectangle(Point2D<int>(thePoint.i, thePoint.j), theDim);
        switch(mShapeType)
        {
        case ShapeType::rect:
                theRect = Rectangle(Point2D<int>(thePoint.i - theDim.w() / 2, thePoint.j - theDim.h() / 2), theDim);
                drawRectOR(img, theRect, mColor, 1, D_DEGREE * (mOrientation + 90.0));
                break;
        case ShapeType::line:
                drawLine(img, thePoint, mOrientation * D_DEGREE, mDim.i * cam.mScale, mColor, 1);
                break;
        case ShapeType::square:
                theRect = Rectangle(Point2D<int>(thePoint.i - theDim.w() / 2, thePoint.j - theDim.h() / 2), theDim);
                drawRectOR(img, theRect, mColor, 1, D_DEGREE * (mOrientation + 90.0));
                break;
        case ShapeType::circle:
                drawCircle(img, thePoint, mDim.i * cam.mScale / 2, mColor, 1);
                break;
        }
        //drawBackground(img, cam, theRect);
}

void LocalizationMapEntity::drawBackground(Image<PixRGB<byte> > &img, const Camera cam, Rectangle theRect)
{
	/*Dims theDim = theRect.dims();
	if(mShapeType.mId ==  ShapeType::line)
		return;
	if(mShapeType.mId == ShapeType::circle)
	{
		drawCircle(img, theRect.topLeft(), mDim.i * cam.mScale, mColor, 1);
	}
	else if(mShapeType.mId == ShapeType::rect || mShapeType.mId == ShapeType::square)
	{
		theRect.topLeft() += Point2D<int> (-theDim.w() / 2, -theDim.h() / 2);
		drawRectOR(img, theRect, mColor, 1, D_DEGREE * (mOrientation + 90.0));
	}*/
}
