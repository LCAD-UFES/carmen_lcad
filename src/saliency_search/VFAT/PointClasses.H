/*! @file VFAT/PointClasses.H [put description here] */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/PointClasses.H $
// $Id: PointClasses.H 6182 2006-01-31 18:41:41Z rjpeters $

class PixelPoint
{
public:
        float x;
        float y;

        PixelPoint(float xval, float yval)
        {
                x = xval;
                y = yval;
        }
};

class Point3D
{
public:
        float x;
        float y;
        float z;

        Point3D(float xval, float yval, float zval)
        {
                x = xval;
                y = yval;
                z = zval;
        }
};

class CameraParams
{
public:
        float x;
        float y;
        float z;
        float theta;
        float phi;
        float f;
        float r; //this param is the distance between the camera center and the image center
        CameraParams(float xval, float yval, float zval, float thetaval, float phival, float fval, float rval)
        {
                x = xval;
                y = yval;
                z = zval;
                theta = thetaval;
                phi = phival;
                f = fval;
                r = rval;
        }
};
