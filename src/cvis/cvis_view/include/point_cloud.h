#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_

#include <carmen/carmen.h>
#include <carmen/stereo_util.h>
#include <gtk_gui.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>

namespace CVIS {

class PointCloud {

public:

	GLfloat *vertices;
	GLfloat *colors;
	int pointCloudSize;
	int pointCloudDim;

	PointCloud(int pointCloudSize, int pointCloudDim);
	virtual ~PointCloud();
	virtual void PopulatePointCloud(void* message);
};

}

#endif /* POINT_CLOUD_H_ */
