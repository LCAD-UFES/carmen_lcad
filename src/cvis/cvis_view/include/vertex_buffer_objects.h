#include <point_cloud.h>
//#include <cuda.h>
//#include <cutil.h>
//#include <cuda_runtime.h>
////#include <cutil_inline.h>
////#include <cutil_gl_inline.h>
//#include <cuda_gl_interop.h>

#ifndef VERTEX_BUFFER_OBJECTS_H_
#define VERTEX_BUFFER_OBJECTS_H_

namespace CVIS {

class PointCloud;

class VertexBufferObjects {

public:

	float* cudaVboPtr;
	GLuint vboPointCloud;
	PointCloud* pointCloud;

	int pointCloudCount;
	int pointCloudNumber;

	bool isCudaEnabled;
	int clean_memory;

	VertexBufferObjects(bool cuda_enable, int device);
	VertexBufferObjects();
	virtual ~VertexBufferObjects();
	void AllocatePointCloudVBO(int point_cloud_size, int point_cloud_dim, int point_cloud_number);
	void DeleteVertexBufferObjects();
	virtual void UploadPointCloudDataToVBO();
	//virtual void UploadPointCloudDataToVBOWithCUDA(void (*external_callback)(float* vertices, float* colors, float* posV, float* posC, int width, int height), int point_cloud_width, int point_cloud_height);

};

}

#endif /* VERTEX_BUFFER_OBJECTS_H_ */
