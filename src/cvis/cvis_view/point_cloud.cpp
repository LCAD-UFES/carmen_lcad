#include <point_cloud.h>

namespace CVIS {

	PointCloud::PointCloud(int point_cloud_size, int point_cloud_dim)
	{
		pointCloudSize = point_cloud_size;
		pointCloudDim = point_cloud_dim;

		this->vertices = (GLfloat*) malloc (point_cloud_size * point_cloud_dim * sizeof(GLfloat));
		this->colors = (GLfloat*) malloc (point_cloud_size * 3 * sizeof(GLfloat));
	}

	PointCloud::~PointCloud()
	{

	}

	void PointCloud::PopulatePointCloud(__attribute__((unused)) void* message)
	{
		//TODO: implements in child class
	}

//	void PointCloud::SetStereoParameters(int camera, int width, int height)
//	{
//		stereoInstance = get_stereo_instance(camera, width, height);
//	}
//
//	void PointCloud::PopulatePointCloudFromStereo(carmen_simple_stereo_disparity_message* message)
//	{
//		CvPoint3D32f *vertices_cloud = (CvPoint3D32f*) malloc (pointCloudSize * sizeof(CvPoint3D32f));
//
//		reprojectTo3D(message->disparity, vertices_cloud, 0, this->stereoInstance);
//
//		if(this->vertices != NULL && this->colors !=NULL)
//		{
//			//copy vertices to class atribute
//			for(int i=0; i<pointCloudSize; i++)
//			{
//				if(vertices_cloud[i].z < 4.0)
//				{
//					this->vertices[3 * i] 	  = vertices_cloud[i].x;
//					this->vertices[3 * i + 1] = vertices_cloud[i].z;
//					this->vertices[3 * i + 2] = vertices_cloud[i].y;
//				}
//				else
//				{
//					this->vertices[3 * i] 	  = 0.0;
//					this->vertices[3 * i + 1] = 0.0;
//					this->vertices[3 * i + 2] = 0.0;
//				}
//			}
//
//			//copy colors to class atribute
//			for(int i=0; i<pointCloudSize; i++)
//			{
//				if((message->reference_image[3 * i] + message->reference_image[3 * i + 1] + message->reference_image[3 * i + 2]) < 3*150)
//				{
//						this->colors[3 * i] 	= message->reference_image[3 * i] / 255.0;
//						this->colors[3 * i + 1] = message->reference_image[3 * i + 1] / 255.0;
//						this->colors[3 * i + 2] = message->reference_image[3 * i + 2] / 255.0;
//				}
//				else
//				{
//						this->colors[3 * i] 	= 0;
//						this->colors[3 * i + 1] = 0;
//						this->colors[3 * i + 2] = 0;
//				}
//			}
//		}
//		else
//		{
//			printf("ERRO: PointCloud class not initialized!\n");
//		}
//	}
}

