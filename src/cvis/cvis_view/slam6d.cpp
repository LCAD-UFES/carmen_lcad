#include <slam6d.h>

namespace CVIS {

	Slam6d::Slam6d(int width, int height, float focal_length, float max_depth) : VertexBufferObjects(true, 0)
	{
		this->width_ = width;
		this->height_ = height;
		this->focal_length_ = focal_length;
		this->max_depth_ = max_depth;

		this->message = NULL;
	}

	Slam6d::~Slam6d()
	{
		if(this->message != NULL)
		{
			free(this->message->depth);
			free(this->message->image);
			free(this->message);
		}
	}

	void Slam6d::PopulatePointCloud(void* message)
	{
		this->rawPointCloud.clear();
		this->transformedPointCloud.clear();

		pcl::PointXYZ point;
		float cx, cy;
		this->message = (carmen_slam6d_pointcloud_message*) message;

		cx = this->width_/2.0f;
		cy = this->height_/2.0f;

		if(this->pointCloud->vertices != NULL && this->pointCloud->colors !=NULL)
		{
			this->pcTransform = ComposePointCloudTransformationMatrix(this->message);

			for(int i=0; i < this->height_; i+=2)
			{
				for(int j=0; j < this->width_; j+=2)
				{
					int pos = i * this->width_ + j;
					float z = this->message->depth[pos] / 1000.0;	//milimeters to meters

					if(z > 0.5 && z < 3.0)
					{
						point.x = z * (j - cx) * (1.0f/this->focal_length_);
						point.y = z * (i - cy) * (1.0f/this->focal_length_);
						point.z = z;

						this->rawPointCloud.push_back(point);
					}
					else
					{
				        point.x = numeric_limits<float>::quiet_NaN ();
				        point.y = numeric_limits<float>::quiet_NaN ();
				        point.z = numeric_limits<float>::quiet_NaN ();

						this->rawPointCloud.push_back(point);
					}
				}
			}

			pcl::transformPointCloud(this->rawPointCloud, this->transformedPointCloud, this->pcTransform);

			for(int i=0; i < this->height_; i+=2)
			{
				for(int j=0; j < this->width_; j+=2)
				{
					int pos = i * this->width_ + j;
					int pos_rel = (i / 2) * (this->width_ / 2) + (j / 2);

					this->pointCloud->vertices[3 * pos_rel] = this->transformedPointCloud[pos_rel].x;
					this->pointCloud->vertices[3 * pos_rel + 1] = this->transformedPointCloud[pos_rel].y;
					this->pointCloud->vertices[3 * pos_rel + 2] = this->transformedPointCloud[pos_rel].z;

					this->pointCloud->colors[3 * pos_rel]     = this->message->image[3 * pos] / 255.0; //r
					this->pointCloud->colors[3 * pos_rel + 1] = this->message->image[3 * pos + 1] / 255.0; //g
					this->pointCloud->colors[3 * pos_rel + 2] = this->message->image[3 * pos + 2] / 255.0; //b
				}
			}
		}
		else
		{
			printf("ERRO: PointCloud class not initialized!\n");
		}
	}

	void Slam6d::UploadPointSlam6dCloudDataToVBOWithCUDA(void (*external_callback)(carmen_slam6d_pointcloud_message* message, float* posV, float* posC, float focal_length))
	{
		if(this->pointCloud != NULL)
		{
			int vertice_size = this->pointCloud->pointCloudSize * this->pointCloud->pointCloudDim;
			int color_size = this->pointCloud->pointCloudSize * 3;
			int pos_vertice = 0;
			int pos_color = 0;

			if(this->pointCloudCount == this->pointCloudNumber)
				this->pointCloudCount = 0;

			// bind vbo to cuda memory
			cudaGLMapBufferObject((void**)&this->cudaVboPtr, this->vboPointCloud);

			// compute vertice and color position on vbo
			pos_vertice = pointCloudCount * vertice_size;
			pos_color = vertice_size * pointCloudNumber + (pointCloudCount * color_size);

			this->pcTransform = ComposePointCloudTransformationMatrix(this->message);

			this->message->rotation[0] = this->pcTransform(0,0);
			this->message->rotation[1] = this->pcTransform(0,1);
			this->message->rotation[2] = this->pcTransform(0,2);
			this->message->rotation[3] = this->pcTransform(1,0);
			this->message->rotation[4] = this->pcTransform(1,1);
			this->message->rotation[5] = this->pcTransform(1,2);
			this->message->rotation[6] = this->pcTransform(2,0);
			this->message->rotation[7] = this->pcTransform(2,1);
			this->message->rotation[8] = this->pcTransform(2,2);

			this->message->position[0] = this->pcTransform(0,3);
			this->message->position[1] = this->pcTransform(1,3);
			this->message->position[2] = this->pcTransform(2,3);

			// create point cloud with cuda
			external_callback(this->message, this->cudaVboPtr + pos_vertice, this->cudaVboPtr + pos_color, this->focal_length_);

			// unbind cuda
			cudaGLUnmapBufferObject(this->vboPointCloud);

			this->pointCloudCount++;
		}
		else
		{
			printf("ERRO: PointCloud VBO not allocated!\n");
		}
	}

	Eigen::Matrix4f Slam6d::ComposePointCloudTransformationMatrix(carmen_slam6d_pointcloud_message* message)
	{
		Eigen::Matrix4f rot_x, transform;

		//rotation
		rot_x(0,0) = 0.0;
		rot_x(0,1) = 0.0;
		rot_x(0,2) = 1.0;
		rot_x(1,0) = -1.0;
		rot_x(1,1) = 0.0;
		rot_x(1,2) = 0.0;
		rot_x(2,0) = 0.0;
		rot_x(2,1) = -1.0;
		rot_x(2,2) = 0.0;

		//translation
		rot_x(0,3) = 0.0;
		rot_x(1,3) = 0.0;
		rot_x(2,3) = 0.0;

		rot_x(3,0) = 0;
		rot_x(3,1) = 0;
		rot_x(3,2) = 0;
		rot_x(3,3) = 1;

		//rotation
		transform(0,0) = message->rotation[0];
		transform(0,1) = message->rotation[1];
		transform(0,2) = message->rotation[2];
		transform(1,0) = message->rotation[3];
		transform(1,1) = message->rotation[4];
		transform(1,2) = message->rotation[5];
		transform(2,0) = message->rotation[6];
		transform(2,1) = message->rotation[7];
		transform(2,2) = message->rotation[8];

		//translation
		transform(0,3) = message->position[0];
		transform(1,3) = message->position[1];
		transform(2,3) = message->position[2];

		transform(3,0) = 0;
		transform(3,1) = 0;
		transform(3,2) = 0;
		transform(3,3) = 1;

		return rot_x * transform;
	}
}
