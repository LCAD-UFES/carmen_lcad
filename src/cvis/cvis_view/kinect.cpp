#include <kinect.h>

namespace CVIS {

	Kinect::Kinect(int width, int height, float focal_length) : VertexBufferObjects(true, 0)
	{
		this->width_ = width;
		this->height_ = height;
		this->focal_length_ = focal_length;

		this->message = NULL;

		if(this->message == NULL)
			this->message = (carmen_kinect_depth_message*) malloc (sizeof(carmen_kinect_depth_message));

		this->message->depth = (float* ) malloc (width * height * sizeof(float));
		this->rand_drop_vector = (double *) calloc (width * height, sizeof(double));

		for(int i = 0; i < width * height; i++)
		{
			this->rand_drop_vector[i] = (double)(50.0 + rand() % 150) / 100.0;
		}

		this->cut_off = 0;
		this->blowup_count = 0;
	}

	Kinect::~Kinect()
	{
		if(this->message != NULL)
		{
			free(this->message->depth);
			free(this->message);
		}
	}

	void Kinect::PopulatePointCloud(void* message,  double cloudR, double cloudG, double cloudB, double foreRange, double backRange)
	{
		float vx, vy, vz;
		float cx, cy;
		this->message = (carmen_kinect_depth_message*) message;

		cx = this->width_/2.0f;
		cy = this->height_/2.0f;

		if(this->pointCloud->vertices != NULL && this->pointCloud->colors !=NULL)
		{

			for(int i=0; i < this->height_; i+=this->cloud_density)
			{
				for(int j=0; j < this->width_; j+=this->cloud_density)
				{
					int pos = i * this->width_ + j;

					float z = this->message->depth[pos];

					if(z > foreRange && z < backRange)
					{
						vx = z * (j - cx) * (1.0f/this->focal_length_);
						vy = z * (i - cy) * (1.0f/this->focal_length_);
						vz = z - 2.0;

						this->pointCloud->vertices[3 * pos] = vx;
						this->pointCloud->vertices[3 * pos + 1] = vz;
						this->pointCloud->vertices[3 * pos + 2] = -vy;

						this->pointCloud->colors[3 * pos]     = cloudR; //r
						this->pointCloud->colors[3 * pos + 1] = cloudG; //g
						this->pointCloud->colors[3 * pos + 2] = cloudB; //b

					}
					else
					{
						this->pointCloud->vertices[3 * pos] = 0;
						this->pointCloud->vertices[3 * pos + 1] = 0;
						this->pointCloud->vertices[3 * pos + 2] = 0;

						this->pointCloud->colors[3 * pos]     = 0; //r
						this->pointCloud->colors[3 * pos + 1] = 0; //g
						this->pointCloud->colors[3 * pos + 2] = 0; //b
					}
				}
			}
		}
		else
		{
			printf("ERRO: PointCloud class not initialized!\n");
		}
	}

	double count_message = 0;
	void Kinect::EffectPointCloudColor(double cor1R, double cor1G, double cor1B,
									   double cor2R, double cor2G, double cor2B,
									   double cor3R, double cor3G, double cor3B, double changeSpeed)
	{
		if(this->pointCloud->vertices != NULL && this->pointCloud->colors !=NULL)
		{
			for(int i=0; i < this->height_; i+=this->cloud_density)
			{
				for(int j=0; j < this->width_; j+=this->cloud_density)
				{
					int pos = i * this->width_ + j;

					int value = (int)count_message % 3;
					switch(value)
					{
						case 0:
							this->pointCloud->colors[3 * pos]     = cor1R; //r
							this->pointCloud->colors[3 * pos + 1] = cor1G; //g
							this->pointCloud->colors[3 * pos + 2] = cor1B; //b
							break;
						case 1:
							this->pointCloud->colors[3 * pos]     = cor2R; //r
							this->pointCloud->colors[3 * pos + 1] = cor2G; //g
							this->pointCloud->colors[3 * pos + 2] = cor2B; //b
							break;
						case 2:
							this->pointCloud->colors[3 * pos]     = cor3R; //r
							this->pointCloud->colors[3 * pos + 1] = cor3G; //g
							this->pointCloud->colors[3 * pos + 2] = cor3B; //b
							break;
					}
				}
			}

			count_message+=changeSpeed;
		}
		else
		{
			printf("ERRO: PointCloud class not initialized!\n");
		}
	}


	void Kinect::EffectPointCloudDrop()
	{
		double vy;
		//cut_off++;

		if(this->pointCloud->vertices != NULL && this->pointCloud->colors !=NULL)
		{
			for(int i=0; i < this->height_; i+=this->cloud_density)
			{
				for(int j=0; j < this->width_; j+=this->cloud_density)
				{
					int pos = i * this->width_ + j;

					vy = this->pointCloud->vertices[3 * pos + 2];

					double value_y = (vy - rand_drop_vector[pos] * 0.1);
					this->pointCloud->vertices[3 * pos + 2] = (value_y <= -1.0) ? -1.0 : value_y;

					if(this->pointCloud->vertices[3 * pos + 2] <= -1.0)
					{
						this->pointCloud->colors[3 * pos] = 0;
						this->pointCloud->colors[3 * pos + 1] = 0;
						this->pointCloud->colors[3 * pos + 2] = 0;
					}
				}
			}
		}
		else
		{
			printf("ERRO: PointCloud class not initialized!\n");
		}
	}

	void Kinect::EffectPointCloudBlowUp()
	{
		if(this->pointCloud->vertices != NULL && this->pointCloud->colors !=NULL)
		{

			for(int i=0; i < this->height_; i+=this->cloud_density)
			{
				for(int j=0; j < this->width_; j+=this->cloud_density)
				{
					int pos = i * this->width_ + j;

					this->pointCloud->vertices[3 * pos] *= rand_drop_vector[pos];
					this->pointCloud->vertices[3 * pos + 1] *= rand_drop_vector[pos];
					this->pointCloud->vertices[3 * pos + 2] *= rand_drop_vector[pos];

					if(this->pointCloud->vertices[3 * pos] > 20.0 || this->pointCloud->vertices[3 * pos + 1] > 20.0 || this->pointCloud->vertices[3 * pos + 2] > 20.0)
					{
						this->pointCloud->colors[3 * pos] = 0;
						this->pointCloud->colors[3 * pos + 1] = 0;
						this->pointCloud->colors[3 * pos + 2] = 0;
					}
				}
			}
		}
		else
		{
			printf("ERRO: PointCloud class not initialized!\n");
		}
	}

	void Kinect::EffectPointCloudNoise()
	{
		int i, k;
		//cut_off++;

		if(this->pointCloud->vertices != NULL && this->pointCloud->colors !=NULL)
		{
			for(k = 0;  k < (rand() % 5); k++)
			{
				i = 10 + rand() % 470;

				for(int l = (i - (rand() % 50)); l < i; l++)
				{
					double randomico = (double)(-50.0 + rand() % 100);

					for(int j=0; j < this->width_; j+=this->cloud_density)
					{
						int pos = l * this->width_ + j;

						this->pointCloud->vertices[3 * pos + 1] += (randomico / 500.0)	;
					}
				}
			}
		}
		else
		{
			printf("ERRO: PointCloud class not initialized!\n");
		}
	}
}
