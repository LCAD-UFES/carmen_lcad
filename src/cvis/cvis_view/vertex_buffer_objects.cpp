#include <vertex_buffer_objects.h>

namespace CVIS {

	VertexBufferObjects::VertexBufferObjects(bool cuda_enable, int device = 0)
	{
		this->vboPointCloud = 0;
		this->pointCloud = NULL;
		this->pointCloudCount = 0;
		this->pointCloudNumber = 0;
		this->clean_memory = 0;

		//this->isCudaEnabled = cuda_enable;

		//if(this->isCudaEnabled)
		//	cudaGLSetGLDevice(device);

		printf("Cuda Slam6d Device: %d\n", device);
	}

	VertexBufferObjects::VertexBufferObjects()
	{
		this->vboPointCloud = 0;
		this->pointCloud = NULL;
		this->pointCloudCount = 0;
		this->pointCloudNumber = 0;
	}

	VertexBufferObjects::~VertexBufferObjects()
	{
	}

	void VertexBufferObjects::AllocatePointCloudVBO(int point_cloud_size, int point_cloud_dim, int point_cloud_number)
	{
		if(this->pointCloud == NULL)
		{
			this->pointCloudNumber = point_cloud_number;
			this->pointCloud = new PointCloud(point_cloud_size, point_cloud_dim);
		}

		long int vertices_size = point_cloud_size * point_cloud_dim * point_cloud_number * sizeof(GLfloat);
		long int colors_size = point_cloud_size * 3 * point_cloud_number * sizeof(GLfloat);

		float* reset = (float*) malloc (vertices_size + colors_size);
		memset(reset, 0, vertices_size + colors_size);

		/* cria um novo VBO array na placa grafica */
		glGenBuffersARB(1, &this->vboPointCloud);

		/* associa o array VBO criado a sua ID antes de usa-lo */
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, this->vboPointCloud);

		/* aloca na placa grafica memoria para a nuvem de ponto mais a cor de cada ponto*/
		glBufferDataARB(GL_ARRAY_BUFFER_ARB, vertices_size + colors_size, reset, GL_DYNAMIC_DRAW_ARB);

		glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

		free(reset);

		//if(this->isCudaEnabled)
		//	cudaGLRegisterBufferObject(this->vboPointCloud);
	}

	void VertexBufferObjects::DeleteVertexBufferObjects()
	{
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, this->vboPointCloud);
		glDeleteBuffersARB(1, &this->vboPointCloud);

		//if(this->isCudaEnabled)
		//	cudaGLUnregisterBufferObject(this->vboPointCloud);
	}

	void VertexBufferObjects::UploadPointCloudDataToVBO()
	{
		if(this->pointCloud != NULL)
		{
			int vertices_size = this->pointCloud->pointCloudSize * this->pointCloud->pointCloudDim * sizeof(GLfloat);
			int colors_size = this->pointCloud->pointCloudSize * 3 * sizeof(GLfloat);

			if(this->pointCloudCount == this->pointCloudNumber)
				this->pointCloudCount = 0;

			/* conecta ao vbo allocado */
			glBindBufferARB(GL_ARRAY_BUFFER_ARB, this->vboPointCloud);

			/* copia os dados dos pontos da nuvem para o inicio do VBO */
			glBufferSubDataARB(GL_ARRAY_BUFFER_ARB, pointCloudCount * vertices_size, vertices_size, this->pointCloud->vertices);

			/* copia dos dados de cor da nuvem apos o vetor de pontos*/
			glBufferSubDataARB(GL_ARRAY_BUFFER_ARB, vertices_size * pointCloudNumber + (pointCloudCount * colors_size), colors_size, this->pointCloud->colors);

			/* desconecta do vbo */
			glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

			pointCloudCount++;

			if(clean_memory)
			{
				memset(this->pointCloud->vertices, 0, vertices_size);
				memset(this->pointCloud->colors, 0, colors_size);

				clean_memory = 0;
			}
		}
		else
		{
			printf("ERRO: PointCloud VBO not allocated!\n");
		}
	}

//	void VertexBufferObjects::UploadPointCloudDataToVBOWithCUDA(void (*external_callback)(float* vertices, float* colors, float* posV, float* posC, int width, int height), int point_cloud_width, int point_cloud_height)
//	{
//		if(this->pointCloud != NULL)
//		{
//			int vertice_size = this->pointCloud->pointCloudSize * this->pointCloud->pointCloudDim;
//			int color_size = this->pointCloud->pointCloudSize * 3;
//			int pos_vertice = 0;
//			int pos_color = 0;
//
//			if(this->pointCloudCount == this->pointCloudNumber)
//				this->pointCloudCount = 0;
//
//			// bind vbo to cuda memory
//			cudaGLMapBufferObject((void**)&this->cudaVboPtr, this->vboPointCloud);
//
//			// compute vertice and color position on vbo
//			pos_vertice = pointCloudCount * vertice_size;
//			pos_color = vertice_size * pointCloudNumber + (pointCloudCount * color_size);
//
//			// create point cloud with cuda
//			external_callback(this->pointCloud->vertices, this->pointCloud->colors, this->cudaVboPtr + pos_vertice, this->cudaVboPtr + pos_color, point_cloud_width, point_cloud_height);
//
//			// unbind cuda
//			cudaGLUnmapBufferObject(this->vboPointCloud);
//
//			this->pointCloudCount++;
//		}
//		else
//		{
//			printf("ERRO: PointCloud VBO not allocated!\n");
//		}
//	}
}
