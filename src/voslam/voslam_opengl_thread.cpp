#include "voslam_opengl_thread.h"

VoslamOpenGLThread::VoslamOpenGLThread() {
}

VoslamOpenGLThread::~VoslamOpenGLThread() {

}

/* static */
void * VoslamOpenGLThread::entryPoint(void *pthis) {
		VoslamOpenGLThread *ptr = static_cast<VoslamOpenGLThread *>(pthis);
        ptr->Run();

        return ptr;
}

void VoslamOpenGLThread::Start(int argc, char **argv, int point_cloud_size, int point_cloud_dim, int number_of_point_clouds, void (*handler_t)(int), int is_cuda_enable, int device) {
        this->argc_ = argc;
        this->argv_ = argv;
        this->point_cloud_size_ = point_cloud_size;
        this->point_cloud_dim_ = point_cloud_dim;
        this->number_of_point_clouds_ = number_of_point_clouds;
        this->handler_t_ = handler_t;
        this->is_cuda_enable_ = is_cuda_enable;
        this->device_ = device;

        this->vertices = (float *) malloc (point_cloud_size * point_cloud_dim * sizeof(float));
        this->colors = (float *) malloc (point_cloud_size * 3 * sizeof(float));

        if (pthread_create(&m_threadId, NULL, entryPoint, this) != 0) {
                // throw an error
        }
}

void VoslamOpenGLThread::Wait() {
        if (pthread_join(m_threadId, NULL) != 0) {
                // throw an error
        }
}

void VoslamOpenGLThread::Abort() {
        if (pthread_cancel(m_threadId) != 0) {
                // throw an error
        }
}

void VoslamOpenGLThread::Detach() {
        if (pthread_detach(m_threadId) != 0) {
        	clear_global_variables();
        }
}

void VoslamOpenGLThread::Run() {
	  initialize_point_cloud_viewer(this->argc_, this->argv_, this->point_cloud_size_, this->point_cloud_dim_,
			  this->number_of_point_clouds_, this->handler_t_, this->is_cuda_enable_, this->device_);
	  allocate_vbo_arrays();

	  run_glut();
}

void VoslamOpenGLThread::UploadChain(VoslamKeyframes keyframe_chain, int depth_width, int depth_height)
{
	int start =  (keyframe_chain.getKeyframesListSize() - this->number_of_point_clouds_);

	if(start < 0)
		start = 0;

	for (int i = start; i < keyframe_chain.getKeyframesListSize(); i++)
	{
		memset(this->vertices, 0, this->point_cloud_size_ * this->point_cloud_dim_ * sizeof(GLfloat));
		memset(this->colors, 0, this->point_cloud_size_ * 3 * sizeof(GLfloat));

		this->CopyVerticesAndColorFromFrame(keyframe_chain.getKeyframe(i), depth_width, depth_height);

		copy_data_to_vertices(this->vertices);
		copy_data_to_colors(this->colors);

		upload_data_to_vbo_array();
	}
}

#ifndef NO_CUDA

void VoslamOpenGLThread::UploadChainCuda(std::vector<carmen_voslam_pointcloud_t> keyframe_chain, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, int rgb_center_x, int rgb_center_y)
{
	upload_chain_depth_data_to_vbo_cuda_cloud(keyframe_chain, depth_width, depth_height, depth_focal_length_x, depth_focal_length_y, rgb_focal_length_x, rgb_focal_length_y, rgb_center_x, rgb_center_y);
}

#endif

pthread_t VoslamOpenGLThread::GetId() {
        return m_threadId;
}

void VoslamOpenGLThread::CopyVerticesAndColorFromFrame(carmen_voslam_pointcloud_t* frame, int width, int height)
{
	int v = 0, c = 0;

	/***********************************************************
	 *** TODO: Coloquei a transformada para a interface aqui ***
	 ***********************************************************/

	tf::Transform world_to_opengl(tf::Quaternion(0, 0, -M_PI/2), tf::Vector3(0, 0, 0));

	for(int i = 0;  i < frame->pointcloud->points.size(); i++)
	{
		tf::Transform point_in_world_reference(tf::Quaternion(0, 0, 0), tf::Vector3(
			frame->pointcloud->points[i].x,
			frame->pointcloud->points[i].y,
			frame->pointcloud->points[i].z
		));

		tf::Transform point_in_opengl_reference = world_to_opengl * point_in_world_reference;

			// BACKUP:
//			this->vertices[v++] = frame->pointcloud->points[i].x;
//			this->vertices[v++] = frame->pointcloud->points[i].y;
//			this->vertices[v++] = frame->pointcloud->points[i].z;

			this->vertices[v++] = point_in_opengl_reference.getOrigin().x();
			this->vertices[v++] = point_in_opengl_reference.getOrigin().y();
			this->vertices[v++] = point_in_opengl_reference.getOrigin().z();

			this->colors[c++] = frame->pointcloud->points[i].r / 255.0f;
			this->colors[c++] = frame->pointcloud->points[i].g / 255.0f;
			this->colors[c++] = frame->pointcloud->points[i].b / 255.0f;
	}
}
