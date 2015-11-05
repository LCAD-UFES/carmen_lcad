#include "slam6d_opengl_thread.h"

Thread::Thread() {
}

Thread::~Thread() {

}

/* static */
void *Thread::entryPoint(void *pthis) {
        Thread *ptr = static_cast<Thread *>(pthis);
        ptr->Run();
}

void Thread::Start(int argc, char **argv, int point_cloud_size, int point_cloud_dim, int number_of_point_clouds, void (*handler_t)(int), int is_cuda_enable, int device) {
        this->argc_ = argc;
        this->argv_ = argv;
        this->point_cloud_size_ = point_cloud_size;
        this->point_cloud_dim_ = point_cloud_dim;
        this->number_of_point_clouds_ = number_of_point_clouds;
        this->handler_t_ = handler_t;
        this->is_cuda_enable_ = is_cuda_enable;
        this->device_ = device;

        if (pthread_create(&m_threadId, NULL, entryPoint, this) != 0) {
                // throw an error
        }
}

void Thread::Wait() {
        if (pthread_join(m_threadId, NULL) != 0) {
                // throw an error
        }
}

void Thread::Abort() {
        if (pthread_cancel(m_threadId) != 0) {
                // throw an error
        }
}

void Thread::Detach() {
        if (pthread_detach(m_threadId) != 0) {
                // throw an error
        }
}

void Thread::Run() {
	  initialize_point_cloud_viewer(this->argc_, this->argv_, this->point_cloud_size_, this->point_cloud_dim_,
			  this->number_of_point_clouds_, this->handler_t_, this->is_cuda_enable_, this->device_);
	  allocate_vbo_arrays();

	  run_glut();
}

void Thread::UploadCuda(unsigned short* depth, unsigned char* color, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, int rgb_center_x, int rgb_center_y,  double* Rmat, double* tvec)
{
	upload_depth_data_to_vbo_cuda_cloud(depth, color, depth_width, depth_height, depth_focal_length_x, depth_focal_length_y, rgb_focal_length_x, rgb_focal_length_y, rgb_center_x, rgb_center_y, Rmat, tvec);
}

void Thread::UploadChainCuda(std::vector<carmen_slam6d_kinect_fused_t> keyframe_chain, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, int rgb_center_x, int rgb_center_y)
{
	upload_chain_depth_data_to_vbo_cuda_cloud(keyframe_chain, depth_width, depth_height, depth_focal_length_x, depth_focal_length_y, rgb_focal_length_x, rgb_focal_length_y, rgb_center_x, rgb_center_y);
}

void Thread::Upload(float* vertices, float* colors)
{
	copy_data_to_vertices(vertices);
	copy_data_to_colors(colors);

	upload_data_to_vbo_array();
}

pthread_t Thread::GetId() {
        return m_threadId;
}
