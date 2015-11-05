#ifndef SLAM6D_OPENGL_THREAD_H_
#define SLAM6D_OPENGL_THREAD_H_

#include <pthread.h>
#include "slam6d_opengl_interface.h"

class Thread {
        public:
                Thread();
                ~Thread();

                void Start(int argc, char **argv, int point_cloud_size, int point_cloud_dim, int number_of_point_clouds, void (*handler_t)(int), int is_cuda_enable, int device);
                void UploadCuda(unsigned short* depth, unsigned char* color, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, int rgb_center_x, int rgb_center_y,  double* Rmat, double* tvec);
                void UploadChainCuda(std::vector<carmen_slam6d_kinect_fused_t> keyframe_chain, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, int rgb_center_x, int rgb_center_y);
		void Upload(float* vertices, float* colors);
                void Wait();
                void Abort();
                void Detach();
                void Run();

                pthread_t GetId();

        protected:
                int argc_;
                char **argv_;
                int point_cloud_size_;
                int point_cloud_dim_;
                int number_of_point_clouds_;
                void (*handler_t_)(int);
                int is_cuda_enable_;
                int device_;


                static void *entryPoint(void *pthis);

        private:
                pthread_t m_threadId;
};

#endif /* SLAM6D_OPENGL_THREAD_H_ */
