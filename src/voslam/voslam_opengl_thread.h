#ifndef CARMEN_VOSLAM_OPENGL_THREAD_H_
#define CARMEN_VOSLAM_OPENGL_THREAD_H_

#include <pthread.h>
#include "voslam_opengl_interface.h"
#include "voslam_keyframes.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

class VoslamOpenGLThread {
        public:
				VoslamOpenGLThread();
                ~VoslamOpenGLThread();

                void Start(int argc, char **argv, int point_cloud_size, int point_cloud_dim, int number_of_point_clouds, void (*handler_t)(int), int is_cuda_enable, int device);
                void UploadChain(VoslamKeyframes keyframe_chain, int depth_width, int depth_height);
                void Wait();
                void Abort();
                void Detach();
                void Run();

#ifndef NO_CUDA
 				void UploadChainCuda(std::vector<carmen_voslam_pointcloud_t> keyframe_chain, int depth_width, int depth_height, float depth_focal_length_x, float depth_focal_length_y, float rgb_focal_length_x, float rgb_focal_length_y, int rgb_center_x, int rgb_center_y);
#endif

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
                GLfloat *vertices;
                GLfloat *colors;

                void CopyVerticesAndColorFromFrame(carmen_voslam_pointcloud_t* frame, int width, int height);
};

#endif /* SLAM6D_OPENGL_THREAD_H_ */
