
#ifndef SLAM6D_MAIN_H_
#define SLAM6D_MAIN_H_

typedef struct {
	bool is_keyframe;
	bool is_loop_closure;
	int loop_partner_index;
	double position[3];
	double rotation[9];
	unsigned short *depth;
	unsigned char* image;
	double depth_timestamp;
	double image_timestamp;
} carmen_slam6d_kinect_fused_t;

#endif /* SLAM6D_MAIN_H_ */
