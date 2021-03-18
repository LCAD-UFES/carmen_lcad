#ifndef __BBOX_H__
#define __BBOX_H__

// struct bbox_t
// {
// 	unsigned int x, y, w, h;	// (x,y) - top-left corner, (w, h) - width & height of bounded box
// 	float prob;					// confidence - probability that the object was found correctly
// 	unsigned int obj_id;		// class of object - from range [0, classes-1]
// 	unsigned int track_id;		// tracking id for video (0 - untracked, 1 - inf - tracked object)
// };

struct bbox_t {
    unsigned int x, y, w, h;       // (x,y) - top-left corner, (w, h) - width & height of bounded box
    float prob;                    // confidence - probability that the object was found correctly
    unsigned int obj_id;           // class of object - from range [0, classes-1]
    unsigned int track_id;         // tracking id for video (0 - untracked, 1 - inf - tracked object)
    unsigned int frames_counter;   // counter of frames on which the object was detected
    float x_3d, y_3d, z_3d;        // center of object (in Meters) if ZED 3D Camera is used
};

#endif