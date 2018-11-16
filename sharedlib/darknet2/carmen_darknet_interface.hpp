#include <vector>


struct bbox_t
{
	unsigned int x, y, w, h;	// (x,y) - top-left corner, (w, h) - width & height of bounded box
	float prob;					// confidence - probability that the object was found correctly
	unsigned int obj_id;		// class of object - from range [0, classes-1]
	unsigned int track_id;		// tracking id for video (0 - untracked, 1 - inf - tracked object)
};


struct image_t
{
	int h;						// height
	int w;						// width
	int c;						// number of chanels (3 - for RGB)
	float *data;				// pointer to the image data
};


void
initialize_YOLO(char* data_file_name, char* list_file_name, char* cfg_file_name, char* weights_file_name,
		void *net, char **classes_names);

void
test_YOLO(void * net, char **classes_names, char * image_path);

////std::vector<bbox_t>
////extract_predictions(image img, detection *dets, int num, float thresh, char **classes_names, int classes_number, char* line);

////std::vector<bbox_t>
//run_YOLO(image *img, void * net, char **classes_name);
