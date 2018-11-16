#include "network.h"     // It can not be inside extern because include cuda and it use templates
extern "C" {
#include "data.h"
#include "demo.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "option_list.h"
#include "stb_image.h"
}
#include "../carmen_darknet_interface.hpp"


void
test_YOLO(void * net, char **classes_names, char * image_path);

void
copy_classes_names(char **classes_names, unsigned int classes_number)
{
	char** classes = (char**) malloc(classes_number * sizeof(char*));;
	printf("1\n");
	for (unsigned int i = 0; i < classes_number; ++i)
	{
		classes[i] = (char *) malloc (strlen(classes_names[i]) * sizeof (char));
		strcpy(classes[i], classes_names[i]);
		printf("---%s\n", classes[i]);
	}
	printf("2\n");
	classes_names = classes;
}

void
initialize_YOLO(char* data_file_name, char* list_file_name, char* cfg_file_name, char* weights_file_name,
		void *net, char **classes_names)
{
	network *network_config;

	list *options = read_data_cfg(data_file_name);

	char *name_list = option_find_str(options, (char*) "names", list_file_name);

	classes_names = get_labels(name_list);

	network_config = load_network(cfg_file_name, weights_file_name, 0);
	set_batch_network(network_config, 1);

	net = network_config;

	copy_classes_names(classes_names, network_config->layers[network_config->n-1].classes);

	//printf("---%d\n", network_config->layers[network_config->n-1].classes);

	//test_YOLO(network_config, classes_names, (char*) "dog.jpeg");

	if (classes_names == NULL)
			printf("NULL\n");

	printf("3\n");
}


void
test_YOLO(void * net, char **classes_names, char * image_path)
{
	char buff[256], *input = buff;
	float nms = 0.45;
	int nboxes = 0;

	strncpy(input, image_path, 256);

	image im = load_image_color(input,0,0);

	network *network_config = (network*) net;

	image **alphabet = load_alphabet();

	image sized = letterbox_image(im, network_config->w, network_config->h);

	//layer l = network_config->layers[network_config->n-1];

	float *X = sized.data;

	network_predict(network_config, X);

	//printf("%s: Predicted in %f seconds.\n", input, what_time_is_it_now()-time);

	detection *dets = get_network_boxes(network_config, im.w, im.h, 0.5, 0.5, 0, 1, &nboxes);

	if (nms)
		do_nms_sort(dets, nboxes, 80, nms);
	//do_nms_sort(dets, nboxes, l.classes, nms); // l.classes do not work when darknet is compiled for cuda set l.classes = 80 to coco dataset

	draw_detections(im, dets, nboxes, 0.5, classes_names, alphabet, 80);

	save_image(im, "predictions");

	free_image(im);
	free_image(sized);
}

//
//std::vector<bbox_t>
//extract_predictions(image img, detection *dets, int num, float thresh, char **classes_names, int classes_number, char* line)
//{
//	int i,j;
//	std::vector<bbox_t> bbox_vector;
//
//	for(i = 0; i < num; ++i)
//	{
//		char labelstr[4096] = {0};
//		int obj_id = -1;
//
//		for(j = 0; j < classes_number; ++j)
//		{
//			if (dets[i].prob[j] > thresh)
//			{
//				if (obj_id < 0)   //TODO Get the class with the higher probability
//				{
//					strcat(labelstr, classes_names[j]);
//					obj_id = j;
//				}
//				else
//				{
//					strcat(labelstr, ", ");
//					strcat(labelstr, classes_names[j]);
//				}
//				//printf("%s: %.0f%%\n", classes_names[j], dets[i].prob[j]*100);
//			}
//		}
//		if (obj_id >= 0)
//		{
//			box b = dets[i].bbox;
//
//			int left  = (b.x - (b.w / 2.0)) * img.w;
//			int right = (b.x + (b.w / 2.0)) * img.w;
//			int top   = (b.y - (b.h / 2.0)) * img.h;
//			int bot   = (b.y + (b.h / 2.0)) * img.h;
//
//			if(left < 0)
//				left = 0;
//			if(right > img.w-1)
//				right = img.w-1;
//			if(top < 0)
//				top = 0;
//			if(bot > img.h-1)
//				bot = img.h-1;
//
//			bbox_t bbox;
//
//			bbox.x = left;
//			bbox.y = top;
//			bbox.w = right - left;
//			bbox.h = bot - top;
//			bbox.obj_id = obj_id;
//			bbox.prob = dets[i].prob[obj_id]*100;
//			bbox.track_id = 0;
//
//			bbox_vector.push_back(bbox);
//		}
//	}
//    return (bbox_vector);
//}


//std::vector<bbox_t>
//run_YOLO(image *img, void * net, char **classes_name)
//{
//	char buff[256], *input = buff;
//	float nms = 0.45;
//	int nboxes = 0;
//
//	//save_image(*img, "predictions");
//
//	network *network_config = (network*) net;
//
//	image sized = letterbox_image(*img, network_config->w, network_config->h);
//
//	//layer l = network_config->layers[network_config->n-1];
//
//	float *X = sized.data;
//
//	network_predict(network_config, X);
//
//	detection *dets = get_network_boxes(network_config, img->w, img->h, 0.5, 0.5, 0, 1, &nboxes);
//
//	if (nms)
//		do_nms_sort(dets, nboxes, 80, nms);
//		//do_nms_sort(dets, nboxes, l.classes, nms); // l.classes do not work when darknet is compiled for cuda set l.classes = 80 to coco dataset
//
//	std::vector<bbox_t> bbox_vector = extract_predictions(*img, dets, nboxes, 0.5, classes_name, 80, input);
//
//	free_image(sized);
//	return (bbox_vector);
//}



/*
struct detector_gpu_t{
	float **probs;
	box *boxes;
	network net;
	image images[FRAMES];
	float *avg;
	float *predictions[FRAMES];
	int demo_index;
	unsigned int *track_id;
};


YOLODLL_API Detector::Detector(std::string cfg_filename, std::string weight_filename, int gpu_id)
{
	int old_gpu_index;
#ifdef GPU
	cudaGetDevice(&old_gpu_index);
#endif

	detector_gpu_ptr = std::make_shared<detector_gpu_t>();
	detector_gpu_t &detector_gpu = *reinterpret_cast<detector_gpu_t *>(detector_gpu_ptr.get());

#ifdef GPU
	cudaSetDevice(gpu_id);
#endif
	network &net = detector_gpu.net;
	net.gpu_index = gpu_id;
	//gpu_index = i;

	char *cfgfile = const_cast<char *>(cfg_filename.data());
	char *weightfile = const_cast<char *>(weight_filename.data());

	net = parse_network_cfg_custom(cfgfile, 1);
	if (weightfile) {
		load_weights(&net, weightfile);
	}
	set_batch_network(&net, 1);
	net.gpu_index = gpu_id;

	layer l = net.layers[net.n - 1];
	int j;

	detector_gpu.avg = (float *)calloc(l.outputs, sizeof(float));
	for (j = 0; j < FRAMES; ++j) detector_gpu.predictions[j] = (float *)calloc(l.outputs, sizeof(float));
	for (j = 0; j < FRAMES; ++j) detector_gpu.images[j] = make_image(1, 1, 3);

	detector_gpu.boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
	detector_gpu.probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
	for (j = 0; j < l.w*l.h*l.n; ++j) detector_gpu.probs[j] = (float *)calloc(l.classes, sizeof(float));

	detector_gpu.track_id = (unsigned int *)calloc(l.classes, sizeof(unsigned int));
	for (j = 0; j < l.classes; ++j) detector_gpu.track_id[j] = 1;

#ifdef GPU
	cudaSetDevice(old_gpu_index);
#endif
}


YOLODLL_API Detector::~Detector()
{
	detector_gpu_t &detector_gpu = *reinterpret_cast<detector_gpu_t *>(detector_gpu_ptr.get());
	layer l = detector_gpu.net.layers[detector_gpu.net.n - 1];

	free(detector_gpu.track_id);

	free(detector_gpu.avg);
	for (int j = 0; j < FRAMES; ++j) free(detector_gpu.predictions[j]);
	for (int j = 0; j < FRAMES; ++j) if(detector_gpu.images[j].data) free(detector_gpu.images[j].data);

	for (int j = 0; j < l.w*l.h*l.n; ++j) free(detector_gpu.probs[j]);
	free(detector_gpu.boxes);
	free(detector_gpu.probs);

	int old_gpu_index;
#ifdef GPU
	cudaGetDevice(&old_gpu_index);
	cudaSetDevice(detector_gpu.net.gpu_index);
#endif

	//free_network(detector_gpu.net);

#ifdef GPU
	cudaSetDevice(old_gpu_index);
#endif
}

YOLODLL_API int Detector::get_net_width() const {
	detector_gpu_t &detector_gpu = *reinterpret_cast<detector_gpu_t *>(detector_gpu_ptr.get());
	return detector_gpu.net.w;
}
YOLODLL_API int Detector::get_net_height() const {
	detector_gpu_t &detector_gpu = *reinterpret_cast<detector_gpu_t *>(detector_gpu_ptr.get());
	return detector_gpu.net.h;
}


YOLODLL_API std::vector<bbox_t> Detector::detect(std::string image_filename, float thresh, bool use_mean)
{
	std::shared_ptr<image_t> image_ptr(new image_t, [](image_t *img) { if (img->data) free(img->data); delete img; });
	*image_ptr = load_image(image_filename);
	return detect(*image_ptr, thresh, use_mean);
}

static image load_image_stb(char *filename, int channels)
{
	int w, h, c;
	unsigned char *data = stbi_load(filename, &w, &h, &c, channels);
	if (!data)
		throw std::runtime_error("file not found");
	if (channels) c = channels;
	int i, j, k;
	image im = make_image(w, h, c);
	for (k = 0; k < c; ++k) {
		for (j = 0; j < h; ++j) {
			for (i = 0; i < w; ++i) {
				int dst_index = i + w*j + w*h*k;
				int src_index = k + c*i + c*w*j;
				im.data[dst_index] = (float)data[src_index] / 255.;
			}
		}
	}
	free(data);
	return im;
}

YOLODLL_API image_t Detector::load_image(std::string image_filename)
{
	char *input = const_cast<char *>(image_filename.data());
	image im = load_image_stb(input, 3);

	image_t img;
	img.c = im.c;
	img.data = im.data;
	img.h = im.h;
	img.w = im.w;

	return img;
}


YOLODLL_API void Detector::free_image(image_t m)
{
	if (m.data) {
		free(m.data);
	}
}

YOLODLL_API std::vector<bbox_t> Detector::detect(image_t img, float thresh, bool use_mean)
{

	detector_gpu_t &detector_gpu = *reinterpret_cast<detector_gpu_t *>(detector_gpu_ptr.get());
	network &net = detector_gpu.net;
	int old_gpu_index;
#ifdef GPU
	cudaGetDevice(&old_gpu_index);
	cudaSetDevice(net.gpu_index);
#endif
	//std::cout << "net.gpu_index = " << net.gpu_index << std::endl;

	//float nms = .4;

	image im;
	im.c = img.c;
	im.data = img.data;
	im.h = img.h;
	im.w = img.w;

	image sized;

	if (net.w == im.w && net.h == im.h) {
		sized = make_image(im.w, im.h, im.c);
		memcpy(sized.data, im.data, im.w*im.h*im.c * sizeof(float));
	}
	else
		sized = resize_image(im, net.w, net.h);

	layer l = net.layers[net.n - 1];

	float *X = sized.data;

	float *prediction = network_predict(&net, X);

	if (use_mean) {
		memcpy(detector_gpu.predictions[detector_gpu.demo_index], prediction, l.outputs * sizeof(float));
		mean_arrays(detector_gpu.predictions, FRAMES, l.outputs, detector_gpu.avg);
		l.output = detector_gpu.avg;
		detector_gpu.demo_index = (detector_gpu.demo_index + 1) % FRAMES;
	}

	get_region_boxes(l, 1, 1, thresh, detector_gpu.probs, detector_gpu.boxes, 0, 0);
	//if (nms) do_nms_sort(detector_gpu.boxes, detector_gpu.probs, l.w*l.h*l.n, l.classes, nms);
	//draw_detections(im, l.w*l.h*l.n, thresh, boxes, probs, names, alphabet, l.classes);

	std::vector<bbox_t> bbox_vec;

	for (size_t i = 0; i < (unsigned int)(l.w*l.h*l.n); ++i) {
		box b = detector_gpu.boxes[i];
		int const obj_id = max_index(detector_gpu.probs[i], l.classes);
		float const prob = detector_gpu.probs[i][obj_id];

		if (prob > thresh)
		{
			bbox_t bbox;
			bbox.x = std::max((double)0, (b.x - b.w / 2.)*im.w);
			bbox.y = std::max((double)0, (b.y - b.h / 2.)*im.h);
			bbox.w = b.w*im.w;
			bbox.h = b.h*im.h;
			bbox.obj_id = obj_id;
			bbox.prob = prob;
			bbox.track_id = 0;

			bbox_vec.push_back(bbox);
		}
	}

	if(sized.data)
		free(sized.data);

#ifdef GPU
	cudaSetDevice(old_gpu_index);
#endif

	return bbox_vec;
}

YOLODLL_API std::vector<bbox_t> Detector::tracking(std::vector<bbox_t> cur_bbox_vec, int const frames_story)
{
	detector_gpu_t &det_gpu = *reinterpret_cast<detector_gpu_t *>(detector_gpu_ptr.get());

	bool prev_track_id_present = false;
	for (auto &i : prev_bbox_vec_deque)
		if (i.size() > 0) prev_track_id_present = true;

	if (!prev_track_id_present) {
		for (size_t i = 0; i < cur_bbox_vec.size(); ++i)
			cur_bbox_vec[i].track_id = det_gpu.track_id[cur_bbox_vec[i].obj_id]++;
		prev_bbox_vec_deque.push_front(cur_bbox_vec);
		if (prev_bbox_vec_deque.size() > (unsigned int)frames_story) prev_bbox_vec_deque.pop_back();
		return cur_bbox_vec;
	}

	std::vector<unsigned int> dist_vec(cur_bbox_vec.size(), std::numeric_limits<unsigned int>::max());

	for (auto &prev_bbox_vec : prev_bbox_vec_deque) {
		for (auto &i : prev_bbox_vec) {
			int cur_index = -1;
			for (size_t m = 0; m < cur_bbox_vec.size(); ++m) {
				bbox_t const& k = cur_bbox_vec[m];
				if (i.obj_id == k.obj_id) {
					float center_x_diff = (float)(i.x + i.w/2) - (float)(k.x + k.w/2);
					float center_y_diff = (float)(i.y + i.h/2) - (float)(k.y + k.h/2);
					unsigned int cur_dist = sqrt(center_x_diff*center_x_diff + center_y_diff*center_y_diff);
					if (cur_dist < 100 && (k.track_id == 0 || dist_vec[m] > cur_dist)) {
						dist_vec[m] = cur_dist;
						cur_index = m;
					}
				}
			}

			bool track_id_absent = !std::any_of(cur_bbox_vec.begin(), cur_bbox_vec.end(),
				[&i](bbox_t const& b) { return b.track_id == i.track_id && b.obj_id == i.obj_id; });

			if (cur_index >= 0 && track_id_absent){
				cur_bbox_vec[cur_index].track_id = i.track_id;
				cur_bbox_vec[cur_index].w = (cur_bbox_vec[cur_index].w + i.w) / 2;
				cur_bbox_vec[cur_index].h = (cur_bbox_vec[cur_index].h + i.h) / 2;
			}
		}
	}

	for (size_t i = 0; i < cur_bbox_vec.size(); ++i)
		if (cur_bbox_vec[i].track_id == 0)
			cur_bbox_vec[i].track_id = det_gpu.track_id[cur_bbox_vec[i].obj_id]++;

	prev_bbox_vec_deque.push_front(cur_bbox_vec);
	if (prev_bbox_vec_deque.size() > (unsigned int)frames_story) prev_bbox_vec_deque.pop_back();

	return cur_bbox_vec;
}
*/
