#include <carmen/carmen_darknet_interface.hpp>
#include "neural_object_detector.hpp"


char **classes_names;
void *network_struct;


int
main(int argc, char **argv)
{
	initialize_YOLO((char*) "../../sharedlib/darknet2/cfg/coco.data", (char*) "../../sharedlib/darknet2/data/names.list",
			(char*) "../../sharedlib/darknet2/cfg/yolov3.cfg", (char*) "../../sharedlib/darknet2/yolov3.weights",
			network_struct, classes_names);

	printf("4\n");

	printf("---%s\n", classes_names[0]);

	if (network_struct == NULL)
			printf("6\n");

	printf("---%s\n", classes_names[0]);

	test_YOLO(network_struct, classes_names, (char*) "dog.jpeg");

	return 0;
}
