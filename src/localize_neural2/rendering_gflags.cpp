#include "rendering_gflags.h"

DEFINE_string(labels_list_file, "",
              "labels file");

DEFINE_string(config_file, "",
              "darknet config file");

DEFINE_string(weights_file, "",
              "network weights file");

DEFINE_string(images_list_file, "",
              "list of images used for training or validation");

DEFINE_string(running_type, "predict",
              "train|valid|predict");

DEFINE_string(output_acc_dir, "lcad_", "directory to save validation's output csv files");
