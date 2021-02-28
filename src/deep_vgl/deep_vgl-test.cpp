#include <iostream>
#include <fstream>
#include <string>
#include "network.h"
#include "utils.h"
#include "parser.h"
#include "option_list.h"
#include "blas.h"
#include "assert.h"
#include "image.h"
#include <sys/time.h>
#include <gflags/gflags.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>

using namespace std;
using namespace cv;

DEFINE_string(poses_and_labels, "", "poses and labels file");

DEFINE_string(config_file, "", "darknet config file");

DEFINE_string(weights_file, "", "network weights file in darknet format");

DEFINE_string(images_list, "", "list of images used for training, test or validation");

cv::Mat image_to_mat(image img)
{

    image copy = copy_image(img);
    constrain_image(copy);  
    int channels = img.c;
    int width = img.w;
    int height = img.h;
    cv::Mat mat = cv::Mat(height, width, CV_8UC(channels));
    int step = mat.step;

    for (int y = 0; y < img.h; ++y) {
        for (int x = 0; x < img.w; ++x) {
            for (int c = 0; c < img.c; ++c) {
                float val = img.data[c*img.h*img.w + y*img.w + x];
                mat.data[y*step + x*img.c + c] = (unsigned char)(val * 255);
            }
        }
    }

    if (mat.channels() == 3) cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    else if (mat.channels() == 4) cv::cvtColor(mat, mat, cv::COLOR_RGBA2BGR);

    return mat;
}

//teste utilizando a lista de imagens do treino para gerar como output o rótulo estimado
void
predict_classifier(char *labels, int classes_qtd, char *cfgfile, char *weightfile, char *filename)
{
    network net = parse_network_cfg_custom(cfgfile, 1, 0);
    if (weightfile)
        load_weights(&net, weightfile);

    set_batch_network(&net, 1);
    srand(2222222);

    fuse_conv_batchnorm(net);
    calculate_binary_weights(net);

    char *name_list = labels;
    int classes = classes_qtd;
    printf(" classes on label.txt = %d, classes in config.txt = %d \n", classes, net.layers[net.n - 3].c);
    layer l = net.layers[net.n - 1];
    if (classes != l.outputs && (l.type == SOFTMAX || l.type == COST))
    {
        printf("\n Error: num of filters = %d in the last conv-layer in cfg-file doesn't match to classes = %d in data-file \n",
               l.outputs, classes);
        getchar();
    }
    int top = 1;

    int i = 0;
    char **names = get_labels(name_list);
    clock_t time;
    int *indexes = (int *) xcalloc(top, sizeof(int));
    char input[1024];

    std::ifstream images_file;
    images_file.open(filename);
    std::string line;
    int last_label = -1;
    int iteration = 0;
    image im;
    image resized;
    image cropped;
    float *X;
    char pose_image_path[1024];
    double pose_X, pose_Y, pose_Yaw;
    
    if (images_file.is_open())
    {
        while (getline(images_file, line))
        {
            iteration++;
            strcpy(input, line.c_str());

            if (last_label == classes-1)
            {
                iteration = 1;
                last_label = 0;
            }

            im = load_image_color(input, 0, 0);
            resized = resize_min(im, net.w);
            cropped = crop_image(resized, (resized.w - net.w) / 2, (resized.h - net.h) / 2, net.w, net.h);

            X = cropped.data;

            double time = get_time_point();
            float *predictions = network_predict(net, X);
            
            top_k(predictions, net.outputs, 1, indexes);

            int index = indexes[0];

            // if (last_label > -1)
            // {
            //     if ((index >= last_label) && (index <= last_label + iteration))
            //     {
            //         iteration = 1;
            //         last_label = index;
            //     }
            //     else
            //     {
            //         index = last_label;
            //     }
            // }
            // else
            // {
            //     last_label = index;
            // }
           
            sscanf(names[index],"%lf %lf %lf %s",&pose_X,&pose_Y,&pose_Yaw, pose_image_path);

            printf("confidence: %4.2f, X: %lf, Y: %lf, Yaw: %lf, predicted_label: %03d, last_right_label: %03d, possible_label: %03d\n", predictions[index], pose_X, pose_Y, pose_Yaw, indexes[0], last_label, last_label + iteration); // output esperado
            
            Mat live_image = imread(input, IMREAD_COLOR);
            Mat pose_image;
            if (predictions[index] > 0.001)
            	pose_image = imread(pose_image_path, IMREAD_COLOR);
            else
            	pose_image = Mat::zeros(Size(live_image.cols, live_image.rows), live_image.type());
            Mat compare_images;
            hconcat(live_image, pose_image, compare_images);
            live_image.release();
            pose_image.release();
            imshow("DeepVGL", compare_images);
            int k = waitKey(1); // Wait for a keystroke in the window          
            compare_images.release();
            free_image(cropped);
            free_image(resized);
            free_image(im);
        }
    }
    else
        printf("Could not open images_file %s\n", filename);
    free_network(net);
}


int
main(int argc, char **argv)
{
    string config_file, weight_file,  poses_list, images_list; 
    ::google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_poses_and_labels == "" || FLAGS_weights_file == "" || FLAGS_config_file == "" || FLAGS_images_list == "")
    {
        printf(" Usage: %s --poses_and_labels config/poses_and_labels.txt  --weights_file config/classifier.weights --config_file config/config.cfg --images_list config/test.txt \n",argv[0]);
        exit(1);
    }
    else 
    {
        poses_list  = FLAGS_poses_and_labels;
        weight_file = FLAGS_weights_file;
        config_file = FLAGS_config_file;
        images_list = FLAGS_images_list;
    }
    int classes_qtd = 0;
    std::ifstream labels_file;
    std::string line;
    

    labels_file.open(poses_list.c_str());
    if (!labels_file)
    {
        printf("Could not open labels file %s\n", poses_list.c_str());
        exit(1);
    }

    while (getline(labels_file, line))
    {
        if (!line.empty())
            classes_qtd++;
    }

    predict_classifier((char *) poses_list.c_str(), classes_qtd, (char *) config_file.c_str(), (char *) weight_file.c_str(), (char *)images_list.c_str());

    return (0);
}
