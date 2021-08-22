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

DEFINE_string(labels, "", "labels file");

DEFINE_string(config_file, "", "darknet config file");

DEFINE_string(weights_file, "", "network weights file in darknet format");

DEFINE_string(images_list, "", "list of images used for training, test or validation");


//teste utilizando a lista de imagens do treino para gerar como output o rótulo estimado
void predict_classifier(char *labels, int classes_qtd,char *cfgfile, char *weightfile, char *filename, int img_qtd)
{
    network net = parse_network_cfg_custom(cfgfile, 1, 0);
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 1);
    srand(2222222);

    fuse_conv_batchnorm(net);
    calculate_binary_weights(net);

    char *name_list = labels;
    int classes = classes_qtd;
    printf(" classes on label.txt = %d, classes in config.txt = %d \n", classes, net.layers[net.n - 3].c);
    layer l = net.layers[net.n - 1];
    if (classes != l.outputs && (l.type == SOFTMAX || l.type == COST)) {
        printf("\n Error: num of filters = %d in the last conv-layer in cfg-file doesn't match to classes = %d in data-file \n",
            l.outputs, classes);
        getchar();
    }
    int top = 1;

    int i = 0;
    char **names = get_labels(name_list);
    clock_t time;
    int* indexes = (int *) xcalloc(top, sizeof(int));
    char buff[1024];
    char *input = buff;
    
    float avg_mae_0 = 0;
    float avg_mae_1 = 0;
    float avg_mae_2 = 0;
    std::string pred_class;
    int mae1g=0,mae1l=0,mae2g=0,mae2l=0;

    float prediction_time=0.0;
    char pred_time[1500];
    char output_avg[1500];

    std::ifstream images_file;
    images_file.open(filename);
    std::string line;
    printf("\nAbrindo images_file %s\n", filename);
    image im;
    image resized;
    image cropped;
    if (images_file.is_open())
    {
        while( getline(images_file,line)){    
            i+=1;        
            strcpy(input, line.c_str());
            im = load_image_color(input, 0, 0);
            resized = resize_min(im, net.w);
            cropped = crop_image(resized, (resized.w - net.w)*0.5, (resized.h - net.h)*0.5, net.w, net.h);
            //printf("%d %d\n", cropped.w, cropped.h);

            float *X = cropped.data;

            double time = get_time_point();
            float *predictions = network_predict(net, X);
            prediction_time = ((double)get_time_point() - time) * 0.001;
            //sprintf(pred_time,"%s: Predicted in %lf milli-seconds.", input, ((double)get_time_point() - time) / 1000);

            //if(net.hierarchy) hierarchy_predictions(predictions, net.outputs, net.hierarchy, 0);
            top_k(predictions, net.outputs, 1, indexes);

            int index = indexes[0];
            std::string::size_type sz;   // alias of size_t
            std::string temp_name = names[index];
            pred_class = temp_name.substr(1,temp_name.length()-1);
            mae1g = std::stoi(pred_class,&sz)+1;
            mae1l = std::stoi(pred_class,&sz)-1;
            mae2g = std::stoi(pred_class,&sz)+2;
            mae2l = std::stoi(pred_class,&sz)-2;
            std::string prefix_class = "B";
            std::string greater1_class = std::to_string(mae1g);
            std::string less1_class = std::to_string(mae1l);
            greater1_class = prefix_class+greater1_class+"E";
            less1_class = prefix_class+less1_class+"E";
            prefix_class = "B";
            std::string greater2_class = std::to_string(mae2g);
            std::string less2_class = std::to_string(mae2l);
            greater2_class = prefix_class+greater2_class+"E";
            less2_class = prefix_class+less2_class+"E";
            if(strstr(input, names[index])){
                avg_mae_0+=1;
                avg_mae_1+=1;
                avg_mae_2+=1;
            }
            else if(strstr(input, greater1_class.c_str()) || strstr(input, less1_class.c_str()) ){
                avg_mae_1+=1;
                avg_mae_2+=1;
            }
            else if(strstr(input, greater2_class.c_str()) || strstr(input, less2_class.c_str()) ){
                avg_mae_2+=1;
            }
            
            // if(net.hierarchy) printf("%d, %s: %f, parent: %s \n",index, names[index], predictions[index], (net.hierarchy->parent[index] >= 0) ? names[net.hierarchy->parent[index]] : "Root");
            // else 
            //sprintf(output_avg,"avg_mae_0: %f; avg_mae_1: %f; avg_mae_2: %f - %s\n",avg_mae_0/(i),avg_mae_1/(i),avg_mae_2/(i),pred_time); // output esperado 
            //int percentual = (i*100/img_qtd);
            //printf("%05d / %d \r",i,img_qtd);
            std::cout<< i <<" / "<<img_qtd<<"\r" <<std::flush;

            free_image(cropped);
            if (resized.data != im.data) {
                free_image(resized);
            }
            free_image(im);
        }
    }
    else
    	printf("Nao foi possivel abrir %s\n", filename);

    free(indexes);
    free_network(net);

    printf("avg_mae_0: %f; avg_mae_1: %f; avg_mae_2: %f - %f mili sec\n",avg_mae_0/(i),avg_mae_1/(i),avg_mae_2/(i),prediction_time);
}

int main(int argc, char **argv)
{
     string config_file, weight_file,  poses_list, images_list; 
    ::google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_labels == "" || FLAGS_weights_file == "" || FLAGS_config_file == "" || FLAGS_images_list == "")
    {
        printf(" Usage: %s --labels config/labels.txt  --weights_file config/classifier.weights --config_file config/config.cfg --images_list config/test.txt \n",argv[0]);
        exit(1);
    }
    else 
    {
        poses_list  = FLAGS_labels;
        weight_file = FLAGS_weights_file;
        config_file = FLAGS_config_file;
        images_list = FLAGS_images_list;
    }
    int classes_qtd = 0, img_qtd=0;
    std::ifstream labels_file,images_file;
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
    
    images_file.open(images_list.c_str());
    while(getline(images_file, line))
    {
        if (!line.empty())
            img_qtd++;
    }



    predict_classifier((char *) poses_list.c_str(), classes_qtd, (char *) config_file.c_str(), (char *) weight_file.c_str(), (char *)images_list.c_str(), img_qtd);

    return 0;
}
