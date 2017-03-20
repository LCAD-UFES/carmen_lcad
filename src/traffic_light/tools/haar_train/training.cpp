#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <set>
#include <sstream>
#include <string.h>
#include <boost/algorithm/string/trim.hpp>

using namespace std;

struct GTData
{
    string name; // name of the file
    int leftCol; // left column of the region of interest (ROI) of the traffic sign
    int topRow; // upper row of the region of interest (ROI) of the traffic sign
    int rightCol; // right column of the region of interest (ROI) of the traffic sign
    int bottomRow; // lower row of the region of interest (ROI) of the traffic sign    
};

vector<GTData> search_traffic_signals_gt(vector<GTData> gt, string name);

vector<GTData>
TSD_readGTData(string aGTFile);

string negatives_file;

void
haar_create_samples_and_training(char *samples, char *training);

int
main(int argc, char** argv)
{
    if (argc != 6)
    {
        cerr << argv[0] << " train.txt negatives.txt gt_train.txt samples.txt training.txt" << endl;
//        cerr << argv[0] << " train.txt gt_train.txt samples.txt training.txt" << endl;
        return EXIT_FAILURE;
    }

    string pos_file = argv[1]; //"train.txt";
    negatives_file = argv[2]; //"negatives.txt";
    string gt_file = argv[3]; //"gt.txt";

    vector<GTData> gt = TSD_readGTData(gt_file);

    ifstream file_pos_samples;
    ofstream positive_train;

    file_pos_samples.open(pos_file.c_str());
    positive_train.open("positives_samples.txt");
    positive_train.clear();

    if (file_pos_samples.is_open() && positive_train.is_open())
    {
        string line;
        getline(file_pos_samples, line);
        while (!file_pos_samples.eof())
        {
            vector<GTData> aux;
            aux.clear();

            //pega todos os ground truth referentes a imagem da linha corrente (line)
            aux = search_traffic_signals_gt(gt, line);

            char newline[10000] = "";
            for (uint i = 0; i < aux.size(); i++)
            {
            	boost::trim(aux.at(i).name);

                if (i == 0)
                {
                    sprintf(newline, "%s %d %d %d %d %d", aux.at(i).name.c_str(), (int) aux.size(), aux.at(i).leftCol,
                                aux.at(i).topRow, (aux.at(i).rightCol - aux.at(i).leftCol), (aux.at(i).bottomRow - aux.at(i).topRow));
                    //test_full/
                } else
                {
                    char str_aux[100];
                    sprintf(str_aux, " %d %d %d %d", aux.at(i).leftCol, aux.at(i).topRow,
                            (aux.at(i).rightCol - aux.at(i).leftCol), (aux.at(i).bottomRow - aux.at(i).topRow));
                    strcat(newline, str_aux);
                }
            }
                      
            if (strlen(newline) != 0)
            {
                positive_train << newline << endl;
            }
            getline(file_pos_samples, line);

        }
        file_pos_samples.close();
        positive_train.close();
        if (system("rm -rf data"))
        {
        }
        cerr << "File of positives samples created" << endl;
        haar_create_samples_and_training(argv[4], argv[5]);
        cerr << "End of create samples and training" << endl;
    }
}

void
haar_create_samples_and_training(char *samples, char *training)
{
    ifstream haar_create_samples;
    haar_create_samples.open(samples);
    string line;

    //samples deve ser o arquivo com os parametros do opencv_createsamples
    if (haar_create_samples.is_open())
    {

        string samples = "opencv_createsamples -info positives_samples.txt -vec vectorfile.vec";

        while (!haar_create_samples.eof())
        {
            getline(haar_create_samples, line);
            samples.append(" ");
            samples.append(line);
        }
        cerr << "Running opencv_create_samples" << endl;
        if (system(samples.c_str()))
        {
        }
        cerr << "Create Samples command " << samples << endl;
    }
    else
    {
        cerr << "Error to open the file of create_samples" << endl;
    }

    ifstream haar_training;
    haar_training.open(training);

    //training deve ser o arquivo com os parametros do opencv_traincascade
    if (haar_training.is_open())
    {
//        if (system("rm data*"))
//        {
//        }

        string training;
        training = "opencv_traincascade -data data/ -vec vectorfile.vec -bg " + negatives_file;

        while (!haar_training.eof())
        {
            getline(haar_training, line);
            training.append(" ");
            training.append(line);
        }
        cerr << "Running opencv_train_cascade" << endl;

        if (system("mkdir data"))
        {
        }
        if (system(training.c_str()))
        {
        }
        cerr << "Training command " << training << endl;
        if (system("mv data/cascade.xml data_g.xml"))
        {
        }
        if (system("rm -rf data"))
        {
        }
    }
    else
    {
        cerr << "Error to open the file of training" << endl;
    }

}

vector<GTData>
TSD_readGTData(string aGTFile)
{
    ifstream file;
    file.open(aGTFile.c_str());

    vector<GTData> data_vector;

    if (file.is_open())
    {
        string line;
        getline(file, line);
        while (!file.eof())
        {
            string s;
            vector<string> strings;
            istringstream iss(line);
            while (getline(iss, s, ' '))
            {
            	boost::trim(s);
                strings.push_back(s);
            }

            GTData data;

            data.name = strings.at(0);
            data.leftCol = atoi(strings.at(1).c_str());
            data.topRow = atoi(strings.at(2).c_str());
            data.rightCol = atoi(strings.at(3).c_str());
            data.bottomRow = atoi(strings.at(4).c_str());

            data_vector.push_back(data);
            getline(file, line);
        }

    }
    else
    {
        cerr << "Can not Open File" << endl;
    }

    return data_vector;
}

vector<GTData>
search_traffic_signals_gt(vector<GTData> gt, string name)
{
    vector<GTData> aux;
    aux.clear();

    boost::trim(name);//this is needed when you use a awk generated file

    for (vector<GTData>::iterator it = gt.begin(); it != gt.end(); ++it)
    {
        if (it.base()->name == name)
        {
            GTData data;
            (data.name) = it.base()->name;
            (data.bottomRow) = it.base()->bottomRow;
            (data.leftCol) = it.base()->leftCol;
            (data.rightCol) = it.base()->rightCol;
            (data.topRow) = it.base()->topRow;

            aux.push_back(data);
        }
    }
    return aux;
}
