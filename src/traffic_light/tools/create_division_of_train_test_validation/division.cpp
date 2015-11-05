#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>

// for filelisting
#include <cstdio>
#include <sys/io.h>
// for fileoutput
#include <string>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>
#include <vector>

using namespace std;
using namespace cv;
vector<string> used;

int
has_used(string image);

int
main(int argc, char** argv)
{
    string input_file;
    ifstream input;
    ofstream output_train;
    ofstream output_test;
    ofstream output_validation;
    vector<string> lista_imagens;

    if (argc != 2)
    {
        cerr << "Use : " << argv[0] << " input_info.txt" << endl;
        return -1;
    }

    input_file = argv[1];


    cerr << "Opening input_file with names of images" << endl;
    input.open(input_file.c_str());
    cerr << "Done." << endl;

    output_train.open("train.txt");
    output_test.open("test.txt");
    output_validation.open("validation.txt");

    output_train.clear();
    output_test.clear();
    output_validation.clear();


    if (input.is_open())
    {
        string line;

        getline(input, line);
        while (!input.eof())
        {
            lista_imagens.push_back(line);
            getline(input, line);
        }
    }

    cout << "Size " << lista_imagens.size() << endl;
    for (uint i = 0; i < lista_imagens.size(); i++)
    {
        cout << "Position: " << i << " Nome " << lista_imagens.at(i) << endl;
    }

    srand(time(NULL));
    for (uint i = 0; i < (lista_imagens.size() / 3) + 2; i++)
    {
        int aux , cont;
        cont = -1;
        while (cont != 0)
        {
            aux = rand() % lista_imagens.size();
            if (!has_used(lista_imagens.at(aux)))
            {
                cont = 0;
            }
            else
            {
                cont = -1;
            }
        }
        used.push_back(lista_imagens.at(aux));
        output_train << lista_imagens.at(aux) << endl;
    }

    for (uint i = 0; i < (lista_imagens.size() / 3); i++)
    {
        int aux, cont;
        cont = -1;
        while (cont != 0)
        {
            aux = rand() % lista_imagens.size();
            if (!has_used(lista_imagens.at(aux)))
            {
                cont = 0;
            }
            else
            {
                cont = -1;
            }
        }
        used.push_back(lista_imagens.at(aux));
        output_test << lista_imagens.at(aux) << endl;
    }
    for (uint i = 0; i < (lista_imagens.size()); i++)
    {
        if (!has_used(lista_imagens.at(i)))
        {
            output_validation << lista_imagens.at(i) << endl;
        }
    }


    return EXIT_SUCCESS;
}

int
has_used(string image)
{
    for (uint i = 0; i < used.size(); i++)
    {
        if (image == used.at(i))
        {
            return 1;
        }
    }
    return 0;
}
