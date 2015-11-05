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

using namespace std;
using namespace cv;

vector<string> train;

string
IntToString(int num)
{
    ostringstream myStream; //creates an ostringstream object
    myStream << num << flush;
    /*
     * outputs the number into the string stream and then flushes
     * the buffer (makes sure the output is put into the stream)
     */
    return (myStream.str()); //returns the string form of the stringstream object
};

int
main(int argc, char** argv)
{
    string input_file;
    string gt_file;
    string output_file;
    ifstream input;
    ifstream gt;
    ofstream output;

    if (argc != 4)
    {
        cerr << argv[0] << " train.txt gt.txt gt_filtered.txt" << endl;
        return -1;
    }

    input_file = argv[1];
    gt_file = argv[2];
    output_file = argv[3];

    cerr << "Opening input_file with names of images" << endl;
    input.open(input_file.c_str());
    gt.open(gt_file.c_str());
    cerr << "Done." << endl;

    output.open(output_file.c_str());


    if (input.is_open())
    {
        string line;

        getline(input, line);
        while (!input.eof())
        {
            train.push_back(line);
            getline(input, line);
        }
    }
    else
    {
        cerr << "Failed to open: " << input_file << endl;
    }

    if (gt.is_open())
    {
        string line;

        getline(gt, line);
        while (!gt.eof())
        {
            string s;
            vector<string> strings;
            istringstream iss(line);
            while (getline(iss, s, ' '))
            {
                cout << "Leitura: " << s << endl;
                strings.push_back(s);
            }
            for (uint i =0 ; i < train.size();i++){
                if (strings.at(0)== train.at(i)){
                    output << line << endl;
                }
            }
            getline(gt, line);
        }
    }
    else
    {
        cerr<<"Failed to open: "<< gt_file << endl;
    }



    return EXIT_SUCCESS;
}
