#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <cstdio>
#include <string>
#include <fstream>
#include <sstream>


using namespace std;
using namespace cv;

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

Mat image;
string window_name = "Roi";

int
main(int argc, char** argv)
{
    int image_number = 0;
    string input_file;
    ifstream input;

    if (argc != 3)
    {
        cerr << argv[0] << " input_info.txt folder" << endl;
        return EXIT_FAILURE;
    }

    input_file = argv[1];

    cerr << "Opening input_file with names of images" << endl;
    input.open(input_file.c_str());
    cerr << "Done." << endl;

    namedWindow(window_name, 1);

    if (input.is_open())
    {
        string line;

        getline(input, line);
        while (!input.eof())
        {

            cout << "Linha: " << line << endl;
            
            string s;
            vector<string> strings;
            istringstream iss(line);
            
            while (getline(iss, s, ' '))
            {
                cout << "Leitura: " << s << endl;
                strings.push_back(s);
            }

            cerr << "Loading image :" << strings.at(0) << endl;
            image = imread(strings.at(0), 1);

            if (!image.empty())
            {
                int x1 = atoi(strings.at(1).c_str());
                int y1 = atoi(strings.at(2).c_str());
                int x2 = atoi(strings.at(3).c_str());
                int y2 = atoi(strings.at(4).c_str());
                // rectangle(image, cvPoint(x1, y1), cvPoint(x2, y2), CV_RGB(255, 0, 255), 5);
                // imshow(window_name, image);

                Rect myROI(x1, y1, x2 - x1, y2 - y1);
                Mat croppedImage;
                Mat(image, myROI).copyTo(croppedImage);

                imshow(window_name, croppedImage);

                waitKey(50);
                
                image_number++;
                imwrite(argv[2]+ IntToString(image_number) + ".png", croppedImage);

            }
            getline(input, line);
            image.~Mat();
        }
    }
    else
    {
        cerr << "Failed to open: " << input_file << endl;
    }

    image.~Mat();
    input.close();
    destroyWindow(window_name);

    return EXIT_SUCCESS;
}
