#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <set>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/contrib/detection_based_tracker.hpp>


using namespace std;
using namespace cv;

Mat image;

struct GTData
{
    string name; // name of the file
    int leftCol; // left column of the region of interest (ROI) of the traffic sign
    int topRow; // upper row of the region of interest (ROI) of the traffic sign
    int rightCol; // right column of the region of interest (ROI) of the traffic sign
    int bottomRow; // lower row of the region of interest (ROI) of the traffic sign    
};

string ts_cascade_name = "data.xml";
CascadeClassifier ts_cascade;
string window_name = "Capture - Traffic Sign detection. Image: ";
RNG rng(12345);

string IntToString(int num);

std::vector<GTData> TSD_readGTData(string aGTFile);

std::vector<GTData> search_traffic_signals_gt(std::vector<GTData> gt, string name);

void haar_create_samples_and_training(char *samples, char *training);

string detectAndDisplay(Mat frame, string image, char *verbose, string aux);

void TSD_testMyDetector(bool verbose, std::string benchmark_file, std::string gt_file, std::string test, std::string result);

int
main(int argc, char **argv)
{

    if (argc != 10)
    {
        cerr << argv[0] << " train.txt gt_train.txt gt_test.txt images.txt verbose result.txt result_files.txt samples.txt training.txt" << endl;
        return -1;
    }

    string pos_file = argv[1]; //"train.txt";
    string gt_file = argv[2]; //"gt.txt";

    std::vector<GTData> gt = TSD_readGTData(gt_file);

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
            std::vector<GTData> aux;
            aux.clear();

            aux = search_traffic_signals_gt(gt, line);

            char newline[10000] = "";
            int pos =0;
            for (uint i = 0; i < aux.size(); i++)
            {                
                //Limiting the size of traffic light x > 8 and y > 15
                if (((aux.at(i).bottomRow - aux.at(i).topRow) > 15) && ((aux.at(i).rightCol - aux.at(i).leftCol) > 8))
                {
                    if (pos == 0)
                    {
                        sprintf(newline, "../%s %d %d %d %d %d", aux.at(i).name.c_str(), (int) aux.size(), aux.at(i).leftCol,
                                aux.at(i).topRow, (aux.at(i).rightCol - aux.at(i).leftCol), (aux.at(i).bottomRow - aux.at(i).topRow));
                        //test_full/
                    }
                    else
                    {
                        char str_aux[100];
                        sprintf(str_aux, " %d %d %d %d", aux.at(i).leftCol, aux.at(i).topRow,
                                (aux.at(i).rightCol - aux.at(i).leftCol), (aux.at(i).bottomRow - aux.at(i).topRow));
                        strcat(newline, str_aux);
                    }
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
        haar_create_samples_and_training(argv[8], argv[9]);
        cerr << "End of create samples and training" << endl;
    }
    Mat frame;
    ifstream file;
    ofstream result_file;
    string image;
    char ikey = 0;
    result_file.open(argv[6]);

    cerr << "Testing" << endl;
    //-- 1. Load the cascades
    if (!ts_cascade.load(ts_cascade_name))
    {
        cerr << "--(!)Error loading" << endl;
        return -1;
    };

    //-- 2. Reading the file of images
    file.open(argv[4]);
    if (file.is_open())
    {
        char *verbos = argv[5];
        getline(file, image);
        //-- 3. Loop to detect the words
        while (!file.eof())
        {
            string aux = image;
            image = "../" + image;
            frame = cvLoadImageM(image.c_str(), 1);

            //-- 4. Apply the classifier to the frame
            if (!frame.empty())
            {
                string newline = detectAndDisplay(frame, image, verbos, aux);
                result_file << newline;
            }
            else
            {
                cerr << " --(!) No image has informed -- Break!" << endl;
                return 1;
            }
            if (verbos[0] == '1')
            {
                //-- 5. Lookin the key for exit or continue
                ikey = waitKey(0);
                switch (ikey)
                {
                case 27:
                {
                    cvDestroyAllWindows();
                    return 0;
                }
                default:
                {
                }
                    cvDestroyAllWindows();
                }
            }
            getline(file, image);
        }
    }
    result_file.close();

    cerr << "Calculation of precision and recall" << endl;

    bool verbose = false;
    TSD_testMyDetector(verbose, argv[7], argv[3], argv[4], argv[6]);

    cerr << "End of calculation of precision and recall" << endl;
    return EXIT_SUCCESS;
}

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
}

std::vector<GTData>
TSD_readGTData(string aGTFile)
{
    ifstream file;
    file.open(aGTFile.c_str());

    std::vector<GTData> data_vector;

    if (file.is_open())
    {
        string line;
        getline(file, line);
        while (!file.eof())
        {
            string s;
            std::vector<string> strings;
            istringstream iss(line);
            while (getline(iss, s, ' '))
            {
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

std::vector<GTData>
search_traffic_signals_gt(std::vector<GTData> gt, string name)
{
    std::vector<GTData> aux;
    aux.clear();

    for (std::vector<GTData>::iterator it = gt.begin(); it != gt.end(); ++it)
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

void
haar_create_samples_and_training(char *samples, char *training)
{
    ifstream haar_create_samples;
    haar_create_samples.open(samples);
    string line;

    if (haar_create_samples.is_open())
    {

        string samples = "opencv_createsamples -info positives_samples.txt -vec std::vectorfile.vec";

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

    if (haar_training.is_open())
    {
        if (system("rm data*"))
        {
        }

        string training = "opencv_traincascade -data data/ -vec std::vectorfile.vec  -bg ../negatives.txt ";

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
        if (system("mv data/cascade.xml data.xml"))
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

string
detectAndDisplay(Mat frame, string image, char *verbose, string aux)
{
    char newline[100000] = "";

    std::vector<Rect> semaphores;
    Mat frame_gray;

    cvtColor(frame, frame_gray, CV_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
    //-- Detect traffic lights  
    ts_cascade.detectMultiScale(frame_gray, semaphores, 1.1, 5, 0, Size(4, 9), Size(70, 130));

    for (size_t i = 0; i < semaphores.size(); i++)
    {
        CvPoint p1, p2;
        p1.x = semaphores[i].x;
        p1.y = semaphores[i].y;
        p2.x = p1.x + semaphores[i].width;
        p2.y = p1.y + semaphores[i].height;
        rectangle(frame, p1, p2, CV_RGB(255, 0, 0), 3, 8, 0);

        sprintf(newline, "%s%s %d %d %d %d\n", newline, aux.c_str(), p1.x, p1.y, p2.x, p2.y);
    }

    cerr << "Image " << image << endl;
    //-- Show what you got
    if (verbose[0] == '1')
    {
        imshow(window_name + image, frame);
    }
    string result = newline;
    //imwrite("image/" + image, frame);
    return result;
}

double
getJaccardCoefficient(int leftCol, int topRow, int rightCol, int bottomRow, int gtLeftCol, int gtTopRow, int gtRightCol, int gtBottomRow)
{
    double jaccCoeff = 0.;

    if (!(leftCol > gtRightCol ||
        rightCol < gtLeftCol ||
        topRow > gtBottomRow ||
        bottomRow < gtTopRow)
        )
    {
        int interLeftCol = std::max<int>(leftCol, gtLeftCol);
        int interTopRow = std::max<int>(topRow, gtTopRow);
        int interRightCol = std::min<int>(rightCol, gtRightCol);
        int interBottomRow = std::min<int>(bottomRow, gtBottomRow);

        const double areaIntersection = (abs(interRightCol - interLeftCol) + 1) * (abs(interBottomRow - interTopRow) + 1);
        const double lhRoiSize = (abs(rightCol - leftCol) + 1) * (abs(bottomRow - topRow) + 1);
        const double rhRoiSize = (abs(gtRightCol - gtLeftCol) + 1) * (abs(gtBottomRow - gtTopRow) + 1);

        jaccCoeff = areaIntersection / (lhRoiSize + rhRoiSize - areaIntersection);
    }
    return jaccCoeff;
};

void
TSD_testMyDetector(bool verbose, std::string benchmark_file, std::string gt_file, std::string test, std::string result)
{
    std::string file = gt_file;
    std::string files_name = test;
    std::vector<GTData> gtData = TSD_readGTData(file);
    std::vector<GTData> resultData = TSD_readGTData(result);

    int TP = 0;
    int FP = 0;
    int FN = 0;

    std::ifstream files;
    files.open(files_name.c_str());
    if (files.is_open())
    {
        std::string line;
        getline(files, line);
        while (!files.eof())
        {
            std::vector<double> leftCols;
            std::vector<double> rightCols;
            std::vector<double> topRows;
            std::vector<double> bottomRows;

            for (uint i = 0; i < resultData.size(); ++i)
            {
                if (resultData[i].name == line)
                {
                    leftCols.push_back(resultData[i].leftCol);
                    rightCols.push_back(resultData[i].rightCol);
                    topRows.push_back(resultData[i].topRow);
                    bottomRows.push_back(resultData[i].bottomRow);
                }
            }


            std::vector<double> gtLeftCols;
            std::vector<double> gtRightCols;
            std::vector<double> gtTopRows;
            std::vector<double> gtBottomRows;
            for (uint i = 0; i < gtData.size(); ++i)
            {
                if (gtData[i].name == line)
                {
                    gtLeftCols.push_back(gtData[i].leftCol);
                    gtRightCols.push_back(gtData[i].rightCol);
                    gtTopRows.push_back(gtData[i].topRow);
                    gtBottomRows.push_back(gtData[i].bottomRow);
                }
            }

            if (verbose)
                std::cout << "Image " << line << ":" << std::endl;

            std::vector<bool> gtSignHit(gtLeftCols.size(), false);
            for (uint roiIdx = 0; roiIdx < leftCols.size(); ++roiIdx)
            {
                double maxJaccCoeff = 0.6;
                int maxGtRoiIdx = -1;
                for (uint gtRoiIdx = 0; gtRoiIdx < gtLeftCols.size(); ++gtRoiIdx)
                {
                    double jaccCoeff = getJaccardCoefficient(leftCols[roiIdx], topRows[roiIdx], rightCols[roiIdx], bottomRows[roiIdx],
                                                             gtLeftCols[gtRoiIdx], gtTopRows[gtRoiIdx], gtRightCols[gtRoiIdx], gtBottomRows[gtRoiIdx]);
                    if (jaccCoeff > maxJaccCoeff)
                    {
                        maxJaccCoeff = jaccCoeff;
                        maxGtRoiIdx = gtRoiIdx;
                    }
                }
                if (maxGtRoiIdx == -1)
                {
                    FP = FP + 1;
                    if (verbose)
                        std::cout << "Miss: cols=" << leftCols[roiIdx] << ".." << rightCols[roiIdx] << ", rows=" << topRows[roiIdx] << ".." << bottomRows[roiIdx] << std::endl;
                }
                else
                {
                    gtSignHit[maxGtRoiIdx] = true;
                    if (verbose)
                        std::cout << "Hit: cols=" << leftCols[roiIdx] << ".." << rightCols[roiIdx] << ", rows=" << topRows[roiIdx] << ".." << bottomRows[roiIdx] << " matches cols=" << gtLeftCols[maxGtRoiIdx] << ".." << gtRightCols[maxGtRoiIdx] << ", rows=" << gtTopRows[maxGtRoiIdx] << ".." << gtBottomRows[maxGtRoiIdx] << std::endl;
                }
            }

            const int sumHits = std::accumulate(gtSignHit.begin(), gtSignHit.end(), 0);
            TP = TP + sumHits;
            FN = FN + gtSignHit.size() - sumHits;
            if (verbose)
                std::cout << "Precision: " << (double) TP / (double) (TP + FP) << ", Recall: " << (double) TP / (double) (TP + FN) << std::endl;
            getline(files, line);
        }

    }

    std::cout << "True Positive = " << TP << ", False Positives = " << FP << ", False Negatives = " << FN << std::endl;
    std::cout << "Precision: " << (double) TP / (double) (TP + FP) << ", Recall: " << (double) TP / (double) (TP + FN) << std::endl;
    std::ofstream file_benchmark;
    file_benchmark.open(benchmark_file.c_str());
    file_benchmark << (double) TP / (double) (TP + FP) << std::endl;
    file_benchmark << (double) TP / (double) (TP + FN) << std::endl;
    file_benchmark << TP << std::endl;
    file_benchmark << FN << std::endl;
    file_benchmark << FP << std::endl;
    file_benchmark.close();

}