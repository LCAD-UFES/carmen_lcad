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

vector<GTData> TSD_readGTData(string aGTFile);

/*
 * 
 */

void TSD_testMyDetector(bool verbose, std::string benchmark_file, std::string gt_file, std::string test, std::string result);

int
main(int argc, char** argv)
{

    if (argc != 5)
    {
        cerr << argv[0] << " result_file.txt gt_test.txt test.txt result.txt" << endl;
        return -1;
    }
    cerr << "Calculation of precision and recall" << endl;

    bool verbose = false;
    TSD_testMyDetector(verbose, argv[1], argv[2], argv[3], argv[4]);

    cerr << "End of calculation of precision and recall" << endl;
    return EXIT_SUCCESS;
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
        boost::trim(line);
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
            boost::trim(line);
        }

    }
    else
    {
        cerr << "Can not Open File" << endl;
    }

    return data_vector;
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

    std::ofstream fn;
    fn.open("fn.txt");
    std::ofstream fp;
    fp.open("fp.txt");
    std::ofstream tp;
    tp.open("tp.txt");

    std::ifstream files;
    files.open(files_name.c_str());
    if (files.is_open())
    {
        std::string line;
        getline(files, line);
        boost::trim(line);
        while (!files.eof())
        {
//            cout << line << endl;
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
                double maxJaccCoeff = 0.5;
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
                    fp << line << " " << leftCols[roiIdx] << " " << topRows[roiIdx] << " " << rightCols[roiIdx] << " " << bottomRows[roiIdx] << endl;
                    if (verbose)
                        std::cout << "Miss: cols=" << leftCols[roiIdx] << ".." << rightCols[roiIdx] << ", rows=" << topRows[roiIdx] << ".." << bottomRows[roiIdx] << std::endl;
                }
                else
                {
                    gtSignHit[maxGtRoiIdx] = true;
                    tp << line << " " << leftCols[roiIdx] << " " << topRows[roiIdx] << " " << rightCols[roiIdx] << " " << bottomRows[roiIdx] << endl;
                    if (verbose)
                        std::cout << "Hit: cols=" << leftCols[roiIdx] << ".." << rightCols[roiIdx] << ", rows=" << topRows[roiIdx] << ".." << bottomRows[roiIdx] << " matches cols=" << gtLeftCols[maxGtRoiIdx] << ".." << gtRightCols[maxGtRoiIdx] << ", rows=" << gtTopRows[maxGtRoiIdx] << ".." << gtBottomRows[maxGtRoiIdx] << std::endl;
                }
            }

            const int sumHits = std::accumulate(gtSignHit.begin(), gtSignHit.end(), 0);

            for (uint i = 0; i < gtSignHit.size(); i++)
            {
                if (gtSignHit[i] == false)
                {
                    fn << line << " " << gtLeftCols[i] << " " << gtTopRows[i] << " " << gtRightCols[i] << " " << gtBottomRows[i] << endl;
                }

            }
            TP = TP + sumHits;
            FN = FN + gtSignHit.size() - sumHits;
            if (verbose)
                std::cout << "Precision: " << (double) TP / (double) (TP + FP) << ", Recall: " << (double) TP / (double) (TP + FN) << std::endl;
            getline(files, line);
            boost::trim(line);

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
    tp.close();
    fn.close();
    fp.close();          
           
}
