#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>

#include "Learn/WiimoteGR/Quantizer.h"
#include "Learn/WiimoteGR/HMMLib.h"
#include "Learn/WiimoteGR/Database.h"
#include "Image/Point2D.H"

using namespace std;
using namespace WiimoteGR;

std::vector<Point2D<int> > square()
{
  std::vector<Point2D<int> > acc;
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(-5,0));
  acc.push_back(Point2D<int>(-5,0));
  acc.push_back(Point2D<int>(-5,0));
  acc.push_back(Point2D<int>(-5,0));

  acc.push_back(Point2D<int>(0,5));
  acc.push_back(Point2D<int>(0,5));
  acc.push_back(Point2D<int>(0,5));
  acc.push_back(Point2D<int>(0,5));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,-5));
  acc.push_back(Point2D<int>(0,-5));
  acc.push_back(Point2D<int>(0,-5));
  acc.push_back(Point2D<int>(0,-5));

  acc.push_back(Point2D<int>(-5,0));
  acc.push_back(Point2D<int>(-5,0));
  acc.push_back(Point2D<int>(-5,0));
  acc.push_back(Point2D<int>(-5,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(5,0));

  acc.push_back(Point2D<int>(0,-5));
  acc.push_back(Point2D<int>(0,-5));
  acc.push_back(Point2D<int>(0,-5));
  acc.push_back(Point2D<int>(0,-5));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,5));
  acc.push_back(Point2D<int>(0,5));
  acc.push_back(Point2D<int>(0,5));
  acc.push_back(Point2D<int>(0,5));

  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));

  return acc;

}

std::vector<Point2D<int> > triangle()
{
  std::vector<Point2D<int> > acc;
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(5,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(-5,0));
  acc.push_back(Point2D<int>(-5,0));
  acc.push_back(Point2D<int>(-5,0));
  acc.push_back(Point2D<int>(-5,0));

  acc.push_back(Point2D<int>(-5,5));
  acc.push_back(Point2D<int>(-5,5));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(5,-5));
  acc.push_back(Point2D<int>(5,-5));

  acc.push_back(Point2D<int>(-5,-5));
  acc.push_back(Point2D<int>(-5,-5));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(5,5));
  acc.push_back(Point2D<int>(5,5));

  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));
  acc.push_back(Point2D<int>(0,0));

  return acc;

}


void Training(Database& db, const Quantizer& quantizer, HMMLib& trainer, HMM& hmm);


int main()
{
    /* initial HMM model */
    const char* initGestureName = "unknown";
    //Quantizer used, contain information of M
    M32Quantizer defaultQuantizer;
    const double OP = 1.0/defaultQuantizer.M;
    //style of model
    const char* initModelStyle = "5 state left to right";
    //not trained initially
    bool initTrained = false;
    //num of states
    const size_t initN = 5;
    //matrices
    double initA[] = { 0.5, 0.5, 0.0, 0.0, 0.0,
                       0.0, 0.5, 0.5, 0.0, 0.0,
                       0.0, 0.0, 0.5, 0.5, 0.0,
                       0.0, 0.0, 0.0, 0.5, 0.5,
                       0.0, 0.0, 0.0, 0.0, 1.0
                     };
    //B: N=5 * M=32
    double initB[] = { OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP,
                       OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP,
                       OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP,
                       OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP,
                       OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP, OP
                     };
    double initPi[] = { 1.0, 0.0, 0.0, 0.0, 0.0};
    
    HMM initHMM(initGestureName, defaultQuantizer, initModelStyle, initTrained, initN, initA, initB, initPi);


    //database of HMMs, seqs gestures
    Database& db = Database::Open();

    //HMM Trainining Library
    HMMLib trainer;

    ////Train the guesters
    //{
    //  //push(copy) initial HMM to vector
    //  HMM tempHMM = initHMM;
    //  tempHMM.gestureName = "square";
    //  //train and save HMM to database
    //  Training(db, defaultQuantizer, trainer, tempHMM);
    //}

    //{
    //  //push(copy) initial HMM to vector
    //  HMM tempHMM = initHMM;
    //  tempHMM.gestureName = "triangle";
    //  //train and save HMM to database
    //  Training(db, defaultQuantizer, trainer, tempHMM);
    //}

    /*
        Testing
    */
    cout << endl << "============================TESTING===============================" << endl;
    vector<HMM> loadedHMMVec;

    //temp sequence for user input
    TimeSlot tempSeq("testingGesture",defaultQuantizer);
    Acceleration tempAcc;

    //load trained HMMsfrom database for testing
    db.LoadHMMs(defaultQuantizer, initModelStyle, true, loadedHMMVec);
    cout << loadedHMMVec.size() << " pre saved HMM loaded for testing, now gestures are:" << endl;
    for(vector<HMM>::iterator i=loadedHMMVec.begin() ; i<loadedHMMVec.end(); i++)
        cout << i->gestureName << " ";
    cout << endl;

    cout << "Start recognition of " << loadedHMMVec.size() << " gesture models." << endl;

    std::vector<Point2D<int> > acc = triangle();

    //Get gesture
    for(size_t i=0; i<acc.size(); i++)
    {
      tempAcc.x=acc[i].i; 
      tempAcc.y=acc[i].j; 
      tempAcc.z=0;

      tempSeq.AddObservableSymbol(defaultQuantizer.Quantize(tempAcc));
      //cout.width(3);
      //cout << defaultQuantizer.Quantize(tempAcc) << " ";
      cout << trainer.Recognize(loadedHMMVec,tempSeq).gestureName << "-> ";
    }

    //show probs to trained HMM
    cout << endl;
    for(size_t i = 0; i < loadedHMMVec.size(); i++)
      cout << "Prob to " << loadedHMMVec[i].gestureName << " model = " << exp(trainer.SeqLogProb(loadedHMMVec[i],tempSeq,false)) << endl;
    tempSeq.ClearObservableSymbols();

    return 0;
}

void Training(Database& db, const Quantizer& quantizer,
    HMMLib& trainer, HMM& hmm){

    size_t M = quantizer.M;
    if(hmm.M == M){
        //training gestures
        vector<Gesture> gestureVec;
        //training sequences
        vector<TimeSlot> seqVec;

        //Load existing gestures for this model
        //db.LoadGestures(hmm.gestureName.c_str(), gestureVec);
        //db.LoadObservationSequences(hmm.gestureName.c_str(),quantizer,seqVec);

        //cout << gestureVec.size() << " samples of " << hmm.gestureName << " gesture loaded" << endl;
        //cout << seqVec.size() << " observation sequences of " << hmm.gestureName << " gesture loaded" << endl;

        //Delete the gustures and start from scratch
        db.DeleteGestures(hmm.gestureName.c_str());
        db.DeleteObservationSequences(hmm.gestureName.c_str(),quantizer);
        cout << "Sequences of gesture " << hmm.gestureName << " deleted from database." << endl;

        //temp gesture for user input
        Gesture tempGesture(hmm.gestureName.c_str());

        trainer.ShowHMM(hmm);
        std::vector<Point2D<int> > acc;
        if (hmm.gestureName == "square")
          acc = square();
        else
          acc = triangle();
        
        //Feed the points in order


        for(size_t j=0; j<acc.size(); j++)
        {

          for(size_t i=0; i<acc.size(); i++)
          {
            Acceleration tempAcc;
            tempAcc.x=acc[(j+i)%acc.size()].i; 
            tempAcc.y=acc[(j+i)%acc.size()].j; 
            tempAcc.z=0; 

            tempGesture.data.push_back(tempAcc);
            cout.width(3);
            cout << quantizer.Quantize(tempAcc) << " ";
          }

          TimeSlot tempSeq(hmm.gestureName.c_str(), quantizer);

          quantizer.Quantize(tempGesture,tempSeq);
          cout << "/ Length = " << tempSeq.o.size() << endl << endl;

          gestureVec.push_back(tempGesture);
          seqVec.push_back(tempSeq);

          db.SaveGesture(tempGesture);
          db.SaveObservationSequence(tempSeq);
          tempGesture.data.clear();
        }


        /*
        Generate .sce file for plotting by scilab
        */
        //genSce(hmm.gestureName,gestureVec);

        //show probs to initial HMM
        for(size_t i = 0; i < seqVec.size(); i++)
            cout << "prob of seqs[" << i << "] to " << hmm.gestureName << " model = " << trainer.SeqLogProb(hmm,seqVec[i],false) << endl;
        //train by multi seq
        cout << hmm.gestureName << " gesture model is trained in " << trainer.EstimateModelBySeqs(hmm, seqVec, 50) << " loops" << endl;

        hmm.trained = true;
        trainer.ShowHMM(hmm);
        db.SaveHMM(hmm);

        //show probs to trained HMM
        for(size_t i = 0; i < seqVec.size(); i++)
            cout << "prob of seqs[" << i << "] to " << hmm.gestureName << " model = " << trainer.SeqLogProb(hmm,seqVec[i],false) << endl;
        
    }else{
        cout << "ERROR hmm.M != quantizer.M" << endl;
    }
}

/*

double rho(double x, double y, double z)
{
    return sqrt(x*x+y*y+z*z);
}

void genSce(string gestureName, vector<Gesture>& gestureVec)
{
    ofstream sceFile;

    for(size_t i=0;i<gestureVec.size();i++){
        
        stringstream ss;
 
        ss << gestureName << i/10 << i%10;

        sceFile.open( (ss.str()+".sce").c_str() );
        if(!sceFile)
            cout << "error" << endl;
        
        sceFile << "t=[0:" << (gestureVec[i].data.size()-1) << "];" << endl;

        sceFile << "x=[";
        for(size_t j=0; j<gestureVec[i].data.size(); j++)
            sceFile << gestureVec[i].data[j].x << " ";
        sceFile << "];" << endl;

        sceFile << "y=[";
        for(size_t j=0; j<gestureVec[i].data.size(); j++)
            sceFile << gestureVec[i].data[j].y << " ";
        sceFile << "];" << endl;

        sceFile << "z=[";
        for(size_t j=0; j<gestureVec[i].data.size(); j++)
            sceFile << gestureVec[i].data[j].z << " ";
        sceFile << "];" << endl;

        sceFile << "r=[";
        for(size_t j=0; j<gestureVec[i].data.size(); j++)
            sceFile << rho(gestureVec[i].data[j].x,gestureVec[i].data[j].y,gestureVec[i].data[j].z) << " ";
        sceFile << "];" << endl;

        sceFile <<  "plot(t,x,\"ko-\",t,y,\"kx-\",t,z,\"k>-\",t,r,\"k.-\");\n"
                    "a=gca();\n"
                    "a.x_label.text=\"time(10ms)\";\n"
                    "a.x_label.font_size = 2;\n"
                    "a.x_label.font_style = 8;\n"
                    "a.y_label.text=\"acceleration(g)\";\n"
                    "a.y_label.font_size = 2;\n"
                    "a.y_label.font_style = 8;\n"

                    "a.title.text = \"" << ss.str() << "\";\n"
                    "a.title.font_size = 4;\n"
                    "a.title.font_style = 5;\n"

                    "a.font_size = 2;\n"
                    "a.x_location = \"middle\";\n"

                    //"l=legend([\"x\",\"y\",\"z\",\"r\"]);\n"
                    //"l.children(1).font_style=1;"
                    << endl;

        sceFile.close();
    }
}
*/
