#ifndef TIMESLOT_H
#define TIMESLOT_H

#include <vector>
#include <string>
#include <map>

#include "HMMLib.h"
#include "Quantizer.h"

using namespace std;

namespace WiimoteGR{
    struct HMM;
    class Quantizer;
    /*
        TimeSlot:
    */
    struct TimeSlot{
    public:
        string gestureName;
        string quantizerName;
        //number of observation symbols, it should match with the related HMM
        size_t M;

        //observable symbol sequence
        vector<size_t> o; //vector size: T

        //for probability computation(for training, too)
        //mapping HMM to prob computation vectors and result
        map< const HMM*,vector< vector<double> > > alpha; //vector size: T*N
        map< const HMM*,vector<double> > scale; //vector size: T
        map< const HMM*,double > logProb;//P(O|HMM)

        //for HMM training, empty if not used
        //no mapping because training is usually done once, and for one HMM
        vector< vector<double> > beta, gamma; //vector size: T*N
        vector< vector< vector<double> > > xi; //vector size T*N*N

        void AddObservableSymbol(size_t obsSymbol){
            o.push_back(obsSymbol);
        }
        void ClearObservableSymbols(){
            o.clear();
            alpha.clear();
            scale.clear();
            logProb.clear();
        }

        TimeSlot(const char* gestureName, const Quantizer& quantizer)
            :gestureName(gestureName), quantizerName(quantizer.name), M(quantizer.M)
        {
        
        }
    };
}

#endif
