#ifndef HMMLIB_H
#define HMMLIB_H

#include <vector>
#include <string>
#include "Learn/WiimoteGR/TimeSlot.h"

using namespace std;

namespace WiimoteGR{
    /*
        Hidden Markov Model, represents a gesture
    */
    struct HMM{
        string gestureName;
        //quantizer used to generate training sequence, corresponding to M,B
        string quantizerName;
        //represents the style of model(N,pi,A),
        //  ex: "3 state circular", "3 state full connection", "5 state left to right", etc...
        string modelStyle;

        bool trained;
        size_t N;  // number of states
        size_t M;  // number of observable symbols
        vector<double> A;  // size = N*N, A[i*N+j] is the transition prob of going from state i at time t to state j at time t+1
        vector<double> B;  // size = N*M, B[i*M+k] is the probability of observable symbol k in state i
        vector<double> pi; // size = N,   pi[i] is the initial state distribution.

        //constructor for Database::LoadHMM()
        HMM(const Quantizer& quantizer, const char* modelStyle, bool trained)
            :quantizerName(quantizer.name), modelStyle(modelStyle), trained(trained), M(quantizer.M)
        { /* nothing */ }

        HMM(const char* gestureName, const Quantizer& quantizer, const char* modelStyle, bool trained, size_t N,
            double *const A_in, double *const B_in, double *const pi_in);
        
        //for const HMM object
        const double& TranProb(size_t i, size_t j) const{
            return A[i*N+j];
        }
        const double& ObsProb(size_t j, size_t k) const{
            return B[j*M+k];
        }

        //for nonconst HMM object
        double& TranProb(size_t i, size_t j){
            return A[i*N+j];
        }
        double& ObsProb(size_t j, size_t k){
            return B[j*M+k];
        }
    };
    
    struct TimeSlot;

    class HMMLib{
    public:
        void ShowHMM(HMM& hmm);
        const HMM& Recognize(const vector<HMM>& HMMVec, TimeSlot& seq);
        double SeqLogProb(const HMM& hmm, TimeSlot& seq, bool restart);
        int EstimateModel(HMM& hmm, TimeSlot& seq, size_t maxIteration = 10);
        int EstimateModelBySeqs(HMM& hmm, vector<TimeSlot>& seqs, size_t maxIteration = 10);
    private:
        bool CheckAndAllocateMemory(const HMM &hmm, vector<TimeSlot>& seqs);
        void Forward(const HMM& hmm, TimeSlot& seq, bool restart);
        void Backward(const HMM& hmm, TimeSlot& seq);
        void ComputeGamma(const HMM& hmm, TimeSlot& seq);
        void ComputeXi(const HMM& hmm, TimeSlot& seq);
    };
}
#endif
