#include <iostream>
#include <cmath>
#include "Learn/WiimoteGR/HMMLib.h"

using namespace std;

namespace WiimoteGR{

    HMM::HMM(const char* gestureName, const Quantizer& quantizer,
        const char* modelStyle, bool trained, size_t N,
        double *const A_in, double *const B_in, double *const pi_in)
        : gestureName(gestureName),
          quantizerName(quantizer.name),
          modelStyle(modelStyle),
          trained(trained),
          N(N),
          M(quantizer.M)
    {
        A.assign(A_in, A_in+N*N);
        B.assign(B_in, B_in+N*M);
        pi.assign(pi_in, pi_in+N);
    }

    void HMMLib::ShowHMM(HMM& hmm)
    {
        cout.width(15);
        cout << "----------------------------------------------------------------------" << endl;
        cout << "A " << (hmm.trained?"trained":"initial") << " HMM of " << hmm.gestureName << " gesture, " << endl;
        cout << "Trained with observation sequences quantized by " << hmm.quantizerName << "." << endl;
        cout << "with " << hmm.N << " states and " << hmm.M << " observable symbols." << endl << endl;
        cout << "pi:" << endl;
        for(size_t i=0; i<hmm.N-1; i++){
            cout.width(15);
            cout << hmm.pi[i];
        }
        cout.width(15);
        cout << hmm.pi[hmm.N-1] << endl;
        cout << endl;
        cout << "A:" << endl;
        for(size_t i=0; i<hmm.N; i++){
            for(size_t j=0; j<hmm.N-1; j++){
                cout.width(15);
                cout << hmm.TranProb(i,j);
            }
            cout.width(15);
            cout << hmm.TranProb(i,hmm.N-1) << endl;
        }
        cout << endl;
        cout << "B^T:" << endl;
        for(size_t k=0; k<hmm.M; k++){
            for(size_t j=0; j<hmm.N-1; j++){
                cout.width(15);
                cout << hmm.ObsProb(j,k);
            }
            cout.width(15);
            cout << hmm.ObsProb(hmm.N-1,k) << endl;
        }
        cout << endl;
    }

    const HMM& HMMLib::Recognize(const vector<HMM>& HMMVec, TimeSlot& seq)
    {
        vector<HMM>::const_iterator recognizedHMM = HMMVec.begin();
        for(vector<HMM>::const_iterator curHMM = HMMVec.begin()+1; curHMM < HMMVec.end(); curHMM++){
            if( SeqLogProb(*curHMM,seq,false) > SeqLogProb(*recognizedHMM,seq,false) )
                recognizedHMM = curHMM;
        }
        return *recognizedHMM;
    }

    double HMMLib::SeqLogProb(const HMM& hmm, TimeSlot& seq, bool restart)
    {
        Forward(hmm, seq, restart);
        return seq.logProb[&hmm];
    }

    int HMMLib::EstimateModel(HMM& hmm, TimeSlot& seq, size_t maxIteration)
    {
        if(hmm.M != seq.M)
            return -1;

        vector<size_t>& o = seq.o;
        //vector< vector<double> >& alpha = seq.alpha[&hmm];
        //vector< vector<double> >& beta = seq.beta;
        vector< vector<double> >& gamma = seq.gamma;
        vector< vector< vector<double> > >& xi = seq.xi;
        
        //length of time
        size_t T = o.size();

        //indices
        size_t i, j, k;
        size_t t;

        size_t loopCount = 0;

        double numeratorA, denominatorA;
        double numeratorB, denominatorB;

        //probability of sequence to hmm at previous estimation
        double logProb_prev;
        //difference of prob between iteration
        double delta;

        /*
        Initialization
        */

        //allocate memory space, alpha and scale will be allocated in Forward()
        seq.beta.resize(T);
        seq.gamma.resize(T);
        seq.xi.resize(T);
        for(size_t t=0; t<T; t++){
            seq.beta[t].resize(hmm.N);
            seq.gamma[t].resize(hmm.N);
            seq.xi[t].resize(hmm.N);
            for(size_t i=0; i<hmm.N; i++)
                seq.xi[t][i].resize(hmm.N);
        }

        //compute probs first time
        Forward(hmm,seq,true);
        Backward(hmm,seq);
        ComputeGamma(hmm,seq);
        ComputeXi(hmm,seq);

        logProb_prev = seq.logProb[&hmm];

        /*
        Iteration
        */
        do{
            // reestimate probility of state i in time t=0
            for(i = 0; i < hmm.N; i++)
                hmm.pi[i] = 0.0001 + 0.9999*gamma[1][i];

            // reestimate transition matrix and prob of symbols to states
            for(i = 0; i < hmm.N; i++) {
                denominatorA = 0.0;
                for(t = 0; t < T - 1; t++)
                    denominatorA += gamma[t][i];

                for(j = 0; j < hmm.N; j++) {
                    numeratorA = 0.0;
                    for(t = 0; t < T - 1; t++)
                        numeratorA += xi[t][i][j];
                    hmm.TranProb(i,j) = 0.0001 + 0.9999*numeratorA/denominatorA;
                }

                denominatorB = denominatorA + gamma[T-1][i];
                for(k = 0; k < hmm.M; k++) {
                    numeratorB = 0.0;
                    for(t = 0; t < T; t++) {
                        if(o[t] == k)
                            numeratorB += gamma[t][i];
                    }
                    hmm.ObsProb(i,k) = 0.0001 + 0.9999*numeratorB/denominatorB;
                }
            }

            // compute probs by new model
            Forward(hmm,seq,true);
            Backward(hmm,seq);
            ComputeGamma(hmm,seq);
            ComputeXi(hmm,seq);

            // delta prob between old and estimated model
            delta = seq.logProb[&hmm] - logProb_prev;
            logProb_prev = seq.logProb[&hmm];
            loopCount++;
        }while(loopCount < maxIteration);//loop utill log probability converged.
        return loopCount;
    }


    int HMMLib::EstimateModelBySeqs(HMM& hmm, vector<TimeSlot>& seqs, size_t maxIteration)
    {
        if( !CheckAndAllocateMemory(hmm,seqs) )
            return -1; //there is wrong sequence

        size_t i, j, k;
        size_t t;
        size_t T;

        size_t loopCount = 0;

        double numeratorA, numeratorA_partial, denominatorA, denominatorA_partial;
        double numeratorB, numeratorB_partial, denominatorB, denominatorB_partial;

        double logProbSum_prev = 0.0;
        double logProbSum = 0.0;
        double delta; //difference of prob between iteration

        /*
        Initialization
        */
        vector<TimeSlot>::iterator curSeq;
        for(curSeq = seqs.begin(); curSeq < seqs.end(); curSeq++){
            Forward(hmm, *curSeq, true);
            Backward(hmm, *curSeq);
            ComputeGamma(hmm, *curSeq);
            ComputeXi(hmm, *curSeq);
            logProbSum_prev += curSeq->logProb[&hmm];
        }
        /*
        Iteration
        */
        do{
            // reestimate probility of state i in time t=0
            //for(i = 0; i < hmm.N; i++)
            //    hmm.pi[i] = 0.001 + 0.999*curSeq->gamma[1][i];

            // reestimate transition matrix and prob of symbols to states
            for(i = 0; i < hmm.N; i++) {
                denominatorA = 0.0;
                denominatorB = 0.0;
                for(curSeq = seqs.begin(); curSeq < seqs.end(); curSeq++){
                    double& logProb = curSeq->logProb[&hmm];
                    vector< vector<double> >& gamma = curSeq->gamma;
                    T = curSeq->o.size();
                    denominatorA_partial = 0.0;
                    for(t = 0; t < T - 1; t++)
                        denominatorA_partial += gamma[t][i];
                    denominatorB_partial = denominatorA_partial + gamma[T-1][i];
                    denominatorA += denominatorA_partial/exp(logProb);
                    denominatorB += denominatorB_partial/exp(logProb);
                }

                for(j = 0; j < hmm.N; j++) {
                    numeratorA = 0.0;
                    for(curSeq = seqs.begin(); curSeq < seqs.end(); curSeq++){
                        vector< vector< vector<double> > >& xi = curSeq->xi;
                        T = curSeq->o.size();
                        numeratorA_partial = 0.0;
                        for(t = 0; t < T - 1; t++)
                            numeratorA_partial += xi[t][i][j];
                        numeratorA += numeratorA_partial/exp(curSeq->logProb[&hmm]);
                    }
                    hmm.TranProb(i,j) = 0.0001 + 0.9999*numeratorA/denominatorA;
                }

                for(k = 0; k < hmm.M; k++) {
                    numeratorB = 0.0;
                    for(curSeq = seqs.begin(); curSeq < seqs.end(); curSeq++){
                        vector< vector<double> >& gamma = curSeq->gamma;
                        vector<size_t>& o = curSeq->o;
                        T = o.size();
                        numeratorB_partial = 0.0;
                        for(t = 0; t < T; t++) {
                            if(o[t] == k)
                                numeratorB_partial += gamma[t][i];
                        }
                        numeratorB += numeratorB_partial/exp(curSeq->logProb[&hmm]);
                    }
                    hmm.ObsProb(i,k) = 0.0001 + 0.9999*numeratorB/denominatorB;
                }
            }

            // compute probs by new model
            for(curSeq = seqs.begin(); curSeq < seqs.end(); curSeq++){

                Forward(hmm, *curSeq, true);
                Backward(hmm, *curSeq);
                ComputeGamma(hmm, *curSeq);
                ComputeXi(hmm, *curSeq);
                logProbSum += curSeq->logProb[&hmm];
            }

            // delta prob between old and estimated model
            delta = logProbSum - logProbSum_prev;
            logProbSum_prev = logProbSum;
            loopCount++;
        }while(loopCount < maxIteration);//loop utill log probability converged.
        return loopCount;
    }

    bool HMMLib::CheckAndAllocateMemory(const HMM &hmm, vector<TimeSlot>& seqs)
    {
        /*
        check validity:
        invalid: one or more of the sequences's M is not match to hmm.M,
                 return false
        */
        vector<TimeSlot>::iterator curSeq;
        for(curSeq = seqs.begin(); curSeq < seqs.end(); curSeq++){
            if( curSeq->M != hmm.M)
                return false;
        }
        size_t N = hmm.N;
        size_t T;
        for(curSeq = seqs.begin(); curSeq < seqs.end(); curSeq++){
            T = curSeq->o.size();
            /*
            allocation of curSeq->alpha and curSeq->scale will be done in Forward()
            for Forward()'s realtime computation style
            */
            curSeq->beta.resize(T);
            curSeq->gamma.resize(T);
            curSeq->xi.resize(T);
            for(size_t t=0; t<T; t++){
                //set values to zero although not necessary
                curSeq->beta[t].resize(N,0);
                curSeq->gamma[t].resize(N,0);
                curSeq->xi[t].resize(N);
                for(size_t i=0; i<N; i++)
                    curSeq->xi[t][i].resize(N,0);
            }
        }
        return true;
    }

    /*
    Forward Algorithm
    compute P(O|HMM) by newly added symbols if restart==false
    else recompute alpha and scale and probability
    */
    void HMMLib::Forward(const HMM& hmm, TimeSlot& seq, bool restart)
    {
        //references for simplifying code
        const HMM* hmmPtr = &hmm;
        vector<size_t>& o = seq.o;
        vector< vector<double> >& alpha = seq.alpha[hmmPtr];
        vector<double>& scale = seq.scale[hmmPtr];
        double& logProb = seq.logProb[hmmPtr];

        size_t i, j; // state indices
        size_t t;    // time index
        double sum;  // partial sum

        //restart forward
        if(restart)
            alpha.clear();

        // number of observable symbols processed,
        // is also beginning of time iteration.
        size_t t_begin = alpha.size();
        size_t T = o.size(); // time length

        // space needed for this forward iteration
        alpha.resize(T);
        scale.resize(T);

        /*
        Only do iteration necessary for newly added
        symbols of input sequence.
        */
        for(t = t_begin; t < T; t++) {
            /*
            Do initialization if forward algorithm haven't been applied to
            the input sequence before.
            */
            if(t == 0){
                logProb = 0.0;
                scale[0] = 0.0;
                alpha[0].resize(hmm.N);
                for(i = 0; i < hmm.N; i++) {
                    alpha[0][i] = hmm.pi[i] * hmm.ObsProb(i,o[0]);
                    scale[0] += alpha[0][i];
                }
                for(i = 0; i < hmm.N; i++)
                    alpha[0][i] /= scale[0];
            }else{
                scale[t] = 0.0;
                alpha[t].resize(hmm.N);
                for(j = 0; j < hmm.N; j++) {
                    sum = 0.0;
                    for(i = 0; i < hmm.N; i++)
                        sum += alpha[t-1][i] * hmm.TranProb(i,j);

                    alpha[t][j] = sum * hmm.ObsProb(j,o[t]);
                    scale[t] += alpha[t][j];
                }
                for(j = 0; j < hmm.N; j++)
                    alpha[t][j] /= scale[t];
            }
        }

        /*
        Compute sequence probability
        */
        for(t = t_begin; t < T; t++){
            logProb += log(scale[t]);
        }
    }

    /*
    Backward Algorithm:
    should not be executed before Forward()
    */
    void HMMLib::Backward(const HMM& hmm, TimeSlot& seq)
    {
        //references for simplifying code
        vector<size_t>& o = seq.o;
        vector< vector<double> >& beta = seq.beta;
        vector<double>& scale = seq.scale[&hmm];

        size_t i, j; // state indices
        size_t t;    // time index
        double sum;  // partial sum

        size_t T = seq.o.size(); // time length

        /*
        1. Initialization
        forward algorithm must finished before now
        */
        for(i = 0; i < hmm.N; i++)
            beta[T-1][i] = 1.0/scale[T-1];

        /*
        2. Induction
        */
        for(t = T - 2; /*t>=0 cannot be used for size_t*/; t--) {
            for(i = 0; i < hmm.N; i++) {
                sum = 0.0;
                for(j = 0; j < hmm.N; j++)
                    sum += hmm.TranProb(i,j)
                    * hmm.ObsProb(j,o[t+1])
                    * beta[t+1][j];
                beta[t][i] = sum/scale[t];
            }
            if(t==0)
                break;
        }

    }

    void HMMLib::ComputeGamma(const HMM& hmm, TimeSlot& seq)
    {
        //references for simplifying code
        //vector<size_t>& o = seq.o;
        vector< vector<double> >& alpha = seq.alpha[&hmm];
        vector< vector<double> >& beta = seq.beta;
        vector< vector<double> >& gamma = seq.gamma;
 
        size_t i, j; // state indices
        size_t t;    // time index
        double denominator;

        size_t T = seq.o.size(); // time length

        for(t = 0; t < T; t++) {
            denominator = 0.0;
            for(j = 0; j < hmm.N; j++) {
                gamma[t][j] = alpha[t][j] * beta[t][j];
                denominator += gamma[t][j];
            }

            for(i = 0; i < hmm.N; i++)
                gamma[t][i] = gamma[t][i]/denominator;
        }
    }
    void HMMLib::ComputeXi(const HMM& hmm, TimeSlot& seq)
    {
        //references for simplifying code
        vector<size_t>& o = seq.o;
        vector< vector<double> >& alpha = seq.alpha[&hmm];
        vector< vector<double> >& beta = seq.beta;
        //vector< vector<double> >& gamma = seq.gamma;
        vector< vector< vector<double> > >& xi = seq.xi;

        size_t i, j; // state indices
        size_t t;    // time index
        double denominator;

        size_t T = seq.o.size(); // time length

        for(t = 0; t < T-1; t++) {
            denominator = 0.0;
            for(i = 0; i < hmm.N; i++)
                for(j = 0; j < hmm.N; j++) {
                    xi[t][i][j] = alpha[t][i] * beta[t+1][j] * hmm.TranProb(i,j) * hmm.ObsProb(j,o[t+1]);
                    denominator += xi[t][i][j];
                }

                for(i = 0; i < hmm.N; i++)
                    for(j = 0; j < hmm.N; j++)
                        xi[t][i][j] /= denominator;
        }
    }
}
