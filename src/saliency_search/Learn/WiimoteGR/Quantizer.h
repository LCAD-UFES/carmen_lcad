#ifndef QUANTIZER_H
#define QUANTIZER_H

#include <vector>
#include <string>
//#include "TimeSlot.h"

using namespace std;

namespace WiimoteGR{

    /*
        Wiimote acceleration data structures
    */
    struct Acceleration{
        double x,y,z;
    };

    struct Gesture{
        string gestureName;
        vector<Acceleration> data;
        Gesture(const char* gestureName):gestureName(gestureName){ /* nothing */ }
    };
    
    struct TimeSlot;

    /*
        Abstract base Quantizer
        Extend it and use it to construct a TimeSlot.
    */
    class Quantizer{
    public:
        const string name;
        const size_t M; //number of observation symbols

        Quantizer(const char* name, size_t M):name(name),M(M) { /*nothing*/ }

        virtual size_t Quantize(const Acceleration& acc) const = 0;
        virtual void Quantize(const Gesture& gesture, TimeSlot& symbolSeq) const;
    };

    /*
        Default quantizer
    */
    class DefaultQuantizer : public Quantizer{
        const double rho_threshold;
        public:
            DefaultQuantizer():Quantizer("M16Quantizer",16),rho_threshold(1.0) {/*nothing*/}

            virtual size_t Quantize(const Acceleration& acc) const;

            virtual void Quantize(const Gesture& gesture, TimeSlot& symbolSeq) const{
                Quantizer::Quantize(gesture,symbolSeq);
            }
    };

    /*
        M32 quantizer
    */
    class M32Quantizer : public Quantizer{
        public:
            M32Quantizer():Quantizer("M32Quantizer",32) {/*nothing*/}

            virtual size_t Quantize(const Acceleration& acc) const;

            virtual void Quantize(const Gesture& gesture, TimeSlot& symbolSeq) const{
                Quantizer::Quantize(gesture,symbolSeq);
            }
    };

}

#endif
