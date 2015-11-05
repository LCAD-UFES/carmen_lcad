#include <cmath>
#include "Learn/WiimoteGR/Quantizer.h"
#include "Learn/WiimoteGR/TimeSlot.h"

namespace WiimoteGR{
    
    void Quantizer::Quantize(const Gesture& gesture, TimeSlot& symbolSeq) const{
        symbolSeq.gestureName = gesture.gestureName;
        symbolSeq.quantizerName = name;
        symbolSeq.M = M;
        symbolSeq.o.clear();
        for(vector<Acceleration>::const_iterator i = gesture.data.begin(); i<gesture.data.end(); i++)
            symbolSeq.o.push_back(Quantize(*i));
    }
    
    size_t DefaultQuantizer::Quantize(const Acceleration& acc) const{
        double rho = sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z);
        return (rho>rho_threshold? 1<<3 : 0) 
             + (acc.x>acc.y? 1<<2 : 0)
             + (acc.y>acc.z? 1<<1 : 0)
             + (acc.z>acc.x? 1 : 0);
    }

    size_t M32Quantizer::Quantize(const Acceleration& acc) const{
        double rho = sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z);
        return (rho>3.0? 3<<3 : rho>2.0? 2<<3 : rho>1.0? 1<<3 :0)
             | (acc.x>acc.y? 1<<2 : 0)
             | (acc.y>acc.z? 1<<1 : 0)
             | (acc.z>acc.x? 1 : 0);
    }

}
