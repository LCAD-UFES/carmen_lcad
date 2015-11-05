#ifndef P_PARTICLE
#define P_PARTICLE
class pParticle{
public:

        // Co ordinates of the point
        Point2D<float> p;
        // Probability for this particular point
        float prob;
        float orientation;

        pParticle();
        pParticle(const pParticle&);
        pParticle(short int, short int);
        pParticle(short int , short int , float );
//        pParticle(short int , short int , short int );
        pParticle(short int , short int , float , float);

        bool operator< (const pParticle&);
        bool operator> (const pParticle&);
        bool operator== (const pParticle&);

        void setProb(const float);
        void setProb(float&);
        float getProb();

        ~pParticle();

};

inline pParticle::pParticle():p(0,0),prob(0),orientation(0){}
inline pParticle::pParticle(const pParticle& p1):p(p1.p),prob(p1.prob),orientation(p1.orientation) {}
inline pParticle::pParticle(short int x, short int y):p(x,y),prob(0),orientation(0){}
inline pParticle::pParticle(short int x, short int y, float p):p(x,y),prob(p),orientation(0){}
//inline pParticle::pParticle(short int x, short int y, float o):p(x,y),prob(0),orientation(o){}
inline pParticle::pParticle(short int x, short int y, float p, float o):p(x,y),prob(p),orientation(o){}

inline bool pParticle::operator< (const pParticle& p1){
        return (this->prob) < (p1.prob) ;
}
inline bool pParticle::operator> (const pParticle& p1){
        return (this->prob) > (p1.prob);
}
inline bool pParticle::operator== (const pParticle& p1){
        return (this->prob) == (p1.prob);
}

inline void pParticle::setProb(const float p){
        this->prob = p;
}
inline void pParticle::setProb(float& p){
        this->prob = p;
}
inline float pParticle::getProb(){
        return this->prob;
}


inline pParticle::~pParticle(){
        delete &p;
}

#endif
