#ifndef MVOG_MODEL_VOLUME_H
#define MVOG_MODEL_VOLUME_H

#include <algorithm>

namespace MVOG 
{

union Volume2
{
		float bottom_;
		float top_;
		float mass_;  
};

class Volume
{
  private:

		float bottom_;
		float top_;
		float mass_;  

  public:

    float getTop()  const { return top_; }
    float getBot()  const { return bottom_; }
    float getMass() const { return mass_; }

    void setBot (float bot)  { bottom_ = bot;  }
    void setTop (float top)  { top_    = top;  }
    void setMass(float mass) { mass_   = mass; }

    Volume();
    Volume(double bottom, double top);
    virtual ~Volume();
};

bool intersects (const Volume& a, const Volume& b); 
void join (Volume& a, const Volume& b); // merge volume b into a

}; // namespace MVOG

#endif // MVOG_MODEL_VOLUME_H
