#include "mvog_model/volume_vector.h"

namespace MVOG
{

Volume::Volume()
{

}

Volume::Volume(double bottom, double top)
{
  if (top >= bottom)
  {
    bottom_ = bottom;
    top_    = top;
  }
  else
  {
    bottom_ = top;
    top_    = bottom;
  }

  if (top_ - bottom_ < 1.0) //FIXME
  {
    double m = bottom_ + (top_ - bottom_)/2.0;
    top_    = m + 0.5;
    bottom_ = m - 0.5;
  }

  mass_   = top_ - bottom_;
}

Volume::~Volume()
{


}

bool intersects (const Volume& a, const Volume& b)
{
  if (b.getBot() <= a.getTop() && a.getTop() <= b.getTop()) return true;
  if (b.getBot() <= a.getBot() && a.getBot() <= b.getTop()) return true;
  return false;
}

void join (Volume& a, const Volume& b)
{
  // merge volume b into a

  a.setTop(std::max(a.getTop(), b.getTop()));
  a.setBot(std::min(a.getBot(), b.getBot()));
  a.setMass(a.getMass() + b.getMass());
}

}
