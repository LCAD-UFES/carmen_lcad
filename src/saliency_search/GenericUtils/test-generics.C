#include "Image/Image.H"
#include "Image/Pixels.H"
#include "GenericUtils/ProcessFunctor.H"
#include "GenericUtils/GenericItem.H"
#include <vector>

//typdef our ProcessFunction and associated GenericItem definitions, the first
// template type is the underlying container type, and the second is all the
// types that could be processed. If a conversion is desired, we will attempt to
// convert from the end to the front.
// ######################################################################
typedef ProcessFunctor<Image, PixRGB<byte>, byte, double, float> ImageFunctor;
typedef GenericItem<Image, PixRGB<byte>, byte, double, float> GenericImage;

// ######################################################################
//a simple class to sum images as an example
// ######################################################################
class SumFunc : public ImageFunctor//implements virtual funcions for PixRGB<byte>, byte, double, float
{
public:
  float getSum() const { return itsSum; };
private :

  //implement for float
  bool process(const Image<float>& feature)
  {
    //take the sum of the image
    itsSum = 0.0;
    Image<float>::const_iterator iter(feature.begin());
    while (iter != feature.end())
      itsSum += *iter++;
    return true;
  }

  //for other types if desired....but if left empty then we will attempt to convert to float

  float itsSum;
};
// ######################################################################
// here is how you would make one generically... now, for all types in Types
// functions will be defined. In the base implementaton, specialization for
// other types not part of Types could be declared (for instance, if you want to
// do something generically for all types but 1.
// ######################################################################
//define a recursive template
template<typename... Types> class GenSumFunc;             
//base case derives from ImageFunctor, accessor function and storage of data
template<> struct GenSumFunc<> : public ImageFunctor       
{
  float getSum() const { return itsSum; };
  float itsSum;

  //specializations for process go here by just defining the function
};
//recursive case creates our funcions
template<typename T, typename... Tail>                    
class GenSumFunc<T, Tail...> : public GenSumFunc<Tail...>
{
private:
  bool process(const Image<T>& feature)
  {
    //take the sum of the image
    T sum = T();
    typename Image<T>::const_iterator iter(feature.begin());
    while (iter != feature.end())
      sum += *iter++;
    
    GenSumFunc<>::itsSum = (float)sum;
    return true;
  }
};

// ######################################################################
int main(const int argc, const char **argv)
{
  //Fill a vector of generic images
  Image<float> I1(10,10,ZEROS);
  I1.setVal(1,1,10.0F);
  Image<double> I2(10,10,ZEROS);
  I2.setVal(1,1,9.0);
  Image<int> I3(10,10,ZEROS);
  I3.setVal(1,8);

  std::vector<GenericImage> fsv;//collectin of generic images
  fsv.push_back(GenericImage::make(I1));
  fsv.push_back(GenericImage::make(I2));
  fsv.push_back(GenericImage::make(I3));

  //iterate through and take the sum, allowing conversions to float and possibly
  //converting to others first depending on the order of the template
  //declaration:
  // element 1) direct call to the virtual function
  //
  // element 2) call the non-op double function (our SumFunc knows about
  // doubles, but didn't implement and function for them, which will dive
  // straight to the non-op base function, and then convert to float since float
  // is last in declaration (we try converting backwards) and it is implemented
  //
  // element 3) Dive straight to the non-op base function (GenericImage doesn't
  // know about ints), and then convert to float since float is last in
  // declaration (we try converting backwards) and it is implemented
  std::vector<GenericImage>::iterator i(fsv.begin());
  while (i != fsv.end())
    {
      SumFunc f; 
      (i++)->runProcessFunctor(f, true);//if the last argument is true, force conversion to float, otherwise no-op
      if (f.hasData())
	LINFO("%3.2f", f.getSum());
    }

  //No conversion for the first two elements se our template defines functions
  //for them automaticall, and then conversion if Image<int> to Image<float> as
  //in the above example;
  i = fsv.begin();
  while (i != fsv.end())
    {
      GenSumFunc<float, double> f; 
      (i++)->runProcessFunctor(f, true);//if the last argument is true, force conversion to float if we              
      if (f.hasData())                  //are not a float or double, otherwise no-op
	LINFO("%3.2f", f.getSum());
    }

  return 0;

};
