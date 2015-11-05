#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "SIFT/VisualObject.H"
#include "Component/OptionManager.H"

class LoweSIFT : public ModelComponent
{
  public:
    LoweSIFT(OptionManager& mgr, 
        std::string const & descrName = "LoweSIFT",
        std::string const & tagName   = "LoweSIFT");

  std::vector<rutz::shared_ptr<Keypoint> > computeKeypoints(Image<PixRGB<byte> > const & img);

  private:
    //! The path the the sift executable binary
    OModelParam<std::string> itsExePath;
};
