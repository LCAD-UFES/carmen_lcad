#ifndef NONLINEARANALYTICCONDITIONALGAUSSIANMEASUREMENT_H_
#define NONLINEARANALYTICCONDITIONALGAUSSIANMEASUREMENT_H_

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{

  class NonLinearAnalyticConditionalGaussianMeasurement : public AnalyticConditionalGaussianAdditiveNoise
  {
    public:
	  NonLinearAnalyticConditionalGaussianMeasurement( const Gaussian& additiveNoise);
      virtual ~NonLinearAnalyticConditionalGaussianMeasurement();

      // redefine virtual functions
      virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
      virtual MatrixWrapper::Matrix          dfGet(unsigned int i)       const;

    private:
      mutable MatrixWrapper::Matrix df;
    };

} // End namespace BFL

#endif /* NONLINEARANALYTICONDITIONALGAUSSIANMEASUREMENT_H_ */
