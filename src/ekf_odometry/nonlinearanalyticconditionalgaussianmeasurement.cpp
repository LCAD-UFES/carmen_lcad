#include "nonlinearanalyticconditionalgaussianmeasurement.h"
#include <stdio.h>
#include <bfl/wrappers/rng/rng.h>

#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
  using namespace MatrixWrapper;

  NonLinearAnalyticConditionalGaussianMeasurement::NonLinearAnalyticConditionalGaussianMeasurement(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise, NUMCONDARGUMENTS_MOBILE),
      df(2,6)
  {
    // initialize df matrix
    for (unsigned int i=1; i<=2; i++){
      for (unsigned int j=1; j<=6; j++){
    	  df(i,j) = 0;
      }
    }
  }

  NonLinearAnalyticConditionalGaussianMeasurement::~NonLinearAnalyticConditionalGaussianMeasurement(){}

  ColumnVector NonLinearAnalyticConditionalGaussianMeasurement::ExpectedValueGet() const
  {
	ColumnVector expected_measurement(2);

    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector meas  = ConditionalArgumentGet(1);
    expected_measurement(1) = sqrt((meas(1) - state(1)) * (meas(1) - state(1)) + (meas(2) - state(2)) * (meas(2) - state(2)));
    expected_measurement(2) = atan2((meas(2) - state(2)), (meas(1) - state(1))) - state(6);

    if(fabs(expected_measurement(2)) >= M_PI)
    	expected_measurement(2) = atan2((meas(2) - state(2)), (meas(1) - state(1))) - fabs(state(6));

    //printf("landmark_measurement - r: %f, theta: %f\n", expected_measurement(1), expected_measurement(2));

    return expected_measurement + AdditiveNoiseMuGet();

  }

  Matrix NonLinearAnalyticConditionalGaussianMeasurement::dfGet(unsigned int i) const
  {
    if (i==0)//derivative to the first conditional argument (x)
    {
    	ColumnVector state = ConditionalArgumentGet(0);
    	ColumnVector meas  = ConditionalArgumentGet(1);

    	double q  = (meas(1) - state(1)) * (meas(1) - state(1)) + (meas(2) - state(2)) * (meas(2) - state(2));

    	df(1,1) = -((meas(1) - state(1)) / sqrt(q));
		df(1,2) = -((meas(2) - state(2)) / sqrt(q));
		df(1,3) = 0;
		df(1,4) = 0;
		df(1,5) = 0;
		df(1,6) = 0;
		df(2,1) = ((meas(2) - state(2)) / q);
		df(2,2) = -((meas(1) - state(1)) / q);
		df(2,3) = 0;
		df(2,4) = 0;
		df(2,5) = 0;
		df(2,6) = -1;

		return df;
    }
    else
      {
	if (i >= NumConditionalArgumentsGet())
	  {
	    cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
	    exit(-BFL_ERRMISUSE);
	  }
	else{
	  cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
	  exit(-BFL_ERRMISUSE);
	}
      }
  }

}//namespace BFL

