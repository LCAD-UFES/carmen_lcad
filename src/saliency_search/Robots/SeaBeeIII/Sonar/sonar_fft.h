#ifndef SEABEE3_SONAR_FFT_H
#define SEABEE3_SONAR_FFT_H

#include <fftw3.h>

namespace SeaBee3_Sonar {

bool isValidADCNumber(int adc_number);

class DataToFFT {
  public:
    DataToFFT(double *data1, double *data2, double *data3, const int len, const double sampling_frequency, const double target_frequency);
    ~DataToFFT();

    double getPhase(int adc_number);
    double getMagnitude(int adc_number);
    int    getTargetBin();

    void Reset();
  private:
    double        *p_data1;
    double        *p_data2;
    double        *p_data3;
    const int     m_fft_length;
    const double  m_sampling_frequency;
    const double  m_target_frequency;
    const int     m_target_bin;
    fftw_plan     m_fft_plan1;
    fftw_plan     m_fft_plan2;
    fftw_plan     m_fft_plan3;
    fftw_complex  *p_fft1_output;
    fftw_complex  *p_fft2_output;
    fftw_complex  *p_fft3_output;
    double        *p_adc1_bin_magnitude;
    double        *p_adc2_bin_magnitude;
    double        *p_adc3_bin_magnitude;
    double        *p_adc1_bin_phase;
    double        *p_adc2_bin_phase;
    double        *p_adc3_bin_phase;
};

} // End namespace SeaBee3_sonar

#endif // SEABEE3_SONAR_FFT_H Defined