#include "sonar_fft.h"
#include <stdio.h>
#include <math.h>

namespace SeaBee3_Sonar {

bool isValidADCNumber(int adc_number) {
  if (adc_number == 1 || adc_number == 2 || adc_number == 3)
    return true;
  //fprintf(stderr, "ERROR: %d is not a valid ADC\n", adc_number);
  return false;
}

DataToFFT::DataToFFT(double *data1, double *data2, double *data3, const int len, const double sampling_frequency, const double target_frequency)
: p_data1(data1), p_data2(data2), p_data3(data3), m_fft_length(len),
  m_sampling_frequency(sampling_frequency), m_target_frequency(target_frequency),
  m_target_bin(m_target_frequency / m_sampling_frequency * m_fft_length) {
  p_fft1_output = (double (*)[2]) fftw_malloc(sizeof(fftw_complex) * ( ((len)/2)+1) );
  p_fft2_output = (double (*)[2]) fftw_malloc(sizeof(fftw_complex) * ( ((len)/2)+1) );
  p_fft3_output = (double (*)[2]) fftw_malloc(sizeof(fftw_complex) * ( ((len)/2)+1) );
  
  p_adc1_bin_magnitude = NULL;
  p_adc2_bin_magnitude = NULL;
  p_adc3_bin_magnitude = NULL;
  p_adc1_bin_phase = NULL;
  p_adc2_bin_phase = NULL;
  p_adc3_bin_phase = NULL;
  
  m_fft_plan1 = fftw_plan_dft_r2c_1d(m_fft_length, p_data1, p_fft1_output, FFTW_ESTIMATE);
  m_fft_plan2 = fftw_plan_dft_r2c_1d(m_fft_length, p_data2, p_fft2_output, FFTW_ESTIMATE);
  m_fft_plan3 = fftw_plan_dft_r2c_1d(m_fft_length, p_data3, p_fft3_output, FFTW_ESTIMATE);
  
  fftw_execute(m_fft_plan1);
  fftw_execute(m_fft_plan2);
  fftw_execute(m_fft_plan3);
}

DataToFFT::~DataToFFT() {
  if (p_fft1_output != NULL)
    fftw_free(p_fft1_output);
  if (p_fft2_output != NULL)
    fftw_free(p_fft2_output);
  if (p_fft3_output != NULL)
    fftw_free(p_fft3_output);
  
  if (p_adc1_bin_magnitude != NULL) {
    delete p_adc1_bin_magnitude;
    p_adc1_bin_magnitude = NULL;
  }
  if (p_adc2_bin_magnitude != NULL) {
    delete p_adc2_bin_magnitude;
    p_adc2_bin_magnitude = NULL;
  }
  if (p_adc3_bin_magnitude != NULL) {
    delete p_adc3_bin_magnitude;
    p_adc3_bin_magnitude = NULL;
  }
  
  if (p_adc1_bin_phase != NULL) {
    delete p_adc1_bin_phase;
    p_adc1_bin_phase = NULL;
  }
  if (p_adc2_bin_phase != NULL) {
    delete p_adc2_bin_phase;
    p_adc2_bin_phase = NULL;
  }
  if (p_adc3_bin_phase != NULL) {
    delete p_adc3_bin_phase;
    p_adc3_bin_phase = NULL;
  }
  
  fftw_destroy_plan(m_fft_plan1);
  fftw_destroy_plan(m_fft_plan2);
  fftw_destroy_plan(m_fft_plan3);
}

double DataToFFT::getPhase(int adc_number) {
  if (!isValidADCNumber(adc_number))
    return -10.0;
  
  switch (adc_number) {
    case 1:
      if (p_adc1_bin_phase == NULL) {
        p_adc1_bin_phase = new double(0.0);
        fprintf(stderr, "Target Bin: %d\n", m_target_bin);
        fprintf(stderr, "%10.5f, %10.5f\n", p_fft1_output[m_target_bin][1], p_fft1_output[m_target_bin][0]);
        *p_adc1_bin_phase = atan2(p_fft1_output[m_target_bin][1], p_fft1_output[m_target_bin][0]);
      }
      return *p_adc1_bin_phase;
      break;
    case 2:
      if (p_adc2_bin_phase == NULL) {
        p_adc2_bin_phase = new double(0.0);
        fprintf(stderr, "%10.5f, %10.5f\n", p_fft2_output[m_target_bin][1], p_fft2_output[m_target_bin][0]);
        *p_adc2_bin_phase = atan2(p_fft2_output[m_target_bin][1], p_fft2_output[m_target_bin][0]);
      }
      return *p_adc2_bin_phase;
      break;
    case 3:
      if (p_adc3_bin_phase == NULL) {
        p_adc3_bin_phase = new double(0.0);
        fprintf(stderr, "%10.5f, %10.5f\n", p_fft3_output[m_target_bin][1], p_fft3_output[m_target_bin][0]);
        *p_adc3_bin_phase = atan2(p_fft3_output[m_target_bin][1], p_fft3_output[m_target_bin][0]);
      }
      return *p_adc3_bin_phase;
      break;
  }
  
  // Values outside of [-M_PI, M_PI) are not possible, thus -10 signals an error
  return -10.0;
}

double DataToFFT::getMagnitude(int adc_number) {
  if (!isValidADCNumber(adc_number))
    return -1.0;
  
  switch (adc_number) {
    case 1:
      if (p_adc1_bin_magnitude == NULL) {
        p_adc1_bin_magnitude = new double(0.0);
        *p_adc1_bin_magnitude = p_fft1_output[m_target_bin][0] * p_fft1_output[m_target_bin][0] +
                                p_fft1_output[m_target_bin][1] * p_fft1_output[m_target_bin][1];
      }
      return *p_adc1_bin_magnitude;
      break;
    case 2:
      if (p_adc2_bin_magnitude == NULL) {
        p_adc2_bin_magnitude = new double(0.0);
        *p_adc2_bin_magnitude = p_fft2_output[m_target_bin][0] * p_fft2_output[m_target_bin][0] +
                                p_fft2_output[m_target_bin][1] * p_fft2_output[m_target_bin][1];
      }
      return *p_adc2_bin_magnitude;
      break;
    case 3:
      if (p_adc3_bin_magnitude == NULL) {
        p_adc3_bin_magnitude = new double(0.0);
        *p_adc3_bin_magnitude = p_fft3_output[m_target_bin][0] * p_fft3_output[m_target_bin][0] +
                                p_fft3_output[m_target_bin][1] * p_fft3_output[m_target_bin][1];
      }
      return *p_adc1_bin_magnitude;
      break;
  }
  
  // Negative values are not possible, thus used as error signal
  return -1.0;
}

int DataToFFT::getTargetBin() {
  return m_target_bin;
}

void DataToFFT::Reset() {
  fftw_execute(m_fft_plan1);
  fftw_execute(m_fft_plan2);
  fftw_execute(m_fft_plan3);
  
  if (p_adc1_bin_magnitude != NULL) {
    delete p_adc1_bin_magnitude;
    p_adc1_bin_magnitude = NULL;
  }
  if (p_adc2_bin_magnitude != NULL) {
    delete p_adc2_bin_magnitude;
    p_adc2_bin_magnitude = NULL;
  }
  if (p_adc3_bin_magnitude != NULL) {
    delete p_adc3_bin_magnitude;
    p_adc3_bin_magnitude = NULL;
  }
  
  if (p_adc1_bin_phase != NULL) {
    delete p_adc1_bin_phase;
    p_adc1_bin_phase = NULL;
  }
  if (p_adc2_bin_phase != NULL) {
    delete p_adc2_bin_phase;
    p_adc2_bin_phase = NULL;
  }
  if (p_adc3_bin_phase != NULL) {
    delete p_adc3_bin_phase;
    p_adc3_bin_phase = NULL;
  }
}


} // End namespace SeaBee3_sonar

