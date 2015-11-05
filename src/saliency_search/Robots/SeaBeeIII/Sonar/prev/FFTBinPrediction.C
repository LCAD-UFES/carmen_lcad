#include "FFTBinPrediction.H"

#include <stdio.h>

FFTBinAffinePrediction::FFTBinAffinePrediction(
                            int cheetah_bitrate, int fft_length,
                            double target_frequency, double slope,
                            double offset, int halfwidth)
 : m_bitrate(cheetah_bitrate), m_fft_length(fft_length),
   m_target_frequency(target_frequency), m_slope(slope),
   m_offset(offset), m_halfwidth(halfwidth) {

  // Use affine function to calculate sampling frequency
  m_sampling_frequency = m_slope * (double)m_bitrate + m_offset;

  // Use sampling frequency to determine FFT bin number for target frequency
  printf("%10.0f\n", m_target_frequency);
  printf("%10.0f\n", m_sampling_frequency);
  printf("%d\n", m_fft_length);
  m_target_bin = (double)m_target_frequency / (double)m_sampling_frequency * (double)m_fft_length;
  printf("%d\n", m_target_bin);
}

FFTBinAffinePrediction::~FFTBinAffinePrediction() {}

// Accessors
int FFTBinAffinePrediction::getBitrate() { return m_bitrate; }
int FFTBinAffinePrediction::getFFTLength() { return m_fft_length; }
double FFTBinAffinePrediction::getTargetFrequency() {
  return m_target_frequency;
}
double FFTBinAffinePrediction::getSlope() { return m_slope; }
double FFTBinAffinePrediction::getOffset() { return m_offset; }
int FFTBinAffinePrediction::getHalfwidth() { return m_halfwidth; }
int FFTBinAffinePrediction::getTargetBin() { return m_target_bin; }
double FFTBinAffinePrediction::getSamplingFrequency() {
  return m_sampling_frequency;
}

// Manipulators
void FFTBinAffinePrediction::setBitrate(int bitrate) {
  m_bitrate = bitrate;
  updateTargetBin();
}
void FFTBinAffinePrediction::setFFTLength(int fft_length) {
  m_fft_length = fft_length;
  updateTargetBin();
}
void FFTBinAffinePrediction::setTargetFrequency(double frequency) {
  m_target_frequency = frequency;
  updateTargetBin();
}
void FFTBinAffinePrediction::setSlope(double slope) {
  m_slope = slope;
  updateTargetBin();
}
void FFTBinAffinePrediction::setOffset(double offset) {
  m_offset = offset;
  updateTargetBin();
}
void FFTBinAffinePrediction::setHalfwidth(int halfwidth) {
  m_halfwidth = halfwidth;
}
void FFTBinAffinePrediction::updateTargetBin() {
  // Use affine function to calculate sampling frequency
  m_sampling_frequency = m_slope * (double)m_bitrate + m_offset;

  // Use sampling frequency to determine FFT bin number for target frequency
  m_target_bin = (double)m_target_frequency / (double)m_sampling_frequency * (double)m_fft_length;
}

