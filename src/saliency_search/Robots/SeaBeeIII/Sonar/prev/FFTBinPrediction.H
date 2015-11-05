#ifndef SEABEA_SONAR_FFTBINPREDICTION_H
#define SEABEA_SONAR_FFTBINPREDICTION_H


// The FFTBinAffinePrediciton class uses an affine function defined by its
// slope and offset to predict the bin number that a frequency will occupy
// in an FFT based on the bitrate selected for the Cheetah SPI device.  The
// affine function has been selected as the prediction modeled based on
// experimental data collected from the TotalPhase Cheetah SPI device.
class FFTBinAffinePrediction {
public:
  FFTBinAffinePrediction(int cheetah_bitrate, int fft_length,
                         double target_frequency, double slope,
                         double offset, int halfwidth);
  ~FFTBinAffinePrediction();

  // Accessors
  int getBitrate();
  int getFFTLength();
  double getTargetFrequency();
  double getSlope();
  double getOffset();
  int getHalfwidth();
  int getTargetBin();
  double getSamplingFrequency();

  // Manipulators
  void setBitrate(int bitrate);
  void setFFTLength(int fft_length);
  void setTargetFrequency(double frequency);
  void setSlope(double slope);
  void setOffset(double offset);
  void setHalfwidth(int halfwidth);
  void updateTargetBin();

private:
  int m_bitrate;               // x in y = m * x + b
  int m_fft_length;
  double m_target_frequency;   // in Hz
  double m_slope;              // m in y = m * x + b
  double m_offset;             // b in y = m * x + b
  int m_halfwidth;             // Window halfwidth about target bin
  double m_sampling_frequency; // y in y = m * x + b
  int m_target_bin;            // Calculated FFT bin
};

#endif // SEABEA_SONAR_FFTBINPREDICTION_H Defined
