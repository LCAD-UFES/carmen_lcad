#ifndef SAMPLEFILTER_H_
#define SAMPLEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 40 Hz

* 0 Hz - 4 Hz
  gain = 1
  desired ripple = 2 dB
  actual ripple = 0.9350979035150214 dB

* 9 Hz - 13 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -44.317028145338206 dB

* 14 Hz - 20 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = -64.3170281453382 dB

*/

#define SAMPLEFILTER_TAP_NUM 15

typedef struct {
  double history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;

void SampleFilter_init(SampleFilter* f);
void SampleFilter_put(SampleFilter* f, double input);
double SampleFilter_get(SampleFilter* f);

#endif
