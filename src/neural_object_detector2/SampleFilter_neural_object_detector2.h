#ifndef SAMPLEFILTER_H_
#define SAMPLEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 20 Hz

* 0 Hz - 0.1 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = -54.46025665027494 dB

* 0.105 Hz - 10 Hz
  gain = 1
  desired ripple = 10 dB
  actual ripple = 59.590236468198114 dB

*/

#define SAMPLEFILTER_TAP_NUM 5

typedef struct {
  double history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;

void SampleFilter_init(SampleFilter* f);
void SampleFilter_put(SampleFilter* f, double input);
double SampleFilter_get(SampleFilter* f);

#endif
