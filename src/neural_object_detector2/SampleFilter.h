#ifndef SAMPLEFILTER_H_
#define SAMPLEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 40 Hz

* 0 Hz - 20 Hz
  gain = 0.06
  desired ripple = 1 dB
  actual ripple = 8.881784197001252e-14 dB

*/

#define SAMPLEFILTER_TAP_NUM 3

typedef struct {
  double history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;

void SampleFilter_init(SampleFilter* f);
void SampleFilter_put(SampleFilter* f, double input);
double SampleFilter_get(SampleFilter* f);

#endif
