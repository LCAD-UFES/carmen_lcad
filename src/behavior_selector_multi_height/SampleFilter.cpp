#include "SampleFilter.h"

//static double filter_taps[SAMPLEFILTER_TAP_NUM] = {
//  -0.00527641662206894,
//  -0.0025058128713441284,
//  -0.0030136238049375324,
//  -0.0035184815623492726,
//  -0.003998551582740363,
//  -0.0044308181688575326,
//  -0.004788845470590836,
//  -0.005046039481750507,
//  -0.005176063733580603,
//  -0.005150421343397102,
//  -0.004942772280441883,
//  -0.004526374503643374,
//  -0.0038768006716494215,
//  -0.0029783497296080397,
//  -0.0018171853902693936,
//  -0.00038548303573217026,
//  0.001323743377452876,
//  0.0033095757631147198,
//  0.005554491761107369,
//  0.008041695896920638
//  };
//
void SampleFilter_init(SampleFilter* f) {
  int i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void SampleFilter_put(SampleFilter* f, double input) {
  f->history[f->last_index++] = input;
  if(f->last_index == SAMPLEFILTER_TAP_NUM)
    f->last_index = 0;
}

double SampleFilter_get(SampleFilter* f) {
  double acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : SAMPLEFILTER_TAP_NUM-1;
    acc += f->history[index];// * filter_taps[i];
  };
  return acc / (double) SAMPLEFILTER_TAP_NUM;
}
