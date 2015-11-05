#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fftw3.h>

//#include "cheetah.h"
#include "TestData.H"
#include "FFTBinPrediction.H"

double sign(double val) {
  if (val < 0)
    return -1.0;
  else if (val > 0)
    return 1.0;
  return 0.0;
}

// Globals
const double SPEED_SOUND_WATER = 1482; // [m/s]
const double SENSOR_SPACING = 0.024;    // [m]

// Cheetah Settings
//Cheetah handle;
int  port, bitrate;
//uint8_t  mode  = 0;

// Peak Detection
double maxIndex1, maxValue1;
double maxIndex2, maxValue2;
double maxIndex3, maxValue3;

// FFT
const int N_FFT = 512;

// Data
//uint8_t  data_in[N_FFT * 3];
//uint8_t  data_out[2];
short  data_in[N_FFT * 3];
short  data_out[2];
int  ret;
int  input, ready_bit, valid_data_point;
int  target_frequency, halfwidth;
double weighted_sum;
double angle_estimate;

int main (int argc, char const *argv[]) {
  if (argc < 3) {
    fprintf(stderr, "usage: async PORT BITRATE [Target Freq. in Hz] [halfwidth]\n");
    return 1;
  }

  port    = atoi(argv[1]);
  bitrate = atoi(argv[2]);

  if (argc >= 4)
    target_frequency = atoi(argv[3]);
  else
    target_frequency = 30000;
  double target_wavelength = (double)SPEED_SOUND_WATER / (double)target_frequency;

  //if (argc >= 5)
  //  halfwidth = atoi(argv[4]);
  //else
    halfwidth = 3;
  //double *gaussian_weights;
  //gaussian_weights = new double[2 * halfwidth + 1];
//  double gaussian_weights[] = {0.006, 0.061, 0.242, 0.383, 0.242, 0.061, 0.006};

//  // Open the device
//  handle = ch_open(port);
//
//  if (handle <= 0) {
//    fprintf(stderr, "Unable to open Cheetah device on port %d\n", port);
//    fprintf(stderr, "Error code = %d (%s)\n", handle, ch_status_string(handle));
//    exit(1);
//  }
//  fprintf(stderr, "Opened Cheetah device on port %d\n", port);
//
//  fprintf(stderr, "Host interface is %s\n",
//      (ch_host_ifce_speed(handle)) ? "high speed" : "full speed");
//
//  // Ensure that the SPI subsystem is configured.
//  ch_spi_configure(handle, CheetahSpiPolarity(mode >> 1), CheetahSpiPhase(mode & 1), CH_SPI_BITORDER_MSB, 0x0);
//  fprintf(stderr, "SPI configuration set to mode %d, %s shift, SS[2:0] active low\n", mode, "MSB");
//
//  // Power the target using the Cheetah adapter's power supply.
//  ch_target_power(handle, CH_TARGET_POWER_ON);
//  ch_sleep_ms(100);
//
//  // Set the bitrate.
//  bitrate = ch_spi_bitrate(handle, bitrate);
//  fprintf(stderr, "Bitrate set to %d kHz\n", bitrate);
//
//  // Make a simple queue to just assert OE.
//  ch_spi_queue_clear(handle);
//  ch_spi_queue_oe(handle, 1);
//  ch_spi_batch_shift(handle, 0, 0);

  // Queue the batch, which is a sequence of SPI packets (back-to-back) each of length 2.
  fprintf(stderr, "Beginning to queue SPI packets...");
  data_out[0] = 0xff;
  data_out[1] = 0xff;
//  ch_spi_queue_clear(handle);
//  
//  for (int i = 0; i < N_FFT * 3; ++i) {
//    // Convert Slave 1
//    ch_spi_queue_ss(handle, 0xF);
//    ch_spi_queue_array(handle, 2, data_out);
//    ch_spi_queue_ss(handle, 0xE);
//    
//    // Convert Slave 2
//    ch_spi_queue_ss(handle, 0xF);
//    ch_spi_queue_array(handle, 2, data_out);
//    ch_spi_queue_ss(handle, 0xD);
//
//    // Convert Slave 3
//    ch_spi_queue_ss(handle, 0xF);
//    ch_spi_queue_array(handle, 2, data_out);
//    ch_spi_queue_ss(handle, 0xB);
//  }
  fprintf(stderr, " Done\n");

  // Calculate Sampling Frequency and Target Bin
  // For affine bin prediction, use m = 10.59, b = -1193.82 and a
  // window-halfwidth of 5.
  FFTBinAffinePrediction bin_predictor(bitrate, N_FFT, target_frequency, 10.59, -1193.82, halfwidth);
  fprintf(stderr, "Target Bin: %d\n", bin_predictor.getTargetBin());

  // Define the Data Vectors
  //double *data1, *data2, *data3;
  fftw_complex *fft1, *fft2, *fft3;
  fftw_plan fft_plan1, fft_plan2, fft_plan3;

  // Allocate Memory for the Data Vectors
  //data1 = (double *) fftw_malloc ( sizeof ( double ) * N_FFT );
  //data2 = (double *) fftw_malloc ( sizeof ( double ) * N_FFT );
  //data3 = (double *) fftw_malloc ( sizeof ( double ) * N_FFT );
  fft1 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( N_FFT / 2 ) + 1) );
  fft2 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( N_FFT / 2 ) + 1) );
  fft3 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( N_FFT / 2 ) + 1) );
  fft_plan1 = fftw_plan_dft_r2c_1d(N_FFT, data1, fft1, FFTW_ESTIMATE);
  fft_plan2 = fftw_plan_dft_r2c_1d(N_FFT, data2, fft2, FFTW_ESTIMATE);
  fft_plan3 = fftw_plan_dft_r2c_1d(N_FFT, data3, fft3, FFTW_ESTIMATE);

//  // Submit the first batch
//  ch_spi_async_submit(handle);

//  while (1) {
    // Submit another batch, while the previous one is in
    // progress.  The application may even clear the current
    // batch queue and queue a different set of SPI
    // transactions before submitting this batch
    // asynchronously.
//    ch_spi_async_submit(handle);
    
    // The application can now perform some other functions
    // while the Cheetah is both finishing the previous batch
    // and shifting the current batch as well.  In order to
    // keep the Cheetah's pipe full, this entire loop must
    // complete AND another batch must be submitted
    // before the current batch completes.
    //ch_sleep_ms(1);
    
    // Collect the previous batch
    // The length of the batch, N_FFT * 6, come from the fact that 3 ADCs
    // are batched and the return data requires 2 bytes.  (2 * 3 = 6)
//    ret = ch_spi_async_collect(handle, N_FFT * 6, data_in);

//    for (int j = 0; j < N_FFT * 6; j += 6) {
//      // SS2 Data
//      input = (data_in[j] << 8) + data_in[j+1];
//      ready_bit = input & 0x4000;
//      valid_data_point = (input & 0x3ffc) >> 2;
//      data2[j/6] = valid_data_point;
//
//      // SS3 Data
//      input = (data_in[j+2] << 8) + data_in[j+3];
//      ready_bit = input & 0x4000;
//      valid_data_point = (input & 0x3ffc) >> 2;
//      data3[j/6] = valid_data_point;
//
//      // SS1 Data
//      input = (data_in[j+4] << 8) + data_in[j+5];
//      ready_bit = input & 0x4000;
//      valid_data_point = (input & 0x3ffc) >> 2;
//      data1[j/6] = valid_data_point;
//    }

    // Perform FFT on current input data
    fftw_execute(fft_plan1);
    fftw_execute(fft_plan2);
    fftw_execute(fft_plan3);

//    // Check the ADC on SS0 for a peak.
//    weighted_sum = 0;
    maxValue1 = 0;
    maxIndex1 = 0;
    maxValue2 = 0;
    maxIndex2 = 0;
    maxValue3 = 0;
    maxIndex3 = 0;
//    for (int i = bin_predictor.getTargetBin() - bin_predictor.getHalfwidth(); i < bin_predictor.getTargetBin() + bin_predictor.getHalfwidth() + 1; ++i) {
    for (int i = 10; i < N_FFT / 2; ++i) {
//      /*fprintf(stderr, "%1.3f * %10.2f + \n", gaussian_weights[i - (bin_predictor.getTargetBin() - bin_predictor.getHalfwidth())], sqrt(
//          pow(fft1[i][0],2.0) +
//          pow(fft1[i][1],2.0)));*/
//      weighted_sum += gaussian_weights[i - (bin_predictor.getTargetBin() - bin_predictor.getHalfwidth())] *
//                      sqrt(pow(fft1[i][0],2.0) + pow(fft1[i][1],2.0));

      if (fft1[i][0] * fft1[i][0] + fft1[i][1] * fft1[i][1] > maxValue1) {
        maxValue1 = fft1[i][0] * fft1[i][0] + fft1[i][1] * fft1[i][1]; //sqrt(pow(fft1[i][0],2.0) + pow(fft1[i][1],2.0));
        maxIndex1 = i;
        }
      if (fft2[i][0] * fft2[i][0] + fft2[i][1] * fft2[i][1] > maxValue2) {
        maxValue2 = fft2[i][0] * fft2[i][0] + fft2[i][1] * fft2[i][1]; //sqrt(pow(fft2[i][0],2.0) + pow(fft2[i][1],2.0));
        maxIndex2 = i;
      }
      if (fft3[i][0] * fft3[i][0] + fft3[i][1] * fft3[i][1] > maxValue3) {
        maxValue3 = fft3[i][0] * fft3[i][0] + fft3[i][1] * fft3[i][1]; //sqrt(pow(fft3[i][0],2.0) + pow(fft3[i][1],2.0));
        maxIndex3 = i;
      }
    }
    fprintf(stderr, "Signal 1 Peak: %1.0f, %10.2f\n", maxIndex1, maxValue1);
    fprintf(stderr, "Signal 2 Peak: %1.0f, %10.2f\n", maxIndex2, maxValue2);
    fprintf(stderr, "Signal 3 Peak: %1.0f, %10.2f\n", maxIndex3, maxValue3);
    double phase1 = atan2((double)fft1[(int)maxIndex1][1], (double)fft1[(int)maxIndex1][0]);
    double phase2 = atan2((double)fft2[(int)maxIndex2][1], (double)fft2[(int)maxIndex2][0]);
    double phase3 = atan2((double)fft3[(int)maxIndex3][1], (double)fft3[(int)maxIndex3][0]);
    
    double delta1 = phase2 - phase1;
    double delta2 = phase3 - phase2;
    double delta3 = phase1 - phase3;
    
    fprintf(stderr, "Deltas: %4.4f, %4.4f, %4.4f\n", delta1, delta2, delta3);
    
    int min_index = 3;
    if (fabs(delta2) < fabs(delta1) && fabs(delta2) < fabs(delta3))
      min_index = 2;
    if (fabs(delta1) < fabs(delta2) && fabs(delta1) < fabs(delta3))
      min_index = 1;
    
    fprintf(stderr, "Min Index: %d\n", min_index);
    
    // testing case
    //min_index = 1;
    double delta_tmp;
    switch (min_index) {
      case 1:
        delta_tmp = delta1;
        if (delta3 > delta2)
          delta_tmp = -1.0 * delta1 + sign(delta_tmp) * 2.0 * M_PI;
        angle_estimate = delta_tmp * (double)(((double)target_wavelength / 2.0) / SENSOR_SPACING) * 180.0 / M_PI / 2.0;
        break;
      case 2:
        delta_tmp = delta2;
        if (delta1 > delta3)
          delta_tmp = -1.0 * delta2 + 2.0 * M_PI;
        angle_estimate = (delta_tmp - 5.0 / 6.0 * M_PI) * ((target_wavelength / 2.0) / SENSOR_SPACING) * 180.0 / M_PI / 2.0;
        break;
      case 3:
        delta_tmp = delta3;
        if (delta2 > delta1)
          delta_tmp = -1.0 * delta3 - 2.0 * M_PI;
        //fprintf(stderr, "%4.8f\n", (double)SPEED_SOUND_WATER);
        //fprintf(stderr, "%4.8f\n", (double)target_frequency);
        //fprintf(stderr, "%4.8f\n", (double)SPEED_SOUND_WATER / (double)target_frequency);
        //fprintf(stderr, "%4.8f\n", (double)target_wavelength);
        //fprintf(stderr, "%4.8f\n", ((double)target_wavelength / 2.0));
        //fprintf(stderr, "%4.8f\n", (((double)target_wavelength / 2.0) / SENSOR_SPACING));
        //fprintf(stderr, "%4.8f\n", 180.0 / M_PI / 2.0);
        //fprintf(stderr, "%4.8f\n", (((double)target_wavelength / 2.0) / SENSOR_SPACING) * 180.0 / M_PI / 2.0);
        angle_estimate = (delta_tmp + 5.0 / 6.0 * M_PI ) * (((double)target_wavelength / 2.0) / SENSOR_SPACING) * 180.0 / M_PI / 2.0;
        break;
      default:
        fprintf(stderr, "ERROR: Invalid min_index for phase difference.\n");
    }
  
    //phase_difference *= ((target_wavelength / 2.0) / SENSOR_SPACING) * 180.0 / M_PI / 2.0;
    fprintf(stderr, "Detected Angle: %4.4f degrees\n", angle_estimate);
//    weighted_sum *= 2 * bin_predictor.getHalfwidth() + 1;

    //fprintf(stderr, "%8.2f\n", weighted_sum);
    //fprintf(stderr, "DC: %10.2f\n", sqrt(pow(fft1[0][0],2.0) + pow(fft1[0][1],2.0)));

//    // Weighted Gaussian Test
//    if (weighted_sum >= 0.01 * sqrt(pow(fft1[0][0],2.0) + pow(fft1[0][1],2.0)))
//      fprintf(stderr, "Peak detected!\n");

//    // Ensures communication is successful
//    if (ret < 0)  fprintf(stderr, "status error: %s\n", ch_status_string(ret));
//    fflush(stderr);

    // The current batch is now shifting out on the SPI
    // interface. The application can again do some more tasks
    // here but this entire loop must finish so that a new
    // batch is armed before the current batch completes.
    //ch_sleep_ms(1);
//  }

  // Clean up allocated memory
  fftw_destroy_plan(fft_plan1);
  fftw_destroy_plan(fft_plan2);
  fftw_destroy_plan(fft_plan3);
//  fftw_free(data1);
//  fftw_free(data2);
//  fftw_free(data3);
  fftw_free(fft1);
  fftw_free(fft2);
  fftw_free(fft3);
  //delete gaussian_weights;

  return 0;
}
