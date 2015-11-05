#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <fftw3.h>

//#include "cheetah.h"
//#include "RealData_nopinger.H"
#include "RealData_pinger.H"
//#include "DummyData256.H"
#include "FFTBinPrediction.H"

double sign(double val) {
  if (val < 0)
    return -1.0;
  else if (val > 0)
    return 1.0;
  return 0.0;
}

// Globals
const double SPEED_SOUND_WATER  = 1482;  // [m/s]
const double SENSOR_SPACING_2_1 = 0.024; // [m]
const double SENSOR_SPACING_3_2 = 0.024; // [m]
const double SENSOR_SPACING_1_3 = 0.024; // [m]
const int DATA_RETENTION_LENGTH = 1024;

// Declare Cheetah Variables
//Cheetah    handle;
int        port, bitrate;
//uint8_t    mode = 0;
//int        ret, input, ready_bit, valid_data_point;

// FFT & Bin Prediction
const int  FP_FFT_LENGTH = 32;  // First-pass FFT length
double     BIN_PREDICTION_M = 10.59;
double     BIN_PREDICTION_B = -1193.82;
int        target_frequency1, target_frequency2;
double     target_wavelength1, target_wavelength2;
double     angle_estimate1, angle_estimate2;
double     bin_current_mag_sq_1;
double     bin_current_mag_sq_2;
double     bin_current_mag_sq_3;
double     adc1_bin_mean, adc2_bin_mean, adc3_bin_mean;
std::vector<bool> FP_bin_indicator;

// Raw Data
//const int  TX_LENGTH = FP_FFT_LENGTH * 3 * 2;
const int  TX_LENGTH = 512 * 3 * 2;
uint8_t    data_in[TX_LENGTH];
uint8_t    data_out[2];

// Hysterisis Vars
const int FP_BIN_HISTORY_LENGTH = 10;//10;
const double MEAN_SCALE_FACTOR = 8.0;
const int FP_MIN_SAMPLE_LENGTH = 128;
int FP_bin_mean_idx = 1;
double FP_adc1_bin_history[FP_BIN_HISTORY_LENGTH];
double FP_adc2_bin_history[FP_BIN_HISTORY_LENGTH];
double FP_adc3_bin_history[FP_BIN_HISTORY_LENGTH];
int    FP_bin_history_fill = 0;
std::vector<double> *adc1_data_history, *adc2_data_history, *adc3_data_history, *tmp_data_buffer;


int main (int argc, char const *argv[]) {
  port = 0;
  bitrate = 10000;

  // Assign Target Frequencies
  target_frequency1  = 30000; //atoi(argv[2]);
  target_frequency2  = 27000; //atoi(argv[3]);
  target_wavelength1 = (double)SPEED_SOUND_WATER / (double)target_frequency1;
  target_wavelength2 = (double)SPEED_SOUND_WATER / (double)target_frequency2;

  // Calculate Sampling Frequency and Target Bin
  // For affine bin prediction, use m = 10.59, b = -1193.82 and a
  // window-HALFWIDTH of 5.
  FFTBinAffinePrediction FP_f1_bin(bitrate,
                                       FP_FFT_LENGTH,
                                       target_frequency1,
                                       BIN_PREDICTION_M,
                                       BIN_PREDICTION_B,
                                       3);
  FFTBinAffinePrediction FP_f2_bin(bitrate,
                                       FP_FFT_LENGTH,
                                       target_frequency2,
                                       BIN_PREDICTION_M,
                                       BIN_PREDICTION_B,
                                       3);
  printf("Sonar: First-pass Target Bin 1: %d\n", FP_f1_bin.getTargetBin());
  printf("Sonar: First-pass Target Bin 2: %d\n", FP_f2_bin.getTargetBin());

  // Declare Data Vectors
  //double *FP_adc1_samples, *FP_adc2_samples, *FP_adc3_samples;
  double FP_adc1_samples[FP_FFT_LENGTH], FP_adc2_samples[FP_FFT_LENGTH], FP_adc3_samples[FP_FFT_LENGTH];
  fftw_complex *FP_fft1, *FP_fft2, *FP_fft3;
  fftw_complex *SP_fft1, *SP_fft2, *SP_fft3;
  fftw_plan FP_fft_plan1, FP_fft_plan2, FP_fft_plan3;
  fftw_plan SP_fft_plan1, SP_fft_plan2, SP_fft_plan3;

  // Allocate Memory for Data Vectors
  FP_fft1 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( FP_FFT_LENGTH / 2 ) + 1) );
  FP_fft2 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( FP_FFT_LENGTH / 2 ) + 1) );
  FP_fft3 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( FP_FFT_LENGTH / 2 ) + 1) );

  adc1_data_history = new std::vector<double>;
  adc2_data_history = new std::vector<double>;
  adc3_data_history = new std::vector<double>;

  FP_fft_plan1 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, FP_adc1_samples, FP_fft1, FFTW_ESTIMATE);
  FP_fft_plan2 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, FP_adc2_samples, FP_fft2, FFTW_ESTIMATE);
  FP_fft_plan3 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, FP_adc3_samples, FP_fft3, FFTW_ESTIMATE);

  int z = 0;
  while (z++ < 700) {
    // Append current data to history vectors
    adc1_data_history->insert(adc1_data_history->end(), data1 + z * 256, data1 + (z + 1) * 256);
    adc2_data_history->insert(adc2_data_history->end(), data2 + z * 256, data2 + (z + 1) * 256);
    adc3_data_history->insert(adc3_data_history->end(), data3 + z * 256, data3 + (z + 1) * 256);

    // Check data history vector length constraints
    if (adc1_data_history->size() > DATA_RETENTION_LENGTH) {
      //printf("Data history is too long, reducing vector sizes\n");
      tmp_data_buffer = new std::vector<double> (adc1_data_history->begin() + adc1_data_history->size() - DATA_RETENTION_LENGTH, adc1_data_history->end());
      delete adc1_data_history;
      adc1_data_history = tmp_data_buffer;
      tmp_data_buffer = NULL;
    }
    if (adc2_data_history->size() > DATA_RETENTION_LENGTH) {
      tmp_data_buffer = new std::vector<double> (adc2_data_history->begin() + adc2_data_history->size() - DATA_RETENTION_LENGTH, adc2_data_history->end());
      delete adc2_data_history;
      adc2_data_history = tmp_data_buffer;
      tmp_data_buffer = NULL;
    }
    if (adc3_data_history->size() > DATA_RETENTION_LENGTH) {
      tmp_data_buffer = new std::vector<double> (adc3_data_history->begin() + adc3_data_history->size() - DATA_RETENTION_LENGTH, adc3_data_history->end());
      delete adc3_data_history;
      adc3_data_history = tmp_data_buffer;
      tmp_data_buffer = NULL;
    }

    for (int vector_idx = 0; vector_idx <= adc1_data_history->size() - FP_FFT_LENGTH; vector_idx += FP_FFT_LENGTH) {
      //fprintf(stderr, "Copying vector memory\n");
      std::copy(adc1_data_history->begin() + vector_idx, adc1_data_history->begin() + vector_idx + FP_FFT_LENGTH, FP_adc1_samples);
      std::copy(adc2_data_history->begin() + vector_idx, adc2_data_history->begin() + vector_idx + FP_FFT_LENGTH, FP_adc2_samples);
      std::copy(adc3_data_history->begin() + vector_idx, adc3_data_history->begin() + vector_idx + FP_FFT_LENGTH, FP_adc3_samples);
      //fprintf(stderr, "Done copying vector memory\n");
      //FP_adc1_samples = data1 + z * FP_FFT_LENGTH;
      //FP_adc2_samples = data2 + z * FP_FFT_LENGTH;
      //FP_adc3_samples = data3 + z * FP_FFT_LENGTH;
      
      /*for (int m = 0; m < FP_FFT_LENGTH; ++m)
        printf("%5.0f ", FP_adc1_samples[m]);
      printf("\n");*/
  
      FP_fft_plan1 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, FP_adc1_samples, FP_fft1, FFTW_ESTIMATE);
      FP_fft_plan2 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, FP_adc2_samples, FP_fft2, FFTW_ESTIMATE);
      FP_fft_plan3 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, FP_adc3_samples, FP_fft3, FFTW_ESTIMATE);
      //FP_fft_plan1 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, (double *)&adc1_data_history[vector_idx], FP_fft1, FFTW_ESTIMATE);
      //FP_fft_plan2 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, (double *)&adc2_data_history[vector_idx], FP_fft2, FFTW_ESTIMATE);
      //FP_fft_plan3 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, (double *)&adc3_data_history[vector_idx], FP_fft3, FFTW_ESTIMATE);
      
      // Perform FFT on current input data
      //fprintf(stderr, "Pre-FFT Notice\n");
      fftw_execute(FP_fft_plan1);
      fftw_execute(FP_fft_plan2);
      fftw_execute(FP_fft_plan3);
      //fprintf(stderr, "FFT Complete\n");
  
      // Calculate Gaussian weighted sums surrounding predicted bin
      int FP_target_bin = FP_f1_bin.getTargetBin();
  
      // Populate current bin magnitude array
      bin_current_mag_sq_1 = FP_fft1[FP_target_bin][0] * FP_fft1[FP_target_bin][0] + FP_fft1[FP_target_bin][1] * FP_fft1[FP_target_bin][1];
      bin_current_mag_sq_2 = FP_fft2[FP_target_bin][0] * FP_fft2[FP_target_bin][0] + FP_fft2[FP_target_bin][1] * FP_fft2[FP_target_bin][1];
      bin_current_mag_sq_3 = FP_fft3[FP_target_bin][0] * FP_fft3[FP_target_bin][0] + FP_fft3[FP_target_bin][1] * FP_fft3[FP_target_bin][1];
      //fprintf(stderr, "Bin Magnitudes Selected\n");
  
      // Index Check
      if (FP_bin_mean_idx >= FP_BIN_HISTORY_LENGTH)
        FP_bin_mean_idx = 0;
  
      // Initilize Mean Array
      if (FP_bin_history_fill < FP_BIN_HISTORY_LENGTH) {
        FP_adc1_bin_history[FP_bin_mean_idx] = bin_current_mag_sq_1;
        FP_adc2_bin_history[FP_bin_mean_idx] = bin_current_mag_sq_2;
        FP_adc3_bin_history[FP_bin_mean_idx] = bin_current_mag_sq_3;
        ++FP_bin_mean_idx;
        
        fftw_destroy_plan(FP_fft_plan1);
        fftw_destroy_plan(FP_fft_plan2);
        fftw_destroy_plan(FP_fft_plan3);
        ++FP_bin_history_fill;
        continue;
      }
      
      adc1_bin_mean = 0;
      adc2_bin_mean = 0;
      adc3_bin_mean = 0;
      
      for (int i = 0; i < FP_BIN_HISTORY_LENGTH; ++i) {
        adc1_bin_mean += FP_adc1_bin_history[i];
        adc2_bin_mean += FP_adc2_bin_history[i];
        adc3_bin_mean += FP_adc3_bin_history[i];
      }
      adc1_bin_mean /= (double)FP_BIN_HISTORY_LENGTH;
      adc2_bin_mean /= (double)FP_BIN_HISTORY_LENGTH;
      adc3_bin_mean /= (double)FP_BIN_HISTORY_LENGTH;
  
      if (bin_current_mag_sq_1 > MEAN_SCALE_FACTOR * adc1_bin_mean &&
          bin_current_mag_sq_2 > MEAN_SCALE_FACTOR * adc2_bin_mean &&
          bin_current_mag_sq_3 > MEAN_SCALE_FACTOR * adc3_bin_mean) {
        FP_bin_indicator.push_back(true);
        printf("1 ");
        printf("%15.0f\n", bin_current_mag_sq_3);
      } else {
        FP_bin_indicator.push_back(false);
        //printf("0 ");
        //printf("%15.0f\n", bin_current_mag_sq_3);
        FP_adc1_bin_history[FP_bin_mean_idx] = bin_current_mag_sq_1;
        FP_adc2_bin_history[FP_bin_mean_idx] = bin_current_mag_sq_2;
        FP_adc3_bin_history[FP_bin_mean_idx] = bin_current_mag_sq_3;
        //printf("%10.0f, %10.0f, %10.0f\n", adc1_bin_mean, adc2_bin_mean, adc3_bin_mean);
        ++FP_bin_mean_idx;
      }
  
      
  
      if (FP_bin_indicator[0] == 0 && FP_bin_indicator[1] == 1) {
        printf("Just had a pulse!\n");
        
        int SP_bin_count = 0;
        while (FP_bin_indicator[SP_bin_count + 1] != 0)
          ++SP_bin_count;
        printf("Signal Bin Count of %d\n", SP_bin_count);
        
        if (SP_bin_count * FP_FFT_LENGTH >= FP_MIN_SAMPLE_LENGTH) {
          int SP_fft_length = SP_bin_count * FP_FFT_LENGTH;
          
          // Copy the sample data into new double array
          double *SP_adc1_samples = new double[SP_fft_length];
          double *SP_adc2_samples = new double[SP_fft_length];
          double *SP_adc3_samples = new double[SP_fft_length];
          //std::copy(adc1_data_history->end() - (SP_bin_count + 1) * FP_FFT_LENGTH, adc1_data_history->end() - FP_FFT_LENGTH, SP_adc1_samples);
          std::copy(adc1_data_history->begin() + vector_idx - (SP_bin_count) * FP_FFT_LENGTH, adc1_data_history->begin() + vector_idx, SP_adc1_samples);
          std::copy(adc2_data_history->begin() + vector_idx - (SP_bin_count) * FP_FFT_LENGTH, adc2_data_history->begin() + vector_idx, SP_adc2_samples);
          std::copy(adc3_data_history->begin() + vector_idx - (SP_bin_count) * FP_FFT_LENGTH, adc3_data_history->begin() + vector_idx, SP_adc3_samples);
          
          // Allocate Memory for Data Arrays
          SP_fft1 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( (SP_fft_length) / 2 ) + 1) );
          SP_fft2 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( (SP_fft_length) / 2 ) + 1) );
          SP_fft3 = (double (*)[2]) fftw_malloc ( sizeof ( fftw_complex ) * (( (SP_fft_length) / 2 ) + 1) );
          
          SP_fft_plan1 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, SP_adc1_samples, SP_fft1, FFTW_ESTIMATE);
          SP_fft_plan2 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, SP_adc1_samples, SP_fft2, FFTW_ESTIMATE);
          SP_fft_plan3 = fftw_plan_dft_r2c_1d(FP_FFT_LENGTH, SP_adc1_samples, SP_fft3, FFTW_ESTIMATE);
          
          // Perform FFT on current input data
          fftw_execute(SP_fft_plan1);
          fftw_execute(SP_fft_plan2);
          fftw_execute(SP_fft_plan3);
          
          FFTBinAffinePrediction SP_f1_bin(bitrate,
                                           SP_fft_length,
                                           target_frequency1,
                                           BIN_PREDICTION_M,
                                           BIN_PREDICTION_B,
                                           3);
          /*FFTBinAffinePrediction SP_f2_bin(bitrate,
                                           SP_fft_length,
                                           target_frequency2,
                                           BIN_PREDICTION_M,
                                           BIN_PREDICTION_B,
                                           3);*/
          
          int SP_target_bin = SP_f1_bin.getTargetBin();
          
          fprintf(stderr, "SP Target Bin: %d\n", SP_target_bin);
          
          // Calculate Phase of each signal found on each ADC
          double phase1 = atan2((double)SP_fft1[(int)SP_target_bin][1], (double)SP_fft1[(int)SP_target_bin][0]);
          double phase2 = atan2((double)SP_fft2[(int)SP_target_bin][1], (double)SP_fft2[(int)SP_target_bin][0]);
          double phase3 = atan2((double)SP_fft3[(int)SP_target_bin][1], (double)SP_fft3[(int)SP_target_bin][0]);
          
          // Calculate usable phase differences
          double delta1 = phase2 - phase1;
          double delta2 = phase3 - phase2;
          double delta3 = phase1 - phase3;
          
          fprintf(stderr, "Phase: %5.5f  %5.5f  %5.5f\n", phase1, phase2, phase3);
          fprintf(stderr, "Deltas: %5.5f  %5.5f  %5.5f\n", delta1, delta2, delta3);
          
          // Free Data Array Memory
          fftw_free(SP_fft1);
          fftw_free(SP_fft2);
          fftw_free(SP_fft3);
          
          // Determine minimum phase difference for pair selection
          int min_index = 3;
          if (fabs(delta2) < fabs(delta1) && fabs(delta2) < fabs(delta3))
            min_index = 2;
          if (fabs(delta1) < fabs(delta2) && fabs(delta1) < fabs(delta3))
            min_index = 1;
          
          double delta_tmp;
          switch (min_index) {
            case 1:
              delta_tmp = delta1;
              if (delta3 > delta2)
                delta_tmp = -1.0 * delta1 + sign(delta_tmp) * 2.0 * M_PI;
              angle_estimate1 = delta_tmp * (double)(((double)target_wavelength1 / 2.0) / SENSOR_SPACING_2_1) * 180.0 / M_PI / 2.0;
              break;
            case 2:
              delta_tmp = delta2;
              if (delta1 > delta3)
                delta_tmp = -1.0 * delta2 + 2.0 * M_PI;
              angle_estimate1 = (delta_tmp - 4.0 / 3.0 * M_PI) * ((target_wavelength1 / 2.0) / SENSOR_SPACING_3_2) * 180.0 / M_PI / 2.0;
              break;
            case 3:
              delta_tmp = delta3;
              if (delta2 > delta1)
                delta_tmp = -1.0 * delta3 - 2.0 * M_PI;
              angle_estimate1 = (delta_tmp + 4.0 / 3.0 * M_PI ) * (((double)target_wavelength1 / 2.0) / SENSOR_SPACING_1_3) * 180.0 / M_PI / 2.0;
              break;
            default:
              fprintf(stderr, "Sonar: Invalid min_index for phase difference.");
          }
          fprintf(stderr, "Min Index: %d\n", min_index);
          fprintf(stderr, "Angle Estimate (1): %3.2f\n", angle_estimate1);
          
          // DEBUG
          for (int blah = 0; blah < SP_bin_count * FP_FFT_LENGTH; ++blah)
            printf("%5.0f", SP_adc1_samples[blah]);
          printf("\n");
          
        }
      }
  
      if (FP_bin_indicator.size() > FP_BIN_HISTORY_LENGTH)
        FP_bin_indicator.erase(FP_bin_indicator.begin());
    }
  }
  fftw_destroy_plan(FP_fft_plan1);
  fftw_destroy_plan(FP_fft_plan2);
  fftw_destroy_plan(FP_fft_plan3);
  fftw_destroy_plan(SP_fft_plan1);
  fftw_destroy_plan(SP_fft_plan2);
  fftw_destroy_plan(SP_fft_plan3);

  return 0;
}

