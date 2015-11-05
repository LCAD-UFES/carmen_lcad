#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <fftw3.h>
#include <cstring>

#include "sonar_utils.h"
#include "sonar_fft.h"
#include "cheetah.h"

// Globals
const double SPEED_SOUND_WATER     = 1482;  // [m/s]
const double SENSOR_SPACING_2_1    = 0.024; // [m]
const double SENSOR_SPACING_3_2    = 0.024; // [m]
const double SENSOR_SPACING_1_3    = 0.024; // [m]
const int    DATA_BLOCK_SIZE       = 512;
const int    DATA_RETENTION_LENGTH = 3 * DATA_BLOCK_SIZE;

// Declare Cheetah Variables
Cheetah    handle;
int        port, bitrate;
uint8_t    mode = 0;
int        ret, input, ready_bit, valid_data_point;

// FFT & Bin Prediction
const int  FP_FFT_LENGTH    = 32;  // First-pass FFT length
double     BIN_PREDICTION_M = 10.59;
double     BIN_PREDICTION_B = -1193.82;

int        target_frequency1, target_frequency2;
double     target_wavelength1, target_wavelength2;
double     angle_estimate1, angle_estimate2;
double     bin_current_mag_sq_1, bin_current_mag_sq_2, bin_current_mag_sq_3;
double     adc1_bin_mean, adc2_bin_mean, adc3_bin_mean;
std::vector<bool> FP_bin_indicator;

// Raw Data
const int  TX_LENGTH = DATA_BLOCK_SIZE * 3 * 2;
uint8_t    data_in[TX_LENGTH];
uint8_t    data_out[2];

// Hysterisis Vars
const int FP_BIN_HISTORY_LENGTH = 10;
const double MEAN_SCALE_FACTOR = 10.0;
const int FP_MIN_SAMPLE_LENGTH = 128;
int FP_bin_mean_idx = 1;
double FP_adc1_bin_history[FP_BIN_HISTORY_LENGTH];
double FP_adc2_bin_history[FP_BIN_HISTORY_LENGTH];
double FP_adc3_bin_history[FP_BIN_HISTORY_LENGTH];
int    FP_bin_history_fill = 0;
std::vector<double> *adc1_data_history, *adc2_data_history, *adc3_data_history, *tmp_data_buffer;


int main (int argc, char const *argv[]) {
  //fprintf(stderr, "Starting up...\n");
  port = 0;
  bitrate = 10000;
  
  double sampling_frequency = SeaBee3_Sonar::getSamplingFrequency(bitrate, BIN_PREDICTION_M, BIN_PREDICTION_B);

  // Assign Target Frequencies
  target_frequency1  = 25000; //atoi(argv[2]);
  target_frequency2  = 23000; //atoi(argv[3]);
  target_wavelength1 = (double)SPEED_SOUND_WATER / (double)target_frequency1;
  target_wavelength2 = (double)SPEED_SOUND_WATER / (double)target_frequency2;

  // Declare Data Vectors
  double FP_adc1_samples[FP_FFT_LENGTH], FP_adc2_samples[FP_FFT_LENGTH], FP_adc3_samples[FP_FFT_LENGTH];

  adc1_data_history = new std::vector<double>;
  adc2_data_history = new std::vector<double>;
  adc3_data_history = new std::vector<double>;
  
  // Define and allocate the Data Vectors
  double *data1, *data2, *data3;
  data1 = (double *) fftw_malloc(sizeof(double) * DATA_BLOCK_SIZE);
  data2 = (double *) fftw_malloc(sizeof(double) * DATA_BLOCK_SIZE);
  data3 = (double *) fftw_malloc(sizeof(double) * DATA_BLOCK_SIZE);
  memset(data1, 0, sizeof(double) * DATA_BLOCK_SIZE);
  memset(data2, 0, sizeof(double) * DATA_BLOCK_SIZE);
  memset(data3, 0, sizeof(double) * DATA_BLOCK_SIZE);
  
  // Open the device
  handle = ch_open(port);

  if (handle <= 0) {
    fprintf(stderr, "Unable to open Cheetah device on port %d\n", port);
    fprintf(stderr, "Error code = %d (%s)\n", handle, ch_status_string(handle));
    exit(1);
  }
  fprintf(stderr, "Opened Cheetah device on port %d\n", port);

  fprintf(stderr, "Host interface is %s\n",
      (ch_host_ifce_speed(handle)) ? "high speed" : "full speed");

  // Ensure that the SPI subsystem is configured.
  ch_spi_configure(handle, CheetahSpiPolarity(mode >> 1), CheetahSpiPhase(mode & 1), CH_SPI_BITORDER_MSB, 0x0);
  fprintf(stderr, "SPI configuration set to mode %d, %s shift, SS[2:0] active low\n", mode, "MSB");

  // Power the target using the Cheetah adapter's power supply.
  ch_target_power(handle, CH_TARGET_POWER_ON);
  ch_sleep_ms(100);

  // Set the bitrate.
  bitrate = ch_spi_bitrate(handle, bitrate);
  fprintf(stderr, "Bitrate set to %d kHz\n", bitrate);

  // Make a simple queue to just assert OE.
  ch_spi_queue_clear(handle);
  ch_spi_queue_oe(handle, 1);
  ch_spi_batch_shift(handle, 0, 0);

  // Queue the batch, which is a sequence of SPI packets (back-to-back) each of length 2.
  fprintf(stderr, "Beginning to queue SPI packets...");
  data_out[0] = 0xff;
  data_out[1] = 0xff;
  ch_spi_queue_clear(handle);
  
  for (int i = 0; i < DATA_BLOCK_SIZE; ++i) {
    // Convert Slave 1
    ch_spi_queue_ss(handle, 0xF);
    ch_spi_queue_array(handle, 2, data_out);
    ch_spi_queue_ss(handle, 0xE);
    
    // Convert Slave 2
    ch_spi_queue_ss(handle, 0xF);
    ch_spi_queue_array(handle, 2, data_out);
    ch_spi_queue_ss(handle, 0xD);
    
    // Convert Slave 3
    ch_spi_queue_ss(handle, 0xF);
    ch_spi_queue_array(handle, 2, data_out);
    ch_spi_queue_ss(handle, 0xB);
  }
  fprintf(stderr, " Done\n");
  
  // Submit the first batch
  ch_spi_async_submit(handle);

  int z = 0;
  while (true) {
    //fprintf(stderr, "Starting Loop!\n");
    // Submit another batch, while the previous one is in
    // progress.  The application may even clear the current
    // batch queue and queue a different set of SPI
    // transactions before submitting this batch
    // asynchronously.
    ch_spi_async_submit(handle);
    
    // The application can now perform some other functions
    // while the Cheetah is both finishing the previous batch
    // and shifting the current batch as well.  In order to
    // keep the Cheetah's pipe full, this entire loop must
    // complete AND another batch must be submitted
    // before the current batch completes.
    //ch_sleep_ms(1);
    
    // Collect the previous batch
    // The length of the batch, FP_FFT_LENGTH * 6, come from the fact that 3 ADCs
    // are batched and the return data requires 2 bytes.  (2 * 3 = 6)
    ret = ch_spi_async_collect(handle, TX_LENGTH, data_in);

    int data_idx = 0;
    for (int j = 0; j < TX_LENGTH; j += 6) {
      // SS3 Data
      input = (data_in[j] << 8) + data_in[j+1];
      ready_bit = input & 0x4000;
      valid_data_point = (input & 0x3ffc) >> 2;
      data3[data_idx] = valid_data_point;

      // SS1 Data
      input = (data_in[j+2] << 8) + data_in[j+3];
      ready_bit = input & 0x4000;
      valid_data_point = (input & 0x3ffc) >> 2;
      data1[data_idx] = valid_data_point;

      // SS2 Data
      input = (data_in[j+4] << 8) + data_in[j+5];
      ready_bit = input & 0x4000;
      valid_data_point = (input & 0x3ffc) >> 2;
      data2[data_idx] = valid_data_point;
      ++data_idx;
    }
    
    // Append current data to history vectors
    adc1_data_history->insert(adc1_data_history->end(), data1, data1 + DATA_BLOCK_SIZE);
    adc2_data_history->insert(adc2_data_history->end(), data2, data2 + DATA_BLOCK_SIZE);
    adc3_data_history->insert(adc3_data_history->end(), data3, data3 + DATA_BLOCK_SIZE);

    
    //printf("Vector Length: %d\n", adc1_data_history->size());
    for (int vector_idx = 0; vector_idx <= adc1_data_history->size() - FP_FFT_LENGTH; vector_idx += FP_FFT_LENGTH) {
      //fprintf(stderr, "Copying vector memory\n");
      std::copy(adc1_data_history->begin() + vector_idx, adc1_data_history->begin() + vector_idx + FP_FFT_LENGTH, FP_adc1_samples);
      std::copy(adc2_data_history->begin() + vector_idx, adc2_data_history->begin() + vector_idx + FP_FFT_LENGTH, FP_adc2_samples);
      std::copy(adc3_data_history->begin() + vector_idx, adc3_data_history->begin() + vector_idx + FP_FFT_LENGTH, FP_adc3_samples);
      //fprintf(stderr, "Done copying vector memory\n");
      
/*      for (int i = 0; i < FP_FFT_LENGTH; ++i)
        fprintf(stderr, "%5.0f", FP_adc1_samples[i]);
      fprintf(stderr, "\n");*/
      
      //fprintf(stderr, "Prepping DataToFFT\n");
      SeaBee3_Sonar::DataToFFT fp_fft_f1 = SeaBee3_Sonar::DataToFFT(FP_adc1_samples, FP_adc2_samples, FP_adc3_samples,
                                                                    FP_FFT_LENGTH, sampling_frequency, target_frequency1);
      SeaBee3_Sonar::DataToFFT fp_fft_f2 = SeaBee3_Sonar::DataToFFT(FP_adc1_samples, FP_adc2_samples, FP_adc3_samples,
                                                                    FP_FFT_LENGTH, sampling_frequency, target_frequency2);
      //fprintf(stderr, "Done prepping DataToFFT\n");
      
      bin_current_mag_sq_1 = fp_fft_f1.getMagnitude(1);
      bin_current_mag_sq_2 = fp_fft_f1.getMagnitude(2);
      bin_current_mag_sq_3 = fp_fft_f1.getMagnitude(3);
  
      // Index Check
      if (FP_bin_mean_idx >= FP_BIN_HISTORY_LENGTH)
        FP_bin_mean_idx = 0;
  
      // Initilize Mean Array
      if (FP_bin_history_fill < FP_BIN_HISTORY_LENGTH) {
        FP_adc1_bin_history[FP_bin_mean_idx] = bin_current_mag_sq_1;
        FP_adc2_bin_history[FP_bin_mean_idx] = bin_current_mag_sq_2;
        FP_adc3_bin_history[FP_bin_mean_idx] = bin_current_mag_sq_3;
        ++FP_bin_mean_idx;
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
          
          std::copy(adc1_data_history->begin() + vector_idx - (SP_bin_count) * FP_FFT_LENGTH, adc1_data_history->begin() + vector_idx, SP_adc1_samples);
          std::copy(adc2_data_history->begin() + vector_idx - (SP_bin_count) * FP_FFT_LENGTH, adc2_data_history->begin() + vector_idx, SP_adc2_samples);
          std::copy(adc3_data_history->begin() + vector_idx - (SP_bin_count) * FP_FFT_LENGTH, adc3_data_history->begin() + vector_idx, SP_adc3_samples);
          
          // Delete all data up to and including the detected ping
          adc1_data_history->erase(adc1_data_history->begin(), adc1_data_history->begin() + vector_idx);
          adc2_data_history->erase(adc2_data_history->begin(), adc2_data_history->begin() + vector_idx);
          adc3_data_history->erase(adc3_data_history->begin(), adc3_data_history->begin() + vector_idx);
/*          // DEBUG
          for (int blah = 0; blah < FP_FFT_LENGTH; ++blah)
            printf("%5.0f", FP_adc1_samples[blah]);
          printf("\n\n\n\n");*/
          
          SeaBee3_Sonar::DataToFFT sp_fft_f1 = SeaBee3_Sonar::DataToFFT(FP_adc1_samples, FP_adc2_samples, FP_adc3_samples,
                                          FP_FFT_LENGTH, sampling_frequency, target_frequency1);
          SeaBee3_Sonar::DataToFFT sp_fft_f2 = SeaBee3_Sonar::DataToFFT(FP_adc1_samples, FP_adc2_samples, FP_adc3_samples,
                                          FP_FFT_LENGTH, sampling_frequency, target_frequency2);
          
          fprintf(stderr, "SP Target Bin: %d\n", sp_fft_f1.getTargetBin());
          
          double phase1 = sp_fft_f1.getPhase(1);
          double phase2 = sp_fft_f1.getPhase(2);
          double phase3 = sp_fft_f1.getPhase(3);
          
          // Calculate usable phase differences
          double delta1 = phase2 - phase1;
          double delta2 = phase3 - phase2;
          double delta3 = phase1 - phase3;
          
          fprintf(stderr, "Phase: %5.5f  %5.5f  %5.5f\n", phase1, phase2, phase3);
          fprintf(stderr, "Deltas: %5.5f  %5.5f  %5.5f\n", delta1, delta2, delta3);
          
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
                delta_tmp = -1.0 * delta1 + SeaBee3_Sonar::sign(delta_tmp) * 2.0 * M_PI;
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
          
/*          // DEBUG
          for (int blah = 0; blah < SP_bin_count * FP_FFT_LENGTH; ++blah)
            printf("%5.0f", SP_adc1_samples[blah]);
          printf("\n");*/
          
        }
      }
  
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
    
    if (FP_bin_indicator.size() > FP_BIN_HISTORY_LENGTH)
      FP_bin_indicator.erase(FP_bin_indicator.begin());
    }
  }

  return 0;
}

