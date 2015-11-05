#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "cheetah.h"
#include <fftw3.h>
#include "CImg.h"

using namespace cimg_library;

// Cheetah
Cheetah handle;
int	port	= 0;
int	bitrate	= 0;
uint8_t	mode	= 0;

// FFT
const int N_FFT = 512;
const size_t MAX_TX_LENGTH = (N_FFT) * 3 * 2;
fftw_complex	*data, *fft_result;
fftw_plan	plan_forward;

// Data
uint8_t	data_in[MAX_TX_LENGTH];
uint8_t	data_out[2];
int	ret;
//int	txnlen   = atoi(argv[3]);
int	input;
int	ready_bit;
int	valid_data_point;




int main (int argc, char const *argv[]) {
	if (argc < 3) {
		printf("usage: async PORT BITRATE\n");
		return 1;
	}

	port	 = atoi(argv[1]);
	bitrate  = atoi(argv[2]);

	// Open the device
	handle = ch_open(port);

	if (handle <= 0) {
		printf("Unable to open Cheetah device on port %d\n", port);
		printf("Error code = %d (%s)\n", handle, ch_status_string(handle));
		exit(1);
	}
	printf("Opened Cheetah device on port %d\n", port);

	printf("Host interface is %s\n",
		  (ch_host_ifce_speed(handle)) ? "high speed" : "full speed");

	// Ensure that the SPI subsystem is configured.
	ch_spi_configure(handle, CheetahSpiPolarity(mode >> 1), CheetahSpiPhase(mode & 1), CH_SPI_BITORDER_MSB, 0x0);
	printf("SPI configuration set to mode %d, %s shift, SS[2:0] active low\n", mode, "MSB");

	// Power the target using the Cheetah adapter's power supply.
	ch_target_power(handle, CH_TARGET_POWER_ON);
	ch_sleep_ms(100);

	// Set the bitrate.
	bitrate = ch_spi_bitrate(handle, bitrate);
	printf("Bitrate set to %d kHz\n", bitrate);

	// Make a simple queue to just assert OE.
	ch_spi_queue_clear(handle);
	ch_spi_queue_oe(handle, 1);
	ch_spi_batch_shift(handle, 0, 0);

	fprintf(stderr, "Beginning to queue SPI packets...");
	// Queue the batch, which is a sequence of SPI packets (back-to-back) each of length 2.
	data_out[0] = 0xff;
	data_out[1] = 0xff;
	ch_spi_queue_clear(handle);
	
	for (int i = 0; i < N_FFT * 3; ++i) {
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

	// Define the Data Vectors
	CImg<int> data1(1, N_FFT, 1, 1, 0);
	CImg<int> data2(1, N_FFT, 1, 1, 0);
	CImg<int> data3(1, N_FFT, 1, 1, 0);
	CImg<float> result(1, N_FFT, 1, 1, 0);
	CImgList<float> fft_output;
	CImgList<float> display_set;
	CImgDisplay window(600,800);


	// Submit the first batch
	ch_spi_async_submit(handle);

	while (1) {
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
		ret = ch_spi_async_collect(handle, N_FFT * 6, data_in);	

		for (int j = 0; j < N_FFT * 6;j += 6) {

			// SS3 Data
			input = (data_in[j] << 8) + data_in[j+1];
			ready_bit = input & 0x4000;
			valid_data_point = (input & 0x3ffc) >> 2;
			data3(1,j/6) = valid_data_point;

			// SS2 Data
			input = (data_in[j+2] << 8) + data_in[j+3];
			ready_bit = input & 0x4000;
			valid_data_point = (input & 0x3ffc) >> 2;
			data2(1,j/6) = valid_data_point;

			// SS1 Data
			input = (data_in[j+4] << 8) + data_in[j+5];
			ready_bit = input & 0x4000;
			valid_data_point = (input & 0x3ffc) >> 2;
			data1(1,j/6) = valid_data_point;
		}
		//(data1,data2,data3).display();
		fft_output = data2.get_FFT('y');
		result = fft_output[0].sqr() + fft_output[1].sqr();
		result = result.crop(0,10,0,200);
		display_set.push_back(result);
		if (display_set.size() > 100)
			display_set.pop_front();
		window.resize(false).display(display_set);

		// Ensures communication is successful
		if (ret < 0)  printf("status error: %s\n", ch_status_string(ret));
		fflush(stdout);

	}

	return 0;
}
