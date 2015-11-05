/*=========================================================================
| (c) 2005-2007  Total Phase, Inc.
|--------------------------------------------------------------------------
| Project : Cheetah Sample Code
| File    : async.c
|--------------------------------------------------------------------------
| Use the asynchronous interface of the Cheetah host adapter
|--------------------------------------------------------------------------
| Redistribution and use of this file in source and binary forms, with
| or without modification, are permitted.
|
| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
| FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
| COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
| INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
| BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
| LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
| CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
| LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
| ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
| POSSIBILITY OF SUCH DAMAGE.
 ========================================================================*/

//=========================================================================
// INCLUDES
//=========================================================================
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cheetah.h"
#include <fftw3.h>

#ifdef _WIN32
#include <time.h>
#else
#include <sys/time.h>
#endif


//=========================================================================
// CONSTANTS
//=========================================================================
// Uncomment to show the data returned by the shifting device
#define SHOW_DATA

// Add a delay between bytes by changing this constant (in nanoseconds)
#define BYTE_DELAY 0


#define MAX_TX_LENGTH 3072

const int N_FFT = 1024;



//=========================================================================
// UTILITY FUNCTIONS
//=========================================================================
static s64 _timeMillis () {
#ifdef _WIN32
    return ((s64)clock()) * 1000 / CLOCKS_PER_SEC;
#else
    struct timeval tv;
    gettimeofday(&tv, 0);
    return ((s64)tv.tv_sec * 1000L) + (s64)(tv.tv_usec / 1000L);
#endif
}


#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef min
	#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

//=========================================================================
// FUNCTIONS
//=========================================================================
static void _blast_async (Cheetah handle, u32 txnlen, u32 iter) {
    double elapsed, cycle_duration;
    u32    i,j,k;
    int    count = 0;
    u08    data_out[2];
//    u08    data_out[4];
//    u08    *data_in;
    u08    data_in[MAX_TX_LENGTH];
    int    input;
    int    ready_bit;
    int    valid_data_point;
    s64    start, itteration_start;
    int    ret;
    fftw_complex    *data, *fft_result;
    //fftw_plan       plan_forward;


    data        = ( fftw_complex* ) fftw_malloc( sizeof( fftw_complex ) * N_FFT );
    fft_result  = ( fftw_complex* ) fftw_malloc( sizeof( fftw_complex ) * N_FFT );


    // Opens file to write out data as column vector
    FILE *data1_output, *data2_output, *data3_output;
    data1_output = fopen("data.1.dat", "w");
    data2_output = fopen("data.2.dat", "w");
    data3_output = fopen("data.3.dat", "w");
    if (data1_output == NULL || data2_output == NULL || data3_output == NULL) {
        fprintf(stderr, "Could not open files!!!");
        exit(1);
    }

    // Make a simple queue to just assert OE.
    ch_spi_queue_clear(handle);
    ch_spi_queue_oe(handle, 1);
    ch_spi_batch_shift(handle, 0, 0);

    fprintf(stderr, "Beginning to queue SPI packets...");
    // Queue the batch which is a sequence of SPI packets
    // (back-to-back) each of length 2.
    data_out[0] = 0xff;
    data_out[1] = 0xff;
    ch_spi_queue_clear(handle);
//    for (i = 0; i < N_FFT * txnlen; ++i) {
    for (i = 0; i < N_FFT * 3; ++i) {
        //ch_spi_queue_ss(handle, 0x1);
    
        
        //data_out[2] = (count >>  8) & 0xff;
        //data_out[3] = (count >>  0) & 0xff;
    
        ++count;
        
        //ch_spi_queue_array(handle, 2, data_out);
        //ch_spi_queue_ss(handle, 0x0);
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
    
    fprintf(stderr, "Beginning asynchronous SPI packet transmission...\n");
    start = _timeMillis();
    itteration_start = _timeMillis();

    // First, submit first batch 
    ch_spi_async_submit(handle);

    for (i = 0; i < iter; ++i) {
        cycle_duration = ((double)(_timeMillis() - itteration_start)) / 1000;
        itteration_start = _timeMillis();
        fprintf(stderr, "Packet %d...\n", i);
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
        ch_sleep_ms(1);
        
        // Collect the previous batch
        //ret = ch_spi_async_collect(handle, txnlen, data_in);
        ret = ch_spi_async_collect(handle, N_FFT * 3, data_in);        
        //elapsed = ((double)(_timeMillis() - start)) / 1000;


        //data_in = (u08 *)malloc(3); 
        for (j = 0; j < N_FFT * 3; j += 6) {
//        for (j = 0; j < N_FFT; j +=2 ) {
            // SS2 Data
            input = (data_in[j] << 8) + data_in[j+1];
            ready_bit = input & 0x4000;
            valid_data_point = (input & 0x3ffc) >> 2;
//            valid_data[j/2] = valid_data_point;
            fprintf(data2_output, "%d\n", valid_data_point);

            // SS3 Data
            input = (data_in[j+2] << 8) + data_in[j+3];
            ready_bit = input & 0x4000;
            valid_data_point = (input & 0x3ffc) >> 2;
//            valid_data[j/2 + 1] = valid_data_point;
            fprintf(data3_output, "%d\n", valid_data_point);
//            fprintf(data1_output, "%d ", valid_data_point);
//            data[j][0] = (double)valid_data_point;
//            data[j][1] = 0.0;

            // SS1 Data
            input = (data_in[j+4] << 8) + data_in[j+5];
            ready_bit = input & 0x4000;
            valid_data_point = (input & 0x3ffc) >> 2;
//            valid_data[j/2 + 2] = valid_data_point;
            fprintf(data1_output, "%d\n", valid_data_point);
        }

        // Ensures communication is successful
        if (ret < 0)  printf("status error: %s\n", ch_status_string(ret));
        fflush(stdout);

        //plan_forward  = fftw_plan_dft_1d( N_FFT, data, fft_result, FFTW_FORWARD, FFTW_ESTIMATE );
        //fftw_execute ( plan_forward );

        // Print FFT result
        //printf("Sampling frequency: %.6lf Hz\n", N_FFT / cycle_duration);
        //printf("Expected Index: %1.0f\n", 21000 * cycle_duration); 

        //fprintf(stderr, "Output FFT Result\n");
        //for( k = max(21000 * cycle_duration - 20, 0) ; k < min(21000 * cycle_duration + 20, N_FFT); k++ ) {
            //fprintf(data1_output, "%2.2f ", sqrt(pow(fft_result[k][0],2) + pow(fft_result[k][1],2)));
            //fprintf(stderr, "|fft_result[%d]| = %2.2f\n",
            //    k, sqrt(pow(fft_result[k][0],2) + pow(fft_result[k][1],2)) );
        //}

        //for( k = 0 ; k < N_FFT; k++ ) {
        //    fprintf(data1_output, "%2.2f ", data[k][0]);
        //}
	k=0;
        //fprintf(data1_output, "\n");
        //fprintf(stderr, "End of FFT Results\n");


        // The current batch is now shifting out on the SPI
        // interface. The application can again do some more tasks
        // here but this entire loop must finish so that a new
        // batch is armed before the current batch completes.
        ch_sleep_ms(1);
    }
    elapsed = ((double)(_timeMillis() - start)) / 1000;
    fprintf(stderr, "Transmission complete.  Data Collected.\n");

    fclose(data1_output); data1_output = NULL;
    fclose(data2_output); data2_output = NULL;
    fclose(data3_output); data3_output = NULL;
    
    printf("Took %.2lf seconds to get the data.\n", elapsed);
    fflush(stdout);
    printf("Sampling frequency: %.6lf Hz\n", 3 * 3 * N_FFT * iter / elapsed);
    fflush(stdout);

    // Collect batch the last batch
    //ret = ch_spi_async_collect(handle, 0, 0);
    //elapsed = ((double)(_timeMillis() - start)) / 1000;
    //printf("collected batch #%03d in %.5lf seconds\n", i+1, elapsed);
    //if (ret < 0)  printf("status error: %s\n", ch_status_string(ret));
    //fflush(stdout);
}


//=========================================================================
// MAIN PROGRAM
//=========================================================================
int main (int argc, char *argv[]) {
    Cheetah handle;
    int port     = 0;
    int bitrate  = 0;
    u08 mode     = 0;
    u32 txnlen;
    u32 iter;

    if (argc < 5) {
        printf("usage: async PORT BITRATE TXN_LENGTH ITER\n");
        printf("\n");
        printf("TXN_LENGTH is the number of SPI packets, each of length\n");
        printf("4 to queue in a single batch.\n");
        printf("\n");
        printf("ITER is the number of batches to process asynchronously.\n");
        return 1;
    }

    port     = atoi(argv[1]);
    bitrate  = atoi(argv[2]);
    txnlen   = atoi(argv[3]);
    iter     = atoi(argv[4]);
    
    // Open the device
    handle = ch_open(port);
    if (handle <= 0) {
        printf("Unable to open Cheetah device on port %d\n", port);
        printf("Error code = %d (%s)\n", handle, ch_status_string(handle));
        return 1;
    }
    printf("Opened Cheetah device on port %d\n", port);

    printf("Host interface is %s\n",
           (ch_host_ifce_speed(handle)) ? "high speed" : "full speed");

    // Ensure that the SPI subsystem is configured.
    ch_spi_configure(handle, (mode >> 1), mode & 1, CH_SPI_BITORDER_MSB, 0x0);
    printf("SPI configuration set to mode %d, %s shift, SS[2:0] active low\n",
           mode, "MSB");
    fflush(stdout);

    // Power the target using the Cheetah adapter's power supply.
    ch_target_power(handle, CH_TARGET_POWER_ON);
    ch_sleep_ms(100);

    // Set the bitrate.
    bitrate = ch_spi_bitrate(handle, bitrate);
    printf("Bitrate set to %d kHz\n", bitrate);
    fflush(stdout);

    _blast_async(handle, txnlen, iter);
    
    // Close and exit.
    ch_close(handle);
    return 0;
}
