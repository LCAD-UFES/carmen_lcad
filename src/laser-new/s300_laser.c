#include "s300_laser.h"
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <signal.h>
#include <carmen/carmenserial.h>

#define	MAX_MEAS_DIST_MASK 0x1FFF;
#define	IS_REFLECT_MASK  0x2000;
#define SICK_S300_NUM_SCAN_POINTS 541

const unsigned char header_bytes[10] = {0,0,0,0,0,0,0,0,255,7};
unsigned char serial_buf[8192];
unsigned int header_state = 0;	

const unsigned short crc_LookUpTable[256] = { 
   0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 
   0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 
   0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 
   0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 
   0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 
   0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 
   0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 
   0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 
   0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 
   0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 
   0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 
   0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 
   0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 
   0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 
   0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 
   0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 
   0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 
   0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067, 
   0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 
   0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 
   0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 
   0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
   0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 
   0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 
   0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 
   0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 
   0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 
   0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 
   0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 
   0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 
   0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 
   0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0 
 }; 


#ifdef USE_TCP862
#include <carmen/tcp862.h>
#define serial_configure(_fd, _baudrate, _parity) tcp862_setBaud(_fd,_baudrate)
#define serial_readn(_fd,_buf,_size)   tcp862_readn(_fd,_buf,_size)
#define serial_writen(_fd,_buf,_size)  tcp862_writen(_fd,_buf,_size)
#define serial_ClearInputBuffer(_fd) tcp862_clearInputBuffer(_fd)
#else
#define serial_configure(_fd, _baudrate, _parity)  carmen_serial_configure(_fd, _baudrate, _parity)
#define serial_readn(_fd,_buf,_size)  carmen_serial_readn(_fd,_buf,_size)
#define serial_writen(_fd,_buf,_size) carmen_serial_writen(_fd,_buf,_size)
#define serial_ClearInputBuffer(_fd)  carmen_serial_ClearInputBuffer(_fd)
#endif


// S300 checksum algorithm
static unsigned short s300_compute_checksum(unsigned char *CommData, int uLen) {
	
	int CounterWord; 
	unsigned short CrcValue=0xFFFF;

	for (CounterWord = 0; CounterWord < uLen; CounterWord++)  { 
		CrcValue = (CrcValue << 8) ^ crc_LookUpTable[ (((unsigned char)(CrcValue >> 8)) ^ *CommData) ]; 
		CommData++; 
	} 

	return (CrcValue); 	
}


unsigned int get_unsigned_word(unsigned char msb, unsigned char lsb) {
		return (msb << 8) | lsb;
}

unsigned int s300_find_header(unsigned char input, struct timeval *timestamp){

  switch(header_state) {
	case 0:
		if (input == header_bytes[0]) {
			header_state++;
			if (timestamp) gettimeofday(timestamp, NULL);
		}

    	break;
	case 1:
		if (input == header_bytes[1]) header_state++;
		else header_state = 0;
    	break;
	case 2:
		if (input == header_bytes[2]) header_state++;
		else header_state = 0;
    	break;
	case 3:
		if (input == header_bytes[3]) header_state++;
		else header_state = 0;
    	break;
	case 4:
		if (input == header_bytes[4]) header_state++;
		else header_state = 0;
    	break;
	case 5:
		if (input == header_bytes[5]) header_state++;
		else header_state = 0;
    	break;
	// bytes 6 and 7 are skipped, they contain the packet size
	case 6:
		header_state++;
    	break;
	case 7:
		header_state++;
    	break;
	case 8:
		if (input == header_bytes[8]) header_state++;
		else header_state = 0;
    	break;
	case 9:
		if (input == header_bytes[9]) return 1;
		else {
			header_state = 0;
	    	break;
		}
	default:
	  header_state = 0;
      break;
	}

  return 0;
}


unsigned char s300_get_next_scan(
				     s300_laser_t* sick,
				     unsigned int * n_ranges,
				     unsigned int* range, 
				     struct timeval *tv ) {
						 
  	unsigned char*t=serial_buf+header_state;
	int i;
	int telegram_size = 0;
	unsigned int crc_read;
	unsigned int crc_calc;
						 
	
	/* S300 header format in continuous mode:
	   
       | 00 00 00 00 |   4 byte reply header
       | 00 00 |         data block number (fixed)
       | xx xx |         size of data telegram (should be dec 1104)
       | FF 07 |         fixed
       | xx xx |         protocol version
       | 0x 00 |         status: 00 00 = normal, 01 00 = lockout
       | xx xx xx xx |   scan number
       | xx xx |         telegram number
       | BB BB |         fixed
       | 11 11 |         fixed
	      ...            data
       | xx xx |         CRC
	   
	   Header: bytes 0 to 23
	   Data:   bytes 24 to 1101
	   CRC:    bytes 1102, 1103
	
	 */
	
	
	// read data from serial
	serial_readn(sick->fd, t, 1);
						 
	if (s300_find_header( serial_buf[header_state], tv) ) {
		t++;
		serial_readn(sick->fd, t, 1098);	// read rest of telegram
	}
	else {
		return 0;
	}

	// check weather size of telegram is as expected
	// (this check could be left out?)
	telegram_size = 2 * get_unsigned_word(serial_buf[6], serial_buf[7]);
	if (telegram_size  != 1104 ) {
		fprintf(stderr, "wrong no of data bytes\n");
		return 0;
	};	


	crc_read = get_unsigned_word(serial_buf[1107], serial_buf[1106]);
	crc_calc = s300_compute_checksum(&serial_buf[4], 1102);
	
	if (crc_calc == crc_read) {
		
		if (n_ranges) *n_ranges = SICK_S300_NUM_SCAN_POINTS;
			
		// read range data
		for(i = 0; i < SICK_S300_NUM_SCAN_POINTS; i++) {
			if (range){
				range[i] = 
					get_unsigned_word(
						 serial_buf[24 + 2 * i + 1],
						 serial_buf[24 + 2 * i    ]
					) & MAX_MEAS_DIST_MASK; 	// masking some bits
			}
			else {
				fprintf(stderr,"ERROR: range buffer not valid!\n");
				return 0;
			}
			//printf ("range[540]: %d\n", range[540]);
		}
	}
	else {
		fprintf(stderr, "CRC error.\n");
	}

	// done parsing, find next header
	header_state = 0;
	return 1;
}


int s300_connect(s300_laser_t* sick, char* filename, int baudrate) {

  if(carmen_serial_connect(&(sick->fd),filename)<0){
    fprintf(stderr,"error\n");
    fprintf(stderr,"  carmen_serial_connect failed!\n");
    return -1;
  }
  
  fprintf(stderr,"connected\n");
  serial_configure(sick->fd, baudrate, "8N1");
  carmen_serial_set_low_latency(sick->fd);  
  //fprintf(stderr,"Serial Configured\n");
  
  serial_ClearInputBuffer(sick->fd);

  return 1;
}
