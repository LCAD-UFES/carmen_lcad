#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>


#ifdef __cplusplus
extern "C" {
#endif

#ifndef DYNAMIXEL_LIB_H
#define DYNAMIXEL_LIB_H
////////////////// Start dxl_hal.c /////////////////////////////
int dxl_hal_open(int deviceIndex, float baudrate);
void dxl_hal_close();
int dxl_hal_set_baud( float baudrate );
void dxl_hal_clear();
int dxl_hal_tx( unsigned char *pPacket, int numPacket );
int dxl_hal_rx( unsigned char *pPacket, int numPacket );
void dxl_hal_set_timeout( int NumRcvByte );
int dxl_hal_timeout();

static int	gSocket_fd	= -1;
static long	glStartTime	= 0;
static float	gfRcvWaitTime	= 0.0f;
static float	gfByteTransTime	= 0.0f;
static char	gDeviceName[20];

inline int dxl_hal_open(int deviceIndex, float baudrate)
{
	struct termios newtio;
	struct serial_struct serinfo;
	char dev_name[100] = {0, };

	sprintf(dev_name, "/dev/ttyUSB%d", deviceIndex);

	strcpy(gDeviceName, dev_name);
	memset(&newtio, 0, sizeof(newtio));
	dxl_hal_close();
	
	if((gSocket_fd = open(gDeviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
		fprintf(stderr, "device open error: %s\n", dev_name);
		goto DXL_HAL_OPEN_ERROR;
	}

	newtio.c_cflag		= B38400|CS8|CLOCAL|CREAD;
	newtio.c_iflag		= IGNPAR;
	newtio.c_oflag		= 0;
	newtio.c_lflag		= 0;
	newtio.c_cc[VTIME]	= 0;	// time-out value (TIME * 0.1 Second) 0 : disable
	newtio.c_cc[VMIN]	= 0;	// MIN is the minimum number of characters to read the return

	tcflush(gSocket_fd, TCIFLUSH);
	tcsetattr(gSocket_fd, TCSANOW, &newtio);
	
	if(gSocket_fd == -1)
		return 0;
	
	if(ioctl(gSocket_fd, TIOCGSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot get serial info\n");
		return 0;
	}
	
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / baudrate;
	
	if(ioctl(gSocket_fd, TIOCSSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot set serial info\n");
		return 0;
	}
	
	dxl_hal_close();
	
	gfByteTransTime = (float)((1000.0f / baudrate) * 12.0f);
	
	strcpy(gDeviceName, dev_name);
	memset(&newtio, 0, sizeof(newtio));
	dxl_hal_close();
	
	if((gSocket_fd = open(gDeviceName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
		fprintf(stderr, "device open error: %s\n", dev_name);
		goto DXL_HAL_OPEN_ERROR;
	}

	newtio.c_cflag		= B38400|CS8|CLOCAL|CREAD;
	newtio.c_iflag		= IGNPAR;
	newtio.c_oflag		= 0;
	newtio.c_lflag		= 0;
	newtio.c_cc[VTIME]	= 0;	// time-out value (TIME * 0.1 Second) 0 : disable
	newtio.c_cc[VMIN]	= 0;	// MIN is the minimum number of characters to read the return

	tcflush(gSocket_fd, TCIFLUSH);
	tcsetattr(gSocket_fd, TCSANOW, &newtio);
	
	return 1;

DXL_HAL_OPEN_ERROR:
	dxl_hal_close();
	return 0;
}

inline void dxl_hal_close()
{
	if(gSocket_fd != -1)
		close(gSocket_fd);
	gSocket_fd = -1;
}

inline int dxl_hal_set_baud( float baudrate )
{
	struct serial_struct serinfo;
	
	if(gSocket_fd == -1)
		return 0;
	
	if(ioctl(gSocket_fd, TIOCGSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot get serial info\n");
		return 0;
	}
	
	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / baudrate;
	
	if(ioctl(gSocket_fd, TIOCSSERIAL, &serinfo) < 0) {
		fprintf(stderr, "Cannot set serial info\n");
		return 0;
	}
	
	//dxl_hal_close();
	//dxl_hal_open(gDeviceName, baudrate);
	
	gfByteTransTime = (float)((1000.0f / baudrate) * 12.0f);
	return 1;
}

inline void dxl_hal_clear(void)
{
	tcflush(gSocket_fd, TCIFLUSH);
}

inline int dxl_hal_tx( unsigned char *pPacket, int numPacket )
{
	return write(gSocket_fd, pPacket, numPacket);
}

inline int dxl_hal_rx( unsigned char *pPacket, int numPacket )
{
	memset(pPacket, 0, numPacket);
	return read(gSocket_fd, pPacket, numPacket);
}

static inline long myclock()
{
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

inline void dxl_hal_set_timeout( int NumRcvByte )
{
	glStartTime = myclock();
	gfRcvWaitTime = (float)(gfByteTransTime*(float)NumRcvByte + 5.0f);
}

inline int dxl_hal_timeout(void)
{
	long time;
	
	time = myclock() - glStartTime;
	
	if(time > gfRcvWaitTime)
		return 1;
	else if(time < 0)
		glStartTime = myclock();
		
	return 0;
}

////////////////// End dxl_hal.c /////////////////////////////

///////////// device control methods ////////////////////////
int dxl_initialize(int deviceIndex, int baudnum );
void dxl_terminate();


///////////// set/get packet methods //////////////////////////
#define MAXNUM_TXPARAM		(150)
#define MAXNUM_RXPARAM		(60)

void dxl_set_txpacket_id(int id);
#define BROADCAST_ID		(254)

void dxl_set_txpacket_instruction(int instruction);
#define INST_PING			(1)
#define INST_READ			(2)
#define INST_WRITE			(3)
#define INST_REG_WRITE		(4)
#define INST_ACTION			(5)
#define INST_RESET			(6)
#define INST_SYNC_WRITE		(131)

void dxl_set_txpacket_parameter(int index, int value);
void dxl_set_txpacket_length(int length);

int dxl_get_rxpacket_error(int errbit);
#define ERRBIT_VOLTAGE		(1)
#define ERRBIT_ANGLE		(2)
#define ERRBIT_OVERHEAT		(4)
#define ERRBIT_RANGE		(8)
#define ERRBIT_CHECKSUM		(16)
#define ERRBIT_OVERLOAD		(32)
#define ERRBIT_INSTRUCTION	(64)

int dxl_get_rxpacket_length(void);
int dxl_get_rxpacket_parameter(int index);


// utility for value
int dxl_makeword(int lowbyte, int highbyte);
int dxl_get_lowbyte(int word);
int dxl_get_highbyte(int word);


////////// packet communication methods ///////////////////////
void dxl_tx_packet(void);
void dxl_rx_packet(void);
void dxl_txrx_packet(void);

int dxl_get_result(void);
#define	COMM_TXSUCCESS		(0)
#define COMM_RXSUCCESS		(1)
#define COMM_TXFAIL		(2)
#define COMM_RXFAIL		(3)
#define COMM_TXERROR		(4)
#define COMM_RXWAITING		(5)
#define COMM_RXTIMEOUT		(6)
#define COMM_RXCORRUPT		(7)


//////////// high communication methods ///////////////////////
void dxl_ping(int id);
int dxl_read_byte(int id, int address);
void dxl_write_byte(int id, int address, int value);
int dxl_read_word(int id, int address);
void dxl_write_word(int id, int address, int value);


////////////////// dynamixel.c ////////////////////////////////
#define ID					(2)
#define LENGTH				(3)
#define INSTRUCTION			(4)
#define ERRBIT				(4)
#define PARAMETER			(5)
#define DEFAULT_BAUDNUMBER	(1)


static unsigned char gbInstructionPacket[MAXNUM_TXPARAM+10] = {0};
static unsigned char gbStatusPacket[MAXNUM_RXPARAM+10] = {0};
static unsigned char gbRxPacketLength = 0;
static unsigned char gbRxGetLength = 0;
static int gbCommStatus = COMM_RXSUCCESS;
static int giBusUsing = 0;


inline int dxl_initialize(int deviceIndex, int baudnum )
{
	float baudrate;	
	baudrate = 2000000.0f / (float)(baudnum + 1);
	if( dxl_hal_open(deviceIndex, baudrate) == 0 )
		return 0;

	gbCommStatus = COMM_RXSUCCESS;
	giBusUsing = 0;
	return 1;
}

inline void dxl_terminate(void)
{
	dxl_hal_close();
}

inline void dxl_tx_packet(void)
{
	unsigned char i;
	unsigned char TxNumByte, RealTxNumByte;
	unsigned char checksum = 0;

	if( giBusUsing == 1 )
		return;
	
	giBusUsing = 1;

	if( gbInstructionPacket[LENGTH] > (MAXNUM_TXPARAM+2) )
	{
		gbCommStatus = COMM_TXERROR;
		giBusUsing = 0;
		return;
	}
	
	if( gbInstructionPacket[INSTRUCTION] != INST_PING
		&& gbInstructionPacket[INSTRUCTION] != INST_READ
		&& gbInstructionPacket[INSTRUCTION] != INST_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_REG_WRITE
		&& gbInstructionPacket[INSTRUCTION] != INST_ACTION
		&& gbInstructionPacket[INSTRUCTION] != INST_RESET
		&& gbInstructionPacket[INSTRUCTION] != INST_SYNC_WRITE )
	{
		gbCommStatus = COMM_TXERROR;
		giBusUsing = 0;
		return;
	}
	
	gbInstructionPacket[0] = 0xff;
	gbInstructionPacket[1] = 0xff;
	for( i=0; i<(gbInstructionPacket[LENGTH]+1); i++ )
		checksum += gbInstructionPacket[i+2];
	gbInstructionPacket[gbInstructionPacket[LENGTH]+3] = ~checksum;
	
	if( gbCommStatus == COMM_RXTIMEOUT || gbCommStatus == COMM_RXCORRUPT )
		dxl_hal_clear();

	TxNumByte = gbInstructionPacket[LENGTH] + 4;
	RealTxNumByte = dxl_hal_tx( (unsigned char*)gbInstructionPacket, TxNumByte );

	if( TxNumByte != RealTxNumByte )
	{
		gbCommStatus = COMM_TXFAIL;
		giBusUsing = 0;
		return;
	}

	if( gbInstructionPacket[INSTRUCTION] == INST_READ )
		dxl_hal_set_timeout( gbInstructionPacket[PARAMETER+1] + 6 );
	else
		dxl_hal_set_timeout( 6 );

	gbCommStatus = COMM_TXSUCCESS;
}

inline void dxl_rx_packet(void)
{
	unsigned char i, j, nRead;
	unsigned char checksum = 0;

	if( giBusUsing == 0 )
		return;

	if( gbInstructionPacket[ID] == BROADCAST_ID )
	{
		gbCommStatus = COMM_RXSUCCESS;
		giBusUsing = 0;
		return;
	}
	
	if( gbCommStatus == COMM_TXSUCCESS )
	{
		gbRxGetLength = 0;
		gbRxPacketLength = 6;
	}
	
	nRead = dxl_hal_rx( (unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength );
	gbRxGetLength += nRead;
	if( gbRxGetLength < gbRxPacketLength )
	{
		if( dxl_hal_timeout() == 1 )
		{
			if(gbRxGetLength == 0)
				gbCommStatus = COMM_RXTIMEOUT;
			else
				gbCommStatus = COMM_RXCORRUPT;
			giBusUsing = 0;
			return;
		}
	}
	
	// Find packet header
	for( i=0; i<(gbRxGetLength-1); i++ )
	{
		if( gbStatusPacket[i] == 0xff && gbStatusPacket[i+1] == 0xff )
		{
			break;
		}
		else if( i == gbRxGetLength-2 && gbStatusPacket[gbRxGetLength-1] == 0xff )
		{
			break;
		}
	}	
	if( i > 0 )
	{
		for( j=0; j<(gbRxGetLength-i); j++ )
			gbStatusPacket[j] = gbStatusPacket[j + i];
			
		gbRxGetLength -= i;		
	}

	if( gbRxGetLength < gbRxPacketLength )
	{
		gbCommStatus = COMM_RXWAITING;
		return;
	}

	// Check id pairing
	if( gbInstructionPacket[ID] != gbStatusPacket[ID])
	{
		gbCommStatus = COMM_RXCORRUPT;
		giBusUsing = 0;
		return;
	}
	
	gbRxPacketLength = gbStatusPacket[LENGTH] + 4;
	if( gbRxGetLength < gbRxPacketLength )
	{
		nRead = dxl_hal_rx( (unsigned char*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength - gbRxGetLength );
		gbRxGetLength += nRead;
		if( gbRxGetLength < gbRxPacketLength )
		{
			gbCommStatus = COMM_RXWAITING;
			return;
		}
	}

	// Check checksum
	for( i=0; i<(gbStatusPacket[LENGTH]+1); i++ )
		checksum += gbStatusPacket[i+2];
	checksum = ~checksum;

	if( gbStatusPacket[gbStatusPacket[LENGTH]+3] != checksum )
	{
		gbCommStatus = COMM_RXCORRUPT;
		giBusUsing = 0;
		return;
	}
	
	gbCommStatus = COMM_RXSUCCESS;
	giBusUsing = 0;
}

inline void dxl_txrx_packet(void)
{
	dxl_tx_packet();

	if( gbCommStatus != COMM_TXSUCCESS )
		return;	
	
	do{
		dxl_rx_packet();		
	}while( gbCommStatus == COMM_RXWAITING );	
}

inline int dxl_get_result(void)
{
	return gbCommStatus;
}

inline void dxl_set_txpacket_id( int id )
{
	gbInstructionPacket[ID] = (unsigned char)id;
}

inline void dxl_set_txpacket_instruction( int instruction )
{
	gbInstructionPacket[INSTRUCTION] = (unsigned char)instruction;
}

inline void dxl_set_txpacket_parameter( int index, int value )
{
	gbInstructionPacket[PARAMETER+index] = (unsigned char)value;
}

inline void dxl_set_txpacket_length( int length )
{
	gbInstructionPacket[LENGTH] = (unsigned char)length;
}

inline int dxl_get_rxpacket_error( int errbit )
{
	if( gbStatusPacket[ERRBIT] & (unsigned char)errbit )
		return 1;

	return 0;
}

inline int dxl_get_rxpacket_length(void)
{
	return (int)gbStatusPacket[LENGTH];
}

inline int dxl_get_rxpacket_parameter( int index )
{
	return (int)gbStatusPacket[PARAMETER+index];
}

inline int dxl_makeword( int lowbyte, int highbyte )
{
	unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}

inline int dxl_get_lowbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff;
	return (int)temp;
}

inline int dxl_get_highbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff00;
	temp = temp >> 8;
	return (int)temp;
}

inline void dxl_ping( int id )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_PING;
	gbInstructionPacket[LENGTH] = 2;
	
	dxl_txrx_packet();
}

inline int dxl_read_byte( int id, int address )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 1;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();

	return (int)gbStatusPacket[PARAMETER];
}

inline void dxl_write_byte( int id, int address, int value )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)value;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();
}

inline int dxl_read_word( int id, int address )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 2;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_txrx_packet();

	return dxl_makeword((int)gbStatusPacket[PARAMETER], (int)gbStatusPacket[PARAMETER+1]);
}

inline void dxl_write_word( int id, int address, int value )
{
	while(giBusUsing);

	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)dxl_get_lowbyte(value);
	gbInstructionPacket[PARAMETER+2] = (unsigned char)dxl_get_highbyte(value);
	gbInstructionPacket[LENGTH] = 5;
	
	dxl_txrx_packet();
}

////////////////// End dynamixel.c ////////////////////////////////








#endif





#ifdef __cplusplus
}
#endif

