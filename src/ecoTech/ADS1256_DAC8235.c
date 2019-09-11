#include "ADS1256_DAC8235.h"

UBYTE ScanMode = 0;

/******************************************************************************
function:   Module reset
parameter:
Info:
******************************************************************************/
static void ADS1256_reset(void)
{
    DEV_Digital_Write(DEV_RST_PIN, 1);
    DEV_Delay_ms(200);
    DEV_Digital_Write(DEV_RST_PIN, 0);
    DEV_Delay_ms(200);
    DEV_Digital_Write(DEV_RST_PIN, 1);
}

/******************************************************************************
function:   send command
parameter:
        Cmd: command
Info:
******************************************************************************/
static void ADS1256_WriteCmd(UBYTE Cmd)
{
    DEV_Digital_Write(DEV_CS_PIN, 0);
    DEV_SPI_WriteByte(Cmd);
    DEV_Digital_Write(DEV_CS_PIN, 1);
}

/******************************************************************************
function:   Write a data to the destination register
parameter:
        Reg : Target register
        data: Written data
Info:
******************************************************************************/
static void ADS1256_WriteReg(UBYTE Reg, UBYTE data)
{
    DEV_Digital_Write(DEV_CS_PIN, 0);
    DEV_SPI_WriteByte(CMD_WREG | Reg);
    DEV_SPI_WriteByte(0x00);
    DEV_SPI_WriteByte(data);
    DEV_Digital_Write(DEV_CS_PIN, 1);
}

/******************************************************************************
function:   Read a data from the destination register
parameter:
        Reg : Target register
Info:
    Return the read data
******************************************************************************/
static UBYTE ADS1256_Read_data(UBYTE Reg)
{
    UBYTE temp = 0;
    DEV_Digital_Write(DEV_CS_PIN, 0);
    DEV_SPI_WriteByte(CMD_RREG | Reg);
    DEV_SPI_WriteByte(0x00);
    DEV_Delay_ms(1);
    temp = DEV_SPI_ReadByte();
    DEV_Digital_Write(DEV_CS_PIN, 1);
    return temp;
}

/******************************************************************************
function:   Waiting for a busy end
parameter:
Info:
    Timeout indicates that the operation is not working properly.
******************************************************************************/
static void ADS1256_WaitDRDY(void)
{
    UDOUBLE i = 0;
    for(i=0;i<4000000;i++){
        if(DEV_Digital_Read(DEV_DRDY_PIN) == 0)
            break;
    }
    if(i >= 4000000){
       printf("Time Out ...\r\n");
    }
}

/******************************************************************************
function:  Read device ID
parameter:
Info:
******************************************************************************/
UBYTE ADS1256_ReadChipID(void)
{
    UBYTE id;
    ADS1256_WaitDRDY();
    id = ADS1256_Read_data(REG_STATUS);
    return id>>4;
}

/******************************************************************************
function:  Configure ADC gain and sampling speed
parameter:
    gain : Enumeration type gain
    drate: Enumeration type sampling speed
Info:
******************************************************************************/
void ADS1256_ConfigADC(ADS1256_GAIN gain, ADS1256_DRATE drate)
{
    ADS1256_WaitDRDY();
    UBYTE buf[4] = {0,0,0,0};
    buf[0] = (0<<3) | (1<<2) | (1<<1);
    buf[1] = 0x08;
    buf[2] = (0<<5) | (0<<3) | (gain<<0);
    buf[3] = ADS1256_DRATE_E[drate];
    DEV_Digital_Write(DEV_CS_PIN, 0);
    DEV_SPI_WriteByte(CMD_WREG | 0);
    DEV_SPI_WriteByte(0x03);

    DEV_SPI_WriteByte(buf[0]);
    DEV_SPI_WriteByte(buf[1]);
    DEV_SPI_WriteByte(buf[2]);
    DEV_SPI_WriteByte(buf[3]);
    DEV_Digital_Write(DEV_CS_PIN, 1);
    DEV_Delay_ms(1);
}

/******************************************************************************
function:  Set the channel to be read
parameter:
    Channal : Set channel number
Info:
******************************************************************************/
static void ADS1256_SetChannal(UBYTE Channal)
{
    if(Channal > 7){
        return ;
    }
    ADS1256_WriteReg(REG_MUX, (Channal<<4) | (1<<3));
}

void ADS1256_SetDiffChannal(UBYTE Channal)
{
    if (Channal == 0){
        ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	//DiffChannal  AIN0-AIN1
    }
    else if(Channal == 1){
        ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	//DiffChannal   AIN2-AIN3
    }
    else if(Channal == 2){
        ADS1256_WriteReg(REG_MUX, (4 << 4) | 5); 	//DiffChannal    AIN4-AIN5
    }
    else if(Channal == 3){
        ADS1256_WriteReg(REG_MUX, (6 << 4) | 7); 	//DiffChannal   AIN6-AIN7
    }
}

/******************************************************************************
function:  Setting mode
parameter:
    Mode : 0 Single-ended input
           1 channel1 Differential input
Info:
******************************************************************************/
void ADS1256_SetMode(UBYTE Mode)
{
    if(Mode == 0){
        ScanMode = 0;
    }
    else{
        ScanMode = 1;
    }
}

/******************************************************************************
function:  Device initialization
parameter:
Info:
******************************************************************************/
UBYTE ADS1256_init(void)
{
    ADS1256_reset();
    if(ADS1256_ReadChipID() == 3){
        printf("ID Read success \r\n");
    }
    else{
        printf("ID Read failed \r\n");
        return 1;
    }
    ADS1256_ConfigADC(ADS1256_GAIN_1, ADS1256_30000SPS);


    // printf("ADD 0x02  = 0x%x\r\n",ADS1256_Read_data(0x02));
    // printf("ADD 0x03  = 0x%x\r\n",ADS1256_Read_data(0x03));
    return 0;
}

/******************************************************************************
function:  Read ADC data
parameter:
Info:
******************************************************************************/
static UDOUBLE ADS1256_Read_ADC_Data(void)
{
    UDOUBLE read = 0;
    UBYTE buf[3] = {0,0,0};

    ADS1256_WaitDRDY();
    DEV_Delay_ms(1);
    DEV_Digital_Write(DEV_CS_PIN, 0);
    DEV_SPI_WriteByte(CMD_RDATA);
    DEV_Delay_ms(1);
    buf[0] = DEV_SPI_ReadByte();
    buf[1] = DEV_SPI_ReadByte();
    buf[2] = DEV_SPI_ReadByte();
    DEV_Digital_Write(DEV_CS_PIN, 1);
    read = ((UDOUBLE)buf[0] << 16) & 0x00FF0000;
    read |= ((UDOUBLE)buf[1] << 8);  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
    read |= buf[2];
    //printf("%d  %d  %d \r\n",buf[0],buf[1],buf[2]);
    if (read & 0x800000)
        read &= 0xFF000000;
    return read;
}

/******************************************************************************
function:  Read ADC specified channel data
parameter:
    Channel: Channel number
Info:
******************************************************************************/
UDOUBLE ADS1256_GetChannalValue(UBYTE Channel)
{
    UDOUBLE Value = 0;
    while(DEV_Digital_Read(DEV_DRDY_PIN) == 1);
    if(ScanMode == 0){// 0  Single-ended input  8 channel1 Differential input  4 channe
        if(Channel>=8){
            return 0;
        }
        ADS1256_SetChannal(Channel);
        ADS1256_WriteCmd(CMD_SYNC);
        ADS1256_WriteCmd(CMD_WAKEUP);
        Value = ADS1256_Read_ADC_Data();
    }
    else{
        if(Channel>=4){
            return 0;
        }
        ADS1256_SetDiffChannal(Channel);
        ADS1256_WriteCmd(CMD_SYNC);
        ADS1256_WriteCmd(CMD_WAKEUP);
        Value = ADS1256_Read_ADC_Data();
    }
    return Value;
}

/******************************************************************************
function:  Read data from all channels
parameter:
    ADC_Value : ADC Value
Info:
******************************************************************************/
void ADS1256_GetAll(UDOUBLE *ADC_Value)
{
    UBYTE i;
    for(i = 0; i<8; i++){
        ADC_Value[i] = ADS1256_GetChannalValue(i);
    }

}

void Write_DAC8532(UBYTE Channel, UWORD Data)
{
    DEV_Digital_Write(DEV_CS1_PIN, 1);
    DEV_Digital_Write(DEV_CS1_PIN, 0);
    DEV_SPI_WriteByte(Channel);
    DEV_SPI_WriteByte((Data>>8));
    DEV_SPI_WriteByte((Data&0xff));
    DEV_Digital_Write(DEV_CS1_PIN, 1);
}

void DAC8532_Out_Voltage(UBYTE Channel, float Voltage)
{
    UWORD temp = 0;
    if((Voltage <= DAC_VREF) && (Voltage >= 0)){
        temp = (UWORD)(Voltage * DAC_Value_MAX / DAC_VREF);
        Write_DAC8532(Channel, temp);
    }
}

/******************************************************************************
function:	Initialization pin
parameter:
Info:
******************************************************************************/
static void DEV_GPIOConfig(void)
{
    //output
	bcm2835_gpio_fsel(DEV_RST_PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(DEV_CS_PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(DEV_CS1_PIN, BCM2835_GPIO_FSEL_OUTP);

    //intput
	bcm2835_gpio_fsel(DEV_DRDY_PIN,BCM2835_GPIO_FSEL_INPT);

}

/******************************************************************************
function:	Module Initialize, the BCM2835 library and initialize the pins, SPI protocol
parameter:
Info:
******************************************************************************/
UBYTE DEV_ModuleInit(void)
{
    if(!bcm2835_init()) {
        printf("bcm2835 init failed  !!! \r\n");
        return 1;
    }
//    else {
//        printf("bcm2835 init success !!! \r\n");
//    }

	DEV_GPIOConfig();
	printf ("1!\n");

	bcm2835_spi_begin();                                         //Start spi interface, set spi pin for the reuse function
    printf ("2!\n");
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);     //High first transmission
    printf ("3!\n");
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                  //spi mode 0
    printf ("4!\n");
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);   //Frequency
    return 0;
}

/******************************************************************************
function:	Module exits, closes SPI and BCM2835 library
parameter:
Info:
******************************************************************************/
void DEV_ModuleExit(void)
{
    bcm2835_spi_end();
    bcm2835_close();
}
