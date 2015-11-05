#include "scorbot.h"
#include "buffered_serial.h"

#define READ_ENCODER_VALUE       10
#define READ_MICROSWITCHES       11
#define READ_PWM_VALUE           12
#define READ_TRAJECTORY          13
#define READ_CONTROL_PARAMS      15
#define SET_DESTINATION          20
#define RESET_ENCODER            21
#define READ_ALL_ENCODERS        22
#define READ_ALL_PWMS            23
#define SET_CONTROL_PARAMS       25
#define ENABLE_MOTORS            30
#define DISABLE_MOTORS           31

#define LOOP_STATUS_TEXT         'l'
#define DEBUG                    'd'

#define FAILURE                  254
#define SUCCESS                  255

BufferedSerial pc(USBTX, USBRX, 128);



int main()
{
  Scorbot arm;

  //create and set the serial baud rate to the PC
  pc.baud(115200);
  
  uint8_t command, motor, length, index;
  uint8_t * param_buffer;
  long position, duration;
  
  while (1)
  {
    command = pc.getc();
    
    switch (command)
    {
      case READ_ENCODER_VALUE:
        motor = pc.getc();
        if (motor < NUM_MOTORS)
        {
          pc.writeLong(arm.readEncoder(motor));
          pc.putc(SUCCESS);
        }
        else
        {
          pc.writeLong(0);
          pc.putc(FAILURE);  
        }
        break;
      case READ_MICROSWITCHES:
        pc.putc(arm.readMicroswitches());
        pc.putc(SUCCESS);
        break;
      case READ_PWM_VALUE:
        motor = pc.getc();
        if (motor < NUM_MOTORS)
        {
          pc.writeLong((long)(arm.readPWMDuty(motor) * 100000.0));
          pc.putc(SUCCESS);
        }
        else
        {
          pc.writeLong(0);
          pc.putc(FAILURE);
        }
        break;
      case READ_TRAJECTORY:
        motor = pc.getc();
        if (motor < NUM_MOTORS)
        {
          pc.writeLong(arm.readTargetPosition(motor));
          pc.writeLong(arm.readTargetVelocity(motor));
          pc.putc(SUCCESS);
        }
        else
          pc.putc(FAILURE);
          
        break;
      case READ_CONTROL_PARAMS:
        motor = pc.getc();
        if (motor < NUM_MOTORS)
        {
          param_buffer = arm.readControlParameters(motor, length);
          pc.putc(length);
          for (index = 0; index < length; index++)
            pc.putc(param_buffer[index]);
          delete[] param_buffer;
          pc.putc(SUCCESS);
        }
        else
          pc.putc(FAILURE);
        break;
      /* SET_DESTINATION */
      case SET_DESTINATION:
        motor = pc.getc();
        position = pc.readLong();
        duration = pc.readLong();
        if (motor < NUM_MOTORS)
        {
          arm.setDestination(motor, position, duration);
          pc.putc(SUCCESS);
        }
        else
          pc.putc(FAILURE);
        break;
      /* SET_CONTROL_PARAMS */
      case RESET_ENCODER:
        arm.resetEncoders();
        pc.putc(SUCCESS);
        break;
      /* READ_ALL_ENCODERS */
      case READ_ALL_ENCODERS:
        pc.putc(READ_ALL_ENCODERS);
        pc.putc(4*NUM_MOTORS);
        for(index=0; index<NUM_MOTORS; index++)
            pc.writeLong(arm.readEncoder(index));
        pc.putc(255);
        break;
      case READ_ALL_PWMS:
        pc.putc(READ_ALL_PWMS);
        pc.putc(4*NUM_MOTORS);
        for (index = 0; index < NUM_MOTORS; index++)
          pc.writeLong((long)(arm.readPWMDuty(index) * 100000.0));
        pc.putc(255);
        break;
      /* SET_CONTROL_PARAMS */
      case SET_CONTROL_PARAMS:
        motor = pc.getc();
        length = pc.getc();
        param_buffer = new uint8_t[length];
        pc.readBytes(param_buffer, length);
        if (motor < 7 && arm.setControlParameters(motor, length, param_buffer))
          pc.putc(SUCCESS);
        else
          pc.putc(FAILURE);
        break;
      /* ENABLE_MOTORS */
      case ENABLE_MOTORS:
        arm.enableMotors();
        pc.putc(SUCCESS);
        break;
      /* DISABLE_MOTORS */
      case DISABLE_MOTORS:
        arm.disableMotors();
        pc.putc(SUCCESS);
        break;
      /* LOOP_STATUS_TEXT */
      case LOOP_STATUS_TEXT:
        while (!pc.readable())
        {
          pc.printf("--------------------------------------------------------------------------------\n");
          pc.printf("                                 Scorbot Status                                 \n");
          pc.printf("--------------------------------------------------------------------------------\n");
          pc.printf("\tEncoder 1 = %d\t\t\tEncoder 2 = %d\n", arm.readEncoder(0), arm.readEncoder(1));
          pc.printf("\tEncoder 3 = %d\t\t\tEncoder 4 = %d\n", arm.readEncoder(2), arm.readEncoder(3));
          pc.printf("\tEncoder 5 = %d\t\t\tEncoder 6 = %d\n", arm.readEncoder(4), arm.readEncoder(5));
          pc.printf("\tEncoder 7 = %d\n", arm.readEncoder(6));
          pc.printf("--------------------------------------------------------------------------------\n");
          pc.printf("   MS 1 = %u     MS 2 = %u     MS 3 = %u     MS 4 = %u     MS 5 = %u     MS 7 = %u\n",
            arm.readMicroswitch(0), arm.readMicroswitch(1), arm.readMicroswitch(2), 
            arm.readMicroswitch(3), arm.readMicroswitch(4), arm.readMicroswitch(6));
          pc.printf("--------------------------------------------------------------------------------\n");
          pc.printf("                                                                                \n");
          pc.printf("--------------------------------------------------------------------------------\n");
          wait_ms(500);
        }
        break;
      case DEBUG:
        motor =0;
        while (!pc.readable())
        {
           pc.printf("hi = %u\n", motor);
           motor++;
           wait_ms(10);
        }
        break;
    }
  }
}
