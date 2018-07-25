
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include "linux/i2c-dev.h"
#include "LSM9DS0.h"
#include "LSM9DS1.h"

#ifdef __cplusplus
extern "C"
{
#endif


void readBlock(uint8_t command, uint8_t size, uint8_t *data);
void selectDevice(int file, int addr);
void readACC(int a[]);
void readMAG(int m[]);
void readGYR(int g[]);
void writeAccReg(uint8_t reg, uint8_t value);
void writeMagReg(uint8_t reg, uint8_t value);
void writeGyrReg(uint8_t reg, uint8_t value);
void detectIMU();
void enableIMU();

#ifdef __cplusplus
}
#endif