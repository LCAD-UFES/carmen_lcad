#ifndef __XSENSCORE_H
#define __XSENSCORE_H

#include <cmtdef.h>
#include <xsens_time.h>
#include <xsens_list.h>
#include <cmtscan.h>
#include <cmt3.h>

typedef struct{
    double x, y, z;
} xsens_axis;

typedef struct{
    double pitch, roll, yaw;
} xsens_euler;

typedef struct{
    double data[4];
} xsens_quat;

typedef struct{
    double data[3][3];
} xsens_matrix;

typedef struct {

    xsens_axis acc;    
    xsens_axis gyr;
    xsens_axis mag;

    xsens_euler eulerData;
    xsens_quat quatData;
    xsens_matrix matrixData;
  
    double temp;
    unsigned short count;
  
    unsigned long long timestamp;
  
} xsens_global;  

/*
int doHardwareScan(xsens::Cmt3 &, CmtDeviceId []);
int doHardwareScan_Manual(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[], char* portName);
void doMtSettings(xsens::Cmt3 &, CmtOutputMode &, CmtOutputSettings &, CmtDeviceId []);
*/

void initializeXsens(xsens::Cmt3&, CmtOutputMode&, CmtOutputSettings&, unsigned long&, char* portName = NULL);
void getDataFromXsens(xsens::Cmt3&, xsens::Packet* packet, CmtOutputMode& mode, CmtOutputSettings& settings, xsens_global& data);
void shutDownXsens(xsens::Cmt3&);

#endif
