
#ifndef TOOL_H
#define TOOL_H

#include <stdio.h>
#include "../ros_inc/macro.h"
#include "../ros_inc/serial.h"
#include "../ros_inc/protocol.h"

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    u16 data;
    u16 dataY;
    u16 dataZ;
    u16 data3;
    u16 data4;
    u8 data5;
    u8 data6;
}Data_TEMP_HandleType;
typedef struct
{
    u16 dataID;
    u8 dataLen;
    u16 cPower;
    u8 shoot;
    u8 holding;
    
}
Data_Robo_Status;
typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve0;
    short accX;
    short accY;
    short accZ;
    short gyroX;
    short gyroY;
    short gyroZ;
    short magX;
    short magY;
    short magZ;
    u16 reserve1;

}Data_RAW_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    float distanceX; //X轴运动距离
    float distanceY; //Y轴运动距离
    float distanceZ; //Z轴运动距离

}Data_DISTANCE_HandleType;
typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float accX;
    float accY;
    float accZ;
}Data_CAL_ACC_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float gyroX;
    float gyroY;
    float gyroZ;
}Data_CAL_GYRO_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float magX;
    float magY;
    float magZ;
}Data_CAL_MAG_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float accX;
    float accY;
    float accZ;
}Data_KAL_ACC_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float gyroX;
    float gyroY;
    float gyroZ;
}Data_KAL_GYRO_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float magX;
    float magY;
    float magZ;
}Data_KAL_MAG_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    union
    {
        unsigned int uint_x;
        float        float_x;
    } Q0;
    union
    {
        unsigned int uint_x;
        float        float_x;
    } Q1;
    union
    {
        unsigned int uint_x;
        float        float_x;
    } Q2;
    union
    {
        unsigned int uint_x;
        float        float_x;
    } Q3;
}Data_Quaternion_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float roll;
    float pitch;
    float yaw;
}Data_Euler_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
    float g;
    float h;
    float i;
}Data_RoMax_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    u32 packerCounter;
}Data_PacketCounter_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float DT;
}Data_DT_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float ErrorAll;
}Data_MAG_EA_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 percent;

}Data_MAG_Strength_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u16 OS_Time_us;
    u32 OS_Time_ms;
}Data_OS_Time_ms_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    u32 status;
}Data_Status_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    double longlatX;
    double longlatY;
}Data_Position_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    float longlatZ;
}Data_Altitude_HandleType;

typedef struct
{
    u16 dataID;
    u8 dataLen;
    u8 reserve;
    u32 ms;
    u16 year;
    u8 month;
    u8 day;
    u8 hour;
    u8 minute;
    u8 second;
    u8 flag;
}Data_UTCTime_HandleType;





typedef struct
{
    Data_TEMP_HandleType temperature;//上一次上位机fa 送的控制信息
    Data_RAW_HandleType  accRawData;//车体实际速度信息
    Data_RAW_HandleType  gyroRawData;//轮速信息
    Data_RAW_HandleType  magRawData;//持球杆高度信息，是否持球
    Data_CAL_ACC_HandleType accCal;
    Data_CAL_GYRO_HandleType gyroCal;
    Data_CAL_MAG_HandleType magCal;
    Data_KAL_ACC_HandleType accKal;
    Data_KAL_GYRO_HandleType gyroKal;
    Data_KAL_MAG_HandleType magKal;
    Data_Quaternion_HandleType quat;
    Data_Euler_HandleType euler;
    Data_RoMax_HandleType romatix;
    Data_PacketCounter_HandleType packetCounter;
    Data_OS_Time_ms_HandleType tick;
    Data_Status_HandleType status;
    Data_UTCTime_HandleType UTC_time;
    //Data_CpuUsage_HandleTypeDef CpuUsage;
    Data_KAL_ACC_HandleType accLinear;
    Data_DT_HandleType dt;
    Data_MAG_Strength_HandleType magStrength;
    Data_MAG_EA_HandleType magEA;
    Data_Position_HandleType position;
    Data_Altitude_HandleType altitude;
    Data_DISTANCE_HandleType distance;
    Data_Robo_Status status2;
}Data;

extern int Align(unsigned char nFD);
extern int IMU_Align(unsigned char nFD);
extern int GetFrame(unsigned char nFD, unsigned char * tmpBuf, int frameLen);
extern void ParserDataPacket(Data *DataHandle, u8 *pBuffer, u16 dataLen, FILE *fpLog);
extern void IMU_ParserDataPacket(Data *DataHandle, u8 *pBuffer, u16 dataLen, FILE *fpLog);
extern bool ValidFrame(unsigned char * tmpBuf,int frameLen);
extern bool IMU_ValidFrame(unsigned char * tmpBuf,int frameLen);
extern int  FillFrameHead(unsigned char * tmpBuf);
extern int  IMU_FillFrameHead(unsigned char * tmpBuf);
#endif //_TOOL_H
