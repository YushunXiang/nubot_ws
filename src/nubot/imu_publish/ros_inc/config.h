#ifndef CONFIG_H
#define CONFIG_H

#define DATAPACKET_CNT_ROS 6
#define DATAPACKET_CNT     24


extern unsigned char  InitConfig(const char* fileName);
extern unsigned char  IMU_InitConfig(const char* fileName);
#endif //CONFIG_H
