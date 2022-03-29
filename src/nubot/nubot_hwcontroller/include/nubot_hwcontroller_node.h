#ifndef BALL_HANDLE_H
#define BALL_HANDLE_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <realtime_tools/realtime_publisher.h>

//#include "nubot_hwcontroller/DebugInfo.h"
#include "nubot_common/VelCmd.h"
#include "nubot_common/OdoInfo.h"
#include "nubot_common/Shoot.h"
#include "nubot_common/BallHandle.h"
#include "nubot_common/IMUInfo.h"

#define Send_Data_F 3000



using namespace std;

class Nubot_HWController
{
public:
    Nubot_HWController();
    ~Nubot_HWController();
    // write info
    short Cmd_Vx,Cmd_Vy,Cmd_W;
    short Shoot_Height,Shoot_Strength,Chasis_Mode,BallHandle_Mode;
    short ValidBit;
    short Imu_ValidBit;

    // read info
    double Real_Vx,Real_Vy,Real_w;
    double Real_angle;
    bool BallIsHolding;
    bool PowerState;
    bool RobotStuck; //const

    //atom link
    unsigned char nFD;  //ljt 2021-5-15
    unsigned char IMU_nFD;

    ros::ServiceServer ballhandle_service_;
    ros::ServiceServer shoot_service_;

   // void SerialWrite(const nubot_common::VelCmd::ConstPtr& cmd);
    void SerialWrite();
    void SerialWrite(const nubot_common::VelCmd::ConstPtr& cmd);
    void SerialWrite(const int ShootPower);//, const int ShootDep);
    void SerialWrite(const bool PowerSwitch);
    void SerialRead();
    void IMU_SerialRead();
    void Angle_Processing();
    bool BallHandleControlService(nubot_common::BallHandle::Request  &req,
                                  nubot_common::BallHandle::Response &res);
    bool ShootControlServive(nubot_common::Shoot::Request  &req,
                             nubot_common::Shoot::Response &res);

private:
    ros::NodeHandle nh;
    serial::Serial serial;
    ros::Subscriber Velcmd_sub_;
    ros::Timer timer1;
//    realtime_tools::RealtimePublisher<nubot_hwcontroller::DebugInfo> *DebugInfo_pub;
    realtime_tools::RealtimePublisher<nubot_common::OdoInfo> *OdoInfo_pub;
    realtime_tools::RealtimePublisher<nubot_common::IMUInfo> *IMUInfo_pub;

};


#endif