#include <stdio.h>
#include <string>   
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "nubot_hwcontroller_node.h"
#include "../../imu_publish/ros_inc/serial.h"
#include "../../imu_publish/ros_inc/macro.h"
#include "../../imu_publish/ros_inc/protocol.h"
#include "../../imu_publish/ros_inc/config.h"
#include "../../imu_publish/ros_inc/tool.h"

unsigned char frmBuf[256] = { 0 }; 
unsigned char * dataBuf = NULL;
int packLengthFW;//包头长度
int pkgLen;//数据包长
unsigned char IMU_frmBuf[256] = { 0 }; 
unsigned char * IMU_dataBuf = NULL;
int IMU_packLengthFW;//包头长度
int IMU_pkgLen;//数据包长
Data DataHandle; //
Data IMU_DataHandle;
FILE *fpLog = NULL;
FILE *IMU_fpLog = NULL;
float Odo_X=0;
float Odo_Y=0;
float Robot_Angle = 0;
float IMU_Angle;
float temp_Angle=0;
bool Angle_Mark = false;
short Holding_Ball = 0;
template <class Type>//暂时没用
//--------里程计调试--------//
//float Odo_X;
//float Odo_Y;
//------------------------//

Type stringToNum(const string& str)
{
    istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

Nubot_HWController::Nubot_HWController()
{
   Shoot_Height=11; //击球杆高度
   Shoot_Strength=20;  //击球力度
   Chasis_Mode=2;   //底盘模式 
   BallHandle_Mode=3;  //持球模式与击球
   ValidBit = 0; //有效位初始化为0
   Imu_ValidBit =0;
    IMU_nFD = IMU_InitConfig("/home/hawking/nubot_ws/src/nubot/nubot_hwcontroller/src/cfg1.json");  //放到了这个文件夹下
   
   nFD = InitConfig("/home/hawking/nubot_ws/src/nubot/nubot_hwcontroller/src/cfg2.json");  //放到了这个文件夹下
   
   ROS_INFO("nFD:%d IMU_nFD:%d",nFD,IMU_nFD);

   // 注册里程计以及调试信息topic
   // DebugInfo_pub = new realtime_tools::RealtimePublisher<nubot_hwcontroller::DebugInfo>(nh, "/nubotdriver/debug", 1);
   OdoInfo_pub = new realtime_tools::RealtimePublisher<nubot_common::OdoInfo>(nh, "/nubotdriver/odoinfo", 1);
   IMUInfo_pub = new realtime_tools::RealtimePublisher<nubot_common::IMUInfo>(nh, "/nubotdriver/angle", 1);
   // 接受底盘速度指令
   Velcmd_sub_ = nh.subscribe("/nubotcontrol/velcmd",10,&Nubot_HWController::SerialWrite,this);

   // 注册主动带球以及射门服务
   ballhandle_service_ = nh.advertiseService("BallHandle",&Nubot_HWController::BallHandleControlService,this); //持球
   shoot_service_ = nh.advertiseService("Shoot",&Nubot_HWController::ShootControlServive,this); //射门
}

Nubot_HWController::~Nubot_HWController()
{

}

void Nubot_HWController::SerialWrite(const nubot_common::VelCmd::ConstPtr& cmd)//ROS回调函数，修改3个速度
{
    Cmd_Vx = (short)(cmd->Vx*2);//-150 ~ 150 
    Cmd_Vy = -(short)(cmd->Vy*2);
    Cmd_W = -(short)(cmd->w*2);
    // Cmd_Vx = 0;
    // Cmd_Vy = 0;
    // Cmd_W = 30;
}

void Nubot_HWController::SerialWrite()//这个函数负责将速度发给下位机
{
    u8 MADDR = 0xFF;//目标地址，初始为广播形式
    u8 classID = 0x05;//操作类型
    u8 msgID = 0x01;//信息类型
    signed char payloadData[44]; 
    u16 payloadLen = 44; //数据长度
    Shoot_Strength = 300;//射球力度

    ROS_INFO("ShootInfo: Height,Strength,Mode: %d,%d,%d",Shoot_Height,Shoot_Strength,BallHandle_Mode);
    ROS_INFO("SerialWrite: Vx,Vy,W: %d,%d,%d",Cmd_Vx,Cmd_Vy,Cmd_W);
    if(BallHandle_Mode==1) ROS_INFO("Shoot!!!");
    // if(!(cmd->stop_ == false && cmd->Vx == 0 && cmd->Vy == 0 && cmd->w == 0))
    // {
        int fake_speed=0;
        payloadData[fake_speed++]=0x1E;
        payloadData[fake_speed++]=0x0C;
        payloadData[fake_speed++] = (unsigned char)(Cmd_Vy&0xFF);
        payloadData[fake_speed++] = (unsigned char)(Cmd_Vy>>8);
        payloadData[fake_speed++] = (unsigned char)(Cmd_Vx&0xFF);
        payloadData[fake_speed++] = (unsigned char)(Cmd_Vx>>8);
        payloadData[fake_speed++] = (unsigned char)(Cmd_W&0xFF);
        payloadData[fake_speed++] = (unsigned char)(Cmd_W>>8);
        payloadData[fake_speed++] = (unsigned char)(Shoot_Height&0xFF);      //击球干高度
        payloadData[fake_speed++] = (unsigned char)(Shoot_Height>>8);
        payloadData[fake_speed++] = (unsigned char)(Shoot_Strength&0xFF);    //击球力度
        payloadData[fake_speed++] = (unsigned char)(Shoot_Strength>>8);
        payloadData[fake_speed++] = (unsigned char)(BallHandle_Mode);           //持球模式 2：无力  3：持球  1：击球 
        payloadData[fake_speed++] = (unsigned char)(Chasis_Mode);        //底盘模式 2：无力  3：底盘  1：工程
       // payloadData[fake_speed++] = (unsigned char)(ValidBit);               //有效位（与下位机）
       // payloadData[fake_speed++] = (unsigned char)(Imu_ValidBit);            //有效位（与九轴）
        payloadData[fake_speed++]=0x19;
        payloadData[fake_speed++]=0x0C;
        for(;fake_speed<28;fake_speed++)
        {
            payloadData[fake_speed]='k';
        } 
        payloadData[fake_speed++]=0x22;
        payloadData[fake_speed++]=0x08;
        for(;fake_speed<38;fake_speed++)
        {
            payloadData[fake_speed]='k';
        }
        payloadData[fake_speed++]=0x31;
        payloadData[fake_speed++]=0x04;
        for(;fake_speed<44;fake_speed++)
        {
            payloadData[fake_speed]='k';
        }
        int jieshou = AtomCmd_Compose_Send(nFD, MADDR, classID, msgID, (u8*)payloadData, payloadLen);
        if(jieshou != 0) printf("Send Data Error!\n");
    // BallHandle_Mode = 3;//发送后将模式设为持球
        // if(BallHandle_Mode == 1)
        // {
        //     BallHandle_Mode=3;//如果状态为击球 就重置为持球状态
        // }
}

void Nubot_HWController::SerialRead()  //待改，tty
{
   // if(nFD<0) nFD = OpenCom(name->valuestring,115200);
    ValidBit = 0;
    GetFrame(nFD, dataBuf,packLengthFW+EMPTY_LEN);
    if(!ValidFrame(dataBuf,pkgLen))//判断是否有效
    {
        
        ValidBit = 1;
        packLengthFW = Align(nFD);
        FillFrameHead(dataBuf);
        GetFrame(nFD, dataBuf+HEAD_LEN,packLengthFW+TAIL_LEN);
        ParserDataPacket(&DataHandle, &dataBuf[HEAD_LEN], packLengthFW,fpLog);

        //除了是否持球，其他暂时没有用到
        OdoInfo_pub->msg_.header.stamp = ros::Time::now();
        OdoInfo_pub->msg_.Vx=0;
        OdoInfo_pub->msg_.Vy=0;
        OdoInfo_pub->msg_.w =0;
        OdoInfo_pub->msg_.angle=0;
        OdoInfo_pub->msg_.BallIsHolding = DataHandle.status2.holding;//是否持球
        ROS_INFO("Holding_Ball: %d",DataHandle.status2.holding);
        OdoInfo_pub->msg_.RobotStuck =0;
        OdoInfo_pub->msg_.PowerState =0;
        OdoInfo_pub->unlockAndPublish();
    }
    else
    {
        printf("Read Frame Invalid\n");
    }
}
void Nubot_HWController::IMU_SerialRead()  //待改，tty
{
    Imu_ValidBit = 0;
    printf("Start read IMU\n");
      IMU_packLengthFW = IMU_Align(IMU_nFD);
      printf("Start out:%d\n",IMU_packLengthFW);
    IMU_pkgLen = IMU_packLengthFW + EMPTY_LEN;
    printf("Length：%ddataBuf = &frmBuf[0];\n",IMU_packLengthFW);
    //GetFrame(IMU_nFD, IMU_dataBuf,IMU_packLengthFW+EMPTY_LEN);
    GetFrame(IMU_nFD, IMU_dataBuf,IMU_packLengthFW+EMPTY_LEN);
printf("Start GetFrame\n");
    IMU_FillFrameHead(IMU_dataBuf);
    
    
    
    if(!IMU_ValidFrame(IMU_dataBuf,IMU_pkgLen))//判断是否有效
    {
        float Odo_X;
        float Odo_Y;
       // float IMU_Angle;
        double pos_x;
        double pos_y;
        printf("valid packet!\n");
        Imu_ValidBit = 1;
        IMU_packLengthFW = IMU_Align(IMU_nFD);
        FillFrameHead(IMU_dataBuf);
        GetFrame(IMU_nFD, IMU_dataBuf+HEAD_LEN,IMU_packLengthFW+TAIL_LEN);
        IMU_ParserDataPacket(&IMU_DataHandle, &IMU_dataBuf[HEAD_LEN], IMU_packLengthFW,IMU_fpLog);
        Odo_X = IMU_DataHandle.distance.distanceX;
        Odo_Y = IMU_DataHandle.distance.distanceY;
        IMU_Angle = IMU_DataHandle.euler.yaw;
        pos_x = IMU_DataHandle.position.longlatX;
        pos_y = IMU_DataHandle.position.longlatY;
        ROS_INFO("SerialRead_IMU:Odo_X,Odo_y: %f,%f",Odo_X,Odo_Y);
        ROS_INFO("SerialRead_IMU:IMU_Angle: %f",IMU_Angle);//九轴角度
        ROS_INFO("SerialRead_IMU:position:%f,%f",pos_x,pos_y);

       
    }
    else
    {
        printf("Read IMU Frame Invalid\n");
    }
}
bool Nubot_HWController::BallHandleControlService(nubot_common::BallHandle::Request  &req,
                             nubot_common::BallHandle::Response &res)
{
    return true;
}

bool Nubot_HWController::ShootControlServive(nubot_common::Shoot::Request  &req,
                        nubot_common::Shoot::Response &res)
{
   if(req.strength==0)
   {
       Shoot_Strength = req.strength;
       Shoot_Height   = req.ShootPos;
       res.ShootIsDone = 0;
   }
   else
   {
       Shoot_Strength = req.strength;
       Shoot_Height   = req.ShootPos;
       BallHandle_Mode = 1;//设为射门模式
       res.ShootIsDone = 1;
   }
   return true;
}

void Nubot_HWController::Angle_Processing()
{

    if(!Angle_Mark)
    {
        Angle_Mark = true;
        Robot_Angle = 0;
        temp_Angle = IMU_Angle;

    }
    else
    {
        Robot_Angle = IMU_Angle - temp_Angle;

    }

    if(Robot_Angle<=-180)
    {
        Robot_Angle = Robot_Angle +360;
    }
    else if (Robot_Angle >=180)
    {
        Robot_Angle = Robot_Angle-360;
    }
    ROS_INFO("Robot_Angle:%f",Robot_Angle);
    Robot_Angle = Robot_Angle*0.0174532925; //  pi/180
     
    IMUInfo_pub->msg_.header.stamp = ros::Time::now();
    IMUInfo_pub->msg_.OdoA = Robot_Angle;
    IMUInfo_pub->unlockAndPublish();

}
int main(int argc,char** argv)
{
    ros::init(argc,argv, "nubot_hwcontroller_node");
    Nubot_HWController controller;

    dataBuf = &frmBuf[0];
    IMU_dataBuf = &frmBuf[0];
    // packLengthFW = Align(controller.nFD);//这里面有问题
    pkgLen = packLengthFW + EMPTY_LEN;
    fpLog = fopen("HWController.log","w");
   // IMU_fpLog = fopen("HWController_IMU.log","w");
    FillFrameHead(dataBuf);

    // GetFrame(controller.nFD, dataBuf,packLengthFW+EMPTY_LEN);//等待下位机的握手数据
    // while(!ValidFrame(dataBuf,pkgLen))
    // {
    //     packLengthFW = Align(controller.nFD);
    //     FillFrameHead(dataBuf);
    //     GetFrame(controller.nFD, dataBuf+HEAD_LEN,packLengthFW+TAIL_LEN);
    // }
    // //parser a whole frame to generate ros publish data 
    // ParserDataPacket(&DataHandle, &dataBuf[HEAD_LEN], packLengthFW,fpLog);//解包

    ros::Rate loop_rate(Send_Data_F/2); //100hz，和下位机同频
    while(ros::ok())
    {
        controller.IMU_SerialRead();  //九轴
        controller.SerialRead();
        controller.SerialWrite();
        controller.Angle_Processing(); //朝向角
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
//说明：
//每次先调用SerialRead，再调用SerialWrite。在SerialRead中进行检验，若从下位机收到包且包有效，则将ValidBit置为1。
//循环频率为Send_Data_F，当前设为100Hz。
//将回调函数和发送数据置下位机独立开来，回调函数更新速度信息，若未更新则速度不变。