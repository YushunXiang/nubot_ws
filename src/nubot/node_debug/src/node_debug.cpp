#include <ros/ros.h>
#include "nubot_common/VelCmd.h"

using namespace std;


void callback(const nubot_common::VelCmd::ConstPtr& cmd)
{
    ROS_INFO("######## Node Debug ######### v : %d,%d,%d",(short)(cmd->Vx),(short)(cmd->Vy),(short)(cmd->w));

}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"node_debug");
    ros::NodeHandle debug;

    ros::Subscriber node_debug_sub = debug.subscribe("/nubotcontrol/velcmd",10,callback);
    ros::spin();

    return 0;
}