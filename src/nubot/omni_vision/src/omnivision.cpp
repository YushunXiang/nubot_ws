// #include <iostream>
#include <fstream>
#include <sstream>
#include<iostream>
//#include<ifstream>
//#include<ofstream>
#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/transfer.h"
#include "nubot/omni_vision/scanpoints.h"
#include "nubot/omni_vision/whites.h"
#include "nubot/omni_vision/optimise.h"
#include "nubot/omni_vision/glocalization.h"
#include "nubot/omni_vision/odometry.h"
#include "nubot/omni_vision/localization.h"
#include "nubot/omni_vision/obstacles.h"
#include "nubot/omni_vision/colorsegment.h"
#include "nubot/omni_vision/ballfinder.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

#include <nubot_common/OdoInfo.h>
#include <nubot_common/RobotInfo.h>
#include <nubot_common/BallInfo.h>
#include <nubot_common/IMUInfo.h>
#include <nubot_common/ObstaclesInfo.h>
#include <nubot_common/OminiVisionInfo.h>
#include <omni_vision/OmniVisionConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sched.h>
#define DEFAULT_PRIO    80
using namespace cv;
using namespace nubot;

#ifndef H_LOW_HIGH
    #define H_LOW_HIGH      360
    #define H_HIGH_HIGH     360
    #define NUMS_PTS_LINE_HIGH  10
    #define FILTER_WIDTH_HIGH   10
    #define MERGE_WAVE_HIGH 10
    #define H_OF_PEAK_INDEX_HIGH 360
    #define WINDOW_NAME "WINDOW"
#endif

// whites中创建的调节bar的个控制为1个．
int NUM_OF_BAR=0;
float imu_angle=0;

namespace encodings=sensor_msgs::image_encodings;
namespace nubot {

struct RobotInformation
{
    DPoint2d realvtrans_;
    DPoint2d worldvtrans_;
    DPoint   visual_location_;
    DPoint   final_location_;
    Angle    angle_;
    double   angular_velocity_;
    bool     isglobal_;
};

class Omni_Vision
{

public:

    OmniImage        * imginfo_;
    Transfer         * tranfer_;
    ScanPoints       * scanpts_;
    Whites           * whites_;
    Optimise         * optimise_;
    Globallocalization * glocation_;
    Odometry         * odometry_;
    Localization     * location_;
    Obstacles        * obstacles_;
    BallFinder       * ball_finder_;
    ColorSegment     * colorsegment_;
    FieldInformation field_info_;

    RobotInformation robot;


    image_transport::Subscriber camera_sub_;
    ros::Subscriber odo_info_sub_;
    ros::Subscriber imu_info_sub_;

    ros::Publisher  ballinfo_pub_;
    ros::Publisher  robotinfo_pub_;
    ros::Publisher  obstaclesinfo_pub_;
    ros::Publisher  omin_vision_pub_;

    nubot_common::BallInfo        ball_info_;
    nubot_common::RobotInfo       robot_info_;
    nubot_common::ObstaclesInfo   obstacles_info_;
    nubot_common::OminiVisionInfo omin_vision_info_;
    dynamic_reconfigure::Server<omni_vision::OmniVisionConfig> reconfigureServer_;

    bool is_show_ball_;
    bool is_show_whites_;
    bool is_show_obstacles_;
    bool is_show_scan_points;
    bool is_show_result_;

    bool is_restart_;
    bool is_robot_stuck_;
    bool is_power_off_;
    cv::Mat field_image_;

    ros::Time receive_time_;
    int Agent_ID_;
public:

    Omni_Vision(int argc, char **argv)
    {
        char * environment = getenv("HOME");
        std::stringstream ss,env;
        env<<environment;

        char * agent_id_;
        if((agent_id_ = getenv("AGENT"))==NULL)
        {
            ROS_ERROR("this agent number is not read by robot");
            return ;
        }
        Agent_ID_ = atoi(agent_id_);
        ss<<Agent_ID_;

        std::string calibration_path=env.str()+"/nubot_ws/src/nubot/omni_vision/calib_results";

        if(argc>1)
            calibration_path=argv[1];

        ROS_INFO("initialize the omni_vision  process");
        imginfo_     = new OmniImage(calibration_path+"/"+ss.str()+"/ROI.xml");
        tranfer_     = new Transfer(calibration_path+"/"+ss.str()+"/mirror_calib.xml",*imginfo_);
        scanpts_     = new ScanPoints(*imginfo_);
        whites_      = new Whites(*scanpts_,*tranfer_);

        optimise_    = new Optimise(calibration_path+"/"+ss.str()+"/errortable.bin",
                                    calibration_path+"/"+ss.str()+"/Diff_X.bin",
                                    calibration_path+"/"+ss.str()+"/Diff_Y.bin",
                                    *tranfer_);
        glocation_   = new Globallocalization(*optimise_);
        odometry_    = new Odometry();
        location_    = new Localization(*optimise_);
        obstacles_   = new Obstacles(*scanpts_,*tranfer_);
        ball_finder_ = new BallFinder(*tranfer_);
        colorsegment_= new ColorSegment(calibration_path+"/"+ss.str()+"/CTable.dat");

        is_show_result_=true;
        is_show_ball_=true;
        is_show_obstacles_=true;
        is_show_whites_=true;
        is_show_scan_points=true;//**
        is_robot_stuck_ = false;

        robot.isglobal_=true;
        is_restart_=false;
        is_power_off_=false;
//        if(WIDTH_RATIO<1)
//            field_image_ = cv::imread(calibration_path+"/"+ss.str()+"/field_mine.bmp");
//        else
        field_image_ = cv::imread(calibration_path+"/"+ss.str()+"/field.jpg");

        ros::NodeHandle nh;
        image_transport::ImageTransport image_transport_(nh);
        camera_sub_= image_transport_.subscribe("/tiscamera/image_raw", 1, &Omni_Vision::imageCallback, this);
        ros::NodeHandle local_nh;
        odo_info_sub_ = local_nh.subscribe("/nubotdriver/odoinfo", 1, &Omni_Vision::odometryupdate,this);
        imu_info_sub_ = local_nh.subscribe("/nubotdriver/angle", 1, &Omni_Vision::angleupdate,this);

        ros::NodeHandle node;
        omin_vision_pub_   = node.advertise<nubot_common::OminiVisionInfo>("/omnivision/OmniVisionInfo",1);
        reconfigureServer_.setCallback(boost::bind(&Omni_Vision::configure, this, _1, _2));
    }
    ~Omni_Vision()
    {


    }

public:

    void
    configure(const omni_vision::OmniVisionConfig & config, uint32_t level)
    {
        ROS_INFO("Reconfigure request received");
        is_show_result_     = config.show;
        is_show_ball_       = config.ball;
        is_show_whites_     = config.white;
        is_show_obstacles_  = config.obstacle;
        is_show_scan_points = config.scan;
//        ROS_INFO("screen:%d, ball:%d, whites:%d, obs:%d, scan:%d", is_show_result_, is_show_ball_, is_show_whites_, is_show_obstacles_, is_show_scan_points);
        int  obstacle_thres = config.obsthres;
        double obstacle_length_thres = config.obs_length_thres;
        double obstacle_basic_thres = config.obs_basic_thres;
        obstacles_->setObsThres(obstacle_thres,obstacle_length_thres,obstacle_basic_thres);
        ROS_INFO("Reconfigure request end");
    }

    void
    publish()
    {
        robot_info_.header.stamp = receive_time_;
        robot_info_.header.seq++;
        robot_info_.heading.theta = robot.angle_.radian_;
        robot_info_.pos.x         = robot.final_location_.x_;
        robot_info_.pos.y         = robot.final_location_.y_;
        robot_info_.vtrans.x      = robot.worldvtrans_.x_;
        robot_info_.vtrans.y      = robot.worldvtrans_.y_;
        robot_info_.vrot          = robot.angular_velocity_;
        robot_info_.AgentID       = Agent_ID_;
        robot_info_.isstuck       = is_robot_stuck_;
        robot_info_.isvalid       = is_power_off_;
        if(is_show_result_)
        {
            ROS_INFO("[SELF](x:%.f, y:%.f, a:%d, vx:%.1f, vy:%.1f, w:%.1f, wp:%d ,g:%d, poff:%d)",
                     robot.final_location_.x_,robot.final_location_.y_,int(robot.angle_.degree()),robot.worldvtrans_.x_,
                     robot.worldvtrans_.y_,robot_info_.vrot,int(whites_->img_white_.size()),int(robot.isglobal_),int(is_power_off_));
        }

        ball_info_.header.stamp = receive_time_;
        ball_info_.header.seq++;
        ball_info_.pos.x =  ball_finder_->get_ball_global_loc().x_;
        ball_info_.pos.y =  ball_finder_->get_ball_global_loc().y_;
        ball_info_.real_pos.angle  = ball_finder_->get_ball_real_loc().angle_.radian_;
        ball_info_.real_pos.radius = ball_finder_->get_ball_real_loc().radius_;
        if(is_show_result_)
        {
            ROS_INFO("[BALL](x:%.f, y:%.f, vx:%.1f, vy:%.1f, k:%d ,as:%d)",ball_info_.pos.x,ball_info_.pos.y, ball_info_.velocity.x,
                     ball_info_.velocity.y,ball_info_.pos_known, ball_finder_->ball_area_.area_size_);
        }
        obstacles_info_.header.stamp= ros::Time::now();
        obstacles_info_.header.seq++;
        obstacles_info_.pos.clear();
        obstacles_info_.polar_pos.clear();
        int length= obstacles_->real_obstacles_.size();
        nubot_common::Point2d point;
        nubot_common::PPoint  polar_point;
        for(int i = 0 ; i < length ; i++)
        {
            DPoint & pt=obstacles_->world_obstacles_[i];
            PPoint & polar_pt= obstacles_->real_obstacles_[i];
            point.x=pt.x_;
            point.y=pt.y_;
            polar_point.angle=polar_pt.angle_.radian_;
            polar_point.radius=polar_pt.radius_;
            obstacles_info_.pos.push_back(point);
            obstacles_info_.polar_pos.push_back(polar_point);
        }
        if(is_show_result_)
        {
            for(int i = 0 ;i < OBS_VALUABLE_NUMBER ;i++)
            {
                if (i<obstacles_info_.pos.size())
                    ROS_INFO("[OBS-%d](%.f, %.f)", i, obstacles_info_.pos[i].x,obstacles_info_.pos[i].y);
                //  else
                //       ROS_INFO("obs_omni(%.f, %.f)",-10000.0,-10000.0);
            }
        }
        omin_vision_info_.header.stamp = robot_info_.header.stamp;
        omin_vision_info_.header.seq++;
        omin_vision_info_.ballinfo=ball_info_;
        omin_vision_info_.obstacleinfo=obstacles_info_;
        omin_vision_info_.robotinfo.clear();
        omin_vision_info_.robotinfo.push_back(robot_info_);
        omin_vision_pub_.publish(omin_vision_info_);
    }
    void static nullFun(int,void *){

    }

    // void static changeIT(int value,void *){
    //     for (int i=0;i<I_MIN;i++)
	//         whites_->t_new[i]=float(T_MIN);
    //     for (int i=I_MIN;i<=I_MAX;i++)
    //         whites_->t_new[i]=float((cos(float(i-I_MIN)*SINGLEPI_CONSTANT/float(I_MAX-I_MIN)+SINGLEPI_CONSTANT)+1)*(T_MAX-T_MIN)/2+T_MIN);
    //     for (int i=I_MAX+1;i<256;i++)
    //         whites_->t_new[i]=float(T_MAX);
    // }
    void
    process()
    {
        if(!colorsegment_->Segment(imginfo_->yuv_image_))
            return;

        //get the colors of the scan_points
        scanpts_->process();
        
        if(NUM_OF_BAR==0){
            namedWindow(WINDOW_NAME,CV_WINDOW_NORMAL);
            char TrackbarName_H_LOW          [50],
                 TrackbarName_H_HIGH         [50],
                 TrackbarName_NUMS_PTS_LINE  [50],
                 TrackbarName_FILTER_WIDTH   [50],
                 TrackbarName_MERGE_WAVE     [50],
                 TrackbarName_H_OF_PEAK_INDEX[50]
                //  TrackbarName_I_MAX          [50],
                //  TrackbarName_I_MIN          [50],
                //  TrackbarName_T_MAX          [50],
                //  TrackbarName_T_MIN          [50],
                ;
            int& H_LOW          =whites_->h_low_;
            int& H_HIGH         =whites_->h_high_;
            int& NUMS_PTS_LINE  =whites_->nums_pts_line_;
            int& FILTER_WIDTH   =whites_->filter_width_;  
            int& MERGE_WAVE     =whites_->merge_wave_;
            int& H_OF_PEAK_INDEX=whites_->h_of_peak_index_;
            

            sprintf(TrackbarName_H_LOW          ,"TrackbarName_H_LOW           %d",H_LOW          );
            sprintf(TrackbarName_H_HIGH         ,"TrackbarName_H_HIGH          %d",H_HIGH         );
            sprintf(TrackbarName_NUMS_PTS_LINE  ,"TrackbarName_NUMS_PTS_LINE   %d",NUMS_PTS_LINE  );
            sprintf(TrackbarName_FILTER_WIDTH   ,"TrackbarName_FILTER_WIDTH    %d",FILTER_WIDTH   );
            sprintf(TrackbarName_MERGE_WAVE     ,"TrackbarName_MERGE_WAVE      %d",MERGE_WAVE     );
            sprintf(TrackbarName_H_OF_PEAK_INDEX,"TrackbarName_H_OF_PEAK_INDEX %d",H_OF_PEAK_INDEX);
            // sprintf(TrackbarName_I_MAX          ,"TrackbarName_I_MAX           %d",I_MAX          );
            // sprintf(TrackbarName_I_MIN          ,"TrackbarName_I_MIN           %d",I_MIN          );
            // sprintf(TrackbarName_T_MAX          ,"TrackbarName_T_MAX           %d",T_MAX          );
            // sprintf(TrackbarName_T_MIN          ,"TrackbarName_T_MIN           %d",T_MIN          );
            createTrackbar(TrackbarName_H_LOW          ,WINDOW_NAME,&H_LOW               ,H_LOW_HIGH,          nullFun);
            createTrackbar(TrackbarName_H_HIGH         ,WINDOW_NAME,&H_HIGH              ,H_HIGH_HIGH,         nullFun);
            createTrackbar(TrackbarName_NUMS_PTS_LINE  ,WINDOW_NAME,&NUMS_PTS_LINE       ,NUMS_PTS_LINE_HIGH,  nullFun);
            createTrackbar(TrackbarName_FILTER_WIDTH   ,WINDOW_NAME,&FILTER_WIDTH        ,FILTER_WIDTH_HIGH,   nullFun);
            createTrackbar(TrackbarName_MERGE_WAVE     ,WINDOW_NAME,&MERGE_WAVE          ,MERGE_WAVE_HIGH,     nullFun);
            createTrackbar(TrackbarName_H_OF_PEAK_INDEX,WINDOW_NAME,&H_OF_PEAK_INDEX     ,H_OF_PEAK_INDEX_HIGH,nullFun);
            // createTrackbar(TrackbarName_I_MAX          ,WINDOW_NAME,&I_MAX               ,I_MAX              ,changeIT);
            // createTrackbar(TrackbarName_I_MIN          ,WINDOW_NAME,&I_MIN               ,I_MIN              ,changeIT);
            // createTrackbar(TrackbarName_T_MAX          ,WINDOW_NAME,&T_MAX               ,T_MAX              ,changeIT);
            // createTrackbar(TrackbarName_T_MIN          ,WINDOW_NAME,&T_MIN               ,T_MIN              ,changeIT);
            NUM_OF_BAR=1;

        }
        
        //detect the white points
        whites_->process();

        if(whites_->img_white_.size()<=0)
        {
            //  is_restart_=true;
            ROS_WARN("don't detect the white points");
            return ;
        }
        
        if(is_power_off_)
            is_restart_=true;
        if(is_restart_)
        {
            is_restart_=false;
            robot.isglobal_=true;
        }
        //localization
        if(robot.isglobal_)
        {
            ROS_INFO("start global localization");
            robot.isglobal_=glocation_->process(imu_angle,whites_->weights_,whites_->robot_white_,robot.visual_location_,robot.angle_);
        }
        else
        {
//
            DPoint delta_loc=odometry_->getWorldLocaton();
            Angle  delta_ang=odometry_->getDeltaAngle();
            odometry_->clear(robot.angle_);
            robot.realvtrans_       = odometry_->getRealVelocity();
            robot.worldvtrans_      = odometry_->getWorldVelocity();
            robot.angular_velocity_ = odometry_->getAngularVelocity();
        //    ROS_INFO("[real]%f,%f [world]%f,%f [angular]%f", robot.realvtrans_.x_, robot.realvtrans_.y_, robot.worldvtrans_.x_, robot.worldvtrans_.y_, robot.angular_velocity_);
        //    ROS_INFO("[delta_loc]%f, %f, [delta_ang]%f", delta_loc.x_, delta_loc.y_, delta_ang.radian_);
            location_->process(whites_->weights_,whites_->robot_white_,robot.visual_location_,robot.angle_,delta_loc,delta_ang);
        }
        PPoint correct_pt = PPoint(tranfer_->get_offset().angle(),tranfer_->get_offset().norm());
        tranfer_->calculateWorldCoordinates(correct_pt,robot.visual_location_,robot.angle_,robot.final_location_);
        // robot.final_location_.x_ = -5.2040;
        // robot.final_location_.y_ = 0.2674;
        // robot.angle_.radian_ = -0.0211;
        ROS_INFO("Final location:   [%f], [%f] ,[%lf]",robot.final_location_.x_,robot.final_location_.y_,robot.angle_.radian_);

        obstacles_->process(colorsegment_->segment_result_,robot.final_location_,robot.angle_);
        ball_info_.pos_known=ball_finder_->Process(colorsegment_->segment_result_,robot.final_location_,robot.angle_);
        //ROS_INFO("Ball location:    %d  ,%d",ball_info_.pos_known.x,ball_info_.pos_known.y);
        if(is_show_whites_ || is_show_obstacles_ || is_show_ball_ )
        {
            if(field_image_.empty())
                ROS_INFO("Field.bmp is empty");
            cv::Mat image  = field_image_.clone();
            cv::Mat orgnal = imginfo_->getBGRImage().clone();
            static double length = 700;
            static double width  = 500;
//            if(WIDTH_RATIO<1)
//            {
//                length = 1920;
//                width  = 882;
//            }
            if(is_show_scan_points)
                scanpts_->showScanPoints();

            if(is_show_obstacles_)
            {
                obstacles_->showObstacles(orgnal);
                if(!field_image_.empty())
                    obstacles_->showObstacles(image,robot.final_location_,robot.angle_,length,width);
            }
               if(is_show_ball_)
            {
                //ROS_INFO("ball is found!");
                ball_finder_->showBall(orgnal);
                if(!field_image_.empty())
                    ball_finder_->showBall(image,robot.final_location_,robot.angle_,length,width);
            }
            if(is_show_whites_)
            {
                whites_->showWhitePoints(orgnal);
                //robot.angle_=0;  //mark direction
                if(!field_image_.empty())
                    whites_->showWhitePoints(image,robot.final_location_,robot.angle_,length,width);
            }
         
        }
    }

    void
    imageCallback(const sensor_msgs::ImageConstPtr& _image_msg)
    {
        ros::Time start = ros::Time::now();
        receive_time_ = _image_msg->header.stamp;
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr=cv_bridge::toCvShare(_image_msg,encodings::BGR8);
        Mat orignalimage=cv_ptr->image;
        bool isthreechannels=imginfo_->setBGRImage(orignalimage);
        if(!isthreechannels)
        {
            ROS_WARN("the image doesn't have three channels");
            return ;
        }
        imginfo_->bgr2yuv();

        process();
        publish();
        //   ROS_INFO("start omni_vision imageCallback");
        static ros::Time time_before = ros::Time::now();
        ros::Duration duration  = ros::Time::now() - time_before;
        ros::Duration duration1 = ros::Time::now() - start;
        time_before = ros::Time::now();
//        ROS_INFO("omni_time: %d %d %d",int(1.0/duration.toSec()),int(1.0/duration1.toSec()),int(whites_->img_white_.size()));
    }

    void
    odometryupdate(const nubot_common::OdoInfo & _odoinfo_msg)
    {
        static std::vector<double> motor_data(nubot::MOTOR_NUMS_CONST,0);
        static double gyro_data;
        static ros::Time time_before = _odoinfo_msg.header.stamp;
        ros::Time time_update = _odoinfo_msg.header.stamp;
        motor_data[0]=(double)_odoinfo_msg.Vx;
        motor_data[1]=(double)_odoinfo_msg.Vy;
        motor_data[2]=(double)_odoinfo_msg.w;
        gyro_data =(double)_odoinfo_msg.angle;
        is_power_off_   = !_odoinfo_msg.PowerState; // 0-off 1-on
        is_robot_stuck_ = _odoinfo_msg.RobotStuck;
        ROS_INFO("vx:%f,vy:%f,w:%f,angle:%f,ipo:%d,irs:%d",motor_data[0],motor_data[1],motor_data[2],gyro_data,is_power_off_,is_robot_stuck_);
        ros::Duration duration= time_update-time_before;
        time_before = time_update ;
        double secs=duration.toSec();
        odometry_->process(motor_data,secs);
    }

    void angleupdate(const nubot_common::IMUInfo & _angle_msg)
    {
        imu_angle = _angle_msg.OdoA;
    }

};


}// end of namespace nubot


int main(int argc, char **argv)
{ 
    struct sched_param schedp;
    // ofstream output;
    memset(&schedp, 0, sizeof(schedp));

    schedp.sched_priority = DEFAULT_PRIO;
    if (sched_setscheduler(0, SCHED_FIFO, &schedp)) {
      printf("set scheduler failed.\n");
      sched_setscheduler(0, SCHED_OTHER, &schedp);
    }
    ros::init(argc,argv,"omni_vision_node");
    ros::Time::init();
    // output<<
    ROS_INFO("start omni_vision process");
    nubot::Omni_Vision vision_process(argc, argv);
    ros::spin();
    return 0;
}
