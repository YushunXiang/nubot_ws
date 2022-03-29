#ifndef __NUBOT_VISION_OBSTACLES_H_
#define __NUBOT_VISION_OBSTACLES_H_


#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/scanpoints.h"
#include "nubot/omni_vision/transfer.h"
#include "nubot/omni_vision/fieldinformation.h"

//Macro definition for color segmentation
#define VISION_COLORSEGMENT_YELLOW	0	//Yellow represent for football
#define VISION_COLORSEGMENT_BLACK	1	//black represent for obstacles
#define VISION_COLORSEGMENT_GREEN	2	//green represent field
#define VISION_COLORSEGMENT_UNKNOWCOLOR	3	//Unknown color
//#define VISION_COLORSEGMENT_RED	2	//red represent for red cone
//#define VISION_COLORSEGMENT_WHITE	3	//white represent for white cone
//#define VISION_COLORSEGMENT_BLUE	4	//blue represent for blue cone
//#define VISION_COLORSEGMENT_GREEN	5	//green represent for green cone
//#define VISION_COLORSEGMENT_UNKNOWCOLOR	6	//Unknown color

namespace nubot
{ 

const int    OBS_MINDARKNESS_CONST  = 50;
const double OBS_WEIGHT_THRES_CONST = 0.04;
const double OBS_RADIUS_CONST       = 30; //40
const double OBS_PARTITION_CONST    = 35;
const int    OBS_MAXDISTANCE_CONST  = 500; //1000;

#ifdef THREEPLAYER
const int    OBS_VALUABLE_NUMBER    = 7; //12
const int    OBS_NUMBER_CONST       = 25; //50
#else
const int    OBS_VALUABLE_NUMBER    = 2;
const int    OBS_NUMBER_CONST       = 10;
#endif

const double OBS_LIGHTFACTOR = 0.05;
const double OBS_LIGHTFACTORINCREASE = 0.3 ;
const double OBS_BLACKCOUNTFACTOR = 1700;
const double OBS_BLACKSIZE =0.6;

class Obstacles
{
public:
    Obstacles(ScanPoints & _scanpts,Transfer & _trans);
    void process(cv::Mat & _segment_result,DPoint & _robot_loc,Angle & _robot_head);
    void detectBlacks(std::vector<DPoint2i> & _pts,std::vector<uchar> & _ColorY,std::vector<DPoint2i> & _black_pts,std::vector<bool> & obs_segment);
    void getWeights();
    void getObstacles();
    void setObsThres(int _obs_thres,double _obstacle_length_thres,double _obstacle_basic_thres);
    void transferCoordinate(DPoint & _robot_loc,Angle & _robot_head);

    void showObstacles(cv::Mat & _img);
    void showObstacles(cv::Mat & _img,DPoint _robot_loc, Angle _angle, int filed_length =640,int filed_width=430);

public:
    ScanPoints  * scanpts_;
    Transfer    * transfer_;
    FieldInformation field_info_;
	std::vector< std::vector<DPoint2i> > black_pts_;
	std::vector<double> weight_;
	std::vector<double> distance_;
    std::vector<PPoint> real_obstacles_;
    std::vector<DPoint> world_obstacles_;
    std::vector< std::vector<bool> > obs_segments_;

private:
	int obsthres_;
    double obstacle_length_thres_;
    double obstacle_basic_thres_;
	uchar miniY_;
	uchar maxY_;
	double interval_radian_;
};



}
#endif  //!__NUBOT_VISION_OBSTACLES_H_

