#ifndef __NUBOT_VISION_WHITES_H_
#define __NUBOT_VISION_WHITES_H_

#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "nubot/omni_vision/scanpoints.h"
#include "nubot/omni_vision/transfer.h"


#ifndef H_LOW_HIGH
    #define H_LOW_HIGH      360
    #define H_HIGH_HIGH     360
    #define NUMS_PTS_LINE_HIGH  10
    #define FILTER_WIDTH_HIGH   10
    #define MERGE_WAVE_HIGH 10
    #define H_OF_PEAK_INDEX_HIGH 360
    #define WINDOW_NAME "WINDOW"
#endif

namespace nubot
{ 

using std::vector;
using std::string;

const int MAX_NUMBRT_CONST = 10000;
const int MIN_NUMBRT_CONST = -10000;

class Whites
{
public:

	struct peak
	{
		int peak_index;
		int left_hollow;
		int right_hollow;
		int near_hollow;
		int width;
		bool boundaries;
	};

public:

    Whites(ScanPoints & _scanpts, Transfer & _coor_transfer);
	
	void process();

    void detectWhitePts(std::vector<DPoint2i> & pts,std::vector<uchar> & _ColorY_Aver,std::vector<DPoint2i> & whites);
	
    void detectWave(std::vector<uchar> & colors,std::vector<bool> & wave_hollow,std::vector<bool> & wave_peak);

    void findNearHollow(vector<uchar> & colors,vector<bool> & wave_peak,vector<bool> & wave_hollow,vector<peak> & peak_count);

    bool IsWhitePoint(std::vector<DPoint2i> & pts,double _color_average,vector<uchar> & colors,peak & peak_count );
	
	void calculateWeights();
	
    void showWhitePoints(cv::Mat & _img);

    void showWhitePoints(cv::Mat &  _img,DPoint _robot_loc, Angle _angle, int filed_length =640,int filed_width=430);

	// 设置whites中的各种阈值参数，作为创建滑动窗口的回调函数．在主函数中调用．
	void setWhites_H_LOW(int,void *);
	void setWhites_H_HIGH(int,void *);
	void setWhites_NUMS_PTS_LINE(int,void *);
	void setWhites_FILTER_WIDTH(int,void *);
	void setWhites_MERGE_WAVE(int,void *);
	void setWhites_H_OF_PEAK_INDEX(int,void *);

    ScanPoints   * scanpts_;
    Transfer     * transfer_;

	vector<DPoint2i> img_white_;
  	vector<PPoint>   robot_white_;
    vector<double>   weights_;

public:

	int I_MAX=200;
	int I_MIN=20;
	int T_MAX=30;
	int T_MIN=10;
	
	int 	h_low_			=40;   //40	;
	int 	h_high_			=100;	// 100	;
	int 	nums_pts_line_	=4;//4	;
	int 	filter_width_	=2;	//2	;          
	int 	merge_wave_		=3;		//3	;
	float 	t_new[256]			;

	// whites函数中的一个阈值
	int h_of_peak_index_	=130;

};

}
#endif  //!__NUBOT_VISION_WHITES_H_

