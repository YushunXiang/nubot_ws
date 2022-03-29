#include "nubot/omni_vision/fieldinformation.h"

using namespace nubot;

FieldInformation::FieldInformation()
{
     xline_.push_back(FIELD_XLINE1);  /** 900，0*/
     xline_.push_back(FIELD_XLINE2);
     xline_.push_back(FIELD_XLINE3);
     xline_.push_back(FIELD_XLINE4);
     xline_.push_back(FIELD_XLINE5);
     xline_.push_back(FIELD_XLINE6);
     xline_.push_back(FIELD_XLINE7); /** -900，6*/

     yline_.push_back(FIELD_YLINE1);  /** 600，0 */
     yline_.push_back(FIELD_YLINE2);
     yline_.push_back(FIELD_YLINE3);
     yline_.push_back(FIELD_YLINE4);
     yline_.push_back(FIELD_YLINE5);
     yline_.push_back(FIELD_YLINE6); /** -600，5 */

     /** 场地中存在的所有的白线线段*/
     x_white_line_.push_back(LineSegment(DPoint(xline_[0],yline_[5]),DPoint(xline_[0],yline_[0])));
     x_white_line_.push_back(LineSegment(DPoint(xline_[1],yline_[3]),DPoint(xline_[1],yline_[2])));
     x_white_line_.push_back(LineSegment(DPoint(xline_[2],yline_[4]),DPoint(xline_[2],yline_[1])));
     x_white_line_.push_back(LineSegment(DPoint(xline_[3],yline_[5]),DPoint(xline_[3],yline_[0])));
     x_white_line_.push_back(LineSegment(DPoint(xline_[4],yline_[4]),DPoint(xline_[4],yline_[1])));
     x_white_line_.push_back(LineSegment(DPoint(xline_[5],yline_[3]),DPoint(xline_[5],yline_[2])));
     x_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[5]),DPoint(xline_[6],yline_[0])));

     y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[5]),DPoint(xline_[0],yline_[5])));
     y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[0]),DPoint(xline_[0],yline_[0])));
     y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[4]),DPoint(xline_[4],yline_[4])));
     y_white_line_.push_back(LineSegment(DPoint(xline_[0],yline_[4]),DPoint(xline_[2],yline_[4])));
     y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[1]),DPoint(xline_[4],yline_[1])));
     y_white_line_.push_back(LineSegment(DPoint(xline_[0],yline_[1]),DPoint(xline_[2],yline_[1])));
     y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[3]),DPoint(xline_[5],yline_[3])));
     y_white_line_.push_back(LineSegment(DPoint(xline_[0],yline_[3]),DPoint(xline_[1],yline_[3])));
     y_white_line_.push_back(LineSegment(DPoint(xline_[6],yline_[2]),DPoint(xline_[5],yline_[2])));
     y_white_line_.push_back(LineSegment(DPoint(xline_[0],yline_[2]),DPoint(xline_[1],yline_[2])));

    centercircle_.radius_=FIELD_CENTER_RADIUS;
    centercircle_.center_=DPoint2d(0,0);

    postcircle_.resize(4);
    for(size_t i=0; i< 4;i++)
       postcircle_[i].radius_=FIELD_POST_RADIUS;
    postcircle_[0].center_=DPoint2d(xline_[0],-yline_[0]);
    postcircle_[1].center_=DPoint2d(xline_[0],yline_[0]);
    postcircle_[2].center_=DPoint2d(-xline_[0],yline_[0]);
    postcircle_[3].center_=DPoint2d(-xline_[0],-yline_[0]);

    /** 守门员处的几个关键位置*/
    oppGoal_[GOAL_UPPER]    = DPoint(xline_[0],100);
    oppGoal_[GOAL_MIDUPPER] = DPoint(xline_[0],40);
    oppGoal_[GOAL_MIDDLE]   = DPoint(xline_[0],0);
    oppGoal_[GOAL_MIDLOWER] = DPoint(xline_[0],-40);
    oppGoal_[GOAL_LOWER]    = DPoint(xline_[0],-100);

    ourGoal_[GOAL_UPPER]    = DPoint(xline_[6],100);
    ourGoal_[GOAL_MIDUPPER] = DPoint(xline_[6],40);
    ourGoal_[GOAL_MIDDLE]   = DPoint(xline_[6],0);
    ourGoal_[GOAL_MIDLOWER] = DPoint(xline_[6],-40);
    ourGoal_[GOAL_LOWER]    = DPoint(xline_[6],-100);

    goalcircle_.resize(2);
    for(size_t i=0; i<2; i++)
        goalcircle_[i].radius_=60;
    goalcircle_[0].center_=DPoint2d(225.0,0.0);
    goalcircle_[1].center_=DPoint2d(-225.0,0.0);
}

FieldInformation::FieldInformation(string infopath)
{

}

bool
FieldInformation::isInInterRect(DPoint world_pt,double shrink)
{
    return (world_pt.x_>xline_[6]-shrink &&
            world_pt.x_<xline_[0]+shrink &&
            world_pt.y_>yline_[5]-shrink &&
            world_pt.y_<yline_[0]+shrink);
}

bool
FieldInformation::isInOuterRect(DPoint world_pt,double shrink)
{
    return (world_pt.x_<xline_[6]-shrink ||
            world_pt.x_>xline_[0]+shrink ||
            world_pt.y_<yline_[5]-shrink ||
            world_pt.y_>yline_[0]+shrink);
}


bool
FieldInformation::isOppfield(DPoint world_pt)
{
    return (world_pt.x_>0);
}

bool
FieldInformation::isOurfield(DPoint world_pt)
{
    return (world_pt.x_<0);
}
