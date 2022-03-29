#ifndef __NUBOT_VISION_FIELDINFOMATION_H_
#define __NUBOT_VISION_FIELDINFOMATION_H_

#include "nubot/core/core.hpp"
#include <opencv2/opencv.hpp>

namespace nubot
{
using std::vector;
using std::string;

enum GoalLocation
{
     GOAL_UPPER     = 0,
     GOAL_MIDUPPER  = 1,
     GOAL_MIDDLE    = 2,
     GOAL_MIDLOWER  = 3,
     GOAL_LOWER     = 4,
};

class FieldInformation
{
public:

    FieldInformation();
    FieldInformation(string infopath);

    bool isInInterRect(DPoint world_pt,double shrink=0);
    bool isInOuterRect(DPoint world_pt,double shrink=0);
    bool isOppfield(DPoint world_pt);
    bool isOurfield(DPoint world_pt);

    std::vector<int> xline_;
    std::vector<int> yline_;

    std::vector<Circle > postcircle_;
    std::vector<LineSegment> x_white_line_;
    std::vector<LineSegment> y_white_line_;

    DPoint oppGoal_[5]; /** 5 the number of GoalLocation */
    DPoint ourGoal_[5];
    Circle centercircle_;

    std::vector<Circle > goalcircle_;
};



}
#endif  //!__NUBOT_VISION_FIELDINFOMATION_H_

