#ifndef AUTO_ODOMETRY_H
#define AUTO_ODOMETRY_H
#include <time.h>
#include "agv_solver/car_model.h"
#include "agv_solver/Model_to_Base.h"
#include "agv_solver/Base_to_Model.h"

#include "agv_solver/VW_Center.h"
#include "agv_solver/Base_to_OneDiff.h"
#include "agv_solver/Base_to_TwoDiff.h"
#include "agv_solver/Base_to_FourDiff.h"
#include "agv_solver/Base_to_OneHelm.h"
#include "agv_solver/Base_to_TwoHelm.h"
#include "agv_solver/Base_to_FourHelm.h"
#include "agv_solver/Base_to_Mecanum.h"
#include "agv_solver/Base_to_JointMecan.h"
#include "agv_solver/Base_to_Carlike.h"

#include "agv_solver/OneDiff_to_Base.h"
#include "agv_solver/TwoDiff_to_Base.h"
#include "agv_solver/FourDiff_to_Base.h"
#include "agv_solver/OneHelm_to_Base.h"
#include "agv_solver/TwoHelm_to_Base.h"
#include "agv_solver/FourHelm_to_Base.h"
#include "agv_solver/Mecanum_to_Base.h"
#include "agv_solver/JointMecan_to_Base.h"
#include "agv_solver/Carlike_to_Base.h"
#include "agv_solver/DualMGM_to_plc.h"
#include <ros/ros.h>
#include <string>

using std::string;
class Odometry_Adjust
{
public:
    Odometry_Adjust()
    {
        Theta1 = 0.0f;
        TimeStart = 0.0f;
        Theta2 = 0.0f;
        TimeEnd = 0.0f;
        W_Start = false;
        W_Stop = false;
        DrivePosFinish = false;
        DriveNegFinish = false;
        FdbkPosFinish = false;
        FdbkNegFinish = false;
        /*DriveRatio.LDPos = 0.0f;
        DriveRatio.RDPos = 0.0f;
        DriveRatio.LDNeg = 0.0f;
        DriveRatio.RDNeg = 0.0f;
        FbRatio.LFbPos = 0.0f;
        FbRatio.RFbPos = 0.0f;
        FbRatio.LFbNeg = 0.0f;
        FbRatio.RFbNeg = 0.0f;*/
		Odom_Clac_Finish();
    };
    void  Init();
    float CalcAngleVelocity();
    void  CalcVelocityCurr();
    void  DriveVelocityRatio();
    void  FeedbackOffset();
    void  Odom_Clac_Finish();
    void  Auto_Odom_Proc(float yaw);
    //void Auto_Odom_FbackV(agv_solver::Base_to_Model base_to_model);
    void Auto_Odom_FbackV(const agv_solver::Base_to_Model& base_to_model);
public:
    float  Theta1;
    double TimeStart;
    float  Theta2;
    double TimeEnd;
    bool  W_Start;
    bool  W_Stop;
    CoordCalcW   AngVelocityCurr;
    CoordVelCurr VelocityCurr;
    CoordDRatio  DriveRatio;
    CoordFbRatio FbRatio;
    float CurrentTheta;
    float AngleVelocity_temp;
    float VelocityFb_left;
    float VelocityFb_right;
    float VelocityFb_left1;
    float VelocityFb_right1;
    float VelocityFb_left2;
    float VelocityFb_right2;
    float VelocityExp_left;
    float VelocityExp_right;
    bool  DrivePosFinish;
    bool  DriveNegFinish;
    bool  FdbkPosFinish;
    bool  FdbkNegFinish;
    //float FeedbackVl;
    //float FeedbackVr;
public:
	agv_solver::Base_to_Model base_to_model;
};

template<typename T>
T getParam(const string& name,const T& defaultValue)
{
    T v;
    if(ros::param::get(name,v))
    {
        ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
        return v;
    }
    else
        ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
    return defaultValue;
}
#endif // SOLVER_H
