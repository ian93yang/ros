#include "agv_solver/auto_odometry.h"
//#include <chrono>
#include <ros/ros.h>
#include <time.h>
#include <iostream>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std;

void Odometry_Adjust::Init()
{
    //static uint8_t enable_last= 0;
    //int8_t enable = base_to_model.vw_exp.Enable;
    //if ((enable==1) && (enable_last==0) && (base_to_model.vw_exp.Mode ==1))
    //{
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

    //}

    //VelocityRatio.LWPos = 0.0f;
    //VelocityRatio.RWPos = 0.0f;
    //enable_last = base_to_model.vw_exp.Enable;
}

void Odometry_Adjust::Auto_Odom_Proc(float yaw)
{
    CurrentTheta = yaw;
}

float Odometry_Adjust::CalcAngleVelocity()
{
    ROS_INFO("CurrentTheta = %f", CurrentTheta);

    if ((Theta1 == 0.0)&&(TimeStart == 0.0))
    {
        W_Start = false;
    }
    if ((Theta2 == 0.0)&&(TimeEnd == 0.0))
    {
        W_Stop = false;
    }
    if ((fabs(CurrentTheta) > 0.785)&&(W_Start == false))
    {
        VelocityFb_left1 = base_to_model.one_diff.Vl;
        VelocityFb_right1 = base_to_model.one_diff.Vr;
        Theta1 = CurrentTheta;
        //TimeStart = clock();
        ros::Time::init();
        TimeStart = ros::Time::now().toSec();
        //TimeStart = system_clock::now();
        W_Start = true;
    }
    ROS_INFO("Theta1 = %f", Theta1);
    ROS_INFO("TimeStart = %f", TimeStart);
    ROS_INFO("W_Start = %d", W_Start);
    if ((fabs(CurrentTheta) > 2.356)&&(W_Stop == false))
    {
        VelocityFb_left2 = base_to_model.one_diff.Vl;
        VelocityFb_right2 = base_to_model.one_diff.Vr;
        Theta2 = CurrentTheta;
        //TimeEnd = clock();
        TimeEnd = ros::Time::now().toSec();
        //TimeEnd = system_clock::now();
        W_Stop = true;
    }
    ROS_INFO("Theta2 = %f", Theta2);
    ROS_INFO("TimeEnd = %f", TimeEnd);
    ROS_INFO("W_Stop = %d", W_Stop);
    AngVelocityCurr.DeltaTheta  = Theta2 - Theta1;
    ROS_INFO("DeltaTheta = %f", AngVelocityCurr.DeltaTheta);
    //AngVelocityCurr.DeltaTime = (TimeEnd - TimeStart) / CLOCKS_PER_SEC;
    AngVelocityCurr.DeltaTime = (TimeEnd - TimeStart);
    //AngVelocityCurr.DeltaTime = duration_cast<microseconds>(TimeEnd - TimeStart);
    ROS_INFO("DeltaTime = %f", AngVelocityCurr.DeltaTime);
    if ((W_Start == true) && (W_Stop == true))
    {
        AngleVelocity_temp = fabs (AngVelocityCurr.DeltaTheta / AngVelocityCurr.DeltaTime);
        //W_Start = false;
        //W_Stop = false;
    }
    ROS_INFO("AngleVelocity = %f", AngleVelocity_temp);
    return AngleVelocity_temp;
}
void Odometry_Adjust::CalcVelocityCurr()
{
    //float AngleVelocity_temp;
    float WheelBase_Debug = getParam<double>("OneDiff_Base",600);
    float radius_temp = getParam<float>("RadiusCurr",0);
    VelocityExp_left = getParam<float>("VelocityL",0);
    VelocityExp_right = getParam<float>("VelocityR",0);

    //AngleVelocity_temp = abs(Delta_Angle / time);
    if ((VelocityExp_left > 0.0f)&&(VelocityExp_right > 0.0f))
    {
        if (radius_temp > 0)
        {
            VelocityCurr.LWPos = AngleVelocity_temp * 0.5f * abs(radius_temp);
            VelocityCurr.RWPos = AngleVelocity_temp * (0.5f * abs(radius_temp) + WheelBase_Debug);
            //ROS_INFO("AngleVelocity = %f", AngleVelocity_temp);
            ROS_INFO("VelocityCurrLPos+ = %f", VelocityCurr.LWPos);
            ROS_INFO("VelocityCurrRPos+ = %f", VelocityCurr.RWPos);
        }
        else if (radius_temp < 0)
        {
            VelocityCurr.LWPos = AngleVelocity_temp * (0.5f * abs(radius_temp) + WheelBase_Debug);
            VelocityCurr.RWPos = AngleVelocity_temp * 0.5f * abs(radius_temp);
            ROS_INFO("VelocityCurrLPos- = %f", VelocityCurr.LWPos);
            ROS_INFO("VelocityCurrRPos- = %f", VelocityCurr.RWPos);
        }
    }
    else if ((VelocityExp_left < 0.0f)&&(VelocityExp_right < 0.0f))
    {
        if (radius_temp > 0)
        {
            VelocityCurr.LWNeg = -AngleVelocity_temp * (0.5f * abs(radius_temp) + WheelBase_Debug);
            VelocityCurr.RWNeg = -AngleVelocity_temp * 0.5f * abs(radius_temp);
            ROS_INFO("VelocityCurrLNeg+ = %f", VelocityCurr.LWNeg);
            ROS_INFO("VelocityCurrRNeg+ = %f", VelocityCurr.RWNeg);
        }
        else if (radius_temp < 0)
        {
            VelocityCurr.LWNeg = -AngleVelocity_temp * 0.5f * abs(radius_temp);
            VelocityCurr.RWNeg = -AngleVelocity_temp * (0.5f * abs(radius_temp) + WheelBase_Debug);
            ROS_INFO("VelocityCurrLNeg- = %f", VelocityCurr.LWNeg);
            ROS_INFO("VelocityCurrRNeg- = %f", VelocityCurr.RWNeg);
        }
    }
}

void Odometry_Adjust::DriveVelocityRatio()
{
    VelocityExp_left = getParam<float>("VelocityL",0);
    VelocityExp_right = getParam<float>("VelocityR",0);
    float radius_temp = getParam<float>("RadiusCurr",0);
    if ((VelocityExp_left > 0.0f)&&(VelocityExp_right > 0.0f))
    {
        if (radius_temp > 0)
        {
            DriveRatio.LDPos = VelocityCurr.LWPos / VelocityExp_left - 1.0;
            DriveRatio.RDPos = VelocityCurr.RWPos / VelocityExp_right - 1.0;
            ROS_INFO("+DriveLPOs = %f", DriveRatio.LDPos);
            ROS_INFO("+DriveRPos = %f", DriveRatio.RDPos);
        }
        else if (radius_temp < 0)
        {
            DriveRatio.LDPos = VelocityCurr.LWPos / VelocityExp_left - 1.0;
            DriveRatio.RDPos = VelocityCurr.RWPos / VelocityExp_right - 1.0;
            ROS_INFO("-DriveLPos = %f", DriveRatio.LDPos);
            ROS_INFO("-DriveRPos = %f", DriveRatio.RDPos);
        }
        DrivePosFinish = true;
    }
    else if ((VelocityExp_left < 0.0f)&&(VelocityExp_right < 0.0f))
    {
        if (radius_temp > 0)
        {
            DriveRatio.LDNeg = VelocityCurr.LWNeg / VelocityExp_left - 1.0;
            DriveRatio.RDNeg = VelocityCurr.RWNeg / VelocityExp_right - 1.0;
            ROS_INFO("+DriveLNeg = %f", DriveRatio.LDPos);
            ROS_INFO("+DriveRNeg = %f", DriveRatio.RDPos);
        }
        else if (radius_temp < 0)
        {
            DriveRatio.LDNeg = VelocityCurr.LWNeg / VelocityExp_left - 1.0;
            DriveRatio.RDNeg = VelocityCurr.RWNeg / VelocityExp_right - 1.0;
            ROS_INFO("-DriveLNeg = %f", DriveRatio.LDNeg);
            ROS_INFO("-DriveRNeg = %f", DriveRatio.RDNeg);
        }
        DriveNegFinish = true;
    }
}

void Odometry_Adjust::Auto_Odom_FbackV(const agv_solver::Base_to_Model& base_to_model)
{
    this->base_to_model = base_to_model;
}

void Odometry_Adjust::FeedbackOffset()
{
    VelocityFb_left = 0.5f * (VelocityFb_left1 + VelocityFb_left2);
    VelocityFb_right = 0.5f * (VelocityFb_right1 + VelocityFb_right2);
    ROS_INFO("VelocityFb_left = %f", VelocityFb_left);
    ROS_INFO("VelocityFb_right = %f", VelocityFb_right);
    VelocityExp_left = getParam<float>("VelocityL",0);
    VelocityExp_right = getParam<float>("VelocityR",0);
    if ((VelocityExp_left > 0.0f)&&(VelocityExp_right > 0.0f))
    {
        FbRatio.LFbPos = VelocityCurr.LWPos / VelocityFb_left - 1.0;
        FbRatio.RFbPos = VelocityCurr.RWPos / VelocityFb_right - 1.0;
        ROS_INFO("FbLPos = %f", FbRatio.LFbPos);
        ROS_INFO("FbRPos = %f", FbRatio.RFbPos);
        FdbkPosFinish = true;
    }
    else if ((VelocityExp_left < 0.0f)&&(VelocityExp_right < 0.0f))
    {
        FbRatio.LFbNeg = VelocityCurr.LWNeg / VelocityFb_left - 1.0;
        FbRatio.RFbNeg = VelocityCurr.RWNeg / VelocityFb_right - 1.0;
        ROS_INFO("FbLNeg = %f", FbRatio.LFbNeg);
        ROS_INFO("FbRNeg = %f", FbRatio.LFbNeg);
        FdbkNegFinish = true;
    }
}
void Odometry_Adjust::Odom_Clac_Finish()
{
    if ((getParam<float>("VelocityL",0) > 0.0) && (getParam<float>("VelocityR",0) > 0.0))
    {
        if ((DrivePosFinish == true) && (FdbkPosFinish == true))
        {
            Init();
        }
    }
    else if ((getParam<float>("VelocityL",0) < 0.0) && (getParam<float>("VelocityR",0) < 0.0))
    {
        if ((DriveNegFinish == true) && (FdbkNegFinish == true))
        {
            Init();
        }
    }

//    if ((DrivePosFinish == true) && (FdbkPosFinish == true) && (DriveNegFinish == true) && (FdbkNegFinish == true))
  //  {
        YAML::Node config=YAML::LoadFile("Offset_param.yaml");

        ofstream fout;
        fout.open("Offset_param.yaml",ios::out);
        fout<<"HaHaHaHA:"<<endl;
        for(YAML::const_iterator it= config["DriveOffset"].begin(); it!= config["DriveOffset"].end();++it)
        {
            if ((it->first.as<string>())=="DriveOffsetLeftPos")
            {
                fout<<" "<<(it->first.as<string>())<<":"<<" "<<DriveRatio.LDPos<<endl;
            }
            else if ((it->first.as<string>())=="DriveOffsetRightPos")
            {
                fout<<" "<<(it->first.as<string>())<<":"<<" "<<DriveRatio.RDPos<<endl;
            }
            else if ((it->first.as<string>())=="DriveOffsetLeftNeg")
            {
                fout<<" "<<(it->first.as<string>())<<":"<<" "<<DriveRatio.LDNeg<<endl;
            }
            else if ((it->first.as<string>())=="DriveOffsetRightNeg")
            {
                fout<<" "<<(it->first.as<string>())<<":"<<" "<<DriveRatio.RDNeg<<endl;
            }
            else
            {
                fout<<" "<<(it->first.as<string>())<<":"<<" "<<(it->second.as<double>())<<endl;
            }
        }
        fout<<'\n';
        fout<<"FeedbackOffset:"<<endl;
        for(YAML::const_iterator it= config["FeedbackOffset"].begin(); it!= config["FeedbackOffset"].end();++it)
        {
            if ((it->first.as<string>())=="FeedbackOffsetLeftPos")
            {
                fout<<" "<<(it->first.as<string>())<<":"<<" "<<FbRatio.LFbPos<<endl;
            }
            else if ((it->first.as<string>())=="FeedbackOffsetRightPos")
            {
                fout<<" "<<(it->first.as<string>())<<":"<<" "<<FbRatio.RFbPos<<endl;
            }
            else if ((it->first.as<string>())=="FeedbackOffsetLeftNeg")
            {
                fout<<" "<<(it->first.as<string>())<<":"<<" "<<FbRatio.LFbNeg<<endl;
            }
            else if ((it->first.as<string>())=="FeedbackOffsetRightNeg")
            {
                fout<<" "<<(it->first.as<string>())<<":"<<" "<<FbRatio.RFbNeg<<endl;
            }
            else
            {
                fout<<" "<<(it->first.as<string>())<<":"<<" "<<(it->second.as<double>())<<endl;
            }
        }
        fout.close();
   // }
}
