#include "agv_solver/solver.h"
//#include "agv_solver/LocalizationResult.h"//A发出实时坐标
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

#include <string>
using namespace std;
double SubscribeAngPublish::angle_velocity =0.0;

SubscribeAngPublish::SubscribeAngPublish()
{
  pub_ = nh.advertise<agv_solver::Model_to_Base>("Model_to_Base", 1);
  pub_old = nh.advertise<agv_solver::DualMGM_to_plc>("DualMGM_2_plc",1);
  sub5 = nh.subscribe("LocalizationResult", 1000, &SubscribeAngPublish::chatterCallback5, this);
  //sub5 = nh.subscribe("LocalizationResult", 1000, &SubscribeAngPublish::chatterCallback5, this);
  sub_base = nh.subscribe("Base_to_Model", 1, &SubscribeAngPublish::callback1, this);
  sub_imu = nh.subscribe("IMU_data", 1, &SubscribeAngPublish::callback2, this);
  car_model = new BaseSolver();
  auto_odom = new Odometry_Adjust();
}

SubscribeAngPublish::~SubscribeAngPublish()
{
  delete car_model;
  delete auto_odom;
}

void SubscribeAngPublish::callback1(const agv_solver::Base_to_Model::ConstPtr &base_to_model)
{
  auto_odom->Auto_Odom_FbackV(*base_to_model);//传入订阅的消息
  ROS_INFO("FbVelocityL = %f", base_to_model->one_diff.Vl);
  ROS_INFO("FbVelocityR= %f", base_to_model->one_diff.Vr);
  ROS_INFO("mode = %d", base_to_model->vw_exp.Mode);
  car_model->DataProc(*base_to_model);
  SetPubMsg(car_model->model_to_base ,car_model->msg_to_plc);
  ROS_INFO("++++++++++++++++++++++++++++");
  ROS_INFO("++++++++++++++++++++++++++++");
  ROS_INFO("++++++++++++++++++++++++++++");
  ROS_INFO("Solver node is running!!!");
  ROS_INFO("Vx_expect = %f", base_to_model->vw_exp.Vx);
  ROS_INFO("Vy_expect = %f", base_to_model->vw_exp.Vy);
  ROS_INFO("W_expect = %f", base_to_model->vw_exp.W);
  
  pub_.publish(output);
  pub_old.publish(odometry_old);

  if (base_to_model->vw_exp.Mode == 1)//调试模式
  {
    /*static uint8_t enable_former = 0;
    if ((base_to_model->vw_exp.Enable)&&(enable_former == 0))
    {
      auto_odom->Init();
    }
    enable_former = base_to_model->vw_exp.Enable;*/
    ROS_INFO("Enable = %d", base_to_model->vw_exp.Enable);
    ROS_INFO("VelocityL = %f", getParam<float>("VelocityL",0));
    ROS_INFO("VelocityR = %f", getParam<float>("VelocityR",0));
    if (fabs(getParam<float>("RadiusCurr",0)) < 0.01)
    //if (fabs(getParam<float>("RadiusCurr",0) < 0.01) && (base_to_model->vw_exp.Enable == 1))
    {
      ROS_INFO("RadiusBefore = %f", getParam<float>("RadiusCurr",0));
      auto_odom->CalcAngleVelocity();
    }
    else if (fabs(getParam<float>("RadiusCurr",0)) >= 0.01)
    //else if (fabs(getParam<float>("RadiusCurr",0) >= 0.01) && (base_to_model->vw_exp.Enable == 0))
    {
      ROS_INFO("RadiusAfter = %f", getParam<float>("RadiusCurr",0));
      auto_odom->CalcVelocityCurr();
      auto_odom->DriveVelocityRatio();
      auto_odom->FeedbackOffset();
      auto_odom->Odom_Clac_Finish();
      
      if ((getParam<float>("VelocityL",0) > 0.0) && (getParam<float>("VelocityR",0) > 0.0))
      {
        car_model->DriveRatio.LDPos = auto_odom->DriveRatio.LDPos;
        car_model->DriveRatio.RDPos = auto_odom->DriveRatio.RDPos;

        car_model->FbRatio.LFbPos = auto_odom->FbRatio.LFbPos;
        car_model->FbRatio.RFbPos = auto_odom->FbRatio.RFbPos;
      }
      else if ((getParam<float>("VelocityL",0) < 0.0) && (getParam<float>("VelocityR",0) < 0.0))
      {
        car_model->DriveRatio.LDNeg = auto_odom->DriveRatio.LDNeg;
        car_model->DriveRatio.RDNeg = auto_odom->DriveRatio.RDNeg;

        car_model->FbRatio.LFbNeg = auto_odom->FbRatio.LFbNeg;
        car_model->FbRatio.RFbNeg = auto_odom->FbRatio.RFbNeg;
      }
      else
      {
        car_model->DriveRatio.LDPos = 0.0;
        car_model->DriveRatio.RDPos = 0.0;

        car_model->FbRatio.LFbNeg = 0.0;
        car_model->FbRatio.RFbNeg = 0.0;
      }
    }
  }
}

void SubscribeAngPublish::callback2(const ImuConstPtr &imu_msg)
{
  angle_velocity = imu_msg->angular_velocity.z;
  car_model->angle_velocity = angle_velocity;
}

void SubscribeAngPublish::SetPubMsg(agv_solver::Model_to_Base &model_to_base, agv_solver::DualMGM_to_plc &odometry)
{
  output = model_to_base;
  odometry_old = odometry;
}

/*void SubscribeAngPublish::SetPubMsg(agv_solver::Model_to_Base &model_to_base)
{
  output = model_to_base;
}*/

void SubscribeAngPublish::chatterCallback5(const agv_solver::LocalizationResult::ConstPtr& msg)//控制计算功能
{
  //AfxGetApp->m_BezierData->Location.x=1000.0*msg->pose_with_covariance.pose.position.x;
  //AfxGetApp->m_BezierData->Location.y=1000.0*msg->pose_with_covariance.pose.position.y;
  tf::Quaternion quat;
  double roll, pitch, yaw;
  tf::quaternionMsgToTF(msg->pose_with_covariance.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  auto_odom->Auto_Odom_Proc(yaw);
}

int main(int argc,char ** argv)
{
  ROS_INFO("agv_solver start!");
  ros::init(argc, argv, "solver");
  SubscribeAngPublish sub_pub;
  //double dia = getParam<double>("OneDiff_Diameter",0.0);
  //ROS_INFO("OneDiff_Diameter = %f",dia);
  ros::spin();
  return 0;
}
