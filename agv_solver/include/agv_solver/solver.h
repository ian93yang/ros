#ifndef SOLVER_H
#define SOLVER_H

#include <ros/ros.h>
#include <stdint.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "agv_solver/car_model.h"
#include "agv_solver/auto_odometry.h"
#include "agv_solver/LocalizationResult.h"//A发出实时坐标

using namespace sensor_msgs;
using namespace message_filters;

class SubscribeAngPublish
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub_;
  ros::Publisher pub_old;
  ros::Subscriber sub5;
  ros::Subscriber sub_base;
  ros::Subscriber sub_imu;
  agv_solver::Model_to_Base output;
  agv_solver::DualMGM_to_plc odometry_old;
  BaseSolver *car_model;
  Odometry_Adjust *auto_odom;
public:
  static double angle_velocity;
public:
  SubscribeAngPublish();
  ~SubscribeAngPublish();
  void callback1(const agv_solver::Base_to_Model::ConstPtr &base_to_model);
  void callback2(const ImuConstPtr &imu_msg);
  void SetPubMsg(agv_solver::Model_to_Base &model_to_base , agv_solver::DualMGM_to_plc &odometry);
  //void SetPubMsg(agv_solver::Model_to_Base &model_to_base);
  void chatterCallback5(const agv_solver::LocalizationResult::ConstPtr& msg);//控制计算功能
};

#endif // SOLVER_H
