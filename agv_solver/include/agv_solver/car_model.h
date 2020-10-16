#ifndef __CAR_MODEL_H
#define __CAR_MODEL_H


using namespace std;
#include <string>
#include <ros/ros.h>
#include <math.h>
#include "body_info.h"

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

#define  PP (2.0*M_PI)
#define  AngleNormal(T) (T>M_PI)?(T-PP):(T<-M_PI?(T+PP):T)

typedef struct tagCoordCalcW
{
    float DeltaTheta;
    float DeltaTime;
}CoordCalcW;

typedef struct tagCoordVelCurr
{
	float LWPos;
	float RWPos;
    float LWNeg;
    float RWNeg;
}CoordVelCurr;

typedef struct tagCoordDRatio
{
	float LDPos;
	float RDPos;
    float LDNeg;
    float RDNeg;
}CoordDRatio;

typedef struct tagCoordFbRatio
{
    float LFbPos;
    float RFbPos;
    float LFbNeg;
    float RFbNeg;
}CoordFbRatio;

class BaseSolver
{
public:
  BaseSolver()
  {
    GetModelParam();
  }
	void BaseSolution();

	void KivaSolution();
	void TwoDiffSolution();
	void FourDiffSolution();
	void OneHelmSolution();
	void TwoHelmSolution();
	void FourHelmSolution();
	void MecanumSolution();
	void JointMecanSolution();
	void CarlikeSolution();

	void BaseOdometer();

	void KivaOdometer();
	void TwoDiffOdometer();
	void FourDiffOdometer();
	void OneHelmOdometer();
	void TwoHelmOdometer();
	void FourHelmOdometer();
	void MecanumOdometer();
	void JointMecanOdometer();
	void CarlikeOdometer();
  void Odometer(const agv_solver::VW_Center vw, Odo_t &odo_cur);
  void GetModelParam();

  void DataProc(agv_solver::Base_to_Model base_to_model)
  {
    this->base_to_model = base_to_model;
	if (base_to_model.vw_exp.Mode != 1)
	{
   		BaseSolution();		
	}
	else
	{
		model_to_base.one_diff.Vl = 0;
		model_to_base.one_diff.Vr = 0;
	}
    BaseOdometer();
  }
private:
	BodyParam_t base_para;
  int Imu_used;
public:
  double angle_velocity;
  CoordDRatio  DriveRatio;
  CoordFbRatio FbRatio; 
	agv_solver::Base_to_Model base_to_model;
	agv_solver::Model_to_Base model_to_base;
	agv_solver::DualMGM_to_plc msg_to_plc;
};

void VeloCalc_Diff(agv_solver::VW_Center vw, const BodyKiva_t &para, agv_solver::OneDiff_to_Base &out_kiva);
void VeloCalc_TwoDriver(agv_solver::VW_Center vw,const BodyDiff_t &para,agv_solver::TwoHelm_to_Base &out_twohelm);
void VeloCalc_Mecanum(agv_solver::VW_Center vw,const BodyMecanum_t &para,agv_solver::Mecanum_to_Base &out_mecanum);

//void Odometer(const agv_solver::VW_Center vw, Odo_t &odo_cur);
void Odometry_Trans(const agv_solver::VW_Center &vw, const Odo_t &odo_pose, nav_msgs::Odometry &odom_nav);
void OdomCalc_TwoDriver(const BodyHelm_t &para,const agv_solver::Base_to_TwoHelm &input_twohelm, agv_solver::VW_Center &VWm);


#endif
