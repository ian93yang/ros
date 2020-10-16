#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "agv_solver/car_model.h"
#include "math.h"
#include <sensor_msgs/Imu.h>
#include "agv_solver/solver.h"

void BaseSolver::BaseSolution()
{
	switch(base_para.model_type){
		case One_Diff:
			KivaSolution();
			break;
		case Two_Diff:
			TwoDiffSolution();
			break;
		case Four_Diff:
			FourDiffSolution();
			break;
		case One_Helm:
			OneHelmSolution();
			break;
		case Two_Helm:
			TwoHelmSolution();
			break;
		case Four_Helm:
			FourHelmSolution();
			break;
		case Mecanum:
			MecanumSolution();
			break;
		case JointMecan:
			JointMecanSolution();
			break;
		case Carlike:
			CarlikeSolution();
			break;
	}
}

void BaseSolver::BaseOdometer()
{
	switch(base_para.model_type){
		case One_Diff:
			KivaOdometer();
			break;
		case Two_Diff:
			TwoDiffOdometer();
			break;
		case Four_Diff:
			FourDiffOdometer();
			break;
		case One_Helm:
			OneHelmOdometer();
			break;
		case Two_Helm:
			TwoHelmOdometer();
			break;
		case Four_Helm:
			FourHelmOdometer();
			break;
		case Mecanum:
			MecanumOdometer();
			break;
		case JointMecan:
			JointMecanSolution();
			break;
		case Carlike:
			CarlikeSolution();
			break;
	}
}

void VeloCalc_Diff(agv_solver::VW_Center vw, const BodyKiva_t &para, agv_solver::OneDiff_to_Base &out_kiva)
{
  out_kiva.Vl = vw.Vx - vw.W*para.WheelBase/2.0;
  out_kiva.Vr = vw.Vx + vw.W*para.WheelBase/2.0;
}

void BaseSolver::KivaSolution()
{
  VeloCalc_Diff(base_to_model.vw_exp, base_para.Kiva, model_to_base.one_diff);
  ROS_INFO("LDPos=%f",DriveRatio.LDPos);
  ROS_INFO("LDNeg=%f",DriveRatio.LDNeg);
  ROS_INFO("RDPos=%f",DriveRatio.RDPos);
  ROS_INFO("RDNeg=%f",DriveRatio.RDNeg);
  if(model_to_base.one_diff.Vl>=0)
  {
	  model_to_base.one_diff.Vl = model_to_base.one_diff.Vl*(1 + DriveRatio.LDPos);
  }
  else
  {
	  model_to_base.one_diff.Vl = model_to_base.one_diff.Vl*(1 + DriveRatio.LDNeg);
  }

  if(model_to_base.one_diff.Vr>=0)
  {
	  model_to_base.one_diff.Vr = model_to_base.one_diff.Vr*(1 + DriveRatio.RDPos);
  }
  else
  {
	  model_to_base.one_diff.Vr = model_to_base.one_diff.Vr*(1 + DriveRatio.RDNeg);
  }
  ROS_INFO("kiva model solution,vl=%f,vr=%f",model_to_base.one_diff.Vl,model_to_base.one_diff.Vr);
  ROS_INFO("=========================");
}

void VeloCalc_TwoDriver(agv_solver::VW_Center vw,const BodyDiff_t &para,agv_solver::TwoHelm_to_Base &out_twohelm)
{
  double Length,Width,Dis,Phi_offset;
  Length = 2.0*para.Longitudinal;
  Width = 2.0*para.Lateral;
  Dis = sqrt(Length*Length+Width*Width);

	Phi_offset = (para.Orientation==1)?asin(Width/Dis):((para.Orientation==2)?(-asin(Width/Dis)):0);
  double Vmx,Vmy,Wm;
	Vmx = vw.Vx*cos(Phi_offset) + vw.Vy*sin(Phi_offset);
	Vmy = -vw.Vx*sin(Phi_offset) + vw.Vy*cos(Phi_offset);
	Wm = vw.W;

  double Vmx1,Vmy1,Vmx2,Vmy2;
	Vmx1 = Vmx;
	Vmx2 = Vmx;
	Vmy1 = Vmy + Wm*Dis/2.0;
	Vmy2 = Vmy - Wm*Dis/2.0;

  double Ang1_m,Ang2_m;

	Ang1_m = fabs(Vmx1)>0.001?atan(Vmy1/Vmx1):((Vmy1>0.001)?1.57:(-1.57));
	Ang2_m = fabs(Vmx2)>0.001?atan(Vmy2/Vmx2):((Vmy2>0.001)?1.57:(-1.57));

  double Ang1_res,Ang2_res,V1_res,V2_res;
  double Vx1_res,Vx2_res,Vy1_res,Vy2_res;
	Ang1_res = Ang1_m + Phi_offset;
	Ang2_res = Ang2_m + Phi_offset;
	Vx1_res = Vmx1*cos(Phi_offset) - Vmy1*sin(Phi_offset);
	Vy1_res = Vmx1*sin(Phi_offset) + Vmy1*cos(Phi_offset);
	Vx2_res = Vmx2*cos(Phi_offset) - Vmy2*sin(Phi_offset);
	Vy2_res = Vmx2*sin(Phi_offset) + Vmy2*cos(Phi_offset);	

	V1_res = (fabs(Ang1_res)>0.001)?Vx1_res:Vy1_res/sin(Ang1_res);
	V2_res = (fabs(Ang2_res)>0.001)?Vx2_res:Vy2_res/sin(Ang2_res);

	out_twohelm.V1 = V1_res;
	out_twohelm.V2 = V2_res;
	out_twohelm.Ang1 = Ang1_res;
	out_twohelm.Ang2 = Ang2_res;
}

void BaseSolver::TwoDiffSolution()
{
	agv_solver::TwoHelm_to_Base virtual_res;
  VeloCalc_TwoDriver(base_to_model.vw_exp, base_para.Diff, virtual_res);
	
  double W1_res,W2_res;
	W1_res = virtual_res.Ang1 - base_to_model.two_diff.Ang1;
	W2_res = virtual_res.Ang2 - base_to_model.two_diff.Ang2;
	
	model_to_base.two_diff.Vl_1 = virtual_res.V1 - W1_res*base_para.Diff.KivaBase.WheelBase/2.0;
	model_to_base.two_diff.Vr_1 = virtual_res.V1 + W1_res*base_para.Diff.KivaBase.WheelBase/2.0;
	model_to_base.two_diff.Vl_2 = virtual_res.V2 - W2_res*base_para.Diff.KivaBase.WheelBase/2.0;
	model_to_base.two_diff.Vr_2 = virtual_res.V2 + W2_res*base_para.Diff.KivaBase.WheelBase/2.0;
}

void BaseSolver::FourDiffSolution()
{
	agv_solver::TwoHelm_to_Base virtual_res1;
	agv_solver::TwoHelm_to_Base virtual_res2;
	//titl left
	base_para.Diff.Orientation = 1;
  VeloCalc_TwoDriver(base_to_model.vw_exp, base_para.Diff, virtual_res1);
	//titl right
	base_para.Diff.Orientation = 2;
  VeloCalc_TwoDriver(base_to_model.vw_exp, base_para.Diff, virtual_res2);

  double W1_res,W2_res,W3_res,W4_res;
	W1_res = virtual_res1.Ang1 - base_to_model.four_diff.Ang1;
	W2_res = virtual_res2.Ang1 - base_to_model.four_diff.Ang2;
	W3_res = virtual_res1.Ang2 - base_to_model.four_diff.Ang3;
	W4_res = virtual_res2.Ang2 - base_to_model.four_diff.Ang4;

	model_to_base.four_diff.Vl_1 = virtual_res1.V1 - W1_res*base_para.Diff.KivaBase.WheelBase/2.0;
	model_to_base.four_diff.Vr_1 = virtual_res1.V1 + W1_res*base_para.Diff.KivaBase.WheelBase/2.0;
	model_to_base.four_diff.Vl_2 = virtual_res2.V1 - W2_res*base_para.Diff.KivaBase.WheelBase/2.0;
	model_to_base.four_diff.Vr_2 = virtual_res2.V1 + W2_res*base_para.Diff.KivaBase.WheelBase/2.0;
	model_to_base.four_diff.Vl_3 = virtual_res1.V2 - W3_res*base_para.Diff.KivaBase.WheelBase/2.0;
	model_to_base.four_diff.Vr_3 = virtual_res1.V2 + W3_res*base_para.Diff.KivaBase.WheelBase/2.0;
	model_to_base.four_diff.Vl_4 = virtual_res2.V2 - W4_res*base_para.Diff.KivaBase.WheelBase/2.0;
	model_to_base.four_diff.Vr_4 = virtual_res2.V2 + W4_res*base_para.Diff.KivaBase.WheelBase/2.0;
}

void BaseSolver::OneHelmSolution()
{
  double Longitudinal,Lateral,Vx,W,Th,Vec;
  Vx = base_to_model.vw_exp.Vx;
  W = base_to_model.vw_exp.W;
	Longitudinal = base_para.Helm.Longitudinal;
	Lateral = (base_para.Helm.Orientation==2)?(-base_para.Helm.Lateral):base_para.Helm.Lateral;
	if (fabs(W)<=0.001){
		Vec = Vx;
		Th = 0;
	}else{
		Th = atan(W/(Vx-W*Lateral));
		//data boundary protect
		if(Vx/W/fabs(Vx/W)>0.5 && Th<-1.0)
			Th = -Th;
		else if(Vx/W/fabs(Vx/W)<-0.5 && Th>1.0)
			Th = -Th;		
	}
	model_to_base.one_helm.Ang = Th;
	model_to_base.one_helm.V = W*Longitudinal/sin(Th);
}

void BaseSolver::TwoHelmSolution()
{
	BodyDiff_t para;
	para.Orientation = base_para.Helm.Orientation;
	para.Longitudinal = base_para.Helm.Longitudinal;
	para.Lateral = base_para.Helm.Lateral;
  VeloCalc_TwoDriver(base_to_model.vw_exp, para, model_to_base.two_helm);
}

void BaseSolver::FourHelmSolution()
{
	BodyDiff_t para;
	//para.Orientation = base_para.Helm.Orientation;
	para.Longitudinal = base_para.Helm.Longitudinal;
	para.Lateral = base_para.Helm.Lateral;

	agv_solver::TwoHelm_to_Base virtual_res1;
	agv_solver::TwoHelm_to_Base virtual_res2;
	//titl left
	para.Orientation = 1;
  VeloCalc_TwoDriver(base_to_model.vw_exp, para, virtual_res1);
	//titl right
	base_para.Diff.Orientation = 2;
  VeloCalc_TwoDriver(base_to_model.vw_exp, para, virtual_res2);

	model_to_base.four_helm.V1 = virtual_res1.V1;
	model_to_base.four_helm.Ang1 = virtual_res1.Ang1;
	model_to_base.four_helm.V2 = virtual_res2.V1;
	model_to_base.four_helm.Ang2 = virtual_res2.Ang2;
	model_to_base.four_helm.V3 = virtual_res1.V2;
	model_to_base.four_helm.Ang3 = virtual_res1.Ang2;
	model_to_base.four_helm.V4 = virtual_res2.V2;
	model_to_base.four_helm.Ang4 = virtual_res2.Ang2;
}

void VeloCalc_Mecanum(agv_solver::VW_Center vw,const BodyMecanum_t &para,agv_solver::Mecanum_to_Base &out_mecanum)
{
  double Length,Width;
  Length = 2.0*para.Longitudinal;
  Width = 2.0*para.Lateral;

	out_mecanum.V1 = vw.Vy - vw.Vx + vw.W*(Length+Width)/2.0;
	out_mecanum.V2 = vw.Vy - vw.Vx - vw.W*(Length+Width)/2.0;
	out_mecanum.V3 = vw.Vy + vw.Vx - vw.W*(Length+Width)/2.0;
	out_mecanum.V4 = vw.Vy + vw.Vx + vw.W*(Length+Width)/2.0;
}

void BaseSolver::MecanumSolution()
{
  VeloCalc_Mecanum(base_to_model.vw_exp, base_para.Mecanum, model_to_base.mecanum);
}

void BaseSolver::JointMecanSolution()
{
  double Dis = base_para.JointMecan.CarDis;
	agv_solver::VW_Center vw1,vw2;
	
  vw1.Vx = base_to_model.vw_exp.Vx;
  vw2.Vx = base_to_model.vw_exp.Vx;
  vw1.Vy = base_to_model.vw_exp.Vy + Dis * base_to_model.vw_exp.W;
  vw2.Vy = base_to_model.vw_exp.Vy - Dis * base_to_model.vw_exp.W;
	vw1.W = 0.0;
	vw2.W = 0.0;

	agv_solver::Mecanum_to_Base mecanum1,mecanum2;
	BodyMecanum_t para;
	para.Longitudinal = base_para.JointMecan.MecanumBase.Longitudinal;
	para.Lateral = base_para.JointMecan.MecanumBase.Lateral;

	VeloCalc_Mecanum(vw1, para, mecanum1);
	VeloCalc_Mecanum(vw2, para, mecanum2);	

	model_to_base.joint_mecan.V1 = mecanum1.V1;
	model_to_base.joint_mecan.V2 = mecanum1.V2;
	model_to_base.joint_mecan.V3 = mecanum1.V3;
	model_to_base.joint_mecan.V4 = mecanum1.V4;
	model_to_base.joint_mecan.V5 = mecanum2.V1;
	model_to_base.joint_mecan.V6 = mecanum2.V2;
	model_to_base.joint_mecan.V7 = mecanum2.V3;
	model_to_base.joint_mecan.V8 = mecanum2.V4;
}

void BaseSolver::CarlikeSolution()
{
  double V,W,Length;
	Length = base_para.Carlike.Longitudinal;
  V = base_to_model.vw_exp.Vx;
  W = base_to_model.vw_exp.W;
	model_to_base.car_like.V = V;
	model_to_base.car_like.Ang = (fabs(W)<0.001)?0:((fabs(V)<0.1)?0:atan(Length*W/V));
}

void BaseSolver::Odometer(const agv_solver::VW_Center vw, Odo_t &odo_cur)
{
  static Odo_t odo_last;
	static int count = 0;
	static double t_now = 0.0;
	static double t_before = 0.0;
	double t_dur;

	t_now = ros::Time::now().toSec();
  t_dur = t_now - t_before;
  if (count <1)
  {
    odo_last.Initial();
    t_dur =0.0;
  }
  count++;
  if (count>10000)
    count =1;
	t_before = t_now;

  ROS_INFO("dur_time = %f",t_dur);
	agv_solver::VW_Center vw_tmp;
  //vw_tmp.W = vw.W;
  ROS_INFO("Imu_used = %d",Imu_used);
  angle_velocity = SubscribeAngPublish::angle_velocity;
  ROS_INFO("Imu angle_velocity = %f",angle_velocity);
  vw_tmp.W = (Imu_used==1)?angle_velocity:vw.W;
	vw_tmp.Vx = vw.Vx*cos(odo_last.Th) - vw.Vy*sin(odo_last.Th);
	vw_tmp.Vy = vw.Vx*sin(odo_last.Th) + vw.Vy*cos(odo_last.Th);

	odo_cur.Th = odo_last.Th + vw_tmp.W * t_dur;
	odo_cur.X = odo_last.X + vw_tmp.Vx * t_dur;
	odo_cur.Y = odo_last.Y + vw_tmp.Vy * t_dur;

  odo_cur.Th = AngleNormal(odo_cur.Th);
  ROS_INFO("odo_cur.Th = %f",odo_cur.Th);
  odo_last = odo_cur;
}

void Odometry_Trans(const agv_solver::VW_Center &vw, const Odo_t &odo_pose, nav_msgs::Odometry &odom_nav)
{
	ros::Time current_time;
	current_time = ros::Time::now();

	tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odo_pose.Th);
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = odo_pose.X;
  odom_trans.transform.translation.y = odo_pose.Y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster.sendTransform(odom_trans);

  odom_nav.header.stamp = current_time;
  odom_nav.header.frame_id = "odom";
  //set the position
	odom_nav.pose.pose.position.x = odo_pose.X;
	odom_nav.pose.pose.position.y = odo_pose.Y;
	odom_nav.pose.pose.position.z = 0.0;
  odom_nav.pose.pose.orientation = odom_quat;

  //set the velocity
  odom_nav.child_frame_id = "base_link";
  odom_nav.twist.twist.linear.x = vw.Vx;
  odom_nav.twist.twist.linear.y = vw.Vx;
  odom_nav.twist.twist.angular.z = vw.W;
}

void BaseSolver::KivaOdometer()
{
	agv_solver::VW_Center vw;
	Odo_t odo_pos;
    ROS_INFO("LFbPos=%f",FbRatio.LFbPos);
	ROS_INFO("LFbNeg=%f",FbRatio.LFbNeg);
	ROS_INFO("RFbPos=%f",FbRatio.RFbPos);
	ROS_INFO("RFbNeg=%f",FbRatio.RFbNeg);
	ROS_INFO("base_to_model.one_diff.Vl: %f",base_to_model.one_diff.Vl);
	ROS_INFO("base_to_model.one_diff.Vr: %f",base_to_model.one_diff.Vr);
	msg_to_plc.kiva_left_target_speed = base_to_model.one_diff.Vl;
	msg_to_plc.kiva_right_target_speed = base_to_model.one_diff.Vr;
	if(base_to_model.one_diff.Vl>=0)
	{
		base_to_model.one_diff.Vl = base_to_model.one_diff.Vl*(1 + FbRatio.LFbPos);
	}
	else
	{
		base_to_model.one_diff.Vl = base_to_model.one_diff.Vl*(1 + FbRatio.LFbNeg);
	}

	if(base_to_model.one_diff.Vr>=0)
	{
		base_to_model.one_diff.Vr = base_to_model.one_diff.Vr*(1 + FbRatio.RFbPos);
	}
	else
	{
		base_to_model.one_diff.Vr = base_to_model.one_diff.Vr*(1 + FbRatio.RFbNeg);
	}
	vw.Vx = (base_to_model.one_diff.Vl + base_to_model.one_diff.Vr)/2.0;
	vw.Vy = 0.0;
	vw.W  = (base_to_model.one_diff.Vr - base_to_model.one_diff.Vl)/base_para.Kiva.WheelBase;
	ROS_INFO("Vx_real: %f",vw.Vx);
	ROS_INFO("Vy_real: %f",vw.Vy);
	ROS_INFO("W_real: %f",vw.W);
	msg_to_plc.fVx = vw.Vx;
	msg_to_plc.fVy = vw.Vy;
	msg_to_plc.fW = vw.W;
	Odometer(vw, odo_pos);
	Odometry_Trans(vw, odo_pos, model_to_base.Odo);
	tf::Quaternion quat;
	tf::quaternionMsgToTF(model_to_base.Odo.pose.pose.orientation, quat);
	double roll,pitch,yaw;
	tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
	ROS_INFO("kiva odometer,x:%f,y:%f,theta:%f\n\n",model_to_base.Odo.pose.pose.position.x,model_to_base.Odo.pose.pose.position.y,yaw);
	msg_to_plc.fOdoX = model_to_base.Odo.pose.pose.position.x;
	msg_to_plc.fOdoY = model_to_base.Odo.pose.pose.position.y;
	msg_to_plc.fOdoPhi = yaw;
}

void OdomCalc_TwoDriver(const BodyHelm_t &para,const agv_solver::Base_to_TwoHelm &input_twohelm, agv_solver::VW_Center &VWm)
{
  double Length,Width,Dis,Phi_offset;
	Length = 2.0*para.Longitudinal;
	Width = 2.0*para.Lateral;
	Dis = sqrt(Length*Length+Width*Width);

	Phi_offset = (para.Orientation==1)?asin(Width/Dis):((para.Orientation==2)?(-asin(Width/Dis)):0);

  double V1, V2, Ang1, Ang2;
	V1 = input_twohelm.V1;
	V2 = input_twohelm.V2;
	Ang1 = input_twohelm.Ang1;
	Ang2 =  input_twohelm.Ang2;

	//helm angle in relation to the axis of the base
  double Th1_axis,Th2_axis;
	Th1_axis = (para.Orientation==1)?(Ang1-Phi_offset):(Ang1+Phi_offset);
	Th2_axis = (para.Orientation==1)?(Ang2-Phi_offset):(Ang2+Phi_offset);

	//calc VW based m coordinate.
	VWm.Vx = (V1*cos(Th1_axis)+V2*cos(Th2_axis))/2.0;
	VWm.Vy = (V1*sin(Th1_axis)+V2*sin(Th2_axis))/2.0;
	VWm.W = (V1*sin(Th1_axis)-V2*sin(Th2_axis))/Dis;
}

void BaseSolver::TwoDiffOdometer()
{
	BodyHelm_t para;
	agv_solver::Base_to_TwoHelm base_helm;
	para.Orientation = base_para.Diff.Orientation;
	para.Longitudinal = base_para.Diff.Longitudinal;
	para.Lateral = base_para.Diff.Lateral;
	base_helm.V1 = (base_to_model.two_diff.Vl_1 + base_to_model.two_diff.Vr_1)/2.0;
	base_helm.V2 = (base_to_model.two_diff.Vl_2 + base_to_model.two_diff.Vr_2)/2.0;
	base_helm.Ang1 = base_to_model.two_diff.Ang1;
	base_helm.Ang2 = base_to_model.two_diff.Ang2;
	
	agv_solver::VW_Center VWm;
	Odo_t odo_pos;
	OdomCalc_TwoDriver(para, base_helm, VWm);
	Odometer(VWm, odo_pos);
	Odometry_Trans(VWm, odo_pos, model_to_base.Odo);
}

void BaseSolver::FourDiffOdometer()
{
	BodyHelm_t para1,para2;
	agv_solver::Base_to_TwoHelm base_helm1,base_helm2;

	para1.Orientation = 1;
	para1.Longitudinal = base_para.Diff.Longitudinal;
	para1.Lateral = base_para.Diff.Lateral;
	base_helm1.V1 = (base_to_model.four_diff.Vl_1 + base_to_model.four_diff.Vr_1)/2.0;
	base_helm1.V2 = (base_to_model.four_diff.Vl_3 + base_to_model.four_diff.Vr_3)/2.0;
	base_helm1.Ang1 = base_to_model.four_diff.Ang1;
	base_helm1.Ang2 = base_to_model.four_diff.Ang3;

	para2.Orientation = 2;
	para2.Longitudinal = base_para.Diff.Longitudinal;
	para2.Lateral = base_para.Diff.Lateral;
	base_helm2.V1 = (base_to_model.four_diff.Vl_2 + base_to_model.four_diff.Vr_2)/2.0;
	base_helm2.V2 = (base_to_model.four_diff.Vl_4 + base_to_model.four_diff.Vr_4)/2.0;
	base_helm2.Ang1 = base_to_model.four_diff.Ang2;
	base_helm2.Ang2 = base_to_model.four_diff.Ang4;

	agv_solver::VW_Center VWm,VWm1,VWm2;
	Odo_t odo_pos;
	OdomCalc_TwoDriver(para1, base_helm1, VWm1);
	OdomCalc_TwoDriver(para2, base_helm2, VWm2);
	VWm.Vx = (VWm1.Vx + VWm2.Vx)/2.0;
	VWm.Vy = (VWm1.Vy + VWm2.Vy)/2.0;
	VWm.W = (VWm1.W + VWm2.W)/2.0;

	Odometer(VWm, odo_pos);
	Odometry_Trans(VWm, odo_pos, model_to_base.Odo);
}

void BaseSolver::OneHelmOdometer()
{
  double Longitudinal,Lateral;
  double V,Th;
	V = base_to_model.one_helm.V;
	Th = base_to_model.one_helm.Ang;
	Longitudinal = base_para.Helm.Longitudinal;
	Lateral = (base_para.Helm.Orientation==2)?(-base_para.Helm.Lateral):base_para.Helm.Lateral;

	agv_solver::VW_Center VW;
	Odo_t odo_pos;

	VW.Vx = V*(Longitudinal*cos(Th)+Lateral*sin(Th))/Longitudinal;
	VW.W = V*sin(Th)/Longitudinal;

	VW.Vy = 0.0;

	Odometer(VW, odo_pos);
	Odometry_Trans(VW, odo_pos, model_to_base.Odo);
}

void BaseSolver::TwoHelmOdometer()
{
	agv_solver::VW_Center VWm;
	Odo_t odo_pos;
	OdomCalc_TwoDriver(base_para.Helm, base_to_model.two_helm, VWm);
	Odometer(VWm, odo_pos);
	Odometry_Trans(VWm, odo_pos, model_to_base.Odo);
}

void BaseSolver::FourHelmOdometer()
{
	BodyHelm_t para1,para2;
	agv_solver::Base_to_TwoHelm base_helm1,base_helm2;

	para1.Orientation = 1;
	para1.Longitudinal = base_para.Helm.Longitudinal;
	para1.Lateral = base_para.Helm.Lateral;
	base_helm1.V1 = base_to_model.four_helm.V1;
	base_helm1.V2 = base_to_model.four_helm.V3;
	base_helm1.Ang1 = base_to_model.four_helm.Ang1;
	base_helm1.Ang2 = base_to_model.four_helm.Ang3;

	para2.Orientation = 2;
	para2.Longitudinal = base_para.Helm.Longitudinal;
	para2.Lateral = base_para.Helm.Lateral;
	base_helm2.V1 = base_to_model.four_helm.V2;
	base_helm2.V2 = base_to_model.four_helm.V4;
	base_helm2.Ang1 = base_to_model.four_helm.Ang2;
	base_helm2.Ang2 = base_to_model.four_helm.Ang4;

	agv_solver::VW_Center VWm,VWm1,VWm2;
	Odo_t odo_pos;
	OdomCalc_TwoDriver(para1, base_helm1, VWm1);
	OdomCalc_TwoDriver(para2, base_helm2, VWm2);
	VWm.Vx = (VWm1.Vx + VWm2.Vx)/2.0;
	VWm.Vy = (VWm1.Vy + VWm2.Vy)/2.0;
	VWm.W = (VWm1.W + VWm2.W)/2.0;

	Odometer(VWm, odo_pos);
	Odometry_Trans(VWm, odo_pos, model_to_base.Odo);
}

void BaseSolver::MecanumOdometer()
{
	BodyHelm_t para;
	agv_solver::Base_to_TwoHelm base_helm;
	//para.Orientation = base_para.Diff.Orientation;
	para.Longitudinal = base_para.Diff.Longitudinal;
	para.Lateral = base_para.Diff.Lateral;
	agv_solver::VW_Center VW;
	VW.Vx = (base_to_model.mecanum.V1 + base_to_model.mecanum.V2 + base_to_model.mecanum.V3 + base_to_model.mecanum.V4)/4.0;
	VW.Vy = (-base_to_model.mecanum.V1 + base_to_model.mecanum.V2 - base_to_model.mecanum.V3 + base_to_model.mecanum.V4)/4.0;
	VW.W = (-base_to_model.mecanum.V1 + base_to_model.mecanum.V2 + base_to_model.mecanum.V3 - base_to_model.mecanum.V4)/(para.Longitudinal + para.Lateral);

	Odo_t odo_pos;
	Odometer(VW, odo_pos);
	Odometry_Trans(VW, odo_pos, model_to_base.Odo);
}

void BaseSolver::JointMecanOdometer()
{
  double Longitudinal = base_para.JointMecan.MecanumBase.Longitudinal;
  double Lateral = base_para.JointMecan.MecanumBase.Lateral;
  double Dis = base_para.JointMecan.CarDis;

	agv_solver::VW_Center VW,VW_1,VW_2;
	VW_1.Vx = (base_to_model.joint_mecan.V1 + base_to_model.joint_mecan.V2 + base_to_model.joint_mecan.V3 + base_to_model.joint_mecan.V4)/4.0;
	VW_1.Vy = (-base_to_model.joint_mecan.V1 + base_to_model.joint_mecan.V2 - base_to_model.joint_mecan.V3 + base_to_model.joint_mecan.V4)/4.0;
	VW_1.W = (-base_to_model.joint_mecan.V1 + base_to_model.joint_mecan.V2 + base_to_model.joint_mecan.V3 - base_to_model.joint_mecan.V4)/(Longitudinal + Lateral);	

	VW_2.Vx = (base_to_model.joint_mecan.V5 + base_to_model.joint_mecan.V6 + base_to_model.joint_mecan.V7 + base_to_model.joint_mecan.V8)/4.0;
	VW_2.Vy = (-base_to_model.joint_mecan.V5 + base_to_model.joint_mecan.V6 - base_to_model.joint_mecan.V7 + base_to_model.joint_mecan.V8)/4.0;
	VW_2.W = (-base_to_model.joint_mecan.V5 + base_to_model.joint_mecan.V6 + base_to_model.joint_mecan.V7 - base_to_model.joint_mecan.V8)/(Longitudinal + Lateral);	

	VW.Vx = (VW_1.Vx + VW_2.Vx)/2.0;
	VW.Vy = (VW_1.Vy + VW_2.Vy)/2.0;
	VW.W = (VW_1.Vy - VW_2.Vy)/Dis;

	Odo_t odo_pos;
	Odometer(VW, odo_pos);
	Odometry_Trans(VW, odo_pos, model_to_base.Odo);
}

void BaseSolver::CarlikeOdometer()
{
	agv_solver::VW_Center VW;
  double Th = base_to_model.car_like.Ang;
  double Vec = base_to_model.car_like.V;

	VW.W = Vec*tan(Th)/base_para.Carlike.Longitudinal;
	VW.Vx = Vec;
	VW.Vy = 0;
	Odo_t odo_pos;
	Odometer(VW, odo_pos);
	Odometry_Trans(VW, odo_pos, model_to_base.Odo);
}

void BaseSolver::GetModelParam()
{
  Imu_used = getParam<int>("Imu",1);
  base_para.model_type = (ModelCfg)getParam<int>("model_cofig",1);
  switch (base_para.model_type) {
    case One_Diff:
      base_para.Kiva.WheelDiameter = getParam<double>("OneDiff_Diameter",200);
      base_para.Kiva.WheelBase = getParam<double>("OneDiff_Base",600);
    break;

    case Two_Diff:
    case Four_Diff:
      base_para.Diff.Orientation = uint8_t(getParam<int>("Orientation",0));
      base_para.Diff.Longitudinal = getParam<double>("Longitudinal",0.0);
      base_para.Diff.Lateral = getParam<double>("Lateral",0.0);
      base_para.Diff.KivaBase.WheelDiameter = getParam<double>("OneDiff_Diameter",0.0);
      base_para.Diff.KivaBase.WheelBase = getParam<double>("OneDiff_Base",0.0);
    break;

    case One_Helm:
    case Two_Helm:
    case Four_Helm:
      base_para.Helm.Orientation = uint8_t(getParam<int>("Orientation",0));
      base_para.Helm.Longitudinal = getParam<double>("Longitudinal",0.0);
      base_para.Helm.Lateral = getParam<double>("Lateral",0.0);
    break;

    case Mecanum:
      base_para.Mecanum.Longitudinal =getParam<double>("Longitudinal",0.0);
      base_para.Mecanum.Lateral =getParam<double>("Lateral",0.0);
    break;

    case JointMecan:
      base_para.JointMecan.CarDis = getParam<double>("CarDis",0.0);
      base_para.JointMecan.MecanumBase.Longitudinal = getParam<double>("Longitudinal",0.0);
      base_para.JointMecan.MecanumBase.Lateral = getParam<double>("Lateral",0.0);
    break;

    case Carlike:
    break;
  }
}
