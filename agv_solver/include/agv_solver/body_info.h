#ifndef __BODY_INFO_H
#define __BODY_INFO_H	

#include <stdint.h>
/*car model configuration*/
typedef enum ModelCfg {
	One_Diff = 1,   
	Two_Diff = 2,  
	Four_Diff = 3,   
	One_Helm = 4,	
	Two_Helm = 5,	
	Four_Helm = 6,   
	Mecanum = 7,   
	JointMecan = 8,   
	Carlike = 9
}ModelCfg_t;

/*body info of kiva or one-diff base*/
typedef struct Body_Kiva
{
	/****
	*infomation about sigle wheel or among wheels
	****/
  double WheelDiameter;
  double WheelBase;
}BodyKiva_t;

/*body info of all diff base*/
typedef struct Body_Diff
{
	/****
	*infomation about sigle kiva-base
	****/
	BodyKiva_t KivaBase;

	/****
	*layout of all driver modules
	*Orientation 0:middle 1:titl-left 2:titl-right
	*distance form wheel to body center along the longitudinal orientation
	*distance form wheel to body center along the lateral orientation
	****/
	uint8_t Orientation;
  double Longitudinal;
  double Lateral;
}BodyDiff_t;


/*body info of all helm base*/
typedef struct Body_Helm
{
	/****
	*layout of all driver modules
	*Orientation 0:middle 1:titl-left 2:titl-right
	*distance form wheel to body center along the longitudinal orientation
	*distance form wheel to body center along the lateral orientation
	****/
	uint8_t Orientation;
  double Longitudinal;
  double Lateral;
}BodyHelm_t;

/*body info of Mecanum base*/
typedef struct Body_Mecanum
{
	/****
	*layout of all driver modules
	*Orientation 0:middle 1:titl-left 2:titl-right
	*distance form wheel to body center along the longitudinal orientation
	*distance form wheel to body center along the lateral orientation
	****/
	uint8_t Orientation;
  double Longitudinal;
  double Lateral;
}BodyMecanum_t;

/*body info of Joint-Mecanum base*/
typedef struct Body_JointMecan
{
	BodyMecanum_t MecanumBase;
	/*distance between two mecanum-base agv along the longitudinal orientation*/
  double CarDis;
}BodyJointMecan_t;

typedef struct Body_Carlike
{
  double Longitudinal;
}BodyCarlike_t;

typedef struct Odo
{
  Odo():X(0.0),Y(0.0),Th(0.0){}
	void Initial(){X=0.0;Y=0.0;Th=0.0;}
  double X;
  double Y;
  double Th;
}Odo_t;

/*Body parameter for vehicle based different base*/
typedef struct Body_Param
{
	ModelCfg_t model_type;

	BodyKiva_t Kiva;
	BodyDiff_t Diff;
	BodyHelm_t Helm;
	BodyMecanum_t Mecanum;
	BodyJointMecan_t JointMecan;
	BodyCarlike_t Carlike;
}BodyParam_t;

#endif
