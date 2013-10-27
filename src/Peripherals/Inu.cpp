#include "Inu.hpp"
#include "OrientationFilter.hpp"
#include "Quaternion.hpp"
#include "Mpu9150.hpp"
#include "../GeneralConfig.hpp"

Inu::Inu(){
	reset();
	posAutoZeroCoef=1.;
	spdAutoZeroCoef=1.;
}
Inu::~Inu(){

}



void Inu::update(){
	int i;
	float updateRate;
	Quaternion eFrameAcl;

	updateRate = mpu9150->getHzUpdateRate();

	eFrameAcl = getE_FrameAccel();

	//update speed.
	eFrameSpeed[0]+= 9.8*eFrameAcl.x/updateRate;
	eFrameSpeed[1]+= 9.8*eFrameAcl.y/updateRate;
	eFrameSpeed[2]+= 9.8*(eFrameAcl.z-1.0)/updateRate;

	//speed auto zero
	for(i=0;i<3;i++){
		eFrameSpeed[i] *= spdAutoZeroCoef;
	}

	for(i=0;i<3;i++){
		eFramePos[i]+=eFrameSpeed[i]/updateRate;

		eFramePos[i] *= posAutoZeroCoef;
	}
}
void Inu::reset(){
	int i;
	for(i=0;i<3;i++){
		eFramePos[i]=0.0;
		eFrameSpeed[i]=0.0;
	}
}
float Inu::getE_FramePos(int axis){
	return eFramePos[axis];
}
float Inu::getE_FrameSpeed(int axis){
	return eFrameSpeed[axis];
}
Quaternion Inu::getB_FrameSpeed(){
	Quaternion qSpeed(0,eFrameSpeed[0],eFrameSpeed[1],eFrameSpeed[2]);
	Quaternion attitude = OrientationFilter::getAttitude();

	attitude.con();
	qSpeed.rotate(attitude);

	return qSpeed;
}
Quaternion Inu::getE_FrameAccel(){
	Quaternion qAccel;
	Quaternion attitude = OrientationFilter::getAttitude();

	qAccel.w=0.0;
	qAccel.x=mpu9150->getG_Accel(0);
	qAccel.y=mpu9150->getG_Accel(1);
	qAccel.z=mpu9150->getG_Accel(2);

	qAccel.rotate(attitude);

	return qAccel;
}
