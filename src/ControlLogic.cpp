/*
 * ControlLogic.cpp
 *
 *  Created on: 2013/08/14
 *      Author: sa
 */
#include "mathf.h"
#include "ControlLogic.hpp"
#include "GeneralConfig.hpp"
#include "Peripherals/OrientationFilter.hpp"
#include "Peripherals/Servo.hpp"

ControlLogic::ControlLogic(){
	pitch = role = yaw = throttle = flaps = 0.0;

	Storage::readGlideAttitude(&degGlidePitch,&degGlideRole);
	Storage::readGlideGain(&glidePitchGain,&glidePitchGainFloor,&glideMinSpeed,&glideRoleGain,&glideYawGain,&glideDeadtime);
	Storage::readGlideGyroGain(&glideGyroPitchGain,&glideGyroRoleGain,&glideGyroYawGain);
	degGlideHeading = 0;
	Storage::readFlareParam(&degFlareAngle,&flareGain,&mFlareStartHeight,&mFlareCompleteHeight);
}

void ControlLogic::fullManual(){
	HalServoMtus::getInstance()->setPitch(pitch);
	HalServoMtus::getInstance()->setRole(role);
	HalServoMtus::getInstance()->setYaw(yaw);
}

float ControlLogic::calcGlideGainCompensation(){
	float speed = 0;
	float glideGain;


	//calc speed.
	for(int i=0;i<3;i++){
		speed += inu->getE_FrameSpeed(i) * inu->getE_FrameSpeed(i);
	}
	speed = sqrtf(speed);

	//flooring speed.
	if(speed<glideMinSpeed){
		speed = glideMinSpeed;
	}

	glideGain = (1/(speed*speed) + glidePitchGainFloor);

	return glideGain;
}

void ControlLogic::glide(){
	static const float headingTau = 0.97;
	float gain;

	float degFlaredGlidePitch;
	float attPitch,attRole,attHeading;
	float pitchCommand,roleCommand,yawCommand;


	Quaternion attitude = OrientationFilter::getAttitude();
	attitude.getRadPitchRoleHeading(&attPitch,&attRole,&attHeading);

	attHeading+=QUAT_PI;

	if(degGlideHeading<0){
		degGlideHeading+=360;
	}else if(360<degGlideHeading){
		degGlideHeading-=360;
	}

	if(degGlideHeading<90 && 3*QUAT_PI/2<attHeading){
		attHeading-=2*QUAT_PI;
	}else if(270<degGlideHeading && attHeading < QUAT_PI/2){
		attHeading+=2*QUAT_PI;
	}

	degGlideHeading = (degGlideHeading*headingTau + attHeading*180/QUAT_PI*(1-headingTau));

	float mHeight = rangeFinder->getM_Range()-0.164;


	//monitor of range finder.
//	stdout->putFloat(degFlaredGlidePitch);
//	stdout->putString(",\t");
//	stdout->putFloat(rangeFinder->getM_Range());
//	stdout->putString("\n\r");

	//monitor of heding.
//	stdout->putFloat(attHeading*180/QUAT_PI);
//	stdout->putString(",");
//	stdout->putFloat(degGlideHeading);
//	stdout->putString("\n\r");

	pitchCommand = glidePitchGain * calcGlideGainCompensation() * (attPitch - degGlidePitch*3.14/180);
	roleCommand =  glideRoleGain * (attRole - degGlideRole*3.14/180);
	yawCommand = glideYawGain * (attHeading - degGlideHeading*3.14/180);


	if(mHeight < mFlareCompleteHeight){
		pitchCommand += flareGain * (attPitch - degFlareAngle*3.14/180);
	}else if(mHeight < mFlareStartHeight){
		pitchCommand += flareGain * (attPitch - degFlareAngle*3.14/180) * (mFlareStartHeight - mHeight)/(mFlareStartHeight-mFlareCompleteHeight);
	}


	pitchCommand += glideGyroPitchGain * calcGlideGainCompensation()* (mpu9150->getRpsGyro(1));
	roleCommand += glideGyroRoleGain * (mpu9150->getRpsGyro(0));
	yawCommand += glideGyroYawGain * (mpu9150->getRpsGyro(2));

	HalServoMtus::getInstance()->setPitch(pitchCommand);
	HalServoMtus::getInstance()->setRole(-roleCommand);
	HalServoMtus::getInstance()->setYaw(yawCommand);
}
