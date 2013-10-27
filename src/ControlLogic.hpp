/*
 * ControlLogic.hpp
 *
 *  Created on: 2013/08/14
 *      Author: sa
 */

#ifndef CONTROLLOGIC_HPP_
#define CONTROLLOGIC_HPP_

class ControlLogic{
private:
	float calcGlideGainCompensation();

public:
	float pitch,role,yaw,throttle,flaps;
	float glideDeadtime;
	float glidePitchGain, glidePitchGainFloor, glideMinSpeed;
	float glideRoleGain,glideYawGain;
	float glideGyroPitchGain,glideGyroRoleGain,glideGyroYawGain;
	float degGlidePitch,degGlideRole,degGlideHeading;
	float degFlareAngle,flareGain,mFlareStartHeight,mFlareCompleteHeight;

	ControlLogic();
	void glide();
	void fullManual();





};

#endif /* CONTROLLOGIC_HPP_ */
