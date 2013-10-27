/*
 * Storage.hpp
 *
 *  Created on: 2013/02/11
 *      Author: sa
 */

#ifndef STORAGE_HPP_
#define STORAGE_HPP_

class Storage{
private:
public:
	static void init();
	static void writeGyroBias(float* gyroBias);
	static void readGyroBias(float* gyroBias);
	static void writeTrim(short* trim,int chNum);	//16 ch max.
	static void readTrim(short* trim,int chNum);	//16 ch max.
	static void writeGlideAttitude(float degGlidePitch,float degGlideRole);
	static void readGlideAttitude(float* degGlidePitch,float* degGlideRole);
	static void writeGlideGain(float glidePitchGain,float glidePitchGainFloor, float glideMinSpeed, float glideRoleGain,float glideHeadingGain, float glideDeadtime);
	static void readGlideGain(float* glidePitchGain,float* glidePitchGainFloor, float* glideMinSpeed, float* glideRoleGain,float* glideHeadingGain, float* glideDeadtime);
	static void writeCompassBias(float* compassBias);
	static void readCompassBias(float* compassBias);
	static void writeGlideGyroGain(float glideGyroPitchGain,float glideGyroRoleGain,float glideGyroHeadingGain);
	static void readGlideGyroGain(float* glideGyroPitchGain,float* glideGyroRoleGain,float* glideGyroHeadingGain);
	static void writeFlareParam(float degFlareAngle,float flareGain,float mFlareStartHeight,float mFlareCompleteHeight);
	static void readFlareParam(float* degFlareAngle,float* flareGain,float* mFlareStartHeight,float* mFlareCompleteHeight);
};

#endif /* STORAGE_HPP_ */
