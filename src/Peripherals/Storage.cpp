/*
 * Storage.cpp
 *
 *  Created on: 2013/02/11
 *      Author: sa
 */

#include "Hals/HalFlash.hpp"
#include "Storage.hpp"
#include "../GeneralConfig.hpp"

#define GYRO_BIAS_BANK 0
#define BYTE_GYRO_BIAS_OFFSET 0
#define BYTE_GYRO_BIAS_LENGTH 16

#define TRIM_BANK 0
#define BYTE_TRIM_OFFSET (BYTE_GYRO_BIAS_OFFSET+BYTE_GYRO_BIAS_LENGTH)
#define BYTE_TRIM_LENGTH 32

#define GLIDE_ATTITUDE_BANK 0
#define BYTE_GLIDE_ATTITUDE_OFFSET (BYTE_TRIM_OFFSET+BYTE_TRIM_LENGTH)
#define BYTE_GLIDE_ATTITUDE_LENGTH 16

#define GLIDE_GAIN_BANK 0
#define BYTE_GLIDE_GAIN_OFFSET (BYTE_GLIDE_ATTITUDE_OFFSET+BYTE_GLIDE_ATTITUDE_LENGTH)
#define BYTE_GLIDE_GAIN_LENGTH 32

#define COMPASS_BIAS_BANK 0
#define BYTE_COMPASS_BIAS_OFFSET (BYTE_GLIDE_GAIN_OFFSET+BYTE_GLIDE_GAIN_LENGTH)
#define BYTE_COMPASS_BIAS_LENGTH 16

#define GLIDE_GYRO_GAIN_BANK 0
#define BYTE_GLIDE_GYRO_GAIN_OFFSET (BYTE_COMPASS_BIAS_OFFSET+BYTE_COMPASS_BIAS_LENGTH)
#define BYTE_GLIDE_GYRO_GAIN_LENGTH 16

#define FLARE_PARAM_BANK 0
#define BYTE_FLARE_PARAM_OFFSET (BYTE_GLIDE_GYRO_GAIN_OFFSET+BYTE_GLIDE_GYRO_GAIN_LENGTH)
#define BYTE_FLARE_PARAM_LENGTH 16

void Storage::init(){
	HalFlash::init();
}

void Storage::writeGyroBias(float* gyroBias){
	char buf[BYTE_GYRO_BIAS_LENGTH];
	float* dummy=(float*)buf;
	int i;

	for(i=0;i<3;i++){
		dummy[i]=gyroBias[i];
	}
	dummy[3]=0.0;

	HalFlash::setbank(GYRO_BIAS_BANK);
	HalFlash::write(buf,BYTE_GYRO_BIAS_OFFSET,BYTE_GYRO_BIAS_LENGTH);
	HalFlash::finalize();

	return;
}

void Storage::readGyroBias(float* gyroBias){
	char buf[BYTE_GYRO_BIAS_LENGTH];
	float* dummy=(float*)buf;
	int i;

	HalFlash::setbank(GYRO_BIAS_BANK);
	HalFlash::read(buf,BYTE_GYRO_BIAS_OFFSET,BYTE_GYRO_BIAS_LENGTH);

	for(i=0;i<3;i++){
		gyroBias[i]=dummy[i];
	}

	return;
}
void Storage::writeTrim(short* trim,int chNum){
	char buf[BYTE_TRIM_LENGTH];
	short* dummy = (short*)buf;

	for(int i=0;i<BYTE_TRIM_LENGTH/2;i++){
		if(i<chNum){
			dummy[i] = trim[i];

			//stdout->putInt(i);
			//stdout->putString(",");
			//stdout->putShort(dummy[i]);
			//stdout->putString("\n\r");
		}else{
			dummy[i] = 0;
		}
	}

	HalFlash::setbank(TRIM_BANK);
	HalFlash::write(buf,BYTE_TRIM_OFFSET,BYTE_TRIM_LENGTH);
	HalFlash::finalize();

	return;
}
void Storage::readTrim(short* trim,int chNum){
	char buf[BYTE_TRIM_LENGTH];
	short* dummy = (short*)buf;

	HalFlash::setbank(TRIM_BANK);
	HalFlash::read(buf,BYTE_TRIM_OFFSET,BYTE_TRIM_LENGTH);

	for(int i=0;i<chNum;i++){
		trim[i] = dummy[i];

		//stdout->putInt(i);
		//stdout->putString(",");
		//stdout->putShort(dummy[i]);
		//stdout->putString("\n\r");
	}

	return;
}

void Storage::writeGlideAttitude(float degGlidePitch,float degGlideRole){
	char buf[BYTE_GLIDE_ATTITUDE_LENGTH];
	float* dummy = (float*)buf;

	dummy[0]=degGlidePitch;
	dummy[1]=degGlideRole;
	dummy[2]=0.0;
	dummy[3]=0.0;

	HalFlash::setbank(GLIDE_ATTITUDE_BANK);
	HalFlash::write(buf,BYTE_GLIDE_ATTITUDE_OFFSET,BYTE_GLIDE_ATTITUDE_LENGTH);
	HalFlash::finalize();

	return;
}
void Storage::readGlideAttitude(float* degGlidePitch,float* degGlideRole){
	char buf[BYTE_GLIDE_ATTITUDE_LENGTH];
	float* dummy = (float*)buf;

	HalFlash::setbank(GLIDE_ATTITUDE_BANK);
	HalFlash::read(buf,BYTE_GLIDE_ATTITUDE_OFFSET,BYTE_GLIDE_ATTITUDE_LENGTH);

	*degGlidePitch = dummy[0];
	*degGlideRole = dummy[1];

	return;
}
void Storage::writeGlideGain(float glidePitchGain,float glidePitchGainFloor, float glideMinSpeed, float glideRoleGain,float glideHeadingGain, float glideDeadtime){
	char buf[BYTE_GLIDE_GAIN_LENGTH];
	float* dummy = (float*)buf;

	dummy[0]=glidePitchGain;
	dummy[1]=glidePitchGainFloor;
	dummy[2]=glideMinSpeed;
	dummy[3]=glideRoleGain;
	dummy[4]=glideHeadingGain;
	dummy[5]=glideDeadtime;
	dummy[6]=0.0;
	dummy[7]=0.0;

	HalFlash::setbank(GLIDE_GAIN_BANK);
	HalFlash::write(buf,BYTE_GLIDE_GAIN_OFFSET,BYTE_GLIDE_GAIN_LENGTH);
	HalFlash::finalize();

	return;
}
void Storage::readGlideGain(float* glidePitchGain,float* glidePitchGainFloor, float* glideMinSpeed, float* glideRoleGain,float* glideHeadingGain, float* glideDeadtime){
	char buf[BYTE_GLIDE_GAIN_LENGTH];
	float* dummy = (float*)buf;

	HalFlash::setbank(GLIDE_GAIN_BANK);
	HalFlash::read(buf,BYTE_GLIDE_GAIN_OFFSET,BYTE_GLIDE_GAIN_LENGTH);

	*glidePitchGain = dummy[0];
	*glidePitchGainFloor = dummy[1];
	*glideMinSpeed = dummy[2];
	*glideRoleGain = dummy[3];
	*glideHeadingGain = dummy[4];
	*glideDeadtime = dummy[5];

	return;
}

void Storage::writeCompassBias(float* compassBias){
	char buf[BYTE_GYRO_BIAS_LENGTH];
	float* dummy=(float*)buf;
	int i;

	for(i=0;i<3;i++){
		dummy[i]=compassBias[i];
	}
	dummy[3]=0.0;

	HalFlash::setbank(COMPASS_BIAS_BANK);
	HalFlash::write(buf,BYTE_COMPASS_BIAS_OFFSET,BYTE_COMPASS_BIAS_LENGTH);
	HalFlash::finalize();

	return;
}
void Storage::readCompassBias(float* compassBias){
	char buf[BYTE_COMPASS_BIAS_LENGTH];
	float* dummy=(float*)buf;
	int i;

	HalFlash::setbank(COMPASS_BIAS_BANK);
	HalFlash::read(buf,BYTE_COMPASS_BIAS_OFFSET,BYTE_COMPASS_BIAS_LENGTH);

	for(i=0;i<3;i++){
		compassBias[i]=dummy[i];
	}

	return;
}
void Storage::writeGlideGyroGain(float glideGyroPitchGain,float glideGyroRoleGain,float glideGyroHeadingGain){
	char buf[BYTE_GLIDE_GYRO_GAIN_LENGTH];
	float* dummy = (float*)buf;

	dummy[0]=glideGyroPitchGain;
	dummy[1]=glideGyroRoleGain;
	dummy[2]=glideGyroHeadingGain;
	dummy[3]=0.0;

	HalFlash::setbank(GLIDE_GYRO_GAIN_BANK);
	HalFlash::write(buf,BYTE_GLIDE_GYRO_GAIN_OFFSET,BYTE_GLIDE_GYRO_GAIN_LENGTH);
	HalFlash::finalize();

	return;
}
void Storage::readGlideGyroGain(float* glideGyroPitchGain,float* glideGyroRoleGain,float* glideGyroHeadingGain){
	char buf[BYTE_GLIDE_GYRO_GAIN_LENGTH];
	float* dummy = (float*)buf;

	HalFlash::setbank(GLIDE_GYRO_GAIN_BANK);
	HalFlash::read(buf,BYTE_GLIDE_GYRO_GAIN_OFFSET,BYTE_GLIDE_GYRO_GAIN_LENGTH);

	*glideGyroPitchGain = dummy[0];
	*glideGyroRoleGain = dummy[1];
	*glideGyroHeadingGain = dummy[2];

	return;
}

void Storage::writeFlareParam(float degFlareAngle,float flareGain,float mFlareStartHeight,float mFlareCompleteHeight){
	char buf[BYTE_FLARE_PARAM_LENGTH];
	float* dummy = (float*)buf;

	dummy[0]=degFlareAngle;
	dummy[1]=flareGain;
	dummy[2]=mFlareStartHeight;
	dummy[3]=mFlareCompleteHeight;

	HalFlash::setbank(FLARE_PARAM_BANK);
	HalFlash::write(buf,BYTE_FLARE_PARAM_OFFSET,BYTE_FLARE_PARAM_LENGTH);
	HalFlash::finalize();

	return;
}
void Storage::readFlareParam(float* degFlareAngle,float* flareGain,float* mFlareStartHeight,float* mFlareCompleteHeight){
	char buf[BYTE_GLIDE_GYRO_GAIN_LENGTH];
	float* dummy = (float*)buf;

	HalFlash::setbank(FLARE_PARAM_BANK);
	HalFlash::read(buf,BYTE_FLARE_PARAM_OFFSET,BYTE_FLARE_PARAM_LENGTH);

	*degFlareAngle = dummy[0];
	*flareGain = dummy[1];
	*mFlareStartHeight = dummy[2];
	*mFlareCompleteHeight=dummy[3];

	return;
}
