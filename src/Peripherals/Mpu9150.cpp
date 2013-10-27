#include <mathf.h>
#include "OrientationFilter.hpp"
#include "Quaternion.hpp"
#include "../Util.hpp"
#include "Hals/HalI2C.hpp"
#include "../GeneralConfig.hpp"
#include "Mpu9150.hpp"
#include "Matrix3f.hpp"
#include "Storage.hpp"
#include "../iodefine.h"

#define MPU9150ADDR (0x68<<1)

#define SELF_TEST_X  0x0D
#define SELF_TEST_Y  0x0E
#define SELF_TEST_Z  0x0F
#define SELF_TEST_A  0x10
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define FF_THR       0x1D
#define FF_DUR       0x1E
#define MOT_THR      0x1F
#define MOT_DUR      0x20
#define ZRMOT_THR    0x21
#define ZRMOT_DUR    0x22
#define FIFO_EN      0x23
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_MST_STATUS 0x36
#define INT_PIN_CFG  0x37
#define INT_ENABLE   0x38
#define INT_STATUS   0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define MOT_DETECT_STATUS 0x61
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET 0x68
#define MOT_DETECT_CTRL 0x69
#define USER_CTRL    0x6A
#define PWR_MGMT_1   0x6B
#define PWR_MGMT_2   0x6C
#define FIFO_COUNTH  0x72
#define FIFO_COUNTL  0x73
#define FIFO_R_W     0x74
#define WHO_AM_I     0x75

#define AK8975_ADDR (0x0C<<1) //or 0D, 0E, 0F;
#define WIA   0x00
#define INFO  0x01
#define ST1   0x02
#define HXL   0x03
#define HXH   0x04
#define HYL   0x05
#define HYH   0x06
#define HZL   0x07
#define HZH   0x08
#define ST2   0x09
#define CNTL  0x0A
#define ASTC  0x0C
#define TS1   0x0D
#define TS2   0x0E
#define I2CDIS 0x0F
#define ASAX  0x10
#define ASAY  0x11
#define ASAZ  0x12



Mpu9150* Mpu9150::instance;
float Mpu9150::gyroSum[3];

Mpu9150::Mpu9150(){
	int i;

	I2C::initRIIC0();
	initBias();
	initSensors();
	initCMT0();
	bmp085 = new Bmp085(updateRate/10);

	Util::myWait(100);

	bmp085->readPressureCalibrationData();

	bmp085->measurePrsTemp();
	Util::myWait(20);
	bmp085->readPrsTemp();
	bmp085->measurePressure();
	Util::myWait(50);
	bmp085->readPressure();
	bmp085->updatePressure();

	measure();
	while(!I2C::isBufferEmpty());
	measure();
	while(!I2C::isBufferEmpty());
	measure();
	while(!I2C::isBufferEmpty());

	OrientationFilter::initOrientationFilter(getRpsGyro(0),getRpsGyro(1),getRpsGyro(2),getG_Accel(0),getG_Accel(1),getG_Accel(2),get_uT_Cmps(0),get_uT_Cmps(1),get_uT_Cmps(2));
	kalmanFilter = (KalmanFilter**)malloc(sizeof(KalmanFilter*)*3);
	kalmanFilter[0] = new KalmanFilter(updateRate,0.01,5.0,1.0,0.01);
	kalmanFilter[1] = new KalmanFilter(updateRate,0.01,5.0,1.0,0.01);
	kalmanFilter[2] = new KalmanFilter(updateRate,0.01,1.0,5.0,0.01);

	for(i=0;i<3;i++){
		speed[i]=0.0;
		position[i]=0.0;
		gyroSum[i]=0.0;
	}

	Mpu9150::instance = this;
}

void Mpu9150::initBias(){
	aclBias[0]= -0.00;
	aclBias[1]= -0.03;
	aclBias[2]= -0.077;
	gyroBias[0]= 0.0;
	gyroBias[1]= 0.0;
	gyroBias[2]= -0.0;
	cmpsBias[0]= (0.0)/2;
	cmpsBias[1]= (0.0)/2;
	cmpsBias[2]= (0.0)/2;
	aclScale[0]=1.0;
	aclScale[1]=1.0;
	aclScale[2]=1.01;
}

void Mpu9150::initCMT0(){
	MSTP(CMT0)=0;
	CMT.CMSTR0.BIT.STR0=0;
	CMT0.CMCR.BIT.CMIE=1;
	CMT0.CMCR.BIT.CKS=0;
	CMT0.CMCOR=60000-1;		//100Hz
	//CMT0.CMCOR=30000-1;		//200Hz

	IEN(CMT0,CMI0)=1;
	IPR(CMT0,CMI0)=GeneralConfig::mpu9150Ipr;
}

void Mpu9150::start(){
	CMT.CMSTR0.BIT.STR0=1;
}

void Mpu9150::stop(){
	CMT.CMSTR0.BIT.STR0=0;
}

void Mpu9150::measure(){
	for(int i=0;i<3;i++){
		this->acl[i] = rawToG_Accel(i);
		this->gyro[i]= rawToRpsGyro(i);
		this->cmps[i]= rawTo_uT_Cmps(i);
	}
	temp = rawToTemp();

	I2C::i2cDevRead(MPU9150ADDR, ACCEL_XOUT_H,&rawAclH[0]);
	I2C::i2cDevRead(MPU9150ADDR, ACCEL_XOUT_L,&rawAclL[0]);
	I2C::i2cDevRead(MPU9150ADDR, ACCEL_YOUT_H,&rawAclH[1]);
	I2C::i2cDevRead(MPU9150ADDR, ACCEL_YOUT_L,&rawAclL[1]);
	I2C::i2cDevRead(MPU9150ADDR, ACCEL_ZOUT_H,&rawAclH[2]);
	I2C::i2cDevRead(MPU9150ADDR, ACCEL_ZOUT_L,&rawAclL[2]);

	I2C::i2cDevRead(MPU9150ADDR, TEMP_OUT_H,&rawTemp_H);
	I2C::i2cDevRead(MPU9150ADDR, TEMP_OUT_L,&rawTemp_L);

	I2C::i2cDevRead(MPU9150ADDR, GYRO_XOUT_H,&rawGyroH[0]);
	I2C::i2cDevRead(MPU9150ADDR, GYRO_XOUT_L,&rawGyroL[0]);
	I2C::i2cDevRead(MPU9150ADDR, GYRO_YOUT_H,&rawGyroH[1]);
	I2C::i2cDevRead(MPU9150ADDR, GYRO_YOUT_L,&rawGyroL[1]);
	I2C::i2cDevRead(MPU9150ADDR, GYRO_ZOUT_H,&rawGyroH[2]);
	I2C::i2cDevRead(MPU9150ADDR, GYRO_ZOUT_L,&rawGyroL[2]);

	I2C::i2cDevRead(AK8975_ADDR, HXL,&rawCmpsL[0]);
	I2C::i2cDevRead(AK8975_ADDR, HXH,&rawCmpsH[0]);
	I2C::i2cDevRead(AK8975_ADDR, HYL,&rawCmpsL[1]);
	I2C::i2cDevRead(AK8975_ADDR, HYH,&rawCmpsH[1]);
	I2C::i2cDevRead(AK8975_ADDR, HZL,&rawCmpsL[2]);
	I2C::i2cDevRead(AK8975_ADDR, HZH,&rawCmpsH[2]);

	I2C::i2cDevWrite(AK8975_ADDR, CNTL, 0x01);
}

void Mpu9150::test(){
	int i;
	unsigned char whoAmI=0;
	unsigned char tmp1=0;
	unsigned char tmp2=0;
	unsigned char tmp3=0;
	unsigned char tmp4=0;
	while(1){
		whoAmI=100;
		measure();


		stdout->putString("temp");
		stdout->putFloat(getTemp());
		stdout->putString("acl");
		for(i=0;i<3;i++){
			stdout->putFloat(getG_Accel(i));
			stdout->putString(",\t");
		}
		stdout->putString("gyro");
		for(i=0;i<3;i++){
			stdout->putFloat(getRpsGyro(i));
			stdout->putString(",\t");
		}
		stdout->putString("cmps");
		for(i=0;i<3;i++){
			stdout->putFloat(get_uT_Cmps(i));
			stdout->putString(",\t");
		}
		stdout->putString("\n\r");


		/*
		I2C::i2cDevRead(MPU9150ADDR, ACCEL_XOUT_L, &whoAmI);
		Util::myWait(100);
		stdout->putShortHex(whoAmI);
		stdout->putString("\n\r");
		 */

		Util::myWait(100);



	}

}

void Mpu9150::initSensors(){
	I2C::i2cDevWrite(MPU9150ADDR, PWR_MGMT_1, 0x80);
	I2C::i2cDevWrite(MPU9150ADDR, SIGNAL_PATH_RESET, 0x07);
	Util::myWait(10);
	I2C::i2cDevWrite(MPU9150ADDR, SIGNAL_PATH_RESET, 0x00);


	I2C::i2cDevWrite(MPU9150ADDR, PWR_MGMT_1, (0));
	I2C::i2cDevWrite(MPU9150ADDR, PWR_MGMT_2, (0));

	I2C::i2cDevWrite(MPU9150ADDR, SMPLRT_DIV,SAMPLERATE_DIVIDER);
	I2C::i2cDevWrite(MPU9150ADDR, CONFIG, 0x01);
	I2C::i2cDevWrite(MPU9150ADDR, FIFO_EN, 0x00);

	I2C::i2cDevWrite(MPU9150ADDR, I2C_MST_CTRL, 0x00);
	I2C::i2cDevWrite(MPU9150ADDR, INT_PIN_CFG, (1<<1));

	I2C::i2cDevWrite(MPU9150ADDR, GYRO_CONFIG, (3<<3));
	I2C::i2cDevWrite(MPU9150ADDR, ACCEL_CONFIG, (3<<3));

	I2C::i2cDevWrite(AK8975_ADDR, CNTL, 0x0F);
	I2C::i2cDevRead(AK8975_ADDR, ASAX, &cmpsAsa[0]);
	I2C::i2cDevRead(AK8975_ADDR, ASAY, &cmpsAsa[1]);
	I2C::i2cDevRead(AK8975_ADDR, ASAZ, &cmpsAsa[2]);

	I2C::i2cDevWrite(AK8975_ADDR, CNTL, 0x01);

}

float Mpu9150::rawToG_Accel(int axis){
	int axis2;
	int sign;
	if(axis == 0){
		axis2 = 1;
		sign = 1;
	}else if(axis == 1){
		axis2 = 0;
		sign = 1;
	}else if(axis == 2){
		axis2 = 2;
		sign = -1;
	}else{
		return 0.0;
	}

	short raw = (short)((rawAclH[axis2]<<8) | (rawAclL[axis2]));
	return (float)sign*aclScale[axis]*(raw)/2048.0 - aclBias[axis];
}
float Mpu9150::rawToRpsGyro(int axis){
	int axis2;
	int sign;
	if(axis == 0){
		axis2 = 1;
		sign = 1;
	}else if(axis == 1){
		axis2 = 0;
		sign = 1;
	}else if(axis == 2){
		axis2 = 2;
		sign = -1;
	}else{
		return 0.0;
	}

	short raw = (rawGyroH[axis2]<<8 | rawGyroL[axis2]);
	return (float)sign*(raw / 16.4 * (QUAT_PI/180.0)) - gyroBias[axis];
}
float Mpu9150::rawTo_uT_Cmps(int axis){
	float coef;
	short hadj;

	int axis2;
	float sign;
	if(axis == 0){
		axis2 = 0;
		sign = 1.0;
	}else if(axis == 1){
		axis2 = 1;
		sign = 1.0;
	}else if(axis == 2){
		axis2 = 2;
		sign = 1.0;
	}else{
		return 0.0;
	}

	coef=((cmpsAsa[axis2]-128.0)*0.5)/128 + 1.0;
	hadj= ((short)(rawCmpsH[axis2]<<8 | rawCmpsL[axis2])) * coef;

	return (float)(sign*hadj * 0.3) - cmpsBias[axis];
}
float Mpu9150::rawToTemp(){
	short raw = (rawTemp_H<<8) | rawTemp_L;
	return (float)(raw/1.0);
}

float Mpu9150::getG_Accel(int axis){
	return acl[axis];
//	int axis2;
//	int sign;
//	if(axis == 0){
//		axis2 = 1;
//		sign = 1;
//	}else if(axis == 1){
//		axis2 = 0;
//		sign = 1;
//	}else if(axis == 2){
//		axis2 = 2;
//		sign = -1;
//	}else{
//		return 0.0;
//	}
//
//	short raw = (short)((rawAclH[axis2]<<8) | (rawAclL[axis2]));
//	return (float)sign*(raw)/2048.0 - aclBias[axis];
}
float Mpu9150::getRpsGyro(int axis){
	return gyro[axis];
//	int axis2;
//	int sign;
//	if(axis == 0){
//		axis2 = 1;
//		sign = 1;
//	}else if(axis == 1){
//		axis2 = 0;
//		sign = 1;
//	}else if(axis == 2){
//		axis2 = 2;
//		sign = -1;
//	}else{
//		return 0.0;
//	}
//
//	short raw = (rawGyroH[axis2]<<8 | rawGyroL[axis2]);
//	return (float)sign*(raw / 16.4 * (QUAT_PI/180.0)) - gyroBias[axis];
}
float Mpu9150::get_uT_Cmps(int axis){
	return cmps[axis];
//	float coef;
//	short hadj;
//
//	int axis2;
//	float sign;
//	if(axis == 0){
//		axis2 = 0;
//		sign = 1.0;
//	}else if(axis == 1){
//		axis2 = 1;
//		sign = 1.0;
//	}else if(axis == 2){
//		axis2 = 2;
//		sign = 1.0;
//	}else{
//		return 0.0;
//	}
//
//	coef=((cmpsAsa[axis2]-128.0)*0.5)/128 + 1.0;
//	hadj= ((short)(rawCmpsH[axis2]<<8 | rawCmpsL[axis2])) * coef;
//
//	return (float)(sign*hadj * 0.3) - cmpsBias[axis];
}
float Mpu9150::getTemp(){
	return temp;
}

float Mpu9150::getHzUpdateRate(){
	return updateRate;
}

float Mpu9150::getMpsKalmanSpeed(int axis){
	return kalmanFilter[axis]->getMpsSpd();
}
float Mpu9150::getM_KalmanPos(int axis){
	return kalmanFilter[axis]->getM_RelPos();
}

void Mpu9150::resetKalmanFilter(){
	for(int i=0;i<3;i++){
		kalmanFilter[i]->reset();
	}
	gps->resetRefPosition();
	bmp085->resetReference();
}


Mpu9150* Mpu9150::getInstance(){
	return instance;
}




#pragma interrupt Mpu9150::CMT0vect(vect=VECT( CMT0, CMI0 ),enable)
void Mpu9150::CMT0vect(){
	static int sensorCounter = -1;
	sensorCounter = (sensorCounter + 1)%10;

	float gyro[3];
	float acl[3];
	float cmps[3];
	int i;


	for(i=0;i<3;i++){
		gyro[i]=Mpu9150::getInstance()->getRpsGyro(i);
		acl[i]=Mpu9150::getInstance()->getG_Accel(i);
		cmps[i]=Mpu9150::getInstance()->get_uT_Cmps(i);
	}

	Mpu9150::getInstance()->measure();
	if(sensorCounter == 0){
		Mpu9150::getInstance()->bmp085->measurePrsTemp();
	}else if(sensorCounter == 2){
		Mpu9150::getInstance()->bmp085->readPrsTemp();
		Mpu9150::getInstance()->bmp085->measurePressure();
	}else if(sensorCounter == 7){
		Mpu9150::getInstance()->bmp085->readPressure();
	}else if(sensorCounter == 8){
		Mpu9150::getInstance()->bmp085->updatePressure();
	}


	OrientationFilter::filterUpdate(gyro[0],gyro[1],gyro[2],acl[0],acl[1],acl[2],cmps[0],cmps[1],cmps[2]);

	Quaternion qAcl = inu->getE_FrameAccel();

	Mpu9150::getInstance()->kalmanFilter[0]->predict(qAcl.x * 9.8);
	Mpu9150::getInstance()->kalmanFilter[1]->predict(qAcl.y * 9.8);
	Mpu9150::getInstance()->kalmanFilter[2]->predict((qAcl.z-1.0) * 9.8);

	if(gps->isNewDataAvailable()){
		gps->clearIsNewDataAvailable();
		Mpu9150::getInstance()->kalmanFilter[0]->update(gps->getMpsSpeedX(),gps->getM_RelativePosX());
		Mpu9150::getInstance()->kalmanFilter[1]->update(gps->getMpsSpeedY(),gps->getM_RelativePosY());
	}
	if(sensorCounter == 9){
		Mpu9150::getInstance()->kalmanFilter[2]->update(Mpu9150::getInstance()->bmp085->getMpsSpeed(),Mpu9150::getInstance()->bmp085->getM_Height());
	}

//
//	if(sensorCounter == 0){
//		stdout->putString("acl:");
//		stdout->putFloat(qAcl.z-1.0);
//		stdout->putString("\tprs");
//		stdout->putFloat(Mpu9150::getInstance()->bmp085->getPaPressure());
//		stdout->putString("\tprs spd");
//		stdout->putFloat(Mpu9150::getInstance()->bmp085->getMpsSpeed());
//		stdout->putString("\tprs hgt");
//		stdout->putFloat(Mpu9150::getInstance()->bmp085->getM_Height());
//		stdout->putString("\tkal spd");
//		stdout->putFloat(Mpu9150::getInstance()->kalmanFilter[2]->getMpsSpd());
//		stdout->putString("\tkal pos");
//		stdout->putFloat(Mpu9150::getInstance()->kalmanFilter[2]->getM_RelPos());
//		stdout->putString("\n\r");
//	}
}

void Mpu9150::gyroCalibration(){
	int i;
	int axis;
	float gyroSum[3];
	int repeat = 1000;

	stop();
	Util::myWait(100);

	stdout->putString("gyro calibration start\n\r");

	for(axis=0;axis<3;axis++){
		gyroSum[axis]=0.0;
	}

	for(axis=0;axis<3;axis++){
		gyroBias[axis]=0.0;
	}

	for(i=0;i<repeat;i++){
		measure();
		while(I2C::isBufferEmpty()!=0){}

		for(axis=0;axis<3;axis++){
			gyroSum[axis]+=getRpsGyro(axis);
		}

		if(i%10==0){
			stdout->putInt2(i);
			stdout->putString("/");
			stdout->putInt2(repeat);
			stdout->putString("\r");
		}
	}
	stdout->putString("\n\r");

	for(axis=0;axis<3;axis++){
		gyroSum[axis]/=repeat;
		gyroBias[axis]=gyroSum[axis];
		stdout->putFloat(gyroSum[axis]);
		stdout->putString("\t");
	}

	Storage::writeGyroBias(gyroBias);

	stdout->putString("\n\r");
	stdout->putString("done.\n\r");

	start();
}

void Mpu9150::compassCalibration(){

	stdout->putString("compass calibration start. rotate heading axis\n\r");

	float cmpsMax[3];
	float cmpsMin[3];
	for(int i=0;i<3;i++){
		cmpsMax[i] = -100000;
		cmpsMin[i] =  100000;
	}


	int cmpsFlag[36];
	for(int i=0;i<36;i++){
		cmpsFlag[i]=0;
	}

	while(1){
		Quaternion q = OrientationFilter::getAttitude();
		float pitch,role,heading;
		int headingIndex;
		q.getRadPitchRoleHeading(&pitch,&role,&heading);
		pitch   = pitch*180/QUAT_PI;
		role    = role*180/QUAT_PI;
		heading = heading*180/QUAT_PI+180.0;
		headingIndex = (((int)(heading/10.0))+36) % 36;

		if(-20<pitch && pitch<20){
			cmpsFlag[headingIndex]=1;

			for(int i=0;i<2;i++){
				if(cmpsMax[i]<get_uT_Cmps(i)){
					cmpsMax[i]=get_uT_Cmps(i);
				}
				if(get_uT_Cmps(i)<cmpsMin[i]){
					cmpsMin[i]=get_uT_Cmps(i);
				}
			}
		}

		int complete=1;
		for(int i=0;i<36;i++){
			if(cmpsFlag[i]==0){
				complete=0;
			}
		}

		if(complete==1){
			break;
		}
	}

	stdout->putString("compass calibration start. rotate roll axis\n\r");

	for(int i=0;i<36;i++){
		cmpsFlag[i]=0;
	}

	while(1){
		Quaternion q = OrientationFilter::getAttitude();
		float pitch,role,heading;
		int roleIndex;
		q.getRadPitchRoleHeading(&pitch,&role,&heading);
		pitch   = pitch*180/QUAT_PI;
		role    = role*180/QUAT_PI+180;
		heading = heading*180/QUAT_PI;
		roleIndex = (((int)(role/10.0))+36) % 36;

		if(-20<pitch && pitch<20){
			cmpsFlag[roleIndex]=1;

			for(int i=2;i<3;i++){
				if(cmpsMax[i]<get_uT_Cmps(i)){
					cmpsMax[i]=get_uT_Cmps(i);
				}
				if(get_uT_Cmps(i)<cmpsMin[i]){
					cmpsMin[i]=get_uT_Cmps(i);
				}
			}
		}

		int complete=1;
		for(int i=0;i<36;i++){
			if(cmpsFlag[i]==0){
				complete=0;
			}
		}

		if(complete==1){
			break;
		}
	}


	for(int i=0;i<3;i++){
		cmpsBias[i] += (cmpsMax[i]+cmpsMin[i])/2;

//		stdout->putFloat(cmpsMax[i]);
//		stdout->putString(",");
//		stdout->putFloat(cmpsMin[i]);
//		stdout->putString("\n\r");

	}

	Storage::writeCompassBias(cmpsBias);

	stdout->putString("compass calibration done.\n\r");

}

void Mpu9150::updateAclBias(){
	static int callCount = 0;
	callCount++;

	if(500<callCount){
		Quaternion gravity = Quaternion(0,0,0,1);
		Quaternion attitude = OrientationFilter::getAttitude();
		Quaternion roter = attitude;
		Quaternion roterCon;
		roterCon.con(roter);

		gravity.mul(roter);
		roterCon.mul(gravity);

		Quaternion bGravity = roterCon;
		bGravity.normalize();

		Quaternion bAcl = Quaternion(0,getG_Accel(0),getG_Accel(1),getG_Accel(2));

		bAcl.x-=bGravity.x;
		bAcl.y-=bGravity.y;
		bAcl.z-=bGravity.z;

		aclBias[0]+=bAcl.x*0.01;
		aclBias[1]+=bAcl.y*0.01;
		aclBias[2]+=bAcl.z*0.01;
	}
}

void Mpu9150::acceloCalibration(){
	while(1){
//		Quaternion attitude = OrientationFilter::getAttitude();
//		Quaternion eAcl = inu->getE_FrameAccel();
//		eAcl.z-=1.0;
//
//		Quaternion roter   =attitude;
//		Quaternion roterCon;
//		roterCon.con(roter);
//		Quaternion bAcl;
//
//		eAcl.mul(roter);
//		roterCon.mul(eAcl);
//
//		bAcl = roterCon;
//
//		eAcl.w=0;
//		eAcl.x=getG_Accel(0);
//		eAcl.y=getG_Accel(1);
//		eAcl.z=getG_Accel(2);
//		eAcl.normalize();
//
//		aclBias[0]+=bAcl.x*0.001;
//		aclBias[1]+=bAcl.y*0.001;
//		aclBias[2]+=bAcl.z*0.001;

		//aclScale[0] *= (1-eAcl.x)  +  eAcl.x * (1-0.001*bAcl.x);
		//aclScale[1] *= (1-eAcl.y)  +  eAcl.y * (1-0.001*bAcl.y);
		//aclScale[2] *= (1-eAcl.z)  +  eAcl.z * (1-0.001*bAcl.z);

		Quaternion gravity = Quaternion(0,0,0,1);
		Quaternion attitude = OrientationFilter::getAttitude();
		Quaternion roter = attitude;
		Quaternion roterCon;
		roterCon.con(roter);

		gravity.mul(roter);
		roterCon.mul(gravity);

		Quaternion bGravity = roterCon;
		bGravity.normalize();

		Quaternion bAcl = Quaternion(0,getG_Accel(0),getG_Accel(1),getG_Accel(2));

		bAcl.x-=bGravity.x;
		bAcl.y-=bGravity.y;
		bAcl.z-=bGravity.z;

		aclBias[0]+=bAcl.x*0.001;
		aclBias[1]+=bAcl.y*0.001;
		aclBias[2]+=bAcl.z*0.001;


		stdout->putFloat(bAcl.x);
		stdout->putString("\t");
		stdout->putFloat(bAcl.y);
		stdout->putString("\t");
		stdout->putFloat(bAcl.z);
		stdout->putString("\t");
		stdout->putFloat(aclBias[0]);
		stdout->putString("\t");
		stdout->putFloat(aclBias[1]);
		stdout->putString("\t");
		stdout->putFloat(aclBias[2]);
		stdout->putString("\t");
		stdout->putFloat(aclScale[0]);
		stdout->putString("\t");
		stdout->putFloat(aclScale[1]);
		stdout->putString("\t");
		stdout->putFloat(aclScale[2]);
		stdout->putString("\n\r");

		Util::myWait(30);
	}
}


//void Mpu9150::compassCalibration(){
//	stdout->putString("compass calibration start. move pcb on 8-shaped loop.\n\r");
//
//	float cmps1[3];
//	float cmps2[3];
//	float cmps3[4];
//
//	Quaternion q1 = OrientationFilter::getAttitude();
//	Quaternion q2;
//	Quaternion q12;
//	Quaternion q3;
//	Quaternion q13;
//	Quaternion q23;
//	Quaternion q2Con;
//
//	for(int i=0;i<3;i++){
//		cmps1[i] = get_uT_Cmps(i);
//	}
//
//	Quaternion q1Con;
//	q1Con.con(q1);
//	q1Con.normalize();
//
//	while(1){
//		q2 = OrientationFilter::getAttitude();
//		for(int i=0;i<3;i++){
//			cmps2[i] = get_uT_Cmps(i);
//		}
//
//		q2.normalize();
//		q1Con.normalize();
//
//		q12 = q2;
//		q12.mul(q1Con);
//		q12.normalize();
//
//		float relativeAngle = 2*acosf(q12.w)*180/QUAT_PI;
//
//		if(85<relativeAngle && relativeAngle<95){
//			break;
//		}
//
//		Util::myWait(10);
//	}
//
//	q2Con.con(q2);
//
//	while(1){
//		q3 = OrientationFilter::getAttitude();
//		for(int i=0;i<3;i++){
//			cmps3[i] = get_uT_Cmps(i);
//		}
//
//		q3.normalize();
//
//		q13 = q3;
//		q13.mul(q1Con);
//		q13.normalize();
//
//		q23 = q3;
//		q23.mul(q2Con);
//		q23.normalize();
//
//		float relativeAngle1 = 2*acosf(q13.w)*180/QUAT_PI;
//		float relativeAngle2 = 2*acosf(q23.w)*180/QUAT_PI;
//
//		if(85<relativeAngle1 && relativeAngle1<95 && 85<relativeAngle2 && relativeAngle2<95){
//			break;
//		}
//
//		Util::myWait(10);
//	}
//
//	float** opArray = (float**)malloc(sizeof(float*)*3);
//	if(opArray == NULL){
//		stdout->putString("malloc error at compassCalibration\n\r");
//	}
//	for(int i=0;i<3;i++){
//		opArray[i] = (float*)malloc(sizeof(float)*3);
//		if(opArray[i] == NULL){
//			stdout->putString("malloc error at compassCalibration\n\r");
//		}
//	}
//	float** mArray = (float**)malloc(sizeof(float)*3);
//	if(mArray == NULL){
//		stdout->putString("malloc error at compassCalibration\n\r");
//	}
//	for(int i=0;i<3;i++){
//		mArray[i] = (float*)malloc(sizeof(float)*1);
//		if(mArray[i] == NULL){
//			stdout->putString("malloc error at compassCalibration\n\r");
//		}
//	}
//
//	opArray[0][0] = q12.x*q12.x + q12.w*q12.w - q12.z*q12.z - q12.y*q12.y -1;
//	opArray[0][1] = 2*q12.x*q12.y + 2*q12.w*q12.z;
//	opArray[0][2] = 2*q12.x*q12.z - 2*q12.w*q12.y;
//
//	opArray[1][0] = 2*q12.x*q12.y - 2*q12.w*q12.z;
//	opArray[1][1] = q12.y*q12.y - q12.z*q12.z + q12.w*q12.w - q12.x*q12.x -1;
//	opArray[1][2] = 2*q12.y*q12.z + 2*q12.w*q12.x;
//
//	opArray[2][0] = 2*q13.x*q13.z + 2*q13.w*q13.y;
//	opArray[2][1] = 2*q13.y*q13.z - 2*q13.w*q13.x;
//	opArray[2][2] = q13.z*q13.z - q13.y*q13.y - q13.x*q13.x - q13.w*q13.w -1;
//
//	mArray[0][0] = cmps1[0] - cmps2[0]*(q12.x*q12.x + q12.w*q12.w - q12.z*q12.z - q12.y*q12.y) - cmps2[1]*(2*q12.x*q12.y + 2*q12.w*q12.z) - cmps2[2]*(2*q12.x*q12.z - 2*q12.w*q12.y);
//	mArray[1][0] = cmps1[1] - cmps2[0]*(2*q12.x*q12.y - 2*q12.w*q12.z) - cmps2[1]*(q12.y*q12.y - q12.z*q12.z + q12.w*q12.w - q12.x*q12.x) - cmps2[2]*(2*q12.y*q12.z + 2*q12.w*q12.x);
//	mArray[2][0] = cmps1[2] - cmps3[0]*(2*q13.x*q13.z + 2*q13.w*q13.y) - cmps3[1]*(2*q13.y*q13.z - 2*q13.w*q13.x) - cmps3[2]*(q13.z*q13.z - q13.y*q13.y - q13.x*q13.x - q13.w*q13.w);
//
//	Matrix3f* op = new Matrix3f(opArray,3,3);
//	Matrix3f* m  = new Matrix3f(mArray,3,1);
//
//	op->inverse();
//
//	Matrix3f* biasMat = op->mul(m);
//
//	float** bias = biasMat->getNums();
//
//	for(int i=0;i<3;i++){
//		cmpsBias[i] -= bias[i][0];
//	}
//
//
//
//	delete op;
//	delete m;
//	delete bias;
//
//	for(int i=0;i<3;i++){
//		free(opArray[i]);
//	}
//	free(opArray);
//	for(int i=0;i<3;i++){
//		free(mArray[i]);
//	}
//	free(mArray);
//
//	stdout->putString("compass calibration done.\n\r");
//}

void Mpu9150::readGyroBias(){
	int i;
	stdout->putString("read ");

	Storage::readGyroBias(gyroBias);

	showGyroBias();
	return;
}

void Mpu9150::showGyroBias(){
	int i;
	stdout->putString("gyro Bias\n\r");
	for(i=0;i<3;i++){
		stdout->putFloat(gyroBias[i]);
		stdout->putString("\t");
	}
	stdout->putString("\n\r");
	return;
}
void Mpu9150::readCompassBias(){
	int i;
	stdout->putString("read ");

	Storage::readCompassBias(cmpsBias);

	showCompassBias();
	return;
}
void Mpu9150::showCompassBias(){
	int i;
	stdout->putString("compass Bias\n\r");
	for(i=0;i<3;i++){
		stdout->putFloat(cmpsBias[i]);
		stdout->putString("\t");
	}
	stdout->putString("\n\r");
	return;
}
