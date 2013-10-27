/*
 * Mpu9150.hpp
 *
 *  Created on: 2012/12/26
 *      Author: sa
 */

#ifndef MPU9150_HPP_
#define MPU9150_HPP_

#include "KalmanFilter.hpp"
#include "Bmp085.hpp"

#define SAMPLERATE_DIVIDER 4	//samplerate = 1kHz / (SAMPLERATE_DIVIDER+1)

class Mpu9150{
private:
	const float updateRate = 100.0;

	static Mpu9150* instance;
	static Mpu9150* getInstance();

	Bmp085* bmp085;
	KalmanFilter** kalmanFilter;

	unsigned char rawAclH[3];
	unsigned char rawAclL[3];
	unsigned char rawGyroH[3];
	unsigned char rawGyroL[3];
	unsigned char rawCmpsH[3];
	unsigned char rawCmpsL[3];
	unsigned char cmpsAsa[3];
	unsigned char rawTemp_H;
	unsigned char rawTemp_L;

	float acl[3];
	float gyro[3];
	float cmps[3];
	float temp;

	float aclBias[3];
	float gyroBias[3];
	float cmpsBias[3];
	static float gyroSum[3];

	float aclScale[3];

	float speed[3];
	float position[3];

	void initSensors();
	void initBias();
	void measure();
	void initCMT0();

	void updateAclBias();

	float rawToG_Accel(int axis);
	float rawToRpsGyro(int axis);
	float rawTo_uT_Cmps(int axis);
	float rawToTemp();
public:
	Mpu9150();
	void test();
	void start();
	void stop();
	float getG_Accel(int axis);
	float getRpsGyro(int axis);
	float get_uT_Cmps(int axis);
	float getTemp();
	float getHzUpdateRate();
	float getMpsKalmanSpeed(int axis);
	float getM_KalmanPos(int axis);
	void resetKalmanFilter();

	void gyroCalibration();
	void compassCalibration();
	void acceloCalibration();
	void readGyroBias();
	void showGyroBias();
	void readCompassBias();
	void showCompassBias();

	static void CMT0vect();
};

#endif /* MPU9150_HPP_ */
