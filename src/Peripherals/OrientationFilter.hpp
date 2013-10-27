#ifndef __ORIENTATION_FILTER
#define __ORIENTATION_FILTER

#include "Quaternion.hpp"

class OrientationFilter{
private:

	static const float sampleFreq;
	static const float gyroMeasError;
	static const float gyroMeasDrift;
	static const float initialBeta;
	static const float lastBeta;
	static const int betaStabilizationTime;
	static float beta;
	static float zeta;

	static Quaternion attitude;

	static volatile float q0;
	static volatile float q1;
	static volatile float q2;
	static volatile float q3;
	static volatile float b_x,b_z;
	static volatile float w_bx,w_by,w_bz;

	static void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	static float invSqrt(float x);
	static void updateAttitude();
	static void updateBeta();
public:
	static void initOrientationFilter(float w_x, float w_y, float w_z,float a_x,float a_y,float a_z,float m_x,float m_y,float m_z);
	static void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z);
	static Quaternion getAttitude();
	static float getGyroBias(int axis);
};



#endif
