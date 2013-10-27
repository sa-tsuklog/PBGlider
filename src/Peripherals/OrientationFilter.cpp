#include "mathf.h"
#include "OrientationFilter.hpp"
#include "Mpu9150.hpp"
#include "../GeneralConfig.hpp"

// System constants
const float OrientationFilter::sampleFreq=100.0; // sampling period in seconds (shown as 10 ms)
const float OrientationFilter::gyroMeasError=(3.14159265358979 * (20.f / 180.0f)); // gyroscope measurement error in rad/s (shown as 5 deg/s)
const float OrientationFilter::gyroMeasDrift=(3.14159265358979 * (20.0f / 180.0f)); // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
const float OrientationFilter::initialBeta=0.5;
const float OrientationFilter::lastBeta=0.01;
const int OrientationFilter::betaStabilizationTime = 400;
float OrientationFilter::zeta=(sqrtf(3.0f / 4.0f) * gyroMeasDrift); // compute zeta  ****not used now.
//float OrientationFilter::beta=0.05;//(sqrtf(3.0f / 4.0f) * gyroMeasError); // compute beta
float OrientationFilter::beta=0.05;

Quaternion OrientationFilter::attitude;

volatile float OrientationFilter::q0;
volatile float OrientationFilter::q1;
volatile float OrientationFilter::q2;
volatile float OrientationFilter::q3;
volatile float OrientationFilter::b_x,OrientationFilter::b_z;
volatile float OrientationFilter::w_bx,OrientationFilter::w_by,OrientationFilter::w_bz;

void OrientationFilter::updateBeta(){
	static int callCount = 0;

	if(betaStabilizationTime<callCount){
		beta=lastBeta;
		return;
	}

	beta = initialBeta * ((float)betaStabilizationTime - callCount)/betaStabilizationTime + lastBeta*((float)callCount)/betaStabilizationTime;


	callCount++;

	return;
}

void OrientationFilter::updateAttitude(){
	Quaternion tmp = Quaternion(q0,q1,q2,q3);
	tmp.normalize();
	attitude=tmp;
}

void OrientationFilter::initOrientationFilter(float w_x, float w_y, float w_z,float a_x,float a_y,float a_z,float m_x,float m_y,float m_z){
	q0=1;
	q1=0;
	q2=0;
	q3=0;

	b_x=0.608;
	b_z=-0.794;

	w_bx=0;
	w_by=0;
	w_bz=0;

	Quaternion vectAccel=Quaternion(0.0f, a_x,a_y,a_z);
    Quaternion vectMag  =Quaternion(0.0f, m_x,m_y,m_z);
    Quaternion vectGravityRef=Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    Quaternion vectAccelAxis=Quaternion(1.0f,0.0f,0.0f,0.0f);
    Quaternion accelRoter;
    float accelAngle;

    Quaternion tmpQuat=Quaternion(1.0f,0.0f,0.0f,0.0f);
    Quaternion tmpQuat2=Quaternion(1.0f,0.0f,0.0f,0.0f);
    float magAngle;
    Quaternion magRoter=Quaternion(1.0f,0.0f,0.0f,0.0f);

    vectAccel.normalize();
    vectMag.normalize();

    accelAngle=acosf(vectAccel.vectDot(vectGravityRef));
    vectAccelAxis=vectAccel;
    vectAccelAxis.vectCross(vectGravityRef);

    vectAccelAxis.normalize();

    accelRoter.w=cosf(accelAngle/2);
    accelRoter.x=sinf(accelAngle/2)*vectAccelAxis.x;
    accelRoter.y=sinf(accelAngle/2)*vectAccelAxis.y;
    accelRoter.z=sinf(accelAngle/2)*vectAccelAxis.z;

    tmpQuat=accelRoter;
    tmpQuat2.con(accelRoter);

    vectMag.mul(tmpQuat2);
    tmpQuat.mul(vectMag);

    magAngle=atan2f(tmpQuat.y,tmpQuat.x);

    magRoter.w=cosf(-magAngle/2);
    magRoter.x=0.0f;
    magRoter.y=0.0f;
    magRoter.z=sinf(-magAngle/2);

    magRoter.mul(accelRoter);

	q0=magRoter.w;
	q1=magRoter.x;
	q2=magRoter.y;
	q3=magRoter.z;
	updateAttitude();
}

void OrientationFilter::filterUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	updateBeta();

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		OrientationFilter::MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
//	// local system variables
//	float norm; // vector norm
//	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
//	float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
//	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
//	J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
//	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
//	float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
//	float h_x, h_y, h_z; // computed flux in the earth frame
//	// axulirary variables to avoid reapeated calcualtions
//	float halfSEq_1,halfSEq_2,halfSEq_3,halfSEq_4;
//	float twoSEq_1,twoSEq_2,twoSEq_3,twoSEq_4;
//	float twob_x,twob_z;
//	float twob_xSEq_1,twob_xSEq_2,twob_xSEq_3,twob_xSEq_4;
//	float twob_zSEq_1,twob_zSEq_2,twob_zSEq_3,twob_zSEq_4;
//	float SEq_1SEq_2,SEq_1SEq_3,SEq_1SEq_4,SEq_2SEq_3,SEq_2SEq_4,SEq_3SEq_4;
//	float twom_x,twom_y,twom_z;
//
//	halfSEq_1 = 0.5f * SEq_1;
//	halfSEq_2 = 0.5f * SEq_2;
//	halfSEq_3 = 0.5f * SEq_3;
//	halfSEq_4 = 0.5f * SEq_4;
//	twoSEq_1 = 2.0f * SEq_1;
//	twoSEq_2 = 2.0f * SEq_2;
//	twoSEq_3 = 2.0f * SEq_3;
//	twoSEq_4 = 2.0f * SEq_4;
//	twob_x = 2.0f * b_x;
//	twob_z = 2.0f * b_z;
//	twob_xSEq_1 = 2.0f * b_x * SEq_1;
//	twob_xSEq_2 = 2.0f * b_x * SEq_2;
//	twob_xSEq_3 = 2.0f * b_x * SEq_3;
//	twob_xSEq_4 = 2.0f * b_x * SEq_4;
//	twob_zSEq_1 = 2.0f * b_z * SEq_1;
//	twob_zSEq_2 = 2.0f * b_z * SEq_2;
//	twob_zSEq_3 = 2.0f * b_z * SEq_3;
//	twob_zSEq_4 = 2.0f * b_z * SEq_4;
//	SEq_1SEq_3 = SEq_1 * SEq_3;
//	SEq_2SEq_4 = SEq_2 * SEq_4;
//	twom_x = 2.0f * m_x;
//	twom_y = 2.0f * m_y;
//	twom_z = 2.0f * m_z;
//
//	// normalise the accelerometer measurement
//	norm = sqrtf(a_x * a_x + a_y * a_y + a_z * a_z);
//	a_x /= norm;
//	a_y /= norm;
//	a_z /= norm;
//	// normalise the magnetometer measurement
//	norm = sqrtf(m_x * m_x + m_y * m_y + m_z * m_z);
//	m_x /= norm;
//	m_y /= norm;
//	m_z /= norm;
//	// compute the objective function and Jacobian
//	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
//	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
//	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
//	f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
//	f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
//	f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
//	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
//	J_12or23 = 2.0f * SEq_4;
//	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
//	J_14or21 = twoSEq_2;
//	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
//	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
//	J_41 = twob_zSEq_3; // negated in matrix multiplication
//	J_42 = twob_zSEq_4;
//	J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
//	J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
//	J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
//	J_52 = twob_xSEq_3 + twob_zSEq_1;
//	J_53 = twob_xSEq_2 + twob_zSEq_4;
//	J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
//	J_61 = twob_xSEq_3;
//	J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
//	J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
//	J_64 = twob_xSEq_2;
//	// compute the gradient (matrix multiplication)
//	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
//	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
//	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
//	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
//	// normalise the gradient to estimate direction of the gyroscope error
//	norm = sqrtf(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
//	SEqHatDot_1 = SEqHatDot_1 / norm;
//	SEqHatDot_2 = SEqHatDot_2 / norm;
//	SEqHatDot_3 = SEqHatDot_3 / norm;
//	SEqHatDot_4 = SEqHatDot_4 / norm;
//	// compute angular estimated direction of the gyroscope error
//	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
//	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
//	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
//	// compute and remove the gyroscope baises
//	w_bx += w_err_x * deltat * zeta;
//	w_by += w_err_y * deltat * zeta;
//	w_bz += w_err_z * deltat * zeta;
//	w_x -= w_bx;
//	w_y -= w_by;
//	w_z -= w_bz;
//	// compute the quaternion rate measured by gyroscopes
//	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
//	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
//	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
//	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
//	// compute then integrate the estimated quaternion rate
//	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
//	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
//	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
//	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
//	// normalise quaternion
//	norm = sqrtf(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
//	SEq_1 /= norm;
//	SEq_2 /= norm;
//	SEq_3 /= norm;
//	SEq_4 /= norm;
//
//	// compute flux in the earth frame
//	SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
//	SEq_1SEq_3 = SEq_1 * SEq_3;
//	SEq_1SEq_4 = SEq_1 * SEq_4;
//	SEq_3SEq_4 = SEq_3 * SEq_4;
//	SEq_2SEq_3 = SEq_2 * SEq_3;
//	SEq_2SEq_4 = SEq_2 * SEq_4;
//	h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
//	h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
//	h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
//	// normalise the flux vector to have only components in the x and z
//	b_x = sqrtf((h_x * h_x) + (h_y * h_y));
//	b_z = h_z;

	updateAttitude();
}

void OrientationFilter::MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

float OrientationFilter::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

Quaternion OrientationFilter::getAttitude(){
	return attitude;
}
float OrientationFilter::getGyroBias(int axis){
	if(axis==0){
		return w_bx;
	}else if(axis==1){
		return w_by;
	}else if(axis==2){
		return w_bz;
	}else{
		return 0;
	}
}
