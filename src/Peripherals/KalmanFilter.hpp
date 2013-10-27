/*
 * KalmanFilter.hpp
 *
 *  Created on: 2013/08/18
 *      Author: sa
 */

#ifndef KALMANFILTER_HPP_
#define KALMANFILTER_HPP_

#include "Matrix3f.hpp"

class KalmanFilter{
private:
	float hzUpdateRate;
	Matrix3f xk;
	Matrix3f uk;
	Matrix3f qk;
	Matrix3f rk;
	Matrix3f fk;
	Matrix3f pk;
	Matrix3f zk;
	Matrix3f hk;
	Matrix3f hkTrans;

	float aclSigma;
	float posSigma;
	float spdSigma;
	float aclBiasSigma;

public:
	KalmanFilter(float hzUpdateRate,float aclSigma,float posSigma,float spdSigma,float aclBiasSigma);
	~KalmanFilter();

	void reset();
	void predict(float gAcl);
	void update(float mpsSpd,float mRelativePos);
	void test();

	float getMpsSpd();
	float getM_RelPos();
};





#endif /* KALMANFILTER_HPP_ */
