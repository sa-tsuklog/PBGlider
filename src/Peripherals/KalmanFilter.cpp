/*
 * KalmanFilter.cpp
 *
 *  Created on: 2013/08/18
 *      Author: sa
 */

#include "KalmanFilter.hpp"
#include "Matrix3f.hpp"

KalmanFilter::KalmanFilter(float hzUpdateRate,float aclSigma,float posSigma,float spdSigma,float aclBiasSigma){
	this->aclSigma = aclSigma;
	this->spdSigma = spdSigma;
	this->posSigma = posSigma;
	this->aclBiasSigma = aclBiasSigma;

	this->hzUpdateRate = hzUpdateRate;
	reset();
}

KalmanFilter::~KalmanFilter(){

}

void KalmanFilter::reset(){
	xk = Matrix3f(0,3,1);
	fk = Matrix3f();
	fk.setNum(1.0/hzUpdateRate,0,1);
	fk.setNum(1.0/hzUpdateRate/hzUpdateRate/2,0,2);
	fk.setNum(1.0/hzUpdateRate,1,2);

	qk = Matrix3f(0,3,3);
	qk.setNum(1.0/hzUpdateRate/hzUpdateRate/hzUpdateRate/hzUpdateRate/4,0,0);
	qk.setNum(1.0/hzUpdateRate/hzUpdateRate/hzUpdateRate/2,0,1);
	qk.setNum(1.0/hzUpdateRate/hzUpdateRate/hzUpdateRate/2,1,0);
	qk.setNum(1.0/hzUpdateRate/hzUpdateRate,1,1);
	qk = qk.mul(aclSigma*aclSigma);
	qk.setNum(aclBiasSigma*aclBiasSigma,2,2);

	rk = Matrix3f(2,2);
	rk.setNum(posSigma*posSigma , 0,0);
	rk.setNum(spdSigma*spdSigma , 1,1);

	pk = Matrix3f();
	pk.setNum(posSigma , 0,0);
	pk.setNum(spdSigma , 1,1);
	pk.setNum(aclBiasSigma, 2,2);

	hk=Matrix3f(2,3);
	hkTrans=hk.trans();

	uk=Matrix3f(3,1);
	zk=Matrix3f(2,1);
}

void KalmanFilter::predict(float gAcl){
	uk.setNum(gAcl/hzUpdateRate/hzUpdateRate/2,0,0);
	uk.setNum(gAcl/hzUpdateRate,1,0);
	uk.setNum(0,2,0);

	xk = fk.mul(xk).add(uk);
	pk = fk.mul(pk).mul(fk.trans()).add(qk);
}

void KalmanFilter::update(float mpsSpd,float mRelativePos){
	zk.setNum(mRelativePos,0,0);
	zk.setNum(mpsSpd,1,0);

	Matrix3f ek;
	Matrix3f sk;
	Matrix3f kk;
	Matrix3f unit=Matrix3f();

	ek = zk.sub(hk.mul(xk));
	sk = rk.add(hk.mul(pk).mul(hkTrans));

	kk = pk.mul(hkTrans.mul(sk.inverse()));
	xk = xk.add(kk.mul(ek));
	pk = (unit.sub(kk.mul(hk))).mul(pk);
}

void KalmanFilter::test(){
	Matrix3f a = Matrix3f();
	a.setNum(2,0,1);
	a.setNum(6,2,1);

	Matrix3f b = a.inverse();

	Matrix3f c = a.mul(b);

	c.print();

}

float KalmanFilter::getMpsSpd(){
	return xk.getNum(1,0);
}
float KalmanFilter::getM_RelPos(){
	return xk.getNum(0,0);
}
