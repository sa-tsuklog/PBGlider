///*
// * KalmanFilter.cpp
// *
// *  Created on: 2013/08/18
// *      Author: sa
// */
//
//#include "KalmanFilter.hpp"
//#include "Matrix3f.hpp"
//
//KalmanFilter::KalmanFilter(float hzUpdateRate){
//	aclSigma = 0.1;
//	spdSigma = 1.0;
//	posSigma = 10.0;
//
//
//	this->hzUpdateRate = hzUpdateRate;
//
//	xk = Matrix3f(0,2,1);
//	fk = Matrix3f();
//	fk.setNum(1.0/hzUpdateRate,0,1);
//	qk = Matrix3f();
//	qk.setNum(1.0/hzUpdateRate/hzUpdateRate/hzUpdateRate/hzUpdateRate/4,0,0);
//	qk.setNum(1.0/hzUpdateRate/hzUpdateRate/hzUpdateRate/2,0,1);
//	qk.setNum(1.0/hzUpdateRate/hzUpdateRate/hzUpdateRate/2,0,1);
//	qk.setNum(1.0/hzUpdateRate/hzUpdateRate,1,1);
//	qk = qk.mul(aclSigma*aclSigma);
//
//	rk = Matrix3f();
//	rk.setNum(posSigma*posSigma , 0,0);
//	rk.setNum(spdSigma*spdSigma , 1,1);
//
//	pk = Matrix3f();
//	pk.setNum(posSigma*posSigma , 0,0);
//	pk.setNum(spdSigma*spdSigma , 1,1);
//}
//
//KalmanFilter::~KalmanFilter(){
//
//}
//
//void KalmanFilter::predict(float gAcl){
//	uk.setNum(gAcl/hzUpdateRate/hzUpdateRate/2,0,0);
//	uk.setNum(gAcl/hzUpdateRate,1,0);
//	uk.setNum(0,2,0);
//
//	xk = fk.mul(xk).add(uk);
//	pk = fk.mul(pk).mul(fk.trans()).add(pk);
//}
//
//void KalmanFilter::update(float mpsSpd,float mRelativePos){
//	zk.setNum(mRelativePos,0,0);
//	zk.setNum(mpsSpd,1,0);
//
//	Matrix3f ek;
//	Matrix3f sk;
//	Matrix3f kk;
//	Matrix3f unit=Matrix3f();
//
//	ek = zk.sub(xk);
//	sk = rk.add(pk);
//
//	kk = pk.mul(sk.inverse());
//	xk = xk.add(kk.mul(ek));
//	pk = (unit.sub(kk)).mul(pk);
//}
//
//void KalmanFilter::test(){
//	Matrix3f a = Matrix3f();
//	a.setNum(2,0,1);
//	a.setNum(6,2,1);
//
//	Matrix3f b = a.inverse();
//
//	Matrix3f c = a.mul(b);
//
//	c.print();
//
//}
//
//float KalmanFilter::getMpsSpd(){
//	return xk.getNum(1,0);
//}
//float KalmanFilter::getM_RelPos(){
//	return xk.getNum(0,0);
//}
