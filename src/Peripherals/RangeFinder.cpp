#include "RangeFinder.hpp"

RangeFinder::RangeFinder(HalRangeFinderMtu* halRangeFinderMtu){
	this->halRangeFinderMtu = halRangeFinderMtu;
}
RangeFinder::~RangeFinder(){
}

void RangeFinder::start(){
	halRangeFinderMtu->start();
}
void RangeFinder::stop(){
	halRangeFinderMtu->stop();
}

float RangeFinder::pulseWidthToM(float pulseWidth){
	return pulseWidth/58.0*1000000*0.01;
}

float RangeFinder::getM_Range(){
	return pulseWidthToM(halRangeFinderMtu->getPulseWidth());
}

float RangeFinder::getMpsSpeed(){
	float mRange=pulseWidthToM(halRangeFinderMtu->getPulseWidth());
	float mRangePrev=pulseWidthToM(halRangeFinderMtu->getPulseWidthPrev());
	return (mRange-mRangePrev)*halRangeFinderMtu->HZ_SAMPLERATE;
}

