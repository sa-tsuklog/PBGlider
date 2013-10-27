/*
 * HalRangeFinderMtu.cpp
 *
 *  Created on: 2012/11/03
 *      Author: sa
 */

#include "HalRangeFinderMtu.hpp"

HalRangeFinderMtu* HalRangeFinderMtu::halRangeFinderMtu=new HalRangeFinderMtu();

void HalRangeFinderMtu::initModuleMtu4(){
	PORT2.DDR.BIT.B4 = 0;
	PORT2.DDR.BIT.B5 = 0;
	PORT2.ICR.BIT.B4 = 1;
	PORT2.ICR.BIT.B5 = 1;

	MSTP(MTU4)=0;

	MTU4.TCR.BIT.TPSC=4;
	MTU4.TCR.BIT.CKEG=2;
	MTU4.TCR.BIT.CCLR=1;//clear by input capture on MTIOC4A

	MTU4.TIORH.BIT.IOA=8;//input capture on rising edge;
	MTU4.TIORL.BIT.IOC=9;//input capture on falling edge;

	MTU4.TIER.BIT.TGIEC=1;
	IEN(MTU4,TGIC4)=1;
	IPR(MTU4,TGIC4)=1;
}

HalRangeFinderMtu::HalRangeFinderMtu(){
	stop();
	initModuleMtu4();
}

HalRangeFinderMtu* HalRangeFinderMtu::getInstance(){
	return HalRangeFinderMtu::halRangeFinderMtu;
}

void HalRangeFinderMtu::start(){
	MTUA.TSTR.BIT.CST4=1;
}

void HalRangeFinderMtu::stop(){
	MTUA.TSTR.BIT.CST4=0;
}

float HalRangeFinderMtu::getPulseWidth(){
	return pulseWidth;
}

float HalRangeFinderMtu::getPulseWidthPrev(){
	return pulseWidthPrev;
}

void HalRangeFinderMtu::updatePulseWidth(){
	pulseWidthPrev=pulseWidth;
	pulseWidth=MTU4.TGRC/(48000000.0/128);
}

#pragma interrupt HalRangeFinderMtu::TGIC4(vect=VECT(MTU4, TGIC4),enable)
void HalRangeFinderMtu::TGIC4(void){
	HalRangeFinderMtu::getInstance()->updatePulseWidth();
}

