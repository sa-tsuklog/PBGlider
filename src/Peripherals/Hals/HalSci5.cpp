#include "HalSCI5.hpp"

#include "../../GeneralConfig.hpp"

HalSci5* HalSci5::halSci5=new HalSci5();

HalSci5::HalSci5(){
	int i;

	PORTC.ICR.BIT.B2 = 1;
	PORTC.DDR.BIT.B2 = 0;
	PORTC.DDR.BIT.B3 = 1;

	MSTP(SCI5) = 0;

	SCI5.SCR.BYTE = 0x00;

	SCI5.SMR.BYTE = 0x00;

	SCI5.SCMR.BIT.SMIF= 0;
	SCI5.SCMR.BIT.SINV= 0;
	SCI5.SCMR.BIT.SDIR= 0;

	SCI5.BRR = 12;
	//SCI5.SEMR.BIT.ABCS = 1;
	for(i=0;i<0x800000;i++);
	SCI5.SCR.BYTE = 0x30;

	SCI5.SCR.BIT.TE=1;
	SCI5.SCR.BIT.TIE=1;

	SCI5.SCR.BIT.RE=1;
	SCI5.SCR.BIT.RIE=1;


	IEN(SCI5,RXI5)=0;
	IEN(SCI5,TXI5)=1;
	IPR(SCI5,TXI5)=GeneralConfig::gpsIpr;
	IPR(SCI5,RXI5)=GeneralConfig::gpsIpr;
}
inline void HalSci5::lock(){
	IEN(SCI5,TXI5)=0;
}
inline void HalSci5::unlock(){
	IEN(SCI5,TXI5)=1;
}

void HalSci5::sendNext(){
	if(SCI5.SSR.BYTE&0x80 && (!isTxBufferEmpty())){
		SCI5.TDR=dequeTxBuffer();
	}
}
void HalSci5::nativeSend(char c){
	if(SCI5.SSR.BYTE&0x80){
		SCI5.TDR=c;
	}
}
void HalSci5::received(){
	char buf = SCI5.RDR;
	SCI5.SSR.BYTE=SCI5.SSR.BYTE&0x80;
	appendRxBuf(buf);
	//stdout->putChar(buf);
}

void HalSci5::start(){
	SCI5.SSR.BYTE=SCI5.SSR.BYTE&0x80;
	IEN(SCI5,RXI5)=1;
}

HalSci5* HalSci5::getInstance(){
	return halSci5;
}

#pragma interrupt HalSci5::txi(vect=VECT(SCI5,TXI5),enable)
void HalSci5::txi(){
	HalSci5::getInstance()->sendNext();
}
#pragma interrupt HalSci5::rxi(vect=VECT(SCI5,RXI5),enable)
void HalSci5::rxi(){
	HalSci5::getInstance()->received();
}
