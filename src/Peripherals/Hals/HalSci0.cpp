#include "HalSCI0.hpp"
#include "../../GeneralConfig.hpp"

HalSci0* HalSci0::halSci0=new HalSci0();

HalSci0::HalSci0(){
	int i;

	PORT2.ICR.BIT.B1 = 1;
	PORT2.DDR.BIT.B1 = 0;

	MSTP(SCI0) = 0;

	SCI0.SCR.BYTE = 0x00;

	SCI0.SMR.BYTE = 0x00;

	SCI0.SCMR.BIT.SMIF= 0;
	SCI0.SCMR.BIT.SINV= 0;
	SCI0.SCMR.BIT.SDIR= 0;

	SCI0.BRR = 12;
	//SCI0.SEMR.BIT.ABCS = 1;
	for(i=0;i<0x800000;i++);
	SCI0.SCR.BYTE = 0x30;

	SCI0.SCR.BIT.TE=1;
	SCI0.SCR.BIT.TIE=1;

	SCI0.SCR.BIT.RE=1;
	SCI0.SCR.BIT.RIE=1;


	IEN(SCI0,RXI0)=0;
	IEN(SCI0,TXI0)=1;
	IPR(SCI0,TXI0)=GeneralConfig::uartIpr;
	IPR(SCI0,RXI0)=GeneralConfig::uartIpr;

	SCI0.SSR.BYTE=(char)(SCI0.SSR.BYTE & 0x80);

	echo =0;
}

inline void HalSci0::lock(){
	IEN(SCI0,TXI0)=0;
}
inline void HalSci0::unlock(){
	IEN(SCI0,TXI0)=1;
}

void HalSci0::sendNext(){
	if((SCI0.SSR.BYTE&0x80) && (!isTxBufferEmpty())){
		SCI0.TDR=dequeTxBuffer();
	}
}
void HalSci0::nativeSend(char c){
	if(SCI0.SSR.BYTE&0x80){
		SCI0.TDR=c;
	}
}
void HalSci0::received(){
	char buf = SCI0.RDR;
	SCI0.SSR.BYTE=(char)(SCI0.SSR.BYTE & 0x80);

	if(echo){
		if(buf == '\r'){
			stdout->putString("\n\r");
		}else{
			stdout->putChar(buf);
		}
	}

	appendRxBuf(buf);
}

void HalSci0::setEcho(int i){
	if(i){
		echo=1;
	}else{
		echo=0;
	}
}

void HalSci0::start(){
	SCI0.SSR.BYTE=(char)(SCI0.SSR.BYTE & 0x80);
	IEN(SCI0,RXI0)=1;
}

char HalSci0::getchar(){
	char buf;
	IEN(SCI0,RXI0)=0;
	while(!SCI0.SSR.BIT.RDRF){}
	buf = SCI0.RDR;
	SCI0.SSR.BYTE=(char)(SCI0.SSR.BYTE & 0x80);
	stdout->putChar(buf);
	IR(SCI0,RXI0)=0;
	IEN(SCI0,RXI0)=1;
	return buf;
}

HalSci0* HalSci0::getInstance(){
	return halSci0;
}

#pragma interrupt HalSci0::txi(vect=VECT(SCI0,TXI0),enable)
void HalSci0::txi(){
	HalSci0::getInstance()->sendNext();
}
#pragma interrupt HalSci0::rxi(vect=VECT(SCI0,RXI0),enable)
void HalSci0::rxi(){
	HalSci0::getInstance()->received();
}
