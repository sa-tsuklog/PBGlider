// I2C.c
// RX62N
#include "mathf.h"
#include "HalI2C.hpp"
#include "../../GeneralConfig.hpp"
#include "../../Util.hpp"

I2C_Packet I2C::i2cPacketBuf[I2C_BUFFER_SIZE];

volatile I2C::I2C_STATUS I2C::i2cStatus;

volatile int I2C::sentPointer=0;
volatile int I2C::writtenPointer=0;

volatile char I2C::overrunFlag=0;
volatile char I2C::bufferEmptyFlag=1;

volatile int I2C::busyCount=0;

void I2C::initRIIC0(void)
{
	i2cStatus=IDLE;
	sentPointer=0;
	writtenPointer=0;

	overrunFlag=0;
	bufferEmptyFlag=1;


	SYSTEM.MSTPCRB.BIT.MSTPB21 = 0;

    RIIC0.ICCR1.BIT.ICE = 0;
	/* Set port direction to input */
	PORT1.DDR.BIT.B3 = 0;			/* SDA0 */
	PORT1.DDR.BIT.B2 = 0;			/* SCL0 */
	/* Enable the input buffer */
	PORT1.ICR.BIT.B3 = 1;			/* SDA0 */
	PORT1.ICR.BIT.B2 = 1;			/* SCL0 */

	RIIC0.ICCR1.BIT.ICE = 0;			/* RIIC disable for RIIC initial */;
	RIIC0.ICCR1.BIT.IICRST = 1;		/* RIIC all reset */
	RIIC0.ICCR1.BIT.IICRST = 0;		/* Clear reset */

	/* Disable all address detection. This sample code is for only master mode */
	RIIC0.ICSER.BYTE = 0x00;

	/* Enable detection of Master Arbitration lost */
	//RIIC0.ICFER.BIT.MALE = 1;
	RIIC0.ICFER.BIT.NACKE=1;

	/* Enable detection of Timeout */
	RIIC0.ICFER.BIT.TMOE = 1;
	RIIC0.ICIER.BIT.TMOIE = 1;	// Level interrupt
	RIIC0.ICMR2.BIT.TMOS = 0; // Long Mode TO
	RIIC0.ICSR2.BIT.TMOF = 0;

	RIIC0.ICIER.BIT.ALIE=1;
	RIIC0.ICIER.BIT.STIE=1;
	RIIC0.ICIER.BIT.SPIE=1;
	RIIC0.ICIER.BIT.NAKIE=1;

	/* Set interrupt for ICU */
	IPR(RIIC0, ICEEI0) = GeneralConfig::i2cIpr;	// ICEEI0 interrupt level
	IR(RIIC0, ICEEI0) = 0;		// ICEEI0 clear interrupt flag
	IEN(RIIC0, ICEEI0) = 1;		// ICEEI0 interrupt enable
	
	RIIC0.ICIER.BIT.RIE=1;
	IPR(RIIC0, ICRXI0)=GeneralConfig::i2cIpr;
	IEN(RIIC0, ICRXI0)=1;
	
	RIIC0.ICIER.BIT.TEIE=1;
	IPR(RIIC0, ICTEI0)=GeneralConfig::i2cIpr;
	IEN(RIIC0, ICTEI0)=1;
	
	RIIC0.ICIER.BIT.TIE=1;
	IPR(RIIC0, ICTXI0)=GeneralConfig::i2cIpr;
	IEN(RIIC0, ICTXI0)=1;
	
	
	RIIC0.ICMR1.BIT.CKS = 2;		/* ��400kbps */
	//RIIC0.ICMR1.BIT.CKS = 4;		/* ��100kbps */
	//RIIC0.ICBRH.BIT.BRH = 7;			/* Set High width of SCL */
	//RIIC0.ICBRL.BIT.BRL = 16;		/* Set Low width of SCL */
	RIIC0.ICBRH.BIT.BRH = 12;			/* Set High width of SCL */
	RIIC0.ICBRL.BIT.BRL = 12;		/* Set Low width of SCL */
	/* ACKWP is protect bit for ACKBT. */
	RIIC0.ICMR3.BIT.ACKWP = 1;		/* Disable protect for ACKBT */
	RIIC0.ICCR1.BIT.ICE = 1;			/* Enable RIIC */
	
	busRescue();
}

void I2C::busRescue(){
	while(RIIC0.ICCR1.BIT.SDAI==0){
		while(RIIC0.ICCR1.BIT.CLO==1){}
		RIIC0.ICCR1.BIT.CLO=1;
	}
	RIIC0.ICCR2.BIT.SP=1;
}

void I2C::busStart(){
	if(!RIIC0.ICCR2.BIT.BBSY){
		////stdout->putString("sendStart\n\r");
		RIIC0.ICCR2.BIT.ST=1;
		busyCount=0;
	}else{
		busyCount++;
		//if(30<busyCount){
			//stdout->putString("busy\n\r");
		//}
	}
}

#pragma interrupt I2C::ICEEI0(vect=VECT(RIIC0, ICEEI0),enable)
void I2C::ICEEI0(void){
	if(RIIC0.ICSR2.BIT.AL==1){
		i2cStatus=IDLE;
		RIIC0.ICSR2.BIT.AL&=0;
		//stdout->putString("AL\n\r");
	}
	if(RIIC0.ICSR2.BIT.STOP==1){
		i2cStatus=IDLE;
		RIIC0.ICSR2.BIT.NACKF&=0;
		RIIC0.ICSR2.BIT.STOP&=0;
		//stdout->putString("stop obserbed\n\r");
		
		sentPointer=(sentPointer+1)%I2C_BUFFER_SIZE;
		volatile int i;
		for(i=0;i<4;i++){}
		
		if(writtenPointer!=sentPointer){
			busStart();
		}else{
			bufferEmptyFlag=1;
		}
	}
	if(RIIC0.ICSR2.BIT.START==1){
		RIIC0.ICSR2.BIT.START&=0;
		
		if(i2cStatus==IDLE){	
			i2cStatus=START_SENT;
			//stdout->putString("start obserbed\n\r");
		}else if(i2cStatus==R_MADR_SENT){
			i2cStatus=RESTART_SENT;
			//stdout->putString("Restart obserbed\n\r");
		}else{
			//stdout->putString("???\n\r");
		}
	}
	if(RIIC0.ICSR2.BIT.NACKF==1){
		i2cStatus=IDLE;
		//unsigned char dummy=RIIC0.ICDRR;
		
		//stdout->putString("nack obserbed\n\r");
		RIIC0.ICCR2.BIT.SP=1;
		RIIC0.ICSR2.BIT.NACKF&=0;
	}
	if(RIIC0.ICSR2.BIT.TMOF==1){
		RIIC0.ICSR2.BIT.TMOF&=0;
		//stdout->putString("TMOF\n\r");
		Util::myWait(10);
		initRIIC0();
	}
}

#pragma interrupt I2C::ICRXI0(vect=VECT( RIIC0, ICRXI0),enable)
void I2C::ICRXI0(){
	unsigned char dummy;
	RIIC0.ICSR2.BIT.RDRF&=0;
	
	if(i2cStatus==SLAR_SET){
		i2cStatus=DUMMY_READ;
		dummy=RIIC0.ICDRR;
		RIIC0.ICMR3.BIT.ACKBT=1;
		RIIC0.ICMR3.BIT.WAIT=1;
		
		//stdout->putString("dummy read:");
		//stdout->putShort(dummy);
		//stdout->putString("\n\r");
	}else if(i2cStatus==DUMMY_READ){
		i2cStatus=DATA_RECEIVED;
		
		RIIC0.ICCR2.BIT.SP=1;
		*(i2cPacketBuf[sentPointer].getP_ReadData())=RIIC0.ICDRR;
		
		//stdout->putString("data read:");
		//stdout->putShort(dummy);
		//stdout->putString("\n\r");
	}
	
	
	
}
#pragma interrupt I2C::ICTEI0(vect=VECT( RIIC0, ICTEI0),enable)
void I2C::ICTEI0(){
	RIIC0.ICSR2.BIT.TEND&=0;
	
	//stdout->putString("ICTEI\n\r");
	if(i2cStatus==R_MADR_SET){
		i2cStatus=R_MADR_SENT;
		RIIC0.ICCR2.BIT.RS=1;
		//stdout->putString("RS\n\r");
	}else if(i2cStatus==DATA_SET){
		RIIC0.ICCR2.BIT.SP=1;
		//stdout->putString("STOP\n\r");
	}
	
	
}
#pragma interrupt I2C::ICTXI0(vect=VECT(RIIC0,ICTXI0),enable)
void I2C::ICTXI0(){
	if(i2cStatus==START_SENT){
		i2cStatus=SLAW_SET;
		RIIC0.ICDRT=i2cPacketBuf[sentPointer].getDevAddress()&0xFE;
		
		//stdout->putString("sending slaw:");
		//stdout->putShortHex(i2cPacketBuf[sentPointer].getDevAddress()&0xFE);
		//stdout->putString("\n\r");
	}else if(i2cStatus==SLAW_SET){
		RIIC0.ICDRT=i2cPacketBuf[sentPointer].getMemAddress();
		if(i2cPacketBuf[sentPointer].getRw()==1){	
			i2cStatus=R_MADR_SET;
			
			
			//stdout->putString("sending r_maddr:");
			//stdout->putShortHex(i2cPacketBuf[sentPointer].getMemAddress());
			//stdout->putString("\n\r");
		}else{
			i2cStatus=W_MADR_SET;
			
			//stdout->putString("sending w_maddr\n\r");
		}
				
	}else if(i2cStatus==RESTART_SENT){
		i2cStatus=SLAR_SET;
		RIIC0.ICDRT=i2cPacketBuf[sentPointer].getDevAddress()|i2cPacketBuf[sentPointer].getRw();
		
		//stdout->putString("SLAR2:");
		//stdout->putShortHex(i2cPacketBuf[sentPointer].getDevAddress()|i2cPacketBuf[sentPointer].getRw());
		//stdout->putString("\n\r");

	}else if(i2cStatus==W_MADR_SET){
		i2cStatus=DATA_SET;		
		RIIC0.ICDRT=i2cPacketBuf[sentPointer].getWriteData();
		//stdout->putString("sending data\n\r");
	}
}

void I2C::appendPacket(I2C_Packet packet){
	while(((writtenPointer+1)%I2C_BUFFER_SIZE)==sentPointer){
		//overrunFlag=1;
		//stdout->putString("i2c buffer overrun\n\r");
	}

	i2cPacketBuf[writtenPointer]=packet;
	writtenPointer=(writtenPointer+1)%I2C_BUFFER_SIZE;

	bufferEmptyFlag=0;

	busStart();
}

void I2C::i2cDevRead(unsigned char sadr, unsigned char madr,unsigned char* pReadData) 
{
	I2C_Packet packet = I2C_Packet::genReadPacket(sadr,madr,pReadData);
	appendPacket(packet);
}

void I2C::i2cDevWrite(unsigned char sadr, unsigned char madr, unsigned char data) 
{
	I2C_Packet packet = I2C_Packet::genWritePacket(sadr,madr,data);
	appendPacket(packet);
}

char I2C::isBufferEmpty(){
	return bufferEmptyFlag;
}
