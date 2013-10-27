/*****************************************/
/*RIIC0                                  */
/*                                       */
/*                                       */
/*****************************************/


#ifndef __I2C
#define __I2C

// I2C.c

#include "../../iodefine.h"
#include "../../GeneralConfig.hpp"
// #include "FuncProto.h"
#include "../Interfaces/I2C_Packet.hpp"

class I2C{
//private:
public:
	enum I2C_STATUS{
		IDLE,
		START_SENT,
		RESTART_SENT,
		
		SLAW_SET,
		
		R_MADR_SET,
		R_MADR_SENT,
		W_MADR_SET,
		
		SLAR_SET,
		
		DUMMY_READ,
		DATA_SET,
		DATA_RECEIVED
	};
	static const int I2C_BUFFER_SIZE = 64;
	
	static I2C_Packet i2cPacketBuf[I2C_BUFFER_SIZE];
	static volatile I2C_STATUS i2cStatus;
	
	static volatile int sentPointer;
	static volatile int writtenPointer;
	
	static volatile char overrunFlag;
	static volatile char bufferEmptyFlag;
	
	static volatile int busyCount;

	//interrupt handlers.
	static void ICEEI0();
	static void RiicTMO();
	static void ICRXI0();
	static void ICTEI0();
	static void ICTXI0();
	static void busStart();
	static void busRescue();
	static void appendPacket(I2C_Packet packet);
public:
	static void initRIIC0();
	static void i2cDevWrite(unsigned char devAddress, unsigned char memAddress, unsigned char data);
	static void i2cDevRead(unsigned char devAddress, unsigned char memAddress, unsigned char* pData);
	static char isBufferEmpty();
	
};

#endif
