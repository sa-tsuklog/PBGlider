#ifndef __I2C_PACKET
#define __I2C_PACKET

class I2C_Packet{
private:
	unsigned char devAddress; //8bit address.
	unsigned char rw;
	unsigned char memAddress;
	unsigned char writeData;
	unsigned char* pReadData;
	
	I2C_Packet(unsigned char devAddress,unsigned char memAddress,unsigned char writeData){
		this->devAddress=devAddress;
		this->memAddress=memAddress;
		this->writeData=writeData;
		this->rw=0;
	}
	I2C_Packet(unsigned char devAddress,unsigned char memAddress,unsigned char* pReadData){
		this->devAddress=devAddress;
		this->memAddress=memAddress;
		this->pReadData=pReadData;
		this->rw=1;
	}
public:
	I2C_Packet(){
		this->devAddress=0;
		this->memAddress=0;
		this->writeData=0;
		this->rw=0;
	}
	static I2C_Packet genWritePacket(unsigned char devAddress,unsigned char memAddress,unsigned char writeData){
		I2C_Packet wPacket(devAddress,memAddress,writeData);
		return wPacket;
	}
	static I2C_Packet genReadPacket(unsigned char devAddress,unsigned char memAddress,unsigned char* pReadData){
		I2C_Packet rPacket(devAddress,memAddress,pReadData);
		return rPacket;
	}
	unsigned char getDevAddress(){
		return this->devAddress;
	}
	unsigned char getRw(){
		return this->rw;
	}
	unsigned char getMemAddress(){
		return this->memAddress;
	}
	unsigned char getWriteData(){
		return this->writeData;
	}
	unsigned char* getP_ReadData(){
		return this->pReadData;
	}
};

#endif
