#ifndef __BUFFERED_SCI
#define __BUFFERED_SCI

#include "../../iodefine.h"
#include "stdlib.h"

class WrapperBufferedSci{
private:
	static const int TX_BUFFERSIZE=256;
	static const bool TX_WAIT_ON_BUFFERFULL = false;

	bool txOverrunFlag;
	int txSendingPointer;
	int txWritingPointer;
	char* txBuffer;

	inline virtual void lock(){};
	inline virtual void unlock(){};
	inline virtual void sendNext(){};
protected:
public:
	WrapperBufferedSci(){
		txOverrunFlag=false;
		txSendingPointer=0;
		txWritingPointer=0;
		txBuffer = new char[TX_BUFFERSIZE];
	}
	~WrapperBufferedSci(){
		delete txBuffer;
	}
	void putChar(char c){
		lock();

		if(TX_WAIT_ON_BUFFERFULL){
			while(((txWritingPointer+1)%TX_BUFFERSIZE)==txSendingPointer){
				sendNext();
			}
		}

		txBuffer[txWritingPointer]=c;
		txWritingPointer=(txWritingPointer+1)%TX_BUFFERSIZE;

		if(txWritingPointer==txSendingPointer){
			txOverrunFlag=1;
		}

		sendNext();

		unlock();
	}
	char dequeTxBuffer(){
		char c = txBuffer[txSendingPointer];
		txSendingPointer = (txSendingPointer+1)%TX_BUFFERSIZE;
		return c;
	}
	bool isTxBufferEmpty(){
		if(txWritingPointer == txSendingPointer){
			return true;
		}else{
			return false;
		}
	}
};

#endif
