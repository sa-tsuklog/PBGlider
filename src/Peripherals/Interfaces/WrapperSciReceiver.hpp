/*
 * SciReceiver.hpp
 *
 *  Created on: 2012/11/03
 *      Author: sa
 */

#ifndef SCIRECEIVER_HPP_
#define SCIRECEIVER_HPP_

#include "../Interfaces/EventHandler.hpp"

class WrapperSciReceiver{
private:
	static const int RX_BUFFERSIZE=128;

	EventHandler* crReceivedHandler;
	EventHandler* bufferFullHandler;

	bool inputDataAvailable;
	int rxReceivedPointer;
	int receivedSize;
	char* rxBuffer0;
	char* rxBuffer1;
	char* currentRxBuf;

	void crReceived(){
		if(crReceivedHandler!=0){
			crReceivedHandler->actionPerformed();
		}
	}
	void rxBufferFull(){
		if(bufferFullHandler!=0){
			bufferFullHandler->actionPerformed();
		}
	}

protected:
	WrapperSciReceiver(){
		inputDataAvailable=false;
		receivedSize = 0;
		rxReceivedPointer=0;
		rxBuffer0 = new char[RX_BUFFERSIZE];
		rxBuffer1 = new char[RX_BUFFERSIZE];
		currentRxBuf = rxBuffer0;
		if(rxBuffer0==0 || rxBuffer1==0){
			//Util::myException("lack of memory of rxBuffers\n\r");
		}
		crReceivedHandler=0;
		bufferFullHandler=0;
	}
	~WrapperSciReceiver(){
		delete rxBuffer0;
		delete rxBuffer1;
	}
	void appendRxBuf(char c){
		if(c=='\r'){
			receivedSize = rxReceivedPointer;
			currentRxBuf[rxReceivedPointer]=0;
			rxReceivedPointer=0;
			if(currentRxBuf == rxBuffer0){
				currentRxBuf = rxBuffer1;
			}else{
				currentRxBuf = rxBuffer0;
			}

			crReceived();
		}else if(c=='\n'){
			//do nothing.
		}else if(c==0x08){
			if(0<rxReceivedPointer){
				currentRxBuf[rxReceivedPointer]='0';
				rxReceivedPointer--;
			}else{
				currentRxBuf[rxReceivedPointer]='0';
			}
		}else{
			currentRxBuf[rxReceivedPointer]=c;
			rxReceivedPointer++;
		}
		if(RX_BUFFERSIZE<=rxReceivedPointer){
			rxBufferFull();
			rxReceivedPointer=0;
		}
	}
public:
	void setCrReceivedHandler(EventHandler* handler){
		this->crReceivedHandler=handler;
	}
	void setBufferfullHandler(EventHandler* handler){
		this->bufferFullHandler=handler;
	}
	void clearBuffer(){
		rxReceivedPointer=0;
	}
	char* getBuffer(){
		if(currentRxBuf == rxBuffer0){
			return rxBuffer1;
		}else{
			return rxBuffer0;
		}
	}
	int getBufferSize(){
		return receivedSize;
	}
	//virtual void start(){};
};

#endif /* SCIRECEIVER_HPP_ */
