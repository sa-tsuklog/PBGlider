/*
 * EventHandler.hpp
 *
 *  Created on: 2012/09/17
 *      Author: sa
 */

#ifndef EVENTHANDLER_HPP_
#define EVENTHANDLER_HPP_

class EventHandler{
public:
	EventHandler(){}
	virtual ~EventHandler(){}
	virtual void actionPerformed()=0;
};

#endif /* EVENTHANDLER_HPP_ */
