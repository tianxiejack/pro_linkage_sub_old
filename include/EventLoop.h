#ifndef __EVENT_LOOP_H__
#define __EVENT_LOOP_H__

/*
*	Function: event loop system
*	Author: SunWJ
*	Date: 20190115
*	Version: V1.0.0
*/
#include "WorkThread.h"
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <vector>
#include <fstream>
#include <string>
#include <pthread.h>
#include <stdlib.h>
#include <time.h>
#include "process51.hpp"
//class CProcess ;

const int MAX_EVENT_LOOP_TIMEOUT = 3;
class EventLoop:public WorkThread
{
public:
	EventLoop(CProcess * ptr);
	EventLoop();
	virtual ~EventLoop();

	struct RunPrm{
		EventLoop *pThis;
	};
	int RunService();
	int StopService();
	struct RunPrm m_prm;
	static void* RunProxy(void* pArg);
	
	int Run();
	int Init();
	
private:
	struct timeval now;
	struct timespec outtime;
	CProcess *m_ptrProc;

};
#endif

