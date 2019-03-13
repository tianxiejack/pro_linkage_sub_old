
#include "EventLoop.h"

extern CProcess *proc;

pthread_cond_t event_cond;
pthread_mutex_t event_mutex;

EventLoop::EventLoop(CProcess *ptr):m_ptrProc(ptr)
{
	
}
EventLoop::EventLoop()
{

}
EventLoop::~EventLoop()
{
	StopService();
	
	if(m_ptrProc != NULL){
		delete m_ptrProc;
		m_ptrProc = NULL;
	}
}
int EventLoop::RunService()
{
	m_prm.pThis = this;
	return RunThread(RunProxy, &m_prm);
}

int EventLoop::StopService()
{
	 SetThreadExit();
	 WaitThreadExit();
	 return 0;
}
int EventLoop::Init()
{
	int ret =0;
	ret = pthread_mutex_init(&event_mutex, NULL);
	if(ret !=0)
	{
		printf("[%s]:====== >> Event Mutex Init Failed !!!\r\n",__FUNCTION__);
		return -1;
	}
	ret = pthread_cond_init(&event_cond, NULL);
	if(ret !=0)
	{
		printf("[%s]:====== >> Event Condition Init Failed !!!\r\n",__FUNCTION__);
		return -1;
	}

	
	return 0;
}
void* EventLoop::RunProxy(void* pArg)
{
	 struct timeval tv;
	 struct RunPrm *pPrm = (struct RunPrm*)pArg;
	 while(pPrm->pThis->m_bRun)
	 {		
	 
	 	tv.tv_sec = 0;
    		tv.tv_usec = (30%1000)*1000;
   		select(0, NULL, NULL, NULL, &tv);
		
		pPrm->pThis->Run();
	 }
	 return NULL;
}
int EventLoop::Run()
{
	printf("\r\n[%s]:waiting for cond signal ... ...\r\n",__FUNCTION__);
	pthread_mutex_lock(&event_mutex);
	//gettimeofday(&now, NULL);
	//outtime.tv_sec = now.tv_sec + MAX_EVENT_LOOP_TIMEOUT;
	//outtime.tv_nsec = now.tv_usec * 1000;
	
	pthread_cond_wait(&event_cond, &event_mutex);//, &outtime);


	do {
		if(m_ptrProc != NULL){
			m_ptrProc->MoveBall();
		}

	}while(0);
	
	pthread_mutex_unlock(&event_mutex);
}

