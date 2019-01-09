
#include "IntrinsicMatrix.h"


IntrinsicMatrix::IntrinsicMatrix()
{
	
}
IntrinsicMatrix::~IntrinsicMatrix()
{
	StopService();
}
int IntrinsicMatrix::RunService()
{
	m_prm.pThis = this;
	return RunThread(RunProxy, &m_prm);
}

int IntrinsicMatrix::StopService()
{
	 SetThreadExit();
	 WaitThreadExit();
	 return 0;
}

void* IntrinsicMatrix::RunProxy(void* pArg)
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
int IntrinsicMatrix::Run()
{



}

