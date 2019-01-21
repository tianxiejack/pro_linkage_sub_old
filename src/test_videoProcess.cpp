
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <glut.h>
#include "process51.hpp"
#include "Ipc.hpp"
#include "msgDriv.h"
#include <sys/time.h>

#include <fcntl.h>     
#include <termios.h>    
#include <errno.h>  
#include <time.h> 
#include <signal.h>
#include "EventLoop.h"

using namespace std;
using namespace cv;

CProcess *proc = NULL;
EventLoop *eventLoop = NULL;
bool startEnable = false;

volatile bool cloneOneFrame = false;

uint32 count1=0;
static void timer_op(int signum)
{   
   if( count1 == 60){
   		cloneOneFrame = true;		
		count1 = 0;	
   }   
   count1 ++;	   
}

int timer_init(void)
{
    struct itimerval value, ovalue;   
    signal(SIGALRM, timer_op);     
    value.it_value.tv_sec = 1;  
    value.it_value.tv_usec = 0;//10000;  
    value.it_interval.tv_sec = 0;  
    value.it_interval.tv_usec = 10000; 
    setitimer(ITIMER_REAL, &value, &ovalue);  //set the timer           
    return 0;
}

int main(int argc, char **argv)
{
	int returnValue;
	struct timeval tv;	
	MSGDRIV_create();
#ifdef __IPC__
	Ipc_pthread_start();
#endif

	if(0 != timer_init()){
		printf("[%s]:XXX: Create timer faliled !!!\r\n",__FUNCTION__);
	}
	
	//proc = new CProcess(vdisWH[0][0], vdisWH[0][1]);
	proc = new CProcess(outputWHF[0], outputWHF[1]);
	if(proc == NULL){
		printf("\r\n[%s]:XXX: Create Main App Failed !!!",__FUNCTION__);
		return -1;
	}
	
	while(false == startEnable)
	{
		tv.tv_sec = 0;
		tv.tv_usec = 50000;
		select( 0, NULL, NULL, NULL, &tv );
	}
	proc->loadIPCParam();
	proc->creat();
	proc->init();
	proc->run();
	
	eventLoop = new EventLoop(proc);
	if(eventLoop == NULL){
		printf("\r\n[%s]:XXX: Create Event Loop Failed !!!",__FUNCTION__);
	}
	else{
		returnValue = eventLoop->Init();
		if(returnValue != -1) {
			eventLoop->RunService();
		}
	}	
	
	glutMainLoop();
	
	if(proc != NULL) {
		proc->destroy();
	}	
	if( eventLoop != NULL ){
		eventLoop->StopService();
	}	
#ifdef __IPC__
	Ipc_pthread_stop();
#endif
    	return 0;
}



//__IPC__
//__MOVE_DETECT__
//__TRACK__

//---------------------------
//__MMT__
//__BLOCK__

