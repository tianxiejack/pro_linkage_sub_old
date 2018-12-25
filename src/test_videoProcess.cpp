
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

using namespace std;
using namespace cv;

bool startEnable = false;

volatile bool cloneOneFrame = false;
uint32 count1,count2,count3,count4,count5;
static void timer_op(int signum)
{   
   if( count1 == 50){
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
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 50000;
	MSGDRIV_create();
#ifdef __IPC__
	Ipc_pthread_start();
#endif
	while(false == startEnable)
	{
		select( 0, NULL, NULL, NULL, &tv );
	};

	timer_init();
	CProcess proc;
	proc.creat();
	proc.init();
	proc.run();
	glutMainLoop();
	proc.destroy();
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

