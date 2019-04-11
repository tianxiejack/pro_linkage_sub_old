#include <glut.h>
#include "VideoProcess.hpp"
#include "vmath.h"
#include "arm_neon.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "app_ctrl.h"
#include "ipc_custom_head.hpp"
#include <vector>
#include <errno.h>
#include <string.h>
#include "EventLoop.h"
using namespace vmath;

extern unsigned char  g_GridMapMode;
extern int capIndex;
extern bool recvMtdConfigData;
extern MTD_Config g_mtdConfig;
extern UI_CONNECT_ACTION g_connectAction;
bool showDetectCorners = false;
SelectMode mouse_workmode = DrawRectangle_Mode;
extern GB_WorkMode g_AppWorkMode;
extern GB_MENU run_Mode;
extern CMD_EXT *msgextInCtrl;
int CVideoProcess::m_mouseEvent = 0;
int CVideoProcess::m_mousex = 0;
int CVideoProcess::m_mousey = 0;
CVideoProcess * CVideoProcess::pThis = NULL;
bool CVideoProcess::m_bTrack = false;
bool CVideoProcess::m_bMtd = false;
bool CVideoProcess::m_bMoveDetect = false;
int CVideoProcess::m_iTrackStat = 0;
int CVideoProcess::m_iTrackLostCnt = 0;
int64 CVideoProcess::tstart = 0;
static int count=0;
int ScalerLarge,ScalerMid,ScalerSmall;
extern SingletonSysParam* g_sysParam;
extern osdbuffer_t disOsdBuf[32];
extern osdbuffer_t disOsdBufbak[32];
extern wchar_t disOsd[32][33];
vector<Mat> imageListForCalibra;

extern bool showDetectCorners;
extern OSA_SemHndl g_detectCorners;
extern volatile bool cloneOneFrame;
extern bool captureOnePicture;
extern int captureCount ;
extern std::vector< cv::Mat > ImageList;
EventLoop *eventLoop = NULL;
extern CProcess *proc ;

int CVideoProcess::MAIN_threadCreate(void)
{
	int iRet = OSA_SOK;
	iRet = OSA_semCreate(&mainProcThrObj.procNotifySem ,1,0) ;
	OSA_assert(iRet == OSA_SOK);

	mainProcThrObj.exitProcThread = false;

	mainProcThrObj.initFlag = true;

	mainProcThrObj.pParent = (void*)this;

	iRet = OSA_thrCreate(&mainProcThrObj.thrHandleProc, mainProcTsk, 0, 0, &mainProcThrObj);

	return iRet;
}


static void copyMat2Mat(cv::Mat src, cv::Mat dst, cv::Point pt)
{
	int i, j;
	uchar *psrc, *pdst;
#pragma UNROLL(4)
	for(i=0; i<src.rows; i++)
	{
		psrc = (uchar*)src.ptr<cv::Vec4b>(i,0);
		pdst =  (uchar*)dst.ptr<cv::Vec4b>(pt.y+i, pt.x);
		memcpy(pdst, psrc, src.cols*sizeof(cv::Vec4b));
	}
}

TARGETBOX mBox[MAX_TARGET_NUMBER];
//static int trktime = 0;


void extractUYVY2Gray(Mat src, Mat dst)
{
	int ImgHeight, ImgWidth,ImgStride;

	ImgWidth = src.cols;
	ImgHeight = src.rows;
	ImgStride = ImgWidth*2;
	uint8_t  *  pDst8_t;
	uint8_t *  pSrc8_t;

	pSrc8_t = (uint8_t*)(src.data);
	pDst8_t = (uint8_t*)(dst.data);
//#pragma UNROLL 4
//#pragma omp parallel for
	for(int y = 0; y < ImgHeight*ImgWidth; y++)
	{
		pDst8_t[y] = pSrc8_t[y*2+1];
	}
}

char trkINFODisplay[256];

void CVideoProcess::main_proc_func()
{
	OSA_printf("%s: Main Proc Tsk Is Entering...\n",__func__);
	unsigned int framecount=0;
	
	float speedx,speedy;
	float optValue;
	UTC_Rect AcqRect;
	
	static bool Movedetect=false;
	Point pt1,pt2,erspt1,erspt2,erspt3,erspt4;
	static UTC_ACQ_param acqRect;
	CMD_EXT tmpCmd={0};
	double value;

#if 1
	
	static int timeFlag = 2;
	static int speedcount = 0;

	time_t timep;  
	struct tm *p;  	
	char file[128];
	const int MvDetectAcqRectWidth  =  80;
	const int MvDetectAcqRectHeight =  80;
	
#endif
	while(mainProcThrObj.exitProcThread ==  false)
	{
		OSA_semWait(&mainProcThrObj.procNotifySem, OSA_TIMEOUT_FOREVER);

		Mat frame = mainFrame[mainProcThrObj.pp^1];
		bool bTrack = mainProcThrObj.cxt[mainProcThrObj.pp^1].bTrack;
		bool bMtd = mainProcThrObj.cxt[mainProcThrObj.pp^1].bMtd;
		bool bMoveDetect = mainProcThrObj.cxt[mainProcThrObj.pp^1].bMoveDetect;
		int chId = mainProcThrObj.cxt[mainProcThrObj.pp^1].chId;
		int iTrackStat = mainProcThrObj.cxt[mainProcThrObj.pp^1].iTrackStat;

		int channel = frame.channels();
		Mat frame_gray;

		cv::Mat	salientMap, sobelMap;

		if((GUN_CHID == chId) && (get_send_mat_stat()))
		{
			printf("%s,%d, cloneSrcImage\n",__FILE__,__LINE__);
			m_autofr.cloneSrcImage(frame);
			set_cloneSrcImage_stat(true);
			set_send_mat_stat(false);
		}
			

		mainProcThrObj.pp ^=1;
		if(!OnPreProcess(chId, frame))
			continue;

		if(!m_bTrack && !m_bMtd &&!m_bMoveDetect){
			OnProcess(chId, frame);
			continue;
		}

		if(chId != m_curSubChId)
			continue;
	
		frame_gray = Mat(frame.rows, frame.cols, CV_8UC1);

		if(channel == 2)
		{
		if((chId == video_gaoqing0)||(chId == video_gaoqing)||(chId == video_gaoqing2)||(chId == video_gaoqing3))
			extractYUYV2Gray2(frame, frame_gray);
		else if(chId == video_pal)
			extractUYVY2Gray(frame, frame_gray);
		}
		else
		{
			memcpy(frame_gray.data, frame.data, frame.cols * frame.rows*channel*sizeof(unsigned char));
		}

		if(bTrack)
		{
		#if __TRACK__
			iTrackStat = ReAcqTarget();
			if(Movedetect&&(iTrackStat==0))
				{
				}
			int64 trktime = 0;
			if(algOsdRect == true)
				trktime = getTickCount();//OSA_getCurTimeInMsec();
			if(m_iTrackStat==2)
			{
				//m_searchmod=1;
			}
			else
			{
				m_searchmod=0;
			}
			m_iTrackStat = process_track(iTrackStat, frame_gray, frame_gray, m_rcTrack);

			putText(m_display.m_imgOsd[msgextInCtrl->SensorStat],trkINFODisplay,
				Point( 10, 25),
				FONT_HERSHEY_TRIPLEX,0.8,
				cvScalar(0,0,0,0), 1
			);
#if 0
			sprintf(trkINFODisplay, "trkStatus:%u,trkErrorX=%f,trkErrorY=%f",
				iTrackStat,m_rcTrack.x,m_rcTrack.y);
			putText(m_display.m_imgOsd[msgextInCtrl->SensorStat],trkINFODisplay,
				Point( 10, 25),
				FONT_HERSHEY_TRIPLEX,0.8,
				cvScalar(255,255,0,255), 1
			);
#endif
			

			UtcGetSceneMV(m_track, &speedx, &speedy);
			UtcGetOptValue(m_track, &optValue);
			
			if(m_iTrackStat == 2)
				m_iTrackLostCnt++;
			else
				m_iTrackLostCnt = 0;

			if(m_iTrackStat == 2)
			{								
				if(m_iTrackLostCnt > 3)
				{					
				}
				else
				{
					m_iTrackStat = 1;
				}
			}

			if(m_display.disptimeEnable == 1)
			{
				putText(m_display.m_imgOsd[1],m_strDisplay,
						Point( m_display.m_imgOsd[1].cols-450, 30),
						FONT_HERSHEY_TRIPLEX,0.8,
						cvScalar(0,0,0,0), 1
						);
				sprintf(m_strDisplay, "TrkStat=%d speedxy: (%0.2f,%0.2f)", m_iTrackStat, speedx,speedy);

				putText(m_display.m_imgOsd[1],m_strDisplay,
						Point( m_display.m_imgOsd[1].cols-450, 30),
						FONT_HERSHEY_TRIPLEX,0.8,
						cvScalar(255,255,0,255), 1
						);
			}
			
			//printf("********m_iTrackStat=%d\n",m_iTrackStat);
			//OSA_printf("ALL-Trk: time = %d ms \n", OSA_getCurTimeInMsec() - trktime);
			if(algOsdRect == true){
				float time = ( (getTickCount() - trktime)/getTickFrequency())*1000;;//OSA_getCurTimeInMsec() - trktime;
				static float totaltime = 0;
				static int count11 = 1;
				totaltime += time;
				if((count11++)%100 == 0)
				{
					OSA_printf("ALL-TRK: time = %f ms \n", totaltime/100 );
					count11 = 1;
					totaltime = 0.0;
				}
			}
		#endif
		}
		else if(bMtd)
		{
			tstart = getTickCount();

			Rect roi;

			if(tvzoomStat)
				{
					roi.x=frame_gray.cols/4;
					roi.y=frame_gray.rows/4;
					roi.width=frame_gray.cols/2;
					roi.height=frame_gray.rows/2;
					//OSA_printf("roiXY(%d,%d),WH(%d,%d)\n",roi.x,roi.y,roi.width,roi.height);
				}
			else
				{
					roi.x=0;
					roi.y=0;
					roi.width=frame_gray.cols;
					roi.height=frame_gray.rows;
				}
		#if __MMT__
			//m_MMTDObj.MMTDProcess(frame_gray, m_tgtBox, m_display.m_imgOsd[1], 0);
			m_MMTDObj.MMTDProcessRect(frame_gray, m_tgtBox, roi, m_display.m_imgOsd[1], 0);
			for(int i=0;i<MAX_TARGET_NUMBER;i++)
			{
				m_mtd[chId]->tg[i].cur_x=m_tgtBox[i].Box.x+m_tgtBox[i].Box.width/2;
				m_mtd[chId]->tg[i].cur_y=m_tgtBox[i].Box.y+m_tgtBox[i].Box.height/2;
				m_mtd[chId]->tg[i].valid=m_tgtBox[i].valid;
				//OSA_printf("ALL-MTD: time  ID %d  valid=%d x=%d y=%d\n",i,m_tgtBox[i].valid,m_tgtBox[i].Box.x,m_tgtBox[i].Box.y);
			}
		#endif	
		
		}
		else if (bMoveDetect)
		{
		#if __MOVE_DETECT__
			if(m_pMovDetector != NULL)
			{
				for(int i = 0; i < MAX_MTDRIGION_NUM; i++)
				{
					m_pMovDetector->setFrame(frame_gray,i,2,minsize,maxsize,sensi);
				}
			}
		#endif
		}
		
		OnProcess(chId, frame);
		framecount++;

	/************************* while ********************************/
	}
	OSA_printf("%s: Main Proc Tsk Is Exit...\n",__func__);
}

int CVideoProcess::MAIN_threadDestroy(void)
{
	int iRet = OSA_SOK;

	mainProcThrObj.exitProcThread = true;
	OSA_semSignal(&mainProcThrObj.procNotifySem);

	iRet = OSA_thrDelete(&mainProcThrObj.thrHandleProc);

	mainProcThrObj.initFlag = false;
	OSA_semDelete(&mainProcThrObj.procNotifySem);

	return iRet;
}

void CVideoProcess::linkage_init()
{
	m_GrayMat.create(1080,1920,CV_8UC1);
	if(m_GrayMat.empty()) 
		cout << "Create m_GrayMat Failed !!" << endl;
	else
		cout << "Create m_GrayMat Success !!" << endl;
	
	m_Gun_GrayMat.create(1080,1920,CV_8UC1);
	if(m_Gun_GrayMat.empty())
		cout << "Create m_Gun_GrayMat Failed !!" << endl;
	else
		cout << "Create m_Gun_GrayMat Success !!" << endl;
	
	m_Gun_GrayMat = Scalar(255);

	m_time_show = m_time_flag = disOsdBuf[osdID_time].ctrl;
	
	m_rgbMat.create(1080,1920,CV_8UC3);
	if( m_rgbMat.empty())
		cout << "Create m_rgbMat Failed !!" << endl;
	else
		cout << "Create m_rgbMat Success !!" << endl;
	
	if(m_camCalibra == NULL){
		cout << "Create CamCalibrate Object Failed !!" << endl;
	}else{
		if( !m_camCalibra->Load_CameraParams("gun_camera_data.yml", "ball_camera_data.yml") )
		cout << " Load Camera Origin IntrinsicParameters Failed !!!" << endl;

		m_camCalibra->Init_CameraParams();
		m_camCalibra->RunService();
	}	

	if(m_detectCorners == NULL) {
		cout << "Create DetectCorners Object Failed !!" << endl;
	}else{
		m_detectCorners->Init();
		m_detectCorners->RunService();
	}

	if(m_intrMatObj != NULL){
		m_intrMatObj->RunService();
	}
}

CcCamCalibra* CVideoProcess::m_camCalibra = new CcCamCalibra();
DetectCorners* CVideoProcess::m_detectCorners = new DetectCorners();
IntrinsicMatrix* CVideoProcess::m_intrMatObj = new IntrinsicMatrix();

bool CVideoProcess::m_bLDown = false;
bool CVideoProcess::m_bIsClickMode = false;

int CVideoProcess::m_staticScreenWidth = outputWHF[0];
int CVideoProcess::m_staticScreenHeight = outputWHF[1];
CVideoProcess::CVideoProcess(int w, int h):m_ScreenWidth(w),m_ScreenHeight(h),
	m_track(NULL),m_curChId(MAIN_CHID),m_curSubChId(-1),adaptiveThred(40),m_display(CDisplayer(w,h)),
	m_backNodePos(cv::Point(-10,-10)),m_curNodeIndex(0)
{
	imageListForCalibra.clear();
	pThis = this;
	memset(m_mtd, 0, sizeof(m_mtd));
	memset(&mainProcThrObj, 0, sizeof(MAIN_ProcThrObj));
	
	detState = TRUE;
	trackEnd = FALSE;
	trackStart = TRUE;
	nextDetect = FALSE;
	lastFrameBox=0;
	moveStat = FALSE;
	m_acqRectW			= 	60;
	m_acqRectH			= 	60;

	m_intervalFrame 			= 0;
	m_intervalFrame_change 	= 0;
	m_bakChId = m_curChId;
	trackchange		=0;
	m_searchmod		=0;
	tvzoomStat		=0;
	wFileFlag			=0;
	preAcpSR	={0};
	algOsdRect = false;

	
#if __MOVE_DETECT__
	m_pMovDetector	=NULL;
	detect_vect_arr.resize(MAX_MTDRIGION_NUM);
	mvList_arr.resize(MAX_MTDRIGION_NUM);
#endif

#if __MMT__
	memset(m_tgtBox, 0, sizeof(TARGETBOX)*MAX_TARGET_NUMBER);
#endif

#if __MOVE_DETECT__
	if(recvMtdConfigData == true)
	{
		detectNum = g_mtdConfig.targetNum;
		maxsize = g_mtdConfig.maxArea;
		minsize = g_mtdConfig.minArea;
		sensi = g_mtdConfig.sensitivity;
	}
	else
	{
		detectNum = 1;
		maxsize = 10000;
		minsize = 9;
		sensi = 30;
	}
	setrigion_flagv20 = mtdcnt = 0;	
	memset(&mtdrigionv20, 0, sizeof(mtdrigionv20));
	memset(grid19x10, 0, sizeof(grid19x10));
	memset(grid19x10_bak, 0, sizeof(grid19x10_bak));
#endif

	m_curChId = video_gaoqing ;
	m_curSubChId = video_gaoqing0 ;
	Set_SelectByRect = false ;
	open_handleCalibra = false ;
	linkage_init();
	m_click = m_draw = m_tempX = m_tempY = 0;
	memset(m_rectn, 0, sizeof(m_rectn));
	memset(mRect, 0, sizeof(mRect));
	setrigon_flag = 0;
	setrigon_polygon = 0;

	m_click_v20L = m_click_v20R = 0;
	memset(&mRectv20L, 0, sizeof(mRectv20L));
	memset(&mRectv20R, 0, sizeof(mRectv20R));

	jcenter_s = get_joycenter();
	joys_click = 0;

	jos_mouse.x = w/2;
	jos_mouse.y = h/2;
	m_gridWidth=(int)((float)(w/16));
	m_gridHeight = (int)((float)(h/12));

	for(int i=0;i<=GRID_COLS_15+3;i++)
	{
		if( (0==i) || ((GRID_COLS_15+2) == i)){
			m_intervalCOl[i] = 0;
		}
		else if( (1==i )|| ((GRID_COLS_15+1)== i))
		{
			m_intervalCOl[i] = m_gridWidth/2;
		}
		else
		{
			m_intervalCOl[i] = m_gridWidth;
		}
	}
#if 0
	InitGridMapNodes();
#else
	InitGridMap16X12();
#endif
	readParams("SaveGridMap.yml");
	read_param_trig();
	m_autofr.create(pnotify_callback);	
}

CVideoProcess::CVideoProcess()
	:m_track(NULL),m_curChId(MAIN_CHID),m_curSubChId(-1),adaptiveThred(40),m_curNodeIndex(0)		
{
	imageListForCalibra.clear();
	pThis = this;
	memset(m_mtd, 0, sizeof(m_mtd));
	memset(&mainProcThrObj, 0, sizeof(MAIN_ProcThrObj));
	
	detState = TRUE;
	trackEnd = FALSE;
	trackStart = TRUE;
	nextDetect = FALSE;
	lastFrameBox=0;
	moveStat = FALSE;
	m_acqRectW			= 	60;
	m_acqRectH			= 	60;

	m_intervalFrame 			= 0;
	m_intervalFrame_change 	= 0;
	m_bakChId = m_curChId;
	trackchange		=0;
	m_searchmod		=0;
	tvzoomStat		=0;
	wFileFlag			=0;
	preAcpSR	={0};
	algOsdRect = false;

	
#if __MOVE_DETECT__
	m_pMovDetector	=NULL;
	detect_vect_arr.resize(MAX_MTDRIGION_NUM);
	mvList_arr.resize(MAX_MTDRIGION_NUM);
#endif

#if __MMT__
	memset(m_tgtBox, 0, sizeof(TARGETBOX)*MAX_TARGET_NUMBER);
#endif

#if __MOVE_DETECT__
	detectNum = 1;
	maxsize = 10000;
	minsize = 9;
	sensi = 30;
	setrigion_flagv20 = mtdcnt = 0;	
	memset(&mtdrigionv20, 0, sizeof(mtdrigionv20));
	memset(grid19x10, 0, sizeof(grid19x10));
	memset(grid19x10_bak, 0, sizeof(grid19x10_bak));
#endif

	m_curChId = video_gaoqing ;
	m_curSubChId = video_gaoqing0 ;
	Set_SelectByRect = false ;
	open_handleCalibra = false ;
	linkage_init();
	m_click = m_draw = m_tempX = m_tempY = 0;
	memset(m_rectn, 0, sizeof(m_rectn));
	memset(mRect, 0, sizeof(mRect));
	setrigon_flag = 0;
	setrigon_polygon = 0;

	m_click_v20L = m_click_v20R = 0;
	memset(&mRectv20L, 0, sizeof(mRectv20L));
	memset(&mRectv20R, 0, sizeof(mRectv20R));

	jcenter_s = get_joycenter();
	joys_click = 0;
}

CVideoProcess::~CVideoProcess()
{
	pThis = NULL;
}

int CVideoProcess::creat()
{
	int i = 0;
	#if __TRACK__
		trackinfo_obj=(Track_InfoObj *)malloc(sizeof(Track_InfoObj));
	#endif
	m_display.m_detectCorners = m_detectCorners;
	m_display.create();

	MultiCh.m_user = this;
	MultiCh.m_usrFunc = callback_process;
	MultiCh.creat();

	MAIN_threadCreate();
	OSA_mutexCreate(&m_mutex);
	OnCreate();

	
#if __MOVE_DETECT__
	if(m_pMovDetector == NULL)
		m_pMovDetector = MvDetector_Create();
	OSA_assert(m_pMovDetector != NULL);
#endif
	return 0;
}

int CVideoProcess::destroy()
{
	stop();
	OSA_mutexDelete(&m_mutex);
	MAIN_threadDestroy();

	MultiCh.destroy();
	m_display.destroy();

	OnDestroy();

#if __MOVE_DETECT__
	DeInitMvDetect();
#endif
	if( eventLoop != NULL )
	{
		eventLoop->StopService();
	}	

	return 0;
}

void CVideoProcess::InitGridMap16X12()
{
	int row_offset = (IMG_WIDTH - (GRID_COLS_15*m_gridWidth) ) / 2;
	int col_offset =  (IMG_HEIGHT - (GRID_ROWS_11*m_gridHeight)) / 2;
	//int row_interval,col_interval;
	int temp_col = row_offset;
	static bool print_once = true;
	for(int i=0; i<=GRID_ROWS_11;i++)
	{
		temp_col = row_offset;
		for(int j=0;j<=GRID_COLS_15+2;j++)
		{		
			m_gridNodes[i][j].pano= 0;
			m_gridNodes[i][j].tilt = 0;
			m_gridNodes[i][j].zoom = 2849; 	//ball camera min zoom value

			//m_gridNodes[i][j].coord_x = row_offset + j*GRID_WIDTH_120;

		
			m_gridNodes[i][j].coord_x = temp_col +m_intervalCOl[j];
			temp_col = m_gridNodes[i][j].coord_x;
			
			m_gridNodes[i][j].coord_y = col_offset + i*m_gridHeight;
			m_gridNodes[i][j].isCircle = true;
			m_gridNodes[i][j].renderFlag= false;

			m_nodePos[i][j].x = m_gridNodes[i][j].coord_x;
			m_nodePos[i][j].y = m_gridNodes[i][j].coord_y;
			
			m_calibratedNodes[i][j].x =m_nodePos[i][j].x;
			m_calibratedNodes[i][j].y =m_nodePos[i][j].y; 
			m_calibratedNodes[i][j].isShow = false;
		}
	}

	m_calibratedNodes[0][0].x = 60;
	m_calibratedNodes[0][0].y = 45;

}
void CVideoProcess::InitGridMapNodes()
{
#if 0
	int row_offset = (IMG_WIDTH - (GRID_COLS*GRID_WIDTH) ) / 2;
	int col_offset =  (IMG_HEIGHT - (GRID_ROWS*GRID_HEIGHT)) / 2;

	for(int i =0; i<= GRID_ROWS; i++)
	{
		for(int j=0; j<= GRID_COLS; j++)
		{
			m_gridNodes[i][j].pano= 0;
			m_gridNodes[i][j].tilt = 0;
			m_gridNodes[i][j].zoom = 2849; // ball camera min zoom value
			m_gridNodes[i][j].coord_x = row_offset + j * GRID_WIDTH;
			m_gridNodes[i][j].coord_y = col_offset + i * GRID_HEIGHT;
			m_gridNodes[i][j].isCircle = true;
			m_nodePos[i][j].x = m_gridNodes[i][j].coord_x;
			m_nodePos[i][j].y = m_gridNodes[i][j].coord_y;
				m_calibratedNodes[i][j].x =m_nodePos[i][j].x;
				m_calibratedNodes[i][j].y =m_nodePos[i][j].y; 
				m_calibratedNodes[i][j].isShow = false;

		}		
	}
	printf("\r\nInit Grid Map Success &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\r\n");
#endif
}


void CVideoProcess::processtimeMenu(int value)
{
	if(0 == value)
	{
		pThis->m_time_show = 1;
		pThis->m_time_flag = 1;
	}
	else if(1 == value)
		pThis->m_time_show = 0;

	pThis->sendIPC_Time(value);
}

void CVideoProcess::sendIPC_Time(int value)
{
	osdtext_t* osd_ipc = ipc_getosdtextstatus_p();

	osd_ipc->osdID[osdID_time] = disOsdBuf[osdID_time].osdID = osdID_time;

	if(value == 0)
		osd_ipc->ctrl[osdID_time] = disOsdBuf[osdID_time].ctrl = 1;
	else if(value == 1)
		osd_ipc->ctrl[osdID_time] = disOsdBuf[osdID_time].ctrl = 0;

	setlocale(LC_ALL, "zh_CN.UTF-8");
	memcpy(&disOsdBufbak[osdID_time],&disOsdBuf[osdID_time],sizeof(osdbuffer_t));
	swprintf(disOsd[osdID_time], 33, L"%s", disOsdBuf[osdID_time].buf);

	SENDST test = {0};
	test.cmd_ID = read_shm_osdtext;
	test.param[0] = osdID_time;
	ipc_sendmsg(&test, IPC_FRIMG_MSG);
}

void CVideoProcess::processsmanualcarliMenu(int value)
{
	//printf("%s start, value = %d\n", __FUNCTION__, value);
	switch(value){
		case 0:
			g_sysParam->getSysParam().cameracalibrate.Enable_handleCalibrate = true;
			g_sysParam->getSysParam().cameracalibrate.Enable_Undistortion = true;
			break;
		case 1:
			g_sysParam->getSysParam().cameracalibrate.Enable_cloneSrcImage = true;
			break;
		case 2:
			g_sysParam->getSysParam().cameracalibrate.Enable_calculateMatrix= true;
			break;
		case 3:
			g_sysParam->getSysParam().cameracalibrate.Enable_saveParameter= true;
			break;
		case 4:
			g_sysParam->getSysParam().cameracalibrate.Enable_handleCalibrate = false;
			g_sysParam->getSysParam().cameracalibrate.Enable_Undistortion = false;	
			g_sysParam->getSysParam().cameracalibrate.Enable_cloneSrcImage = false;	
			g_sysParam->getSysParam().cameracalibrate.Enable_calculateMatrix= false;	
			g_sysParam->getSysParam().cameracalibrate.Enable_saveParameter= false;	
			//g_sysParam->getSysParam().cameracalibrate.Enable_AutoDetectMoveTargets = false;
		break;
		default :
			break;
	}	
}

void CVideoProcess::processsautocarliMenu(int value)
{
	//printf("%s start, value=%d\n", __FUNCTION__, value);
	switch(value){
		#if 0
		case 0:
			g_sysParam->getSysParam().cameracalibrate.Enable_handleCalibrate = true;
			g_sysParam->getSysParam().cameracalibrate.Enable_Undistortion = true;
			break;
		#endif
		case 1:
			g_sysParam->getSysParam().cameracalibrate.Enable_cloneSrcImage = true;
			break;
		case 2:
			g_sysParam->getSysParam().cameracalibrate.Enable_calculateMatrix= true;
			break;
		case 3:
			g_sysParam->getSysParam().cameracalibrate.Enable_saveParameter= true;
			break;
		case 4:
			g_sysParam->getSysParam().cameracalibrate.Enable_handleCalibrate = false;
			g_sysParam->getSysParam().cameracalibrate.Enable_Undistortion = false;	
			g_sysParam->getSysParam().cameracalibrate.Enable_cloneSrcImage = false;	
			g_sysParam->getSysParam().cameracalibrate.Enable_calculateMatrix= false;	
			g_sysParam->getSysParam().cameracalibrate.Enable_saveParameter= false;	
			//g_sysParam->getSysParam().cameracalibrate.Enable_AutoDetectMoveTargets = false;
		break;
		default :
			break;
	}	
}



int CVideoProcess::click_legal(int x, int y)
{
	y = m_ScreenHeight - y;
	if(in_gun_area(x, y))
	{
		click_in_area = 1;	//click in gun area
		return 1;
	}
	else if(in_ball_area(x, y))
	{
		click_in_area = 2; //click in ball area
		return 1;
	}
	else 
	{
		click_in_area = 0; //click in illegal area
		return 0;
	}
}

int CVideoProcess::move_legal(int x, int y)
{
	y = m_ScreenHeight - y;
	if(1 == click_in_area)
	{
		if(in_gun_area(x, y))
			return 1;
		else
			return 0;
	}
	else if(2 == click_in_area)
	{
		if(in_ball_area(x, y))
			return 1;
		else
			return 0;
	}
	else
		return 0;
}

int CVideoProcess::in_gun_area(int x, int y)
{
	int dismode = m_display.displayMode;
	switch(dismode)
	{
		case PREVIEW_MODE:
			if((x > m_ScreenWidth/2 && x < m_ScreenWidth) && (y > m_ScreenHeight/2 && y < m_ScreenHeight))
				return 1;
			else
				return 0;
			break;
		case MAIN_VIEW:
			if((x >= 0 && x <= m_ScreenWidth) && (y >= 0 && y <= m_ScreenHeight/*/2*/ ))
				return 1;
			else
				return 0;
			break;
		case SIDE_BY_SIDE:
			if((x > m_ScreenWidth/2 && x < m_ScreenWidth) && (y > 0 && y < m_ScreenHeight))
				return 1;
			else
				return 0;
			break;
		case LEFT_BALL_RIGHT_GUN:
			if((x > m_ScreenWidth/4 && x < m_ScreenWidth) && (y > 270 && y < m_ScreenHeight))
				return 1;
			else
				return 0;
			break;
		default:
			printf("%s,%d, unknown displayMode:%d\n", __FILE__, __LINE__, dismode);
			return 0;
			break;
	}
}

int CVideoProcess::in_ball_area(int x, int y)
{
	int dismode = m_display.displayMode;
	switch(dismode)
	{
		case PREVIEW_MODE:
			if((x > 0 && x < m_ScreenWidth/2) && (y > m_ScreenHeight/2 && y < m_ScreenHeight))
				return 1;
			else
				return 0;
			break;
		case MAIN_VIEW:
			if((x > m_ScreenWidth/4 && x < (m_ScreenWidth*3/4)) && (y > m_ScreenHeight/2 && y < m_ScreenHeight))
				return 1;
			else
				return 0;
			break;
		case SIDE_BY_SIDE:
			if((x > 0 && x < m_ScreenWidth/2) && (y > 0 && y < m_ScreenHeight))
				return 1;
			else
				return 0;
			break;
		case LEFT_BALL_RIGHT_GUN:
			if((x > 0 && x < m_ScreenWidth/4) && (y > (m_ScreenHeight*3/4) && y < m_ScreenHeight))
				return 1;
			else
				return 0;
			break;
		default:
			printf("%s,%d, unknown displayMode:%d\n", __FILE__, __LINE__, dismode);
			return 0;
			break;
	}
}

mouserect CVideoProcess::map2preview(mouserect rectcur)
{
	int dismode = m_display.displayMode;
	switch(dismode)
	{
		case PREVIEW_MODE:
			return rectcur;
			break;
		case MAIN_VIEW:
			return mappip2preview(rectcur);
			break;
		case SIDE_BY_SIDE:
			return mapsbs2preview(rectcur);
			break;
		case LEFT_BALL_RIGHT_GUN:
			return maplbrg2preview(rectcur);
			break;
		default:
			printf("%s,%d, unknown displayMode:%d\n", __FILE__, __LINE__, dismode);
			return rectcur;
			break;			
	}
}

mouserect CVideoProcess::mappip2preview(mouserect rectcur)
{
	mouserect rectpip;
	mouserect rectpreview;
	if(1 == click_in_area)
	{
		rectpip.x = 0;
		rectpip.y = 0;
		rectpip.w = m_ScreenWidth;
		rectpip.h = m_ScreenHeight;
		
		rectpreview.x = m_ScreenWidth/2;
		rectpreview.y = 0;
		rectpreview.w = m_ScreenWidth/2;
		rectpreview.h = m_ScreenHeight/2;
	}
	else if(2 == click_in_area)
	{
	
		rectpip.x = 1440;
		rectpip.y = 0;
		rectpip.w = m_ScreenWidth/4;
		rectpip.h = 270;
		
		rectpreview.x = 0;
		rectpreview.y = 0;
		rectpreview.w = m_ScreenWidth/2;//960;
		rectpreview.h = m_ScreenHeight/2;
	}

	return maprect(rectcur, rectpip, rectpreview);
}

mouserect CVideoProcess::mapsbs2preview(mouserect rectcur)
{
	mouserect rectsbs;
	mouserect rectpreview;
	if(1 == click_in_area)
	{
		rectsbs.x =m_ScreenWidth/2 ;//960;
		rectsbs.y = 0;
		rectsbs.w =m_ScreenWidth/2;// 960;
		rectsbs.h = m_ScreenHeight;
		
		rectpreview.x = m_ScreenWidth/2;//960;
		rectpreview.y = 0;
		rectpreview.w = m_ScreenWidth/2;//960;
		rectpreview.h = m_ScreenHeight/2;
	}
	else if(2 == click_in_area)
	{
		rectsbs.x = 0;
		rectsbs.y = 0;
		rectsbs.w = m_ScreenWidth/2;//960;
		rectsbs.h = m_ScreenHeight;
		
		rectpreview.x = 0;
		rectpreview.y = 0;
		rectpreview.w = m_ScreenWidth/2;//960;
		rectpreview.h = m_ScreenHeight/2;
	}

	return maprect(rectcur, rectsbs, rectpreview);
}

mouserect CVideoProcess::maplbrg2preview(mouserect rectcur)
{
	mouserect rectlbrg;
	mouserect rectpreview;
	if(1 == click_in_area)
	{
		rectlbrg.x = m_ScreenWidth/4;//480;
		rectlbrg.y = 0;
		rectlbrg.w = 1440;
		rectlbrg.h = 810;
		
		rectpreview.x = m_ScreenWidth/2;//960;
		rectpreview.y = 0;
		rectpreview.w = m_ScreenWidth/2;//960;
		rectpreview.h = m_ScreenHeight/2;
	}
	else if(2 == click_in_area)
	{
		rectlbrg.x = 0;
		rectlbrg.y = 0;
		rectlbrg.w = m_ScreenWidth/4;//480;
		rectlbrg.h = 270;
		
		rectpreview.x = 0;
		rectpreview.y = 0;
		rectpreview.w = m_ScreenWidth/2;//960;
		rectpreview.h = m_ScreenHeight/2;
	}

	return maprect(rectcur, rectlbrg, rectpreview);
}

mouserect CVideoProcess::mapfullscreen2gun(mouserect rectcur)
{
	mouserect rect1080p;
	mouserect rectgun;
	
	int dismode = m_display.displayMode;
	rect1080p.x = 0;
	rect1080p.y = 0;
	rect1080p.w = m_ScreenWidth;
	rect1080p.h = m_ScreenHeight;
	switch(dismode)
	{
		case PREVIEW_MODE:
			rectgun.x =m_ScreenWidth/2;// 960;
			rectgun.y = 0;
			rectgun.w =m_ScreenWidth/2;// 960;
			rectgun.h = m_ScreenHeight/2;
			break;
		case MAIN_VIEW:
			rectgun.x = 0;
			rectgun.y = m_ScreenHeight/2;
			rectgun.w = m_ScreenWidth;
			rectgun.h = m_ScreenHeight/2;
			break;
		case SIDE_BY_SIDE:
			rectgun.x = m_ScreenWidth/2;//960;
			rectgun.y = 0;
			rectgun.w =m_ScreenWidth/2;// 960;
			rectgun.h = m_ScreenHeight;
			break;
		case LEFT_BALL_RIGHT_GUN:
			rectgun.x = m_ScreenWidth/4;//480;
			rectgun.y = 0;
			rectgun.w = 1440;
			rectgun.h = 810;
			break;
		default:
			break;

	}
	
	return maprect(rectcur, rect1080p, rectgun);
}

mouserect CVideoProcess::mapfullscreen2gunv20(mouserect rectcur)
{
	mouserect rect1080p;
	mouserect rectgun;

	rect1080p.x = 0;
	rect1080p.y = 0;
	rect1080p.w = m_ScreenWidth;//1920;
	rect1080p.h = m_ScreenHeight;

	rectgun.x = 0;
	rectgun.y = m_ScreenHeight/2;
	rectgun.w = m_ScreenWidth;//1920;
	rectgun.h = m_ScreenHeight/2;
	
	return maprect(rectcur, rect1080p, rectgun);
}

mouserect CVideoProcess::mapgun2fullscreen(mouserect rectcur)
{
	mouserect rect1080p;
	mouserect rectgun;
	
	int dismode = m_display.displayMode;
	rect1080p.x = 0;
	rect1080p.y = 0;
	rect1080p.w = m_ScreenWidth;//1920;
	rect1080p.h = m_ScreenHeight;
		
	switch(dismode)
	{
		case PREVIEW_MODE:
			rectgun.x =m_ScreenWidth/2;// 960;
			rectgun.y = 0;
			rectgun.w =m_ScreenWidth/2;// 960;
			rectgun.h = m_ScreenHeight/2;
			break;
		case MAIN_VIEW:
			rectgun.x = 0;
			rectgun.y = m_ScreenHeight/2;
			rectgun.w = m_ScreenWidth;//1920;
			rectgun.h = m_ScreenHeight/2;
			break;
		case SIDE_BY_SIDE:
			rectgun.x = m_ScreenWidth/2;//960;
			rectgun.y = 0;
			rectgun.w =m_ScreenWidth/2;// 960;
			rectgun.h = m_ScreenHeight;
			break;
		case LEFT_BALL_RIGHT_GUN:
			rectgun.x = m_ScreenWidth/4;//480;
			rectgun.y = 0;
			rectgun.w = 1440;
			rectgun.h = 810;
			break;
		default:
			break;
	}
	return maprect(rectcur, rectgun, rect1080p);
}

int CVideoProcess::mapgun2fullscreen_point(int *x, int *y)
{
	mouserect rect1080p;
	mouserect rectgun;
	
	int dismode = m_display.displayMode;
	rect1080p.x = 0;
	rect1080p.y = 0;
	rect1080p.w = m_ScreenWidth;//1920;
	rect1080p.h = m_ScreenHeight;
		
	switch(dismode)
	{
		case PREVIEW_MODE:
			rectgun.x =m_ScreenWidth/2;// 960;
			rectgun.y = 0;
			rectgun.w =m_ScreenWidth/2;// 960;
			rectgun.h = m_ScreenHeight/2;
			break;
		case MAIN_VIEW:
			rectgun.x = 0;
			rectgun.y = 0;
			rectgun.w =m_ScreenWidth;// 1920;
			rectgun.h = m_ScreenHeight;
			break;
		case SIDE_BY_SIDE:
			rectgun.x =m_ScreenWidth/2;// 960;
			rectgun.y = 0;
			rectgun.w =m_ScreenWidth/2;// 960;
			rectgun.h = m_ScreenHeight;
			break;
		case LEFT_BALL_RIGHT_GUN:
			rectgun.x = m_ScreenWidth/4;//480;
			rectgun.y = 0;
			rectgun.w = 1440;
			rectgun.h = 810;
			break;
		default:
			break;
	}
	return maprect_point(x, y, rectgun, rect1080p);
}

int CVideoProcess::mapfullscreen2gun_pointv20(int *x, int *y)
{
	mouserect rect1080p;
	mouserect rectgun;
	
	rect1080p.x = 0;
	rect1080p.y = 0;
	rect1080p.w =m_ScreenWidth;// 1920;
	rect1080p.h = m_ScreenHeight;

	rectgun.x = 0;
	rectgun.y = m_ScreenHeight/2;
	rectgun.w =m_ScreenWidth;// 1920;
	rectgun.h = m_ScreenHeight/2;
	
	return maprect_point(x, y, rect1080p, rectgun);
}

mouserect CVideoProcess::maprect(mouserect rectcur,mouserect rectsrc,mouserect rectdest)
{
	mouserect rect_result;

	rect_result.x = (rectcur.x-rectsrc.x)*rectdest.w/rectsrc.w+rectdest.x;
	rect_result.y = (rectcur.y-rectsrc.y)*rectdest.h/rectsrc.h+rectdest.y;
	rect_result.w = rectcur.w*rectdest.w/rectsrc.w;
	rect_result.h = rectcur.h*rectdest.h/rectsrc.h;
	return rect_result;
}

int CVideoProcess::maprect_point(int *x, int *y, mouserect rectsrc,mouserect rectdest)
{
	if(NULL != x)
		*x = (*x-rectsrc.x)*rectdest.w/rectsrc.w+rectdest.x;
	if(NULL != y)
		*y = (*y-rectsrc.y)*rectdest.h/rectsrc.h+rectdest.y;
	return 0;
}

int CVideoProcess::map1080p2normal_point(float *x, float *y)
{
	if(NULL != x)
		*x /= m_ScreenWidth;//1920;
	if(NULL != y)
		*y /= m_ScreenHeight;

	return 0;
}

int CVideoProcess::mapnormal2curchannel_point(float *x, float *y, int w, int h)
{
	if(NULL != x)
		*x *= w;
	if(NULL != y)
		*y *= h;
	
	return 0;
}

int CVideoProcess::map1080p2normal_rect(mouserectf *rect)
{
	if(NULL != rect)
	{
		rect->x /= m_ScreenWidth;//1920;
		rect->w /= m_ScreenWidth;//1920;
		rect->y /= m_ScreenHeight;
		rect->h /= m_ScreenHeight;
		return 0;
	}

	return -1;
}

int CVideoProcess::mapnormal2curchannel_rect(mouserectf *rect, int w, int h)
{
	if(NULL != rect)
	{
		rect->x *= w;
		rect->w *= w;
		rect->y *= h;
		rect->h *= h;
		return 0;
	}
	return -1;
}

bool CVideoProcess::readParams(const char* filename)
{
	char paramName[40];
	int total_LineNumber = 0;
	int ch =0;
	FILE* fp = fopen(filename,"r+");
	if(fp == NULL)
	{
		fprintf(stderr,"Open File: SaveGridMap.yml Failed ===%s\n!!!",strerror(errno));
		return false;
	}
	else
	{
		while((ch = fgetc(fp)) != EOF)
		{
			if(ch == '\n')
			{
				total_LineNumber ++;
			}
		}
		fclose(fp);
		printf("\r\n[%s]: File Total Line Number = %d\r\n",__FUNCTION__,total_LineNumber);
		if( total_LineNumber >= GRIDMAP_FILE_LINENUM)
		{
			m_readfs.open(filename,FileStorage::READ);
			if(m_readfs.isOpened())
			{
			
				for(int i=0;i<=GRID_ROWS_11;i++)
				{
					for(int j=0;j<=GRID_COLS_15+1;j++)
					{			
						sprintf(paramName,"GridMapNode_%d_%d_pano",i,j);				
						m_readfs[paramName] >>m_readGridNodes[i][j].pano;
						m_gridNodes[i][j].pano = m_readGridNodes[i][j].pano;
						memset(paramName,0,sizeof(paramName));
						sprintf(paramName,"GridMapNode_%d_%d_tilt",i,j);	
						m_readfs[paramName] >>m_readGridNodes[i][j].tilt;
						m_gridNodes[i][j].tilt = m_readGridNodes[i][j].tilt;

						memset(paramName,0,sizeof(paramName));
						sprintf(paramName,"GridMapNode_%d_%d_mark",i,j);
						m_readfs[paramName] >>m_readGridNodes[i][j].has_mark;
						m_gridNodes[i][j].has_mark = m_readGridNodes[i][j].has_mark;
						m_calibratedNodes[i][j].isShow = m_gridNodes[i][j].has_mark;
						if(m_readGridNodes[i][j].has_mark==1)
						{
							pThis->addMarkNum();
						}
					}

					int line=0;	
				}				
				m_readfs.release();
				return true;

			}
			return false;
			
		}
	}

}	


int CVideoProcess::read_param_trig()
{
	m_autofr.readParams(app_recommendPoints);
}

bool CVideoProcess::writeParamsForTriangle(const char* filename)
{
}

bool CVideoProcess::writeParams(const char* filename)
{
	char paramName[40];
	memset(paramName,0,sizeof(paramName));
	
	m_writefs.open(filename,FileStorage::WRITE);
	if(m_writefs.isOpened())
	{		
		for(int i=0;i<=GRID_ROWS_11;i++)
		{
			for(int j=0;j<=GRID_COLS_15+1;j++)
			{
				sprintf(paramName,"GridMapNode_%d_%d_pano",i,j);				
				m_writefs<<paramName <<m_gridNodes[i][j].pano;
				memset(paramName,0,sizeof(paramName));
				sprintf(paramName,"GridMapNode_%d_%d_tilt",i,j);	
				m_writefs<<paramName <<m_gridNodes[i][j].tilt;
				memset(paramName,0,sizeof(paramName));
				sprintf(paramName,"GridMapNode_%d_%d_mark",i,j);	
				m_writefs<<paramName <<m_gridNodes[i][j].has_mark;
			}
		}
		m_writefs.release();		
		return true;
	}
	return false;
}
GridMapNode CVideoProcess::AutoLinkMoveBallCamera(int px, int py, int grid_width,int grid_height,bool needChangeZoom)
{
	static int dispatch_cnt =0;
	int offset_y = IMG_HEIGHT/2;
	int x = px;
	int y = py;

	int valid_x = 0;
	int valid_y = 0;
#if 0
	switch(m_display.g_CurDisplayMode)
	{
		case MAIN_VIEW:			
			y = (py-offset_y)*2;
			break;
		default:
			break;
	}
#endif

	if(x<grid_width/2 )
	{
		valid_x = grid_width/2;
	}
	else if(x>IMG_WIDTH-grid_width/2)
	{
		valid_x = IMG_WIDTH-grid_width/2;
	}
	else{
		valid_x = x;
	}

	
	if(y<grid_height/2)
	{
		valid_y =grid_height/2; 
	}
	else if(y>IMG_HEIGHT-grid_height/2)
	{
		valid_y=IMG_HEIGHT-grid_height/2;
	}
	else 
	{
		valid_y = y;
	}
	int current_col ;
	int current_row ;
	if(valid_x>120 && (valid_x<1800))
	{		
		current_col = ((valid_x)/grid_width) % (GRID_COLS_15+1) ;
		
	}
	else
	{
		if(valid_x <= 120)
		{
			current_col = ((valid_x-grid_width/2)/(grid_width/2)) % (GRID_COLS_15+1);			
		}
		else
		{
			//current_col = 15;
			current_col = ((valid_x)/grid_width) % (GRID_COLS_15+1) ;
		}
	}
	current_row = (valid_y-grid_height/2)/grid_height ;
	int current_row_plus = current_row+1;
	int current_col_plus = current_col+1;
	GridMapNode Vp,Vp1,Vp2;
	GridMapNode V0 = m_gridNodes[current_row][current_col];
	GridMapNode V1 = m_gridNodes[current_row][current_col_plus];
	GridMapNode V2 = m_gridNodes[current_row_plus][current_col];
	GridMapNode V3 = m_gridNodes[current_row_plus][current_col_plus];
	
	float X1 = (float)V0.coord_x;
	float Y1 = (float)V0.coord_y;
	float X2 = (float)V1.coord_x;
	float Y2 = (float)V2.coord_y;
	float X = (float)valid_x;
	float Y = (float)valid_y;
	if(V0.pano > V1.pano)
	{
		float angle_left = (MAX_ANGLE-V0.pano)/PER_ANGLE;
		float angle_right = (V1.pano -ZERO_ANGLE)/PER_ANGLE;
		float X0 = X1 +(angle_left * GRID_WIDTH)/(angle_left +angle_right);

		if(X < X0)
		{
			Vp1.pano =(int)( ((X0-X)*(float)V0.pano) /(X0-X1)+((X-X1)*(float)MAX_ANGLE)/(X0-X1));
			Vp2.pano =(int)( ((X0-X)*(float)V2.pano) / (X0-X1) + ((X-X1)*(float)MAX_ANGLE)/(X0-X1));
		}
		else
		{
			Vp1.pano =(int)( ((X2-X)*(float)(0))/(X2-X0) + ((X-X0)*(float)V1.pano)/(X2-X0));
			Vp2.pano = (int)( ((X2-X)*(float)(0))/(X2-X0) + ((X-X0)*(float)V3.pano)/(X2-X0));
		}
	}
	else
	{
		Vp1.pano= (int)((X2-X)*((float)V0.pano)/(X2-X1) + (X-X1)*((float)V1.pano)/(X2-X1));
		Vp2.pano= (int)((X2-X)*((float)V2.pano)/(X2-X1) + (X-X1)*((float)V3.pano)/(X2-X1));
	}
	//Vp1.pano= (int)((X2-X)*((float)V0.pano)/(X2-X1) + (X-X1)*((float)V1.pano)/(X2-X1));
	//Vp2.pano= (int)((X2-X)*((float)V2.pano)/(X2-X1) + (X-X1)*((float)V3.pano)/(X2-X1));

	Vp1.tilt = (int)((X2-X)*((float)V0.tilt)/(X2-X1) + (X-X1)*((float)V1.tilt)/(X2-X1));		
	Vp2.tilt = (int)((X2-X)*((float)V2.tilt)/(X2-X1) + (X-X1)*((float)V3.tilt)/(X2-X1));
	Vp.pano = (int)((Y2-Y)*((float)Vp1.pano)/(Y2-Y1) + (Y-Y1)*((float)Vp2.pano)/(Y2-Y1));
	Vp.tilt = (int)((Y2-Y)*((float)Vp1.tilt)/(Y2-Y1) + (Y-Y1)*((float)Vp2.tilt)/(Y2-Y1));
	Vp.zoom = 65535;	
	if(Vp.tilt < 0)
	{
		Vp.tilt = 32768- Vp.tilt;
	}
	pThis->setQueryZoomFlag(true);

	SENDST trkmsg;
	memset((void*)&trkmsg,0,sizeof(SENDST));
	if(needChangeZoom)
	{
		trkmsg.cmd_ID = acqPosAndZoom;
		memcpy(&trkmsg.param[0],&(Vp.pano), sizeof(int));
		memcpy(&trkmsg.param[4],&(Vp.tilt), sizeof(int)); 
		int currentZoom = pThis->getCurrentZoomValue();
		memcpy(&trkmsg.param[8],&currentZoom, sizeof(int));		
	}
	else
	{
		trkmsg.cmd_ID = speedloop;
		memcpy(&trkmsg.param[0],&(Vp.pano), sizeof(int));
		memcpy(&trkmsg.param[4],&(Vp.tilt), sizeof(int)); 
		dispatch_cnt++;		
	}
	ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
	return Vp;
}

GridMapNode CVideoProcess::getLinearDeviationForSelectRect(int px, int py, int grid_width,int grid_height,bool needChangeZoom)
{
	static int dispatch_cnt =0;
	int offset_y = IMG_HEIGHT/2;
	int x = px;
	int y = py;
#if 0
	switch(m_display.g_CurDisplayMode)
	{
		case MAIN_VIEW:			
			y = (py-offset_y)*2;
			break;
		default:
			break;
	}
#endif

	int valid_x = 0;
	int valid_y = 0;
	if(x<grid_width/2 )
	{
		valid_x = grid_width/2;
	}
	else if(x>IMG_WIDTH-grid_width/2)
	{
		valid_x = IMG_WIDTH-grid_width/2;
	}
	else{
		valid_x = x;
	}

	
	if(y<grid_height/2)
	{
		valid_y =grid_height/2; 
	}
	else if(y>IMG_HEIGHT-grid_height/2)
	{
		valid_y=IMG_HEIGHT-grid_height/2;
	}
	else 
	{
		valid_y = y;
	}
	int current_col ;
	int current_row ;
	if(valid_x>120 && (valid_x<1800))
	{		
		current_col = ((valid_x)/grid_width) % (GRID_COLS_15+1) ;
		
	}
	else
	{
		if(valid_x <= 120){
			current_col = ((valid_x-grid_width/2)/(grid_width/2)) % (GRID_COLS_15+1);			
		}
		else
		{
			//current_col = 15;
			current_col = ((valid_x)/grid_width) % (GRID_COLS_15+1) ;
		}
	}
	current_row = (valid_y-grid_height/2)/grid_height ;
	int current_row_plus = current_row+1;
	int current_col_plus = current_col+1;
	GridMapNode Vp,Vp1,Vp2;
	GridMapNode V0 = m_gridNodes[current_row][current_col];
	GridMapNode V1 = m_gridNodes[current_row][current_col_plus];
	GridMapNode V2 = m_gridNodes[current_row_plus][current_col];
	GridMapNode V3 = m_gridNodes[current_row_plus][current_col_plus];
	
	float X1 = (float)V0.coord_x;
	float Y1 = (float)V0.coord_y;
	float X2 = (float)V1.coord_x;
	float Y2 = (float)V2.coord_y;
	float X = (float)valid_x;
	float Y = (float)valid_y;

	if(V0.pano > V1.pano)
	{
		float angle_left = (MAX_ANGLE-V0.pano)/PER_ANGLE;
		float angle_right = (V1.pano -ZERO_ANGLE)/PER_ANGLE;
		float X0 = X1 +(angle_left * GRID_WIDTH)/(angle_left +angle_right);
		if(X < X0)
		{
			Vp1.pano =(int)( ((X0-X)*(float)V0.pano) /(X0-X1)+((X-X1)*(float)MAX_ANGLE)/(X0-X1));
			Vp2.pano =(int)( ((X0-X)*(float)V2.pano) / (X0-X1) + ((X-X1)*(float)MAX_ANGLE)/(X0-X1));
		}
		else
		{
			Vp1.pano =(int)( ((X2-X)*(float)(0))/(X2-X0) + ((X-X0)*(float)V1.pano)/(X2-X0));
			Vp2.pano = (int)( ((X2-X)*(float)(0))/(X2-X0) + ((X-X0)*(float)V3.pano)/(X2-X0));
		}
	}
	else
	{
		Vp1.pano= (int)((X2-X)*((float)V0.pano)/(X2-X1) + (X-X1)*((float)V1.pano)/(X2-X1));
		Vp2.pano= (int)((X2-X)*((float)V2.pano)/(X2-X1) + (X-X1)*((float)V3.pano)/(X2-X1));
	}
	Vp1.tilt = (int)((X2-X)*((float)V0.tilt)/(X2-X1) + (X-X1)*((float)V1.tilt)/(X2-X1));		
	Vp2.tilt = (int)((X2-X)*((float)V2.tilt)/(X2-X1) + (X-X1)*((float)V3.tilt)/(X2-X1));
	Vp.pano = (int)((Y2-Y)*((float)Vp1.pano)/(Y2-Y1) + (Y-Y1)*((float)Vp2.pano)/(Y2-Y1));
	Vp.tilt = (int)((Y2-Y)*((float)Vp1.tilt)/(Y2-Y1) + (Y-Y1)*((float)Vp2.tilt)/(Y2-Y1));
	Vp.zoom = 65535;	
	if(Vp.tilt < 0)
	{
		Vp.tilt = 32768- Vp.tilt;
	}
	pThis->setQueryZoomFlag(true);
	SENDST trkmsg;
	memset((void*)&trkmsg,0,sizeof(SENDST));
	if(needChangeZoom)
	{
		trkmsg.cmd_ID = acqPosAndZoom;
		memcpy(&trkmsg.param[0],&(Vp.pano), sizeof(int));
		memcpy(&trkmsg.param[4],&(Vp.tilt), sizeof(int)); 
		int currentZoom = pThis->getCurrentZoomValue();
		memcpy(&trkmsg.param[8],&currentZoom, sizeof(int));
		ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
	}
	else
	{
		trkmsg.cmd_ID = speedloop;
		memcpy(&trkmsg.param[0],&(Vp.pano), sizeof(int));
		memcpy(&trkmsg.param[4],&(Vp.tilt), sizeof(int)); 
		dispatch_cnt++;
		ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
	}	
	return Vp;
}

void CVideoProcess::useTrigonometric(int px, int py)
{


}

GridMapNode CVideoProcess::getLinearDeviation(int px, int py, int grid_width,int grid_height,bool needChangeZoom)
{
	int offset_y = IMG_HEIGHT/2;
	int x = px;
	int y = py;
	switch(m_display.g_CurDisplayMode)
	{
		case MAIN_VIEW:			
			y = (py-offset_y)*2;
			break;
		default:
			break;
	}
	int valid_x = 0;
	int valid_y = 0;
	if(x<grid_width/2 )
	{
		valid_x = grid_width/2;
	}
	else if(x>IMG_WIDTH-grid_width/2)
	{
		valid_x = IMG_WIDTH-grid_width/2;
	}
	else
	{
		valid_x = x;
	}	
	if(y<grid_height/2)
	{
		valid_y =grid_height/2; 
	}
	else if(y>IMG_HEIGHT-grid_height/2)
	{
		valid_y=IMG_HEIGHT-grid_height/2;
	}
	else 
	{
		valid_y = y;
	}
	int current_col ;
	int current_row ;
	if(valid_x>120 && (valid_x<1800))
	{		
		current_col = ((valid_x)/grid_width) % (GRID_COLS_15+1) ;		
	}
	else
	{
		if(valid_x <= 120)
		{
			current_col = ((valid_x-grid_width/2)/(grid_width/2)) % (GRID_COLS_15+1);			
		}
		else
		{
			//current_col = 15;
			current_col = ((valid_x)/grid_width) % (GRID_COLS_15+1) ;
		}
	}
	current_row = (valid_y-grid_height/2)/grid_height ;
	int current_row_plus = current_row+1;
	int current_col_plus = current_col+1;
	GridMapNode Vp,Vp1,Vp2;
	GridMapNode V0 = m_gridNodes[current_row][current_col];
	GridMapNode V1 = m_gridNodes[current_row][current_col_plus];
	GridMapNode V2 = m_gridNodes[current_row_plus][current_col];
	GridMapNode V3 = m_gridNodes[current_row_plus][current_col_plus];
	
	float X1 = (float)V0.coord_x;
	float Y1 = (float)V0.coord_y;
	float X2 = (float)V1.coord_x;
	float Y2 = (float)V2.coord_y;
	float X = (float)valid_x;
	float Y = (float)valid_y;

	if(V0.pano > V1.pano)
	{
		float angle_left = (MAX_ANGLE-V0.pano)/PER_ANGLE;
		float angle_right = (V1.pano -ZERO_ANGLE)/PER_ANGLE;
		float X0 = X1 +(angle_left * GRID_WIDTH)/(angle_left +angle_right);

		if(X < X0)
		{
			Vp1.pano =(int)( ((X0-X)*(float)V0.pano) /(X0-X1)+((X-X1)*(float)MAX_ANGLE)/(X0-X1));
			Vp2.pano =(int)( ((X0-X)*(float)V2.pano) / (X0-X1) + ((X-X1)*(float)MAX_ANGLE)/(X0-X1));
		}
		else
		{
			Vp1.pano =(int)( ((X2-X)*(float)(0))/(X2-X0) + ((X-X0)*(float)V1.pano)/(X2-X0));
			Vp2.pano = (int)( ((X2-X)*(float)(0))/(X2-X0) + ((X-X0)*(float)V3.pano)/(X2-X0));
		}
	}
	else
	{
		Vp1.pano= (int)((X2-X)*((float)V0.pano)/(X2-X1) + (X-X1)*((float)V1.pano)/(X2-X1));
		Vp2.pano= (int)((X2-X)*((float)V2.pano)/(X2-X1) + (X-X1)*((float)V3.pano)/(X2-X1));
	}
	Vp1.tilt = (int)((X2-X)*((float)V0.tilt)/(X2-X1) + (X-X1)*((float)V1.tilt)/(X2-X1));
	Vp2.tilt = (int)((X2-X)*((float)V2.tilt)/(X2-X1) + (X-X1)*((float)V3.tilt)/(X2-X1));
	Vp.pano = (int)((Y2-Y)*((float)Vp1.pano)/(Y2-Y1) + (Y-Y1)*((float)Vp2.pano)/(Y2-Y1));
	Vp.tilt = (int)((Y2-Y)*((float)Vp1.tilt)/(Y2-Y1) + (Y-Y1)*((float)Vp2.tilt)/(Y2-Y1));
	Vp.zoom = 65535;		

	if(Vp.tilt < 0)
	{
		Vp.tilt = 32768- Vp.tilt;
	}
#if 0
	printf("\r\n[%s]:V0 =<%d,%d>\r\nV1 =<%d,%d> \r\nV2 =<%d,%d> \r\nV3 =<%d,%d> \r\n ",__func__,
				V0.pano,V0.tilt,V1.pano,V1.tilt,V2.pano,V2.tilt,V3.pano,V3.tilt);
	printf("\r\n[%s]: Click:Screen<%d,%d>row-col <%d,%d>\r\n", __func__,px,py,current_row, current_col);
	printf("\r\n[%s]:<X1,Y1>=<%f,%f>, <X2,Y2>=<%f,%f>\r\n",__func__,X1,Y1,X2,Y2);
	printf("\r\nVp1=<%d,%d>\r\n",Vp1.pano,Vp1.tilt);
	printf("\r\nVp2=<%d,%d>\r\n",Vp2.pano,Vp2.tilt);
	printf("\r\n[%s]:node_PTZ=<%d,%d,%d>\r\n",	__func__, Vp.pano,Vp.tilt,Vp.zoom);
#endif
	pThis->setQueryZoomFlag(true);
	SENDST trkmsg;
	memset((void*)&trkmsg,0,sizeof(SENDST));
	if(needChangeZoom)
	{
		trkmsg.cmd_ID = acqPosAndZoom;
		memcpy(&trkmsg.param[0],&(Vp.pano), sizeof(int));
		memcpy(&trkmsg.param[4],&(Vp.tilt), sizeof(int)); 
		int currentZoom = pThis->getCurrentZoomValue();
		memcpy(&trkmsg.param[8],&currentZoom, sizeof(int));
		ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
	}
	else
	{
		trkmsg.cmd_ID = speedloop;
		memcpy(&trkmsg.param[0],&(Vp.pano), sizeof(int));
		memcpy(&trkmsg.param[4],&(Vp.tilt), sizeof(int)); 
		ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
	}	
	return Vp;
}
GridMapNode CVideoProcess::getLinearDeviation(int px, int py)
{
	int offset_y = IMG_HEIGHT/2;
	int x = px;
	int y = py;
	switch(m_display.g_CurDisplayMode)
	{
		case MAIN_VIEW:			
			y = (py-offset_y)*2;
			break;
		default:
			break;
	}
	int valid_x = 0;
	int valid_y = 0;
	if(x<GRID_WIDTH/2 )
	{
		valid_x = GRID_WIDTH/2;
	}
	else if(x>IMG_WIDTH-GRID_WIDTH/2)
	{
		valid_x = IMG_WIDTH-GRID_WIDTH/2;
	}
	else{
		valid_x = x;
	}
	if(y<GRID_HEIGHT/2)
	{
		valid_y =GRID_HEIGHT/2; 
	}
	else if(y>IMG_HEIGHT-GRID_HEIGHT/2)
	{
		valid_y=IMG_HEIGHT-GRID_HEIGHT/2;
	}
	else {
		valid_y = y;
	}	
	int current_col = ((valid_x-GRID_WIDTH/2)/GRID_WIDTH) % GRID_COLS;
	int current_row = (valid_y-GRID_HEIGHT/2)/GRID_HEIGHT;
	int current_row_plus = current_row+1;
	int current_col_plus = current_col+1;
	GridMapNode Vp,Vp1,Vp2;
	GridMapNode V0 = m_gridNodes[current_row][current_col];
	GridMapNode V1 = m_gridNodes[current_row][current_col_plus];
	GridMapNode V2 = m_gridNodes[current_row_plus][current_col];
	GridMapNode V3 = m_gridNodes[current_row_plus][current_col_plus];
	
	printf("\r\n[%s]:V0 =<%d,%d>\r\nV1 =<%d,%d> \r\nV2 =<%d,%d> \r\nV3 =<%d,%d> \r\n ",__func__,
	V0.pano,V0.tilt,V1.pano,V1.tilt,V2.pano,V2.tilt,V3.pano,V3.tilt);

	
	float X1 = (float)V0.coord_x;
	float Y1 = (float)V0.coord_y;
	float X2 = (float)V1.coord_x;
	float Y2 = (float)V2.coord_y;
	float X = (float)valid_x;
	float Y = (float)valid_y;
#if 0
	if(V0.pano < V1.pano){
		float angle_left = (MAX_ANGLE-V0.pano)/PER_ANGLE;
		float angle_right = (V1.pano -ZERO_ANGLE)/PER_ANGLE;
		float X0 = X1 +(angle_left * GRID_WIDTH)/(angle_left +angle_right);

		if(X < X0)
		{
			Vp1.pano =(int)( ((X0-X)*(float)V0.pano) /(X0-X1)+((X-X1)*(float)MAX_ANGLE)/(X0-X1));
			Vp2.pano =(int)( ((X0-X)*(float)V2.pano) / (X0-X1) + ((X-X1)*(float)MAX_ANGLE)/(X0-X1));
		}
		else{
			Vp1.pano =(int)( ((X2-X)*(float)(0))/(X2-X0) + ((X-X0)*(float)V1.pano)/(X2-X0));
			Vp2.pano = (int)( ((X2-X)*(float)(0))/(X2-X0) + ((X-X0)*(float)V3.pano)/(X2-X0));

		}
	}else{
		Vp1.pano= (int)((X2-X)*((float)V0.pano)/(X2-X1) + (X-X1)*((float)V1.pano)/(X2-X1));
		Vp2.pano= (int)((X2-X)*((float)V2.pano)/(X2-X1) + (X-X1)*((float)V3.pano)/(X2-X1));
	}
#endif
	printf("\r\n[%s]:<X1,Y1>=<%f,%f>, <X2,Y2>=<%f,%f>\r\n",__func__,X1,Y1,X2,Y2);

	Vp1.pano= (int)((X2-X)*((float)V0.pano)/(X2-X1) + (X-X1)*((float)V1.pano)/(X2-X1));
	Vp2.pano= (int)((X2-X)*((float)V2.pano)/(X2-X1) + (X-X1)*((float)V3.pano)/(X2-X1));

	Vp1.tilt = (int)((X2-X)*((float)V0.tilt)/(X2-X1) + (X-X1)*((float)V1.tilt)/(X2-X1));
		
	printf("\r\nVp1=<%d,%d>\r\n",Vp1.pano,Vp1.tilt);
		
	Vp2.tilt = (int)((X2-X)*((float)V2.tilt)/(X2-X1) + (X-X1)*((float)V3.tilt)/(X2-X1));

	printf("\r\nVp2=<%d,%d>\r\n",Vp2.pano,Vp2.tilt);

	Vp.pano = (int)((Y2-Y)*((float)Vp1.pano)/(Y2-Y1) + (Y-Y1)*((float)Vp2.pano)/(Y2-Y1));
	Vp.tilt = (int)((Y2-Y)*((float)Vp1.tilt)/(Y2-Y1) + (Y-Y1)*((float)Vp2.tilt)/(Y2-Y1));
	Vp.zoom = 65535;		

	if(Vp.tilt < 0)
	{
		Vp.tilt = 32768- Vp.tilt;
	}
	printf("\r\n[%s]:node_PTZ=<%d,%d,%d>\r\n",	__func__, Vp.pano,Vp.tilt,Vp.zoom);

	pThis->setQueryZoomFlag(true);

	SENDST trkmsg;
	memset((void*)&trkmsg,0,sizeof(SENDST));
	trkmsg.cmd_ID = acqPosAndZoom;

	memcpy(&trkmsg.param[0],&(Vp.pano), sizeof(int));
	memcpy(&trkmsg.param[4],&(Vp.tilt), sizeof(int)); 
	int currentZoom = pThis->getCurrentZoomValue();
	memcpy(&trkmsg.param[8],&currentZoom, sizeof(int));
	ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
printf("\r\n[%s]: Send PTZ Message to Ball Camera !!",__func__);
	
	return Vp;
}

void CVideoProcess::auto_insertpoint(int x, int y)
{
	printf("%s, %d,%s start\n",__FILE__,__LINE__,__FUNCTION__);
	cv::Point2i inPoint;
	inPoint.x = x;
	inPoint.y = y;
	m_autofr.manualInsertRecommendPoints(inPoint);
}


void CVideoProcess::auto_draw_triangle_point(int x, int y)
{
	printf("%s, %d,%s start\n",__FILE__,__LINE__,__FUNCTION__);
	Point2i point_tmp;
	point_tmp.x = x;
	point_tmp.y = y;
	point_triangle = point_tmp;
	set_print_stat(true);
	set_draw_point_triangle_stat(true);

}


void CVideoProcess::moveball(int x, int y)
{
								SENDST trkmsg={0};
								cv::Point tmp;
								Point2i inPoint, outPoint;
								tmp.x = x;
								tmp.y = y;
								pThis->mapout2inresol(&tmp);
								//pThis->mapgun2fullscreen_point(&tmp.x,&tmp.y);
								inPoint.x = tmp.x;
								inPoint.y = tmp.y;
								pThis->m_autofr.Point2getPos(inPoint, outPoint);
								printf("%s, %d,inPoint(%d,%d),outPoint(%d,%d)\n", __FILE__,__LINE__,inPoint.x,inPoint.y,outPoint.x,outPoint.y);
								
								trkmsg.cmd_ID = acqPosAndZoom;
								memcpy(&trkmsg.param[0],&(outPoint.x), sizeof(int));
								memcpy(&trkmsg.param[4],&(outPoint.y), sizeof(int)); 
								ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);

}

void CVideoProcess::auto_selectpoint(int x, int y)
{
	std::vector<FEATUREPOINT_T> app_recommendPoints_tmp = app_recommendPoints;
	int delta_distance_bak;
	int insert_index = -1;
	int delta_distance;
	
	for(int i = 0; i < app_recommendPoints_tmp.size();  i++)
	{
		int deltax = abs(app_recommendPoints_tmp[i].pixel.x-x);
		int deltay = abs(app_recommendPoints_tmp[i].pixel.y-y);
		
		delta_distance = sqrt(deltax*deltax + deltay*deltay);
			
		if(i == 0)
		{
			delta_distance_bak = delta_distance;
			insert_index = i;
		}
		else
		{
			if(delta_distance < delta_distance_bak)
			{
				delta_distance_bak = delta_distance;
				insert_index = i;
			}

		}
	}
	printf("%s,%d,  delta_distance_bak=%d\n",__FILE__,__LINE__,delta_distance_bak);
	if((insert_index >= 0))
	{
		printf("%s, %d, select pixel(%d, %d),pos(%d,%d)\n", __FILE__,__LINE__,app_recommendPoints_tmp[insert_index].pixel.x,app_recommendPoints_tmp[insert_index].pixel.y,app_recommendPoints_tmp[insert_index].pos.x,app_recommendPoints_tmp[insert_index].pos.y);
		m_autofr.selectPoint(app_recommendPoints_tmp[insert_index].pixel);
		set_trig_PTZflag(1);
	}
}

void CVideoProcess::insertPos(int x, int y)
{
	cv::Point2i inPos;
	inPos.x = x;
	inPos.y = y;
	m_autofr.insertPos(inPos);
}


static Point ptStart,ptEnd;
void CVideoProcess::mouse_event(int button, int state, int x, int y)
{
	unsigned int curId;
	static int tempX=0,tempY=0;
	 static bool isRectangleStartPointValid = false;
	  static bool isRectValid = false;
	
	if(pThis->m_display.g_CurDisplayMode == MAIN_VIEW)
		curId = 1;	
	else
		curId = pThis->m_curChId;

	if((pThis->m_display.g_CurDisplayMode == MAIN_VIEW) && (pThis->m_display.m_menuindex == -1)&&(button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) && pThis->InJoys(x,y))
	{
		pThis->joys_click = 1;
		cv::Point tmp = cv::Point(x, y);
		pThis->mapout2inresol(&tmp);
		pThis->jcenter_s = tmp;
		pThis->sendjoyevent(tmp);

	}
	else if((pThis->m_display.g_CurDisplayMode == MAIN_VIEW) && (pThis->m_display.m_menuindex == -1)&&(button == GLUT_RIGHT_BUTTON && state == GLUT_UP) && pThis->InJoys(x,y) && pThis->joys_click)
	{
		pThis->joys_click = 0;
		pThis->jcenter_s = pThis->get_joycenter();
		pThis->sendjoyevent(pThis->jcenter_s);
	}
	//else if(mouse_workmode == SetMteRigion_Mode)
	else if( pThis->m_display.g_CurDisplayMode == GUN_FULL_SCREEN && 1 == pThis->setrigion_flagv20)
	{
		if(pThis->setrigion_flagv20)
		{
			pThis->mouse_eventv20(button, state, x, y);
			return;
		}
	}	
	else if ( (pThis->m_display.g_CurDisplayMode == TEST_RESULT_VIEW) ) {
		if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN){
			//printf("Click  IN  TEST_RESULT_VIEW  DisplayMode\r\n");
			pThis->Test_Match_result(x,y);
			return ;
		}
	}

	else if(TRIG_INTER_MODE == pThis->m_display.g_CurDisplayMode)
	{
		
		if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
		{
			int stat = pThis->get_manualInsertRecommendPoints_stat();
			if(1 == stat)
			{
        			pThis->auto_insertpoint(x, y);
			}
			else if(2 == stat)
			{
        			pThis->auto_draw_triangle_point(x, y);
				pThis->moveball(x, y);
			}
			else if(0 == stat)
			{
				pThis->auto_selectpoint(x, y);
			}	
       	 }
        	

    }
	else if(pThis->m_display.g_CurDisplayMode == GRID_MAP_VIEW)
	{
		if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN){

			int valid_x = x;
			int valid_y = y;
			
			GridMapNode temp_node = pThis->getLinearDeviation(valid_x,valid_y,GRID_WIDTH_120,GRID_HEIGHT_90,true);

		}
	}
	else {
		if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN &&(g_AppWorkMode == HANDLE_LINK_MODE)) {
			if (pThis->open_handleCalibra) // Press 'y' or 'Y' , set this flag to 1
			{
				pThis->OnMouseLeftDwn(x, y);
			}
			else	
			{
				if(pThis->click_legal(x,y) )
				{
					if(y >(m_staticScreenHeight/2) ) 
					{
						isRectangleStartPointValid = true;
						pThis->m_click = 1;
						pThis->addstartpoint(x, y, curId);

						pThis->LeftPoint.x = x;
						pThis->LeftPoint.y = y;
						ptStart = Point(x,y);							
					}
					
					tempX = x;
					tempY = y;
					m_bLDown = true;					
				}
				else{
					//printf("click illegal!!!\n");
				}
			}
		}
		if(button == GLUT_LEFT_BUTTON && state == GLUT_UP &&(g_AppWorkMode == HANDLE_LINK_MODE))
		{
			if(pThis->move_legal(x,y))
			{
				int tmpY = y;
				int tmpX = x;
				if(tmpY <(m_staticScreenHeight/2 )) {
					tmpY = (m_staticScreenHeight/2);	
				}
				ptEnd = Point(x,y);
				if(abs(ptEnd.x - ptStart.x) > 10) /* If rectangle's width  < 10 pixels , do nothing !*/
				{
					isRectValid = true;					
					pThis->addendpoint(tmpX, tmpY, curId);					
				}
				pThis->m_click = 0;
				pThis->m_draw = 0;

				if( (tempX == x) && (tempY == y) && (m_bLDown== true) ) {
					if(g_AppWorkMode == HANDLE_LINK_MODE )
					{
						m_bLDown = false;
						m_bIsClickMode = true;
						if(y>(m_staticScreenHeight/2)) {
							pThis->CaptureMouseClickPoint(x,y);
						}	
						
						if((y > 0) && (y< m_staticScreenHeight/2)) {
							if((x>(m_staticScreenWidth/4)) && (x <(m_staticScreenWidth*3/4))) {
				/* If user click on Ball Camera Image, then Ball Camera's focus move to the point where user click*/
								pThis->MvBallCamByClickBallIMg(x,y); 	
							}
						}
						else{
				/* If user click on Gun Camera Image, then Ball Camera will move its focus to the projectPoints where user click on Gun Image */
							
							if(1 == g_GridMapMode){
								pThis->getLinearDeviation(x,y,GRID_WIDTH_120,GRID_HEIGHT_90,true);//getLinearDeviation(x,y);
							}
							else if(2 == g_GridMapMode){
								SENDST trkmsg={0};
								cv::Point tmp;
								Point2i inPoint, outPoint;
								tmp.x = x;
								tmp.y = y;
								pThis->mapout2inresol(&tmp);
								//pThis->mapgun2fullscreen_point(&tmp.x,&tmp.y);
								inPoint.x = tmp.x;
								inPoint.y = tmp.y;
								inPoint.y = (inPoint.y - 540) * 2;
								pThis->m_autofr.Point2getPos(inPoint, outPoint);
								printf("%s, %d,inPoint(%d,%d),outPoint(%d,%d)\n", __FILE__,__LINE__,inPoint.x,inPoint.y,outPoint.x,outPoint.y);
								
								trkmsg.cmd_ID = acqPosAndZoom;
								memcpy(&trkmsg.param[0],&(outPoint.x), sizeof(int));
								memcpy(&trkmsg.param[4],&(outPoint.y), sizeof(int)); 
								ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
							}
						}
				      }
				}
				else
				{
					m_bIsClickMode = false;					
					pThis->RightPoint.x = tmpX;
					pThis->RightPoint.y = tmpY;	
					if(g_AppWorkMode == HANDLE_LINK_MODE ) {
						if( ( isRectangleStartPointValid == true ) &&( isRectValid == true )) {
							isRectangleStartPointValid = false;
							isRectValid  = false;
						
						if(1 == g_GridMapMode)
						{
							pThis->MvBallCamUseLinearDeviationSelectRect(tmpX, tmpY, true);
						}
						else if(2 == g_GridMapMode)
						{
								SENDST trkmsg={0};
								cv::Point tmp;
								Point2i inPoint, outPoint;
								tmp.x = x;
								tmp.y = y;
								pThis->mapout2inresol(&tmp);
								//pThis->mapgun2fullscreen_point(&tmp.x,&tmp.y);
								inPoint.x = tmp.x;
								inPoint.y = tmp.y;
								inPoint.y = (inPoint.y - 540) * 2;
								pThis->m_autofr.Point2getPos(inPoint, outPoint);
								printf("%s, %d,inPoint(%d,%d),outPoint(%d,%d)\n", __FILE__,__LINE__,inPoint.x,inPoint.y,outPoint.x,outPoint.y);
								
								trkmsg.cmd_ID = acqPosAndZoom;
								memcpy(&trkmsg.param[0],&(outPoint.x), sizeof(int));
								memcpy(&trkmsg.param[4],&(outPoint.y), sizeof(int)); 
								ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
						}						
						}
					}
				}
			}
			else{
				//printf("move illegal!!!\n");	
			}		
	}
		
		if(button == 3)
		{
			pThis->m_click = 0;
			pThis->m_rectn[curId] = 0;
			pThis->m_draw = 0;
		}
	}
	
}

void CVideoProcess::mousemove_event(GLint xMouse, GLint yMouse)
{

}

void CVideoProcess::mousemotion_event(GLint xMouse, GLint yMouse)
{

	unsigned int curId;
	ptEnd = Point(xMouse,xMouse);
	if(pThis->m_display.g_CurDisplayMode == MAIN_VIEW)
		curId = 1;
	else
		curId = pThis->m_curChId;
	
	if((pThis->m_display.g_CurDisplayMode == MAIN_VIEW) && pThis->InJoys(xMouse,yMouse) && pThis->joys_click)
	{
		cv::Point tmp = cv::Point(xMouse, yMouse);
		pThis->mapout2inresol(&tmp);
		pThis->jcenter_s = tmp;
		pThis->sendjoyevent(tmp);
	}
	else if(MAIN_VIEW == pThis->m_display.g_CurDisplayMode)
	{
		
		float floatx,floaty;
		floatx = xMouse;
		floaty = yMouse;	
		pThis->map1080p2normal_point(&floatx, &floaty);
		pThis->mapnormal2curchannel_point(	&floatx, &floaty, vdisWH[curId][0], vdisWH[curId][1] );	
		if(pThis->m_click == 1 && yMouse > 540 &&(	abs(ptEnd.x - ptStart.x) > 10)	)
		{
			pThis->m_tempX = floatx;
			pThis->m_tempY = floaty;
			pThis->m_draw = 1;
		}
	}
}

void CVideoProcess::draw_mouse_move(GLint xMouse, GLint yMouse)
{
	jos_mouse.x = xMouse;
	jos_mouse.y = yMouse;
}

void CVideoProcess::mouse_eventv20(int button, int state, int x, int y)
{
	//printf("mouse_eventv20 start, button=%d,state=%d,x,y(%d,%d)\n", button, state, x, y);
	
	menu_param_t tmpCmd = {0};
	tmpCmd.Mtdmouseclick.button = button;
	tmpCmd.Mtdmouseclick.state = state;
	tmpCmd.Mtdmouseclick.x = x;
	tmpCmd.Mtdmouseclick.y = y;
	app_ctrl_setMtdRigion(&tmpCmd);
}

void CVideoProcess::menu_event(int value)
{
	switch(value)
    {
		case 0:
			pThis->OnKeyDwn('a');
			break;
		default:
			break;
	}
}
void CVideoProcess::addstartpoint(int x, int y, int curId)
{
	pThis->m_rectn[curId] = 0;
	pThis->mRect[curId][pThis->m_rectn[curId]].x1 = x;
	pThis->mRect[curId][pThis->m_rectn[curId]].y1 = y;
	//cout<<" start:("<<pThis->mRect[curId][pThis->m_rectn[curId]].x1<<","<<pThis->mRect[curId][pThis->m_rectn[curId]].y1<<")"<<endl;
}

void CVideoProcess::addendpoint(int x, int y, int curId)
{
	pThis->mRect[curId][pThis->m_rectn[curId]].x2 = x;
	pThis->mRect[curId][pThis->m_rectn[curId]].y2 = y;
	//cout<<" end:("<<pThis->mRect[curId][pThis->m_rectn[curId]].x2<<","<<pThis->mRect[curId][pThis->m_rectn[curId]].y2<<")\n"<<endl;

	mouserect rectsrc, recvdest;
	rectsrc.x = pThis->mRect[curId][pThis->m_rectn[curId]].x1;
	rectsrc.y = pThis->mRect[curId][pThis->m_rectn[curId]].y1;
	rectsrc.w = pThis->mRect[curId][pThis->m_rectn[curId]].x2 - pThis->mRect[curId][pThis->m_rectn[curId]].x1;
	rectsrc.h = pThis->mRect[curId][pThis->m_rectn[curId]].y2 - pThis->mRect[curId][pThis->m_rectn[curId]].y1;

	recvdest = pThis->map2preview(rectsrc);
		
	pThis->m_rectn[curId]++;  
	if(pThis->m_rectn[curId]>=sizeof(pThis->mRect[0]))
	{
		printf("mouse rect reached maxnum:100!\n");
		pThis->m_rectn[curId]--;
	}
}

void CVideoProcess::processrigionMenu(int value)
{
	//printf("set rigion\n");
	pThis->setrigon_flag = 1;
}

void CVideoProcess::processrigionselMenu(int value)
{
	//printf("%s start, value=%d\n", __FUNCTION__, value);
}

void CVideoProcess::processrigionpolygonMenu(int value)
{
	//printf("%s start, value=%d\n", __FUNCTION__, value);
	pThis->setrigon_polygon = 1;
}

int CVideoProcess::InJoys(int x, int y)
{
	int tmpx = x;
	int tmpy = y;
	cv::Point tmp = cv::Point(x, y);
	int jradius = get_joyradius();
	cv::Point jcenter = get_joycenter();

	mapout2inresol(&tmp);
	tmp.x = abs(tmp.x - jcenter.x);
	tmp.y = abs(tmp.y - jcenter.y);
	if(sqrt(tmp.x * tmp.x + tmp.y * tmp.y) <= jradius)
	{
		//printf("%s,%d,int joy\n", __FILE__,__LINE__);
		return 1;
	}
	else
	{
		//printf("%s,%d, not in joy\n", __FILE__,__LINE__);
		return 0;
	}
}

void CVideoProcess::mapout2inresol(cv::Point *tmppoint)
{
	int outputWHF_bak[3];
	memcpy(&outputWHF_bak, &outputWHF, sizeof(outputWHF_bak));
	tmppoint->x = tmppoint->x * vdisWH[1][0] / outputWHF_bak[0];
	tmppoint->y = tmppoint->y * vdisWH[1][1] / outputWHF_bak[1];	
}

void CVideoProcess::sendjoyevent(cv::Point tmppoint)
{
	SENDST test;
	test.cmd_ID = ipcjoyevent;

	mapcapresol2joy(&tmppoint);
		
	CMD_JOY_EVENT cmd_joyevent;
	cmd_joyevent.type = IPC_JS_EVENT_AXIS;

	static cv::Point tmppoint_old = cv::Point(0,0);

	if(tmppoint.x != tmppoint_old.x)
	{
		cmd_joyevent.number = IPC_MSGID_INPUT_AXISX;
		cmd_joyevent.value = tmppoint.x;
		memcpy(test.param, &cmd_joyevent, sizeof(cmd_joyevent));
		ipc_sendmsg(&test, IPC_FRIMG_MSG);
		tmppoint_old.x = tmppoint.x;
	}

	if(tmppoint.y != tmppoint_old.y)
	{
		cmd_joyevent.number = IPC_MSGID_INPUT_AXISY;
		cmd_joyevent.value = tmppoint.y;
		memcpy(test.param, &cmd_joyevent, sizeof(cmd_joyevent));
		ipc_sendmsg(&test, IPC_FRIMG_MSG);
		tmppoint_old.y = tmppoint.y;
	}
}

void CVideoProcess::mapcapresol2joy(cv::Point *tmppoint)
{
	int jradius = get_joyradius();
	cv::Point jcenter = get_joycenter();
	int ratio = 32767 / jradius;

	tmppoint->x = (tmppoint->x - jcenter.x) * ratio;
	tmppoint->y = (tmppoint->y - jcenter.y) * ratio;
}

int CVideoProcess::get_joyradius()
{
	return vdisWH[0][1]/8;
}

cv::Point CVideoProcess::get_joycenter()
{
	return cv::Point(vdisWH[0][0]/8+20, vdisWH[0][1]/2 - vdisWH[0][0]/8);
}

#if __MOVE_DETECT__
void CVideoProcess::processmaxnumMenu(int value)
{
	if(0 == value)
		pThis->detectNum = 5;
	else if(1 == value)
		pThis->detectNum = 10;
}

void CVideoProcess::processmaxtargetsizeMenu(int value)
{
	if(0 == value)
		pThis->maxsize= 40000;
	else if(1 == value)
		pThis->maxsize= 50000;
}

void CVideoProcess::processmintargetsizeMenu(int value)
{
	if(0 == value)
		pThis->minsize= 100;
	else if(1 == value)
		pThis->minsize= 1000;
}
#endif
void CVideoProcess::keyboard_event(unsigned char key, int x, int y)
{
	pThis->OnKeyDwn(key);

	if(key == 27){
		pThis->destroy();
		exit(0);
	}
}

void CVideoProcess::keySpecial_event( int key, int x, int y)
{
	//pThis->OnKeyDwn((unsigned char)key);
	pThis->OnSpecialKeyDwn(key,x,y);
}

void CVideoProcess::visibility_event(int state)
{
	OSA_printf("visibility event: %d\n", state);
}

void CVideoProcess::close_event(void)
{
	OSA_printf("close event\n");
	pThis->destroy();
}

int CVideoProcess::init()
{
	DS_InitPrm dsInit;
	memset(&dsInit, 0, sizeof(DS_InitPrm));
	dsInit.timefunc = processtimeMenu;	
	dsInit.manualcarli = processsmanualcarliMenu;
	dsInit.autocarli = processsautocarliMenu;	
	dsInit.menufunc = menu_event;
	dsInit.mousefunc = mouse_event;
	dsInit.passivemotionfunc = mousemove_event;
	dsInit.motionfunc = mousemotion_event;
	dsInit.setrigion = processrigionMenu;
	dsInit.rigionsel = processrigionselMenu;
	dsInit.rigionpolygon = processrigionpolygonMenu;
#if __MOVE_DETECT__
	dsInit.maxnum = processmaxnumMenu;
	dsInit.maxsize= processmaxtargetsizeMenu;
	dsInit.minsize= processmintargetsizeMenu;
#endif
	dsInit.disFPS = 30; // 20181219
	dsInit.disSched = 33 ; //   3.5      
//#if (!__IPC__)
	dsInit.keyboardfunc = keyboard_event; 
	dsInit.keySpecialfunc = keySpecial_event;
//#endif	
	dsInit.timerfunc = call_run;
	//dsInit.idlefunc = call_run;
	dsInit.visibilityfunc = visibility_event;
	dsInit.timerfunc_value = 0;
	dsInit.timerInterval = 16;//ms
	dsInit.closefunc = close_event;
	dsInit.bFullScreen = true;
	dsInit.winPosX = 200;
	dsInit.winPosY = 100;
	dsInit.winWidth = vdisWH[1][0];
	dsInit.winHeight = vdisWH[1][1];

	m_display.init(&dsInit);

	m_display.m_bOsd = true;
	m_display.m_crossOsd = true;
	OnInit();
	prichnalid=1;//fir


#if __MOVE_DETECT__
	initMvDetect();
	LoadMtdSelectArea("SaveMtdArea.yml",edge_contours);
#endif

	eventLoop = new EventLoop(proc);
	if(eventLoop == NULL)
	{
		printf("\r\n[%s]:XXX: Create Event Loop Failed !!!",__FUNCTION__);
	}
	else
	{
		int returnValue = eventLoop->Init();
		if(returnValue != -1) 
		{
			eventLoop->RunService();
		}
	}	

//============================================
	if(g_AppWorkMode == AUTO_LINK_MODE){
		CMD_EXT Msg;
		Msg.MtdState[Msg.SensorStat] = ipc_eImgAlg_Enable;	
		app_ctrl_setMtdStat(&Msg);
		MSGAPI_msgsend(mtd);
	}
//============================================
	
	return 0;
}


int CVideoProcess::dynamic_config(int type, int iPrm, void* pPrm)
{
	int iret = 0;
	unsigned int render=0;
	OSA_mutexLock(&m_mutex);

	if(type < CDisplayer::DS_CFG_Max){
		iret = m_display.dynamic_config((CDisplayer::DS_CFG)type, iPrm, pPrm);
	}

	switch(type)
	{
	case VP_CFG_MainChId:
		m_curChId = iPrm;
		m_iTrackStat = 0;
		mainProcThrObj.bFirst = true;
		m_display.dynamic_config(CDisplayer::DS_CFG_ChId, 0, &m_curChId);
		break;
	case VP_CFG_SubChId:
		m_curSubChId = iPrm;
		m_display.dynamic_config(CDisplayer::DS_CFG_ChId, 1, &m_curSubChId);
		break;
	case VP_CFG_TrkEnable:
		m_bTrack = iPrm;
		m_bakChId = m_curChId;
		m_iTrackStat = 0;
		m_iTrackLostCnt = 0;
		mainProcThrObj.bFirst = true;
		if(pPrm == NULL)
		{			
			UTC_RECT_float rc;
			rc.width 	=  60;//m_acqRectW;
			rc.height 	=  60;//m_acqRectH;
			rc.x 		=  msgextInCtrl->AvtPosX[m_curChId] - rc.width/2;
			rc.y 		=  msgextInCtrl->AvtPosY[m_curChId] - rc.height/2;
			m_rcTrack   = rc;
			m_rcAcq 	= rc;
		}
		else
		{
			m_rcTrack = *(UTC_RECT_float*)pPrm;
		}
		break;
	case VP_CFG_MmtEnable:
		m_bMtd = iPrm;
		break;
	case VP_CFG_SubPicpChId:
		m_curSubChId = iPrm;
		if(pPrm!=NULL)
		render=*(int *)pPrm;
		m_display.dynamic_config(CDisplayer::DS_CFG_ChId, render, &m_curSubChId);
		break;
	case VP_CFG_MvDetect:
		m_bMoveDetect = iPrm;
		break;
	default:
		break;
	}

	if(iret == 0)
		OnConfig();

	OSA_mutexUnlock(&m_mutex);

	return iret;
}

#if 1
int CVideoProcess::configEnhFromFile()
{
	string CalibFile;
	CalibFile = "config.yml";

	char calib_x[11] = "config_x";


	FILE *fp = fopen(CalibFile.c_str(), "rt");
	if(fp != NULL)
	{
		fseek(fp, 0, SEEK_END);
		int len = ftell(fp);
		fclose(fp);
		
		if(len < 10)
			return -1;
		else
		{
			
			FileStorage fr(CalibFile, FileStorage::READ);
			if(fr.isOpened())
			{
				
					sprintf(calib_x, "enhmod_%d", 0);
					Enhmod= (int)fr[calib_x];

					sprintf(calib_x, "enhparm_%d", 1);
					Enhparm= (float)fr[calib_x];

					sprintf(calib_x, "mmtdparm_%d", 2);
					DetectGapparm= (int)fr[calib_x];

					sprintf(calib_x, "mmtdparm_%d", 3);
					MinArea= (int)fr[calib_x];

					sprintf(calib_x, "mmtdparm_%d", 4);
					MaxArea= (int)fr[calib_x];

					sprintf(calib_x, "mmtdparm_%d", 5);
					stillPixel= (int)fr[calib_x];

					sprintf(calib_x, "mmtdparm_%d", 6);
					movePixel= (int)fr[calib_x];

					sprintf(calib_x, "mmtdparm_%d", 7);
					lapScaler= (float)fr[calib_x];
				

					sprintf(calib_x, "mmtdparm_%d", 8);
					lumThred= (int)fr[calib_x];
					
					sprintf(calib_x, "timedisp_%d", 9);
					m_display.disptimeEnable= (int)fr[calib_x];
				return 0;
			}else
				return -1;
		}
	}else
		return -1;
}
#endif


int CVideoProcess::run()
{
	MultiCh.run();
	m_display.run();
	
	#if __TRACK__
	m_track = CreateUtcTrk();
	#endif
	
	for(int i=0; i<MAX_CHAN; i++){
		m_mtd[i] = (target_t *)malloc(sizeof(target_t));
		if(m_mtd[i] != NULL)
			memset(m_mtd[i], 0, sizeof(target_t));

		OSA_printf(" %d:%s mtd malloc %p\n", OSA_getCurTimeInMsec(),__func__, m_mtd[0]);
	}
#if __MMT__
	m_MMTDObj.SetTargetNum(MAX_TARGET_NUMBER);
#endif
	OnRun();
	return 0;
}

int CVideoProcess::stop()
{
	if(m_track != NULL)
	{
		#if __TRACK__
			DestroyUtcTrk(m_track);
		#endif
	}
	m_track = NULL;
	
	m_display.stop();
	MultiCh.stop();

	OnStop();
	return 0;
}

void CVideoProcess::call_run(int value)
{
	pThis->process_event(0, 0, NULL);
}

void CVideoProcess::process_event(int type, int iPrm, void *pPrm)
{

	Ontimer();
	if(type == 0)//timer event from display
	{
	}
	
}

int CVideoProcess::callback_process(void *handle, int chId, int virchId, Mat frame)
{
	CVideoProcess *pThis = (CVideoProcess*)handle;
	return pThis->process_frame(chId, virchId, frame);
}

static void extractYUYV2Gray(Mat src, Mat dst)
{
	int ImgHeight, ImgWidth,ImgStride, stride16x8;

	ImgWidth = src.cols;
	ImgHeight = src.rows;
	ImgStride = ImgWidth*2;
	stride16x8 = ImgStride/16;

	OSA_assert((ImgStride&15)==0);
//#pragma omp parallel for
	for(int y = 0; y < ImgHeight; y++)
	{
		uint8x8_t  * __restrict__ pDst8x8_t;
		uint8_t * __restrict__ pSrc8_t;
		pSrc8_t = (uint8_t*)(src.data+ ImgStride*y);
		pDst8x8_t = (uint8x8_t*)(dst.data+ ImgWidth*y);
		for(int x=0; x<stride16x8; x++)
		{
			uint8x8x2_t d;
			d = vld2_u8((uint8_t*)(pSrc8_t+16*x));
			pDst8x8_t[x] = d.val[0];
		}
	}
}

void CVideoProcess::extractYUYV2Gray2(Mat src, Mat dst)
{
	int ImgHeight, ImgWidth,ImgStride;

	ImgWidth = src.cols;
	ImgHeight = src.rows;
	ImgStride = ImgWidth*2;
	uint8_t  *  pDst8_t;
	uint8_t *  pSrc8_t;

	pSrc8_t = (uint8_t*)(src.data);
	pDst8_t = (uint8_t*)(dst.data);
//#pragma UNROLL 4
//#pragma omp parallel for
	for(int y = 0; y < ImgHeight*ImgWidth; y++)
	{
		pDst8_t[y] = pSrc8_t[y*2];
	}
}


#if __TRACK__
void CVideoProcess::Track_fovreacq(int fov,int sensor,int sensorchange)
{
	//UTC_RECT_float rect;
	unsigned int currentx=0;
	unsigned int currenty=0;
	unsigned int TvFov[3] 	= {120,48,16};//Big-Mid-Sml:2400*5%,960*5%,330*5%
	unsigned int FrFov[5] 	= {200,120,50,16,6};//Big-Mid-Sml-SuperSml-Zoom:4000*5%,2400*5%,1000*5%,330*5%,120*5%

	if(sensorchange == 1){
		currentx = msgextInCtrl->AvtPosX[m_curChId];
		currenty = msgextInCtrl->AvtPosY[m_curChId];
	}
	else
	{		
			
		currentx=trackinfo_obj->trackrect.x+trackinfo_obj->trackrect.width/2;
		currenty=trackinfo_obj->trackrect.y+trackinfo_obj->trackrect.height/2;
		
	}
	int prifov=trackinfo_obj->trackfov;
	
	double ratio=tan(3.1415926*fov/36000)/tan(3.1415926*prifov/36000);
	
	unsigned int w=trackinfo_obj->trackrect.width/ratio;
	unsigned int h=trackinfo_obj->trackrect.height/ratio;
	if(sensorchange)
	{
		w=w*vcapWH[sensor^1][0]/vcapWH[sensor][0];
		h=h*vcapWH[sensor^1][1]/vcapWH[sensor][1];

	}
	
	trackinfo_obj->trackfov=fov;

	trackinfo_obj->reAcqRect.width=w;
	trackinfo_obj->reAcqRect.height=h;
	trackinfo_obj->reAcqRect.x=currentx-w/2;
	trackinfo_obj->reAcqRect.y=currenty-h/2;

	//OSA_printf("XY(%f,%f),WH(%f,%f)\n",trackinfo_obj->reAcqRect.x,trackinfo_obj->reAcqRect.y,trackinfo_obj->reAcqRect.width,trackinfo_obj->reAcqRect.height);
}
void CVideoProcess::Track_reacq(UTC_RECT_float & rcTrack,int acqinterval)
{
	m_intervalFrame=acqinterval;
	m_rcAcq=rcTrack;
}


int CVideoProcess::ReAcqTarget()
{
	//printf("m_intervalFrame = %d \n\n",m_intervalFrame);
	int iRet = m_iTrackStat;
	if(m_bakChId != m_curChId){
		iRet = 0;
		m_rcTrack = m_rcAcq;
		m_bakChId = m_curChId;
		m_iTrackLostCnt = 0;	
	}
	
	if(m_intervalFrame > 0){
		m_intervalFrame--;
		if(m_intervalFrame == 0){
			iRet = 0;
			m_rcTrack = m_rcAcq;
			m_iTrackLostCnt = 0;
			OSA_printf("----------------Setting m_intervalFrame----------------\n");
		}
	}

	return iRet;

}

#endif

extern void cutColor(cv::Mat src, cv::Mat &dst, int code);
 int saveCount = 0;
#define TM
#undef TM 
int CVideoProcess::process_frame(int chId, int virchId, Mat frame)
{
	int format = -1;
	if(frame.cols<=0 || frame.rows<=0)
		return 0;
	
//	tstart = getTickCount();
	int  channel= frame.channels();

	static bool copy_once = true;
	

#ifdef TM
	cudaEvent_t	start, stop;
		float elapsedTime;
		( (		cudaEventCreate	(	&start)	) );
		( (		cudaEventCreate	(	&stop)	) );
		( (		cudaEventRecord	(	start,	0)	) );
#endif

	if(channel == 2){
//		cvtColor(frame,frame,CV_YUV2BGR_YUYV);
		if((chId == video_gaoqing0)||(chId == video_gaoqing)||(chId == video_gaoqing2)||(chId == video_gaoqing3))
			format = CV_YUV2BGR_YUYV;
		else if(chId == video_pal)
			format = CV_YUV2BGR_UYVY;
	}
	else {
//		cvtColor(frame,frame,CV_GRAY2BGR);
		format = CV_GRAY2BGR;
	}

	OSA_mutexLock(&m_mutex);


	if(chId == m_curSubChId)
	{
		if((chId==video_pal) && (virchId!= PAL_VIRCHID))
			;
		else
		{
			mainFrame[mainProcThrObj.pp] = frame;
			mainProcThrObj.cxt[mainProcThrObj.pp].bTrack = m_bTrack;
			mainProcThrObj.cxt[mainProcThrObj.pp].bMtd = m_bMtd;
			mainProcThrObj.cxt[mainProcThrObj.pp].bMoveDetect = m_bMoveDetect;
			mainProcThrObj.cxt[mainProcThrObj.pp].iTrackStat = m_iTrackStat;
			mainProcThrObj.cxt[mainProcThrObj.pp].chId = chId;
			if(mainProcThrObj.bFirst){
				mainFrame[mainProcThrObj.pp^1] = frame;
				mainProcThrObj.cxt[mainProcThrObj.pp^1].bTrack = m_bTrack;
				mainProcThrObj.cxt[mainProcThrObj.pp^1].bMtd = m_bMtd;
				mainProcThrObj.cxt[mainProcThrObj.pp^1].bMoveDetect = m_bMoveDetect;
				mainProcThrObj.cxt[mainProcThrObj.pp^1].iTrackStat = m_iTrackStat;
				mainProcThrObj.cxt[mainProcThrObj.pp^1].chId = chId;
				mainProcThrObj.bFirst = false;
			}
			OSA_semSignal(&mainProcThrObj.procNotifySem);
		}
	}

			if(	m_camCalibra->start_cloneVideoSrc == true || g_sysParam->isEnable_cloneSrcImage() ) 
			{
				//m_camCalibra->start_cloneVideoSrc = false;
				//printf("%s : cloneVideoSrc \n",__func__);
				#if !GUN_IMAGE_USEBMP
					if(chId == GUN_CHID){
						m_camCalibra->cloneGunSrcImgae(frame);
						//m_camCalibra->cvtGunYuyv2Bgr();
					}
				#endif
					
				if( chId == BALL_CHID )	{	
					m_camCalibra->cloneBallSrcImgae(frame);
					//m_camCalibra->cvtBallYuyv2Bgr();
				}
			}
			
			if(  chId == 0 && copy_once == true) {
				if(!frame.empty()) {
					frame.copyTo(gun_srcMat_remap);
					copy_once = false;
				}
			}
/********************************************************************************************/
#if 1
	if( (chId == capIndex/*m_display.getCapBMPChannel()*/ ) &&/* (captureOnePicture == true)*/(m_display.savePic_once == true) ){
			m_display.savePic_once = false;
			//captureOnePicture = false;
			memset(m_display.savePicName, 0, 20);
			sprintf(m_display.savePicName,"%02d.bmp",saveCount);
			saveCount ++;
			
			#if 1
				int nsize = imageListForCalibra.size();
				if(nsize<50)
				{
					m_detectCorners->m_cutIMG[nsize] = cv::Mat(frame.rows,frame.cols,CV_8UC3);
					cvtColor(frame,m_detectCorners->m_cutIMG[nsize],CV_YUV2BGR_YUYV);
					imageListForCalibra.push_back(m_detectCorners->m_cutIMG[nsize]);

					ImageList.push_back( m_detectCorners->m_cutIMG[nsize] );
					imwrite(m_display.savePicName,m_detectCorners->m_cutIMG[nsize]);
					m_camCalibra->ImageLists.push_back(m_detectCorners->m_cutIMG[nsize]);					
					m_detectCorners->SetCutDisplay(nsize, true);
				}
			#else
				Mat Dst(1080,1920,CV_8UC3);
				cvtColor(frame,Dst,CV_YUV2BGR_YUYV);
				imwrite(m_display.savePicName,Dst);
			#endif
	}
#endif

/**********************************************************************/
	if( (chId == g_connectAction.CurCalibraCam) && (showDetectCorners == true) ){
		if(cloneOneFrame == true){
			cloneOneFrame = false;
			//m_camCalibra->cloneCornerSrcImgae(frame);
			m_detectCorners->cloneCornerSrcImgae(frame);
			OSA_semSignal(&g_detectCorners);
		}
		if(captureCount > 50) {
			showDetectCorners =false;
		}
	}
/**********************************************************************/
	if(chId == m_curChId || chId == m_curSubChId)
	{
		if((chId == video_pal)&&(virchId != PAL_VIRCHID));
		else
			m_display.display(frame,  chId, format);		
	}
	OSA_mutexUnlock(&m_mutex);

//	OSA_printf("process_frame: chId = %d, time = %f sec \n",chId,  ( (getTickCount() - tstart)/getTickFrequency()) );
	//获得结束时间，并显示结果

#ifdef TM
		(	(	cudaEventRecord(	stop,	0	)	)	);
		(	(	cudaEventSynchronize(	stop)	)	);

		(	cudaEventElapsedTime(	&elapsedTime,	start,	stop)	);
		OSA_printf("ChId = %d, Time to YUV2RGB:	%3.1f	ms \n", chId, elapsedTime);

		(	(	cudaEventDestroy(	start	)	)	);
		(	(	cudaEventDestroy(	stop	)	)	);
#endif
	return 0;
}
#if __TRACK__
int CVideoProcess::process_track(int trackStatus, Mat frame_gray, Mat frame_dis, UTC_RECT_float &rcResult)
{
	IMG_MAT image;
	image.data_u8 = frame_gray.data;
	image.width = frame_gray.cols;
	image.height = frame_gray.rows;
	image.channels = 1;
	image.step[0] = image.width;
	image.dtype = 0;
	image.size = frame_gray.cols*frame_gray.rows;

	if(trackStatus != 0)
	{
		rcResult = UtcTrkProc(m_track, image, &trackStatus);		
	}
	else
	{
		//printf("track********x=%f y=%f w=%f h=%f  ax=%d xy=%d\n",rcResult.x,rcResult.y,rcResult.width,rcResult.height);
		UTC_ACQ_param acq;
		acq.axisX 	=	msgextInCtrl->AvtPosX[m_curChId];// image.width/2;//m_ImageAxisx;//
		acq.axisY 	=	msgextInCtrl->AvtPosY[m_curChId];//image.height/2;//m_ImageAxisy;//
		acq.rcWin.x = 	(int)(rcResult.x);
		acq.rcWin.y = 	(int)(rcResult.y);
		acq.rcWin.width  = (int)(rcResult.width);
		acq.rcWin.height = (int)(rcResult.height);

		if(acq.rcWin.width<0)
			{
				acq.rcWin.width=0;

			}
		else if(acq.rcWin.width>= image.width)
			{
				acq.rcWin.width=60;
			}
		if(acq.rcWin.height<0)
			{
				acq.rcWin.height=0;

			}
		else if(acq.rcWin.height>= image.height)
			{
				acq.rcWin.height=60;
			}
		if(acq.rcWin.x<0)
			{
				acq.rcWin.x=0;
			}
		else if(acq.rcWin.x>image.width-acq.rcWin.width)
			{

				acq.rcWin.x=image.width-acq.rcWin.width;
			}
		if(acq.rcWin.y<0)
			{
				acq.rcWin.y=0;
			}
		else if(acq.rcWin.y>image.height-acq.rcWin.height)
			{

				acq.rcWin.y=image.height-acq.rcWin.height;
			}

		{
			//printf("=========movestat = %d\n",moveStat);
			rcResult = UtcTrkAcq(m_track, image, acq);
			moveStat = false;
		}
		
		trackStatus = 1;
	}

	return trackStatus;
}

#endif

vector<Rect> Box(MAX_TARGET_NUMBER);
int CVideoProcess::process_mtd(ALGMTD_HANDLE pChPrm, Mat frame_gray, Mat frame_dis)
{
	

#if 0
	if(pChPrm != NULL && (pChPrm->state > 0))
	{
//		medium.create(frame_gray.rows, frame_gray.cols, frame_gray.type());

//		MediumFliter(frame_gray.data, medium.data, pChPrm->i_width, pChPrm->i_height);

		GaussFliter(frame_gray.data, pChPrm->Img[0], pChPrm->i_width, pChPrm->i_height);
		

		for(i = 0; i < MAX_SCALER; i++)
		{
			DownSample(pChPrm, pChPrm->Img[i], pChPrm->Img[i+1],
										pChPrm->i_width>>i, pChPrm->i_height>>i);
		}

		IMG_sobel(pChPrm->Img[1], pChPrm->sobel, pChPrm->i_width>>1,	 pChPrm->i_height>>1);

		AutoDetectTarget(pChPrm, frame_gray.data);

		FilterMultiTarget(pChPrm);

	}
#endif
	return 0;
}

#if __MOVE_DETECT__
void	CVideoProcess::initMvDetect()
{
	int	i;
	mouserect recttmp;
	OSA_printf("%s:mvDetect start ", __func__);
	OSA_assert(m_pMovDetector != NULL);

	m_pMovDetector->init(NotifyFunc, (void*)this);
					
	std::vector<cv::Point> polyWarnRoi ;
	polyWarnRoi.resize(4);
	edge_contours.resize(1);
	edge_contours[0].resize(4);

	for(i=0; i<MAX_CHAN; i++)
	{
		recttmp.x = vdisWH[i][0] * min_width_ratio;
		recttmp.y = vdisWH[i][1] * min_height_ratio;
		recttmp.w = vdisWH[i][0] * (max_width_ratio - min_width_ratio);
		recttmp.h = vdisWH[i][1] * (max_height_ratio - min_height_ratio); 

		recttmp = mapfullscreen2gun(recttmp);

		recttmp = mapgun2fullscreen(recttmp);
		polyWarnRoi[0]	= cv::Point(recttmp.x,recttmp.y);
	    polyWarnRoi[1]	= cv::Point(recttmp.x+recttmp.w,recttmp.y);
	    polyWarnRoi[2]	= cv::Point(recttmp.x+recttmp.w,recttmp.y+recttmp.h);
	    polyWarnRoi[3]	= cv::Point(recttmp.x,recttmp.y+recttmp.h);

		m_pMovDetector->setWarnMode(WARN_WARN_MODE, i);
		//m_pMovDetector->setWarningRoi(polyWarnRoi,	i);
	}


		recttmp.x = vdisWH[0][0] * min_width_ratio;
		recttmp.y = vdisWH[0][1] * min_height_ratio;
		recttmp.w = vdisWH[0][0] * (max_width_ratio - min_width_ratio);
		recttmp.h = vdisWH[0][1] * (max_height_ratio - min_height_ratio); 
		recttmp = mapfullscreen2gun(recttmp);
		pThis->edge_contours[0][0].x = recttmp.x;
		pThis->edge_contours[0][0].y = recttmp.y;
		pThis->edge_contours[0][1].x = recttmp.x+recttmp.w;
		pThis->edge_contours[0][1].y = recttmp.y;
		pThis->edge_contours[0][2].x = recttmp.x+recttmp.w;
		pThis->edge_contours[0][2].y = recttmp.y+recttmp.h;
		pThis->edge_contours[0][3].x = recttmp.x;
		pThis->edge_contours[0][3].y = recttmp.y+recttmp.h;

		recttmp = mapgun2fullscreen(recttmp);
		polyWarnRoi[0]	= cv::Point(recttmp.x,recttmp.y);
		polyWarnRoi[1]	= cv::Point(recttmp.x+recttmp.w,recttmp.y);
		polyWarnRoi[2]	= cv::Point(recttmp.x+recttmp.w,recttmp.y+recttmp.h);
		polyWarnRoi[3]	= cv::Point(recttmp.x,recttmp.y+recttmp.h);

		m_pMovDetector->setWarnMode(WARN_WARN_MODE, 0);
		m_pMovDetector->setWarningRoi(polyWarnRoi,	0);
}

void	CVideoProcess::DeInitMvDetect()
{
	if(m_pMovDetector != NULL)
		m_pMovDetector->destroy();
}

void CVideoProcess::NotifyFunc(void *context, int chId)
{
	CVideoProcess *pParent = (CVideoProcess*)context;
	int cnt = pThis->detect_vect_arr.size() > 3 ? 3 : pThis->detect_vect_arr.size();
	pThis->detect_vect_arr[chId].clear();
	pThis->m_pMovDetector->getWarnTarget(pThis->detect_vect_arr[chId],chId);
	
	//pParent->m_display.m_bOsd = true;
	//pThis->m_display.UpDateOsd(0);
}
#endif

void CVideoProcess::getImgRioDelta(unsigned char* pdata,int width ,int height,UTC_Rect rio,double * value)
{
	unsigned char * ptr = pdata;
	int Iij;
	//gray max ,gray min ,gray average,gray delta
	double Imax = 0, Imin = 255, Iave = 0, Idelta = 0;
	for(int i=rio.y;i<rio.height+rio.y;i++)
	{
		for(int j=rio.x;j<rio.width+rio.x;j++)
		{
			Iij	= ptr[i*width+j];
			if(Iij > Imax)
				Imax = Iij;
			if(Iij < Imin)
				Imin = Iij;
			Iave = Iave + Iij;
		}
	}
	Iave = Iave/(rio.width*rio.height);
	for(int i=rio.y;i<rio.height+rio.y;i++)
	{
		for(int j=rio.x;j<rio.width+rio.x;j++)
		{
			Iij 	=  ptr[i*width+j];
			Idelta = Idelta + (Iij-Iave)*(Iij-Iave);
		}
	}
	Idelta = Idelta/(rio.width*rio.height);
	*value = Idelta;
}

/************************************************************************************************************/
void CVideoProcess::set_mouse_show(int param)
{
	mouse_show = param;
}

void CVideoProcess::setMtdState(bool flag)
{
	m_bMoveDetect = flag;
	return;
}
const bool CVideoProcess::getMtdState()
{
	return m_bMoveDetect;
}
void CVideoProcess::set_trig_PTZflag(bool flag)
{
	trig_inter_flag = flag;
	return ;
};
bool CVideoProcess::get_trig_PTZflag()
{
	return trig_inter_flag;
};


void CVideoProcess::SaveMtdSelectArea(const char* filename, std::vector< std::vector< cv::Point > > edge_contours)
{
	char paramName[40];
	memset(paramName,0,sizeof(paramName));
	m_fsWriteMtd.open(filename,FileStorage::WRITE);
	if(m_fsWriteMtd.isOpened())
	{		

		memset(paramName,0,sizeof(paramName));
		sprintf(paramName,"AreaCount");	
		int total_size = edge_contours.size();
		m_fsWriteMtd<< paramName  << total_size;

	
		for(int m = 0; m<edge_contours.size(); m++ )
		{			
			memset(paramName,0,sizeof(paramName));
			sprintf(paramName,"AreaIndex_%d",m);
			int count  =  edge_contours[m].size();
			m_fsWriteMtd<< paramName << count;
		}

				
		for(int i = 0; i < edge_contours.size(); i++)
		{
			for(int j = 0; j < edge_contours[i].size(); j++)
			{
				
				sprintf(paramName,"Point_%d_%d_x",i,j);				
				m_fsWriteMtd<<paramName <<edge_contours[i][j].x;
				
				memset(paramName,0,sizeof(paramName));
				sprintf(paramName,"Point_%d_%d_y",i,j);				
				m_fsWriteMtd<<paramName <<edge_contours[i][j].y;		
			}		
		}		
		m_fsWriteMtd.release();		
		
	}
}

void CVideoProcess::LoadMtdSelectArea(const char* filename, std::vector< std::vector< cv::Point > > &edge_contours)
{
	char paramName[40];
	memset(paramName,0,sizeof(paramName));
	int AreaCount=0;
	int IndexArray[5];	
	edge_contours.clear(); // Clear The Vector
	m_fsReadMtd.open(filename,FileStorage::READ);
	if(m_fsReadMtd.isOpened())
	{
		memset(paramName,0,sizeof(paramName));
		sprintf(paramName,"AreaCount");				
		m_fsReadMtd[paramName] >>AreaCount;
		if(AreaCount !=0)
		{
			for(int i=0; i< AreaCount;i++)
			{
				memset(paramName,0,sizeof(paramName));
				sprintf(paramName,"AreaIndex_%d",i);				
				m_fsReadMtd[paramName] >>IndexArray[i];
			}
		}

		for(int i=0;i<AreaCount;i++)
		{
			edge_contours.push_back(std::vector<cv::Point>());
			for(int j=0;j<IndexArray[i];j++)
			{
				int tmp_x =0,tmp_y=0;
				memset(paramName,0,sizeof(paramName));
				sprintf(paramName,"Point_%d_%d_x",i,j);				
				m_fsReadMtd[paramName] >> tmp_x;

				memset(paramName,0,sizeof(paramName));
				sprintf(paramName,"Point_%d_%d_y",i,j);				
				m_fsReadMtd[paramName] >> tmp_y;
				edge_contours[i].push_back(cv::Point(tmp_x,tmp_y));
			}
		}
#if 1
		for(int m=0;m<edge_contours.size();m++)
		{
			for(int n=0;n<edge_contours[m].size();n++)
			{
				printf("\r\n[%s]:(%d-%d)<%d,%d>\r\n",__func__,m,n,edge_contours[m][n].x,edge_contours[m][n].y);
			}
		}
#endif
	}
}

void CVideoProcess::pnotify_callback(std::vector<FEATUREPOINT_T>& recommendPoints)
{
	printf("%s,%d, pnotify_callback start!\n",__FILE__,__LINE__);
	pThis->app_recommendPoints = recommendPoints;
}
