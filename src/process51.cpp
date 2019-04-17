
#include <glut.h>
#include "process51.hpp"
#include "vmath.h"
#include "msgDriv.h"
#include "app_ctrl.h"

#include "osd_cv.h"
#include "app_status.h"
#include "configable.h"
#include "ipc_custom_head.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace cv;


int capIndex =0;
int gun_resolu[2] = {1920, 1080};
extern bool show_TrigonometricMat;
extern bool show_circle_pointer;
extern uint8 exposure_star;
extern MenuDisplay g_displayMode;
extern bool showDetectCorners;
bool saveOnePicture = false;
extern bool start_calibrate;
extern int captureCount;
OSDSTATUS gConfig_Osd_param = {0};
UTCTRKSTATUS gConfig_Alg_param = {0};
extern int ScalerLarge,ScalerMid,ScalerSmall;
extern LinkagePos_t linkagePos; 
CProcess * CProcess::sThis = NULL;
CProcess* plat = NULL;
int glosttime = 3000;
extern vector<Mat> imageListForCalibra;
OSA_SemHndl g_linkage_getPos;
extern GB_MENU run_Mode;
SENDST trkmsg={0};
extern CamParameters g_camParams;
Point dest_ballPoint = Point(-100,-100);
extern SingletonSysParam* g_sysParam;
extern GB_WorkMode g_AppWorkMode;
extern MTD_Config g_mtdConfig;
extern menu_param_t *msgextMenuCtrl;
extern SelectMode mouse_workmode;
extern bool setComBaud_select ;
extern bool changeComBaud ;
extern CProcess *proc;
extern std::vector< cv::Mat > ImageList;
extern bool recvMtdConfigData;
unsigned char  g_GridMapMode = 2;
bool send_signal_flag = true;

void inputtmp(unsigned char cmdid)
{
	plat->OnKeyDwn(cmdid);
}

void getMmtTg(unsigned char index,int *x,int *y)
{
	*x = (int)plat->m_mtd[0]->tg[index].cur_x%vdisWH[plat->extInCtrl->SensorStat][0];
	*y = (int)plat->m_mtd[0]->tg[index].cur_y%vdisWH[plat->extInCtrl->SensorStat][1];
}

#if __MOVE_DETECT__
void getMtdxy(int *x,int *y,int *w,int *h)
{
	*x = *y = *w = *h = -1;
	if(plat->validMtdRecord[plat->chooseDetect])
	{
		for(int i = 0 ;i<plat->mvListsum.size();i++)
		{
			if(plat->chooseDetect == plat->mvListsum[i].number)
			{
				*x = plat->mvListsum[i].trkobj.targetRect.x + plat->mvListsum[i].trkobj.targetRect.width/2;
				*y = plat->mvListsum[i].trkobj.targetRect.y + plat->mvListsum[i].trkobj.targetRect.height/2;
				*w = plat->mvListsum[i].trkobj.targetRect.width;
				*h  = plat->mvListsum[i].trkobj.targetRect.height;
			}
		}	
	}
}
#endif

CProcess::CProcess():m_bMarkCircle(false),panPos(1024),tiltPos(13657),zoomPos(16),m_cofx(6320),
m_cofy(6200),m_bak_count(0),m_capX(960),m_capY(540),m_bGridMapCalibrate(false),m_AppVersion(VERSION_SOFT)
{
	extInCtrl = (CMD_EXT*)ipc_getimgstatus_p();
	memset(extInCtrl,0,sizeof(CMD_EXT));
	msgextInCtrl = extInCtrl;	
	sThis = this;
	plat = this;
	m_iDelta_X = 1920; // imgae width :1920
	m_iZoom = 2849; // Max view zoom:2849
}

CProcess::CProcess(int window_width, int window_height):m_bRefreshPTZValue(false),m_capX(960),m_capY(540),m_bMarkCircle(false),panPos(1024),tiltPos(13657),zoomPos(16),m_cofx(6320),
m_cofy(6200),m_bak_count(0),m_winWidth(window_width),m_winHeight(window_height),CVideoProcess(window_width,window_height),
m_bGridMapCalibrate(false),m_lastRow(-1),m_lastCol(-1),m_successCalibraNum(0),m_AppVersion(VERSION_SOFT)
{
	extInCtrl = (CMD_EXT*)ipc_getimgstatus_p();
	memset(extInCtrl,0,sizeof(IMGSTATUS));
	memset(renderflag,0,sizeof(renderflag));
	msgextInCtrl = extInCtrl;	
	
	sThis = this;
	plat = this;
	m_iDelta_X = window_width; 
	m_iZoom = 2849; // Max view zoom	
}
CProcess::~CProcess()
{
	sThis=NULL;
}


void CProcess::loadIPCParam()
{
		backMenuposX=0; 
		backMenuposY =0;
		memset(rcTrackBak, 0, sizeof(rcTrackBak));
		memset(tgBak, 0, sizeof(tgBak));
		memset(&extOutAck, 0, sizeof(ACK_EXT));
		memset(&extMenuCtrl, 0, sizeof(menu_param_t));

		m_bakClickPoint = Point(-20,-20);
		m_curClickPoint = Point(-30.-30);
		m_ballDestPoint = Point(-20,-30);
		m_bakballDestPoint = Point(-20,-30);
		prisensorstatus=0;//tv
		m_castTm=0;
		m_bCast=false;
		rememflag=false;
		rememtime=0;
		m_rectSelectPic = Rect(0,540,192,54);
		// default cmd value

		CMD_EXT *pIStuts = extInCtrl;

		if(recvMtdConfigData == true)
		{
			extMenuCtrl.osd_mudnum = g_mtdConfig.targetNum;
			extMenuCtrl.osd_trktime = g_mtdConfig.trackTime;
			extMenuCtrl.osd_maxsize = g_mtdConfig.maxArea;
			extMenuCtrl.osd_minsize = g_mtdConfig.minArea;
			extMenuCtrl.osd_sensi = g_mtdConfig.sensitivity;	
			detectNum = g_mtdConfig.targetNum;
			maxsize = g_mtdConfig.maxArea;
			minsize = g_mtdConfig.minArea;
			sensi = g_mtdConfig.sensitivity;

		}
		else
		{
			extMenuCtrl.osd_mudnum = 1;
			extMenuCtrl.osd_trktime = 1;
			extMenuCtrl.osd_maxsize = 10000;
			extMenuCtrl.osd_minsize = 9;
			extMenuCtrl.osd_sensi = 30;		
		}
		extMenuCtrl.resol_type_tmp = extMenuCtrl.resol_type = oresoltype;
		extMenuCtrl.MenuStat = -1;
		extMenuCtrl.Trig_Inter_Mode = 0;
		memset(extMenuCtrl.Passwd, 0, sizeof(extMenuCtrl.Passwd));
		memset(extMenuCtrl.disPasswd, 0, sizeof(extMenuCtrl.disPasswd));

		int cnt[menumaxid] = {4,5,7,4,7,6,3,5,5,3}; //  "menubuf[menumaxid][7][128]" each element counts
		memset(extMenuCtrl.menuarray, 0, sizeof(extMenuCtrl.menuarray));
		for(int i = 0; i < menumaxid; i++)
		{
			extMenuCtrl.menuarray[i].id = i;
			extMenuCtrl.menuarray[i].pointer = 0;
			extMenuCtrl.menuarray[i].submenu_cnt = cnt[i];
		}
		
			
		for(int i = 0; i < MAX_CHAN; i++)
		{
			pIStuts->opticAxisPosX[i] = vdisWH[i][0]/2;
			pIStuts->opticAxisPosY[i] = vdisWH[i][1]/2;
		}
		
		pIStuts->unitAimW 		= 	AIM_WIDTH;
		pIStuts->unitAimH 		= 	AIM_HEIGHT;
		pIStuts->unitAimX		=	vdisWH[video_pal][0]/2;
		pIStuts->unitAimY		=	vdisWH[video_pal][1]/2;

		pIStuts->SensorStat 	=   MAIN_CHID;
		pIStuts->SensorStatpri  =   pIStuts->SensorStat;
		pIStuts->PicpSensorStatpri	=	pIStuts->PicpSensorStat = 0xFF;
		
		pIStuts->changeSensorFlag = 0;
		crossBak.x = pIStuts->opticAxisPosX[pIStuts->SensorStat ];
		crossBak.y = pIStuts->opticAxisPosY[pIStuts->SensorStat ];
		pIStuts->AvtTrkAimSize= AVT_TRK_AIM_SIZE;

		for(int i = 0; i < MAX_CHAN; i++)
		{
			pIStuts->AvtPosX[i] = pIStuts->AxisPosX[i] = pIStuts->opticAxisPosX[i];
			pIStuts->AvtPosY[i] = pIStuts->AxisPosY[i] = pIStuts->opticAxisPosY[i];
		}
		
		pIStuts->PicpPosStat = 0;
		pIStuts->validChId = MAIN_CHID;
		pIStuts->FovStat=1;

		pIStuts->FrCollimation=2;
		pIStuts->PicpSensorStatpri=2;
		pIStuts->axisMoveStepX = 0;
		pIStuts->axisMoveStepY = 0;

		memset(secBak,0,sizeof(secBak));
		memset(Osdflag,0,sizeof(Osdflag));
		memset(osd_flag, 0, sizeof(osd_flag));
		
		Mmtsendtime=0;

		rendpos[0].x=vdisWH[0][0]*2/3;
		rendpos[0].y=vdisWH[0][1]*2/3;
		rendpos[0].w=vdisWH[0][0]/3;
		rendpos[0].h=vdisWH[0][1]/3;

		rendpos[1].x=vdisWH[0][0]*2/3;
		rendpos[1].y=0;
		rendpos[1].w=vdisWH[0][0]/3;
		rendpos[1].h=vdisWH[0][1]/3;

		rendpos[2].x=0;
		rendpos[2].y=0;
		rendpos[2].w=vdisWH[0][0]/3;
		rendpos[2].h=vdisWH[0][1]/3;

		rendpos[3].x=0;
		rendpos[3].y=vdisWH[0][1]*2/3;
		rendpos[3].w=vdisWH[0][0]/3;
		rendpos[3].h=vdisWH[0][1]/3;


		msgextMenuCtrl = &extMenuCtrl;

		save_flag = 0;
		cnt_down = 10;

		update_param_osd();

		pIStuts->DispGrp[0] = 1;
		pIStuts->DispGrp[1] = 1;
		pIStuts->DispColor[0]=2;
		pIStuts->DispColor[1]=2;

		memset(m_rectnbak, 0, sizeof(m_rectnbak));
		memset(mRectbak, 0, sizeof(mRectbak));
		memset(timearr, 0, sizeof(timearr));
		memset(timearrbak, 0, sizeof(timearrbak));
		timexbak = timeybak = 0;
		
#if __MOVE_DETECT__
		chooseDetect = 10;
#endif
		forwardflag = backflag = false;
		memset(validMtdRecord,0,sizeof(validMtdRecord));
		
		key_point1_cnt =0;
		key_point2_cnt =0;
		AllPoints_Num =0;
		m_bMarkCircle = false ;
		string_cnt1 =0;
		string_cnt2 =0;
		key1_pos = Point(-30,-30);
		key2_pos = Point(-80,-80);
		key1_backup = key1_pos;
		key2_backup = key2_pos;
		
		if(!readParams("SysParm.yml")) {
			printf("read param error\n");
		}
		else
		{	
		#if 0
			 vcapWH[0][0] = m_sysparm.gun_camera.raw;
			 vcapWH[0][1] = m_sysparm.gun_camera.col;
			 vcapWH[1][0] = m_sysparm.ball_camera.raw;
			 vcapWH[1][1] = m_sysparm.ball_camera.col;

			 vdisWH[0][0] = m_sysparm.gun_camera.raw;
			 vdisWH[0][1] = m_sysparm.gun_camera.col;
			 vdisWH[1][0] = m_sysparm.ball_camera.raw;
			 vdisWH[1][1] = m_sysparm.ball_camera.col;
		#endif
		}

		if(m_camCalibra != NULL) {
			m_camCalibra->key_points1.clear();
			m_camCalibra->key_points2.clear();
		}

		OSA_semCreate(&g_linkage_getPos, 1, 0);
}


int  CProcess::WindowstoPiexlx(int x,int channel)
{
	int ret=0;
	ret= cvRound(x*1.0/vdisWH[0][0]*vcapWH[channel][0]);
	 if(ret<0)
	 	{
			ret=0;
	 	}
	 else if(ret>=vcapWH[channel][0])
	 	{
			ret=vcapWH[channel][0];
	 	}


	  return ret;
}


int  CProcess::WindowstoPiexly(int y,int channel)
{
	 int ret=0;
	 ret= cvRound(y*1.0/vdisWH[0][1]*vcapWH[channel][1]);

	  if(ret<0)
	 	{
			ret=0;
	 	}
	 else if(ret>=vcapWH[channel][1])
	 	{
			ret=vcapWH[channel][1];
	 	}
	return  ret;
}



float  CProcess::PiexltoWindowsx(int x,int channel)
{
	 float ret=0;
	 ret= cvRound(x*1.0/vcapWH[channel][0]*vdisWH[channel][0]);
	 if(ret<0)
 	 {
		ret=0;
 	 }
	 else if(ret>=vdisWH[channel][0])
 	 {
		ret=vdisWH[channel][0];
 	 }
	 if(extInCtrl->ImgMmtshow[extInCtrl->SensorStat])
 	 {
		ret =ret*2/3;
 	 }

	 return ret;
}

float  CProcess::PiexltoWindowsy(int y,int channel)
{
	 float ret=0;
	 ret= cvRound(y*1.0/vcapWH[channel][1]*vdisWH[channel][1]);

	 if(ret<0)
 	 {
		ret=0;
 	 }
	 else if(ret>=vdisWH[channel][1])
 	 {
		ret=vdisWH[channel][1];
 	 }

	  if(extInCtrl->ImgMmtshow[extInCtrl->SensorStat])
 	  {
		ret =ret*2/3;
 	  }
	
	return  ret;
}

float  CProcess::PiexltoWindowsxf(float x,int channel)
{
	float ret=0;
	 ret= (x*1.0/vcapWH[channel][0]*vdisWH[channel][0]);
	 if(ret<0)
 	{
		ret=0;
 	}
	 else if(ret>=vdisWH[channel][0])
 	{
		ret=vdisWH[channel][0];
 	}

	  return ret;
}


int  CProcess::PiexltoWindowsxzoom(int x,int channel)
{
	int ret=0;
	 ret= cvRound(x*1.0/vcapWH[channel][0]*vdisWH[channel][0]);
	 if(ret<0)
 	{
		ret=0;
 	}
	 else if(ret>=vdisWH[channel][0])
 	{
		ret=vdisWH[channel][0];
 	}

	if(extInCtrl->ImgMmtshow[extInCtrl->SensorStat])
 	{
		ret =ret*2/3;
 	}

	if(extInCtrl->FovCtrl==5&&extInCtrl->SensorStat==0)
 	{
 		ret=ret-320;
		ret=2*ret;
 	}
	return ret;
}

int  CProcess::PiexltoWindowsyzoom(int y,int channel)
{
	 int ret=0;
	 ret= cvRound(y*1.0/vcapWH[channel][1]*vdisWH[channel][1]);

	  if(ret<0)
 	{
		ret=0;
 	}
	 else if(ret>=vdisWH[channel][1])
 	{
		ret=vdisWH[channel][1];
 	}

	  if(extInCtrl->ImgMmtshow[extInCtrl->SensorStat])
 	{
		ret =ret*2/3;
 	}

	 if(extInCtrl->FovCtrl==5&&extInCtrl->SensorStat==0)
 	{
 		ret=ret-256;
		ret=2*ret;
 	}
	return  ret;
}

int  CProcess::PiexltoWindowsxzoom_TrkRect(int x,int channel)
{
	int ret=0;

	ret= cvRound(x*1.0/vcapWH[channel][0]*vdisWH[channel][0]);
	
	if(ret<0)
	{
		ret=0;
	}
	else if(ret>=vdisWH[channel][0])
	{
		ret=vdisWH[channel][0];
	}

	//result to even
	if((ret%2)==0)
		ret = ret;
	else
		ret = ret+1;
	
	return ret;
}

int  CProcess::PiexltoWindowsyzoom_TrkRect(int y,int channel)
{
	 int ret=0;

	 ret= cvRound(y*1.0/vcapWH[channel][1]*vdisWH[channel][1]);

	if(ret<0)
 	{
		ret=0;
 	}
	 else if(ret>=vdisWH[channel][1])
 	{
		ret=vdisWH[channel][1];
 	}

	if((ret%2)==0)
		ret = ret;
	else
		ret = ret+1;

	return  ret;
}
void CProcess::setDisplayResolution(CDisplayer &displayObject, int w, int h)
{
	displayObject.setDisplayResolution(w, h);
}
void CProcess::OnCreate()
{
	MSGAPI_initial();


		//OnKeyDwn('b');	
		bool ret = false;
		if(false==(ret=m_camCalibra->loadLinkageParams("GenerateCameraParam.yml", g_camParams.imageSize, g_camParams.cameraMatrix_gun, 
					g_camParams.distCoeffs_gun,
		          g_camParams.cameraMatrix_ball, g_camParams.distCoeffs_ball, g_camParams.homography,
		          g_camParams.panPos, g_camParams.tiltPos, g_camParams.zoomPos))){
			cout << "  <<< Load linkage params failed !!!>>> " << endl;  
		}
#if 0
		cout << "****************************************<< CamParam >>********************************************************" << endl;
		cout << "imageSize:\n" << g_camParams.imageSize << endl;
		cout << "cameraMatrix_gun:\n" << g_camParams.cameraMatrix_gun << endl;
		cout << "distCoeffs_gun:\n" << g_camParams.distCoeffs_gun << endl;
		cout << "cameraMatrix_ball:\n" << g_camParams.cameraMatrix_ball << endl;
		cout << "distCoeffs_ball:\n" << g_camParams.distCoeffs_ball << endl;
		cout << "homography:\n" << g_camParams.homography << endl;
		cout << "panPos:\n" << g_camParams.panPos<< endl;
		cout << "tiltPos:\n" << g_camParams.tiltPos<< endl;
		cout << "zoomPos:\n" << g_camParams.zoomPos<< endl;
		cout << "************************************************************************************************" << endl;
 #endif
		if(ret == true) {
			this->Init_CameraMatrix();
		}

		TimerCreate();
}
void CProcess::TimerCreate()
{
	resol_light_id = dtimer.createTimer();
	resol_apply_id = dtimer.createTimer();
	mtdnum_light_id = dtimer.createTimer();
	trktime_light_id = dtimer.createTimer();
	maxsize_light_id = dtimer.createTimer();
	minsize_light_id = dtimer.createTimer();
	sensi_light_id = dtimer.createTimer();
	baud_light_id = dtimer.createTimer();
	mouse_show_id = dtimer.createTimer();

	dtimer.registerTimer(resol_light_id, Tcallback, &resol_light_id);
	dtimer.registerTimer(resol_apply_id, Tcallback, &resol_apply_id);
    	dtimer.registerTimer(mtdnum_light_id, Tcallback, &mtdnum_light_id);
    	dtimer.registerTimer(trktime_light_id, Tcallback, &trktime_light_id);
    	dtimer.registerTimer(maxsize_light_id, Tcallback, &maxsize_light_id);
    	dtimer.registerTimer(minsize_light_id, Tcallback, &minsize_light_id);
    	dtimer.registerTimer(sensi_light_id, Tcallback, &sensi_light_id);
	dtimer.registerTimer(baud_light_id, Tcallback, &baud_light_id);
	dtimer.registerTimer(mouse_show_id, Tcallback, &mouse_show_id);
}


void CProcess::Tcallback(void *p)
{
	static int resol_dianmie = 0;
    	static int mtdnum_dianmie = 0;
    	static int trktime_dianmie = 0;
    	static int maxsize_dianmie = 0;
    	static int minsize_dianmie = 0;
    	static int sensi_dianmie = 0;
	static int baud_dianmie = 0;
	
	unsigned char baudlbuf[MAX_BAUDID][128] = {
		"波特率     2400","波特率     4800","波特率     9600","波特率     115200"};
	unsigned char resolbuf[maxresolid][128] = {
			"格式 1920x1080@60Hz","格式 1024x768@60Hz","格式 1280x1024@60Hz"};
	unsigned char resolapplybuf1[128] = "是否保存当前分辨率?";
	unsigned char resolapplybuf2[128] = "0:取消  1:保存";
	int a = *(int *)p;

	if(a == sThis->resol_light_id)
	{
		if(resol_dianmie)
			swprintf(sThis->m_display.disMenu[submenu_setimg][1], 33, L"%s", resolbuf[sThis->m_display.disresol_type_tmp]);
		else
			swprintf(sThis->m_display.disMenu[submenu_setimg][1], 33, L"格式");
		resol_dianmie = !resol_dianmie;
	}
	else if(a == sThis->resol_apply_id)
	{
		swprintf(sThis->m_display.disMenu[submenu_setimg][4], 33, L"%s %d", resolapplybuf1, sThis->cnt_down--);
		swprintf(sThis->m_display.disMenu[submenu_setimg][5], 33, L"%s", resolapplybuf2);
		if(sThis->cnt_down < 0)
		{
			sThis->dtimer.stopTimer(sThis->resol_apply_id);
			if(sThis->save_flag)
			{
				sThis->save_flag = 0;
				sThis->setresol(sThis->m_display.disresol_type);
				swprintf(sThis->m_display.disMenu[submenu_setimg][1], 33, L"%s", resolbuf[sThis->m_display.disresol_type]);
				memset(sThis->m_display.disMenu[submenu_setimg][4], 0, sizeof(sThis->m_display.disMenu[submenu_setimg][4]));
				memset(sThis->m_display.disMenu[submenu_setimg][5], 0, sizeof(sThis->m_display.disMenu[submenu_setimg][5]));
			}
		}
	}
	else if(a == sThis->mtdnum_light_id)
	{
		if(mtdnum_dianmie)
		{
			if((sThis->extMenuCtrl.osd_mudnum < MIN_MTDTARGET_NUM) || (sThis->extMenuCtrl.osd_mudnum > MAX_MTDTARGET_NUM))
				swprintf(sThis->m_display.disMenu[submenu_mtd][1], 33, L"目标个数     %d(超出范围%d~%d)", sThis->extMenuCtrl.osd_mudnum,MIN_MTDTARGET_NUM,MAX_MTDTARGET_NUM);
			else
				swprintf(sThis->m_display.disMenu[submenu_mtd][1], 33, L"目标个数     %d", sThis->extMenuCtrl.osd_mudnum);
		}
		else
			swprintf(sThis->m_display.disMenu[submenu_mtd][1], 33, L"目标个数");
		mtdnum_dianmie = !mtdnum_dianmie;
	}
	else if(a == sThis->trktime_light_id)
	{
		if(trktime_dianmie)
		{
			if((sThis->extMenuCtrl.osd_trktime < MIN_MTDTRKTIME) || (sThis->extMenuCtrl.osd_trktime > MAX_MTDTRKTIME))
				swprintf(sThis->m_display.disMenu[submenu_mtd][2], 33, L"跟踪持续时间 %d秒(超出范围%d~%d秒)", sThis->extMenuCtrl.osd_trktime,MIN_MTDTRKTIME,MAX_MTDTRKTIME);
			else
				swprintf(sThis->m_display.disMenu[submenu_mtd][2], 33, L"跟踪持续时间 %d秒", sThis->extMenuCtrl.osd_trktime);
		}
		else
			swprintf(sThis->m_display.disMenu[submenu_mtd][2], 33, L"跟踪持续时间  秒");
		trktime_dianmie = !trktime_dianmie;
	}
	else if(a == sThis->maxsize_light_id)
	{
		if(maxsize_dianmie)
		{
			if((sThis->extMenuCtrl.osd_maxsize < sThis->minsize) || (sThis->extMenuCtrl.osd_maxsize > MAX_MTDMAXSIZE))
				swprintf(sThis->m_display.disMenu[submenu_mtd][3], 33, L"最大目标面积 %d(超出范围)", sThis->extMenuCtrl.osd_maxsize);
			else
				swprintf(sThis->m_display.disMenu[submenu_mtd][3], 33, L"最大目标面积 %d", sThis->extMenuCtrl.osd_maxsize);
		}
		else
			swprintf(sThis->m_display.disMenu[submenu_mtd][3], 33, L"最大目标面积      ");
		maxsize_dianmie = !maxsize_dianmie;
	}
	else if(a == sThis->minsize_light_id)
	{
		if(minsize_dianmie)
		{
			if((sThis->extMenuCtrl.osd_minsize < MIN_MTDMINSIZE) || (sThis->extMenuCtrl.osd_minsize > sThis->maxsize))
				swprintf(sThis->m_display.disMenu[submenu_mtd][4], 33, L"最小目标面积 %d(超出范围)", sThis->extMenuCtrl.osd_minsize);
			else
				swprintf(sThis->m_display.disMenu[submenu_mtd][4], 33, L"最小目标面积 %d", sThis->extMenuCtrl.osd_minsize);
		}
		else
			swprintf(sThis->m_display.disMenu[submenu_mtd][4], 33, L"最小目标面积  ");
		minsize_dianmie = !minsize_dianmie;
	}
	else if(a == sThis->sensi_light_id)
	{
		if(sensi_dianmie)
		{
			if((sThis->extMenuCtrl.osd_sensi < MIN_MTDSENSI) || (sThis->extMenuCtrl.osd_sensi > MAX_MTDSENSI))
				swprintf(sThis->m_display.disMenu[submenu_mtd][5], 33, L"灵敏度       %d(超出范围%d~%d)", sThis->extMenuCtrl.osd_sensi,MIN_MTDSENSI,MAX_MTDSENSI);
			else
				swprintf(sThis->m_display.disMenu[submenu_mtd][5], 33, L"灵敏度       %d", sThis->extMenuCtrl.osd_sensi);
		}
		else
			swprintf(sThis->m_display.disMenu[submenu_mtd][5], 33, L"灵敏度");
		sensi_dianmie = !sensi_dianmie;
	}
	else if( a == sThis->baud_light_id ){
		if(baud_dianmie)
			swprintf(sThis->m_display.disMenu[submenu_setcom][0], 33, L"%s", baudlbuf[sThis->m_display.disbaud_type]);
		else
			memset(sThis->m_display.disMenu[submenu_setcom][0], 0, sizeof(sThis->m_display.disMenu[submenu_setcom][0]));
		baud_dianmie = !baud_dianmie;

	}
	else if(a == sThis->mouse_show_id)
	{
		if(sThis->mouse_show)
			sThis->set_mouse_show(0);
	}		
}

void CProcess::Init_CameraMatrix()
{	
    initUndistortRectifyMap(g_camParams.cameraMatrix_gun, g_camParams.distCoeffs_gun, Mat(),
                            g_camParams.cameraMatrix_ball,
                            g_camParams.imageSize, CV_16SC2, g_camParams.map1, g_camParams.map2);
    //Mat imageGun = imread("images/2018-08-16-153720.jpg");
   // Mat imageBall = imread("ballLinkageCalib.bmp");
 
	if(!gun_srcMat_remap.empty()) {
    		remap(gun_srcMat_remap, undisImageGun, g_camParams.map1, g_camParams.map2, INTER_LINEAR);
	}

}

bool CProcess::writeParams(const char* file)
{
	
	return true;
}

bool CProcess::readParams(const char* filename)
{
	int workmode = 0;
	FileStorage fs2( filename, FileStorage::READ );
	bool ret = fs2.isOpened();
	if(!ret)
	{
		cout << filename << " can't opened !\n" << endl;
		return false;
	}
	else
	{
		fs2["gun_camera_raw"] >> m_sysparm.gun_camera.raw;
		fs2["gun_camera_col"] >> m_sysparm.gun_camera.col;
		fs2["ball_camera_raw"] >> m_sysparm.ball_camera.raw;
		fs2["ball_camera_col"] >> m_sysparm.ball_camera.col;

		return ret;
	}
	fs2.release();
    return true;
}

bool CProcess::loadConfigParams(const char* filename)
{
	int workmode = 0;
	int mtdstate =0;
	FileStorage fs2( filename, FileStorage::READ );
	bool ret = fs2.isOpened();
	if(!ret)
	{
		cout << filename << " can't opened !\n" << endl;
		return false;
	}
	else
	{
		fs2["workmode"] >> workmode;
		fs2["mtdstate"] >> mtdstate;
		g_AppWorkMode = (GB_WorkMode)workmode;
		
		if(mtdstate == 1)
		{
			setWorkMode(AUTO_LINK_MODE);
		}
		
		setMtdState(bool(mtdstate));		
		//printf("\r\n[%s]:========-----======----======== Load Work Mode : <%d>\r\n",__func__, workmode);
		return ret;
	}
	fs2.release();
    return true;
}

bool CProcess::saveConfigParams(const char* filename)
{
	int workmode = 0;
	int mtdstate = 0;
	FileStorage fs2( filename, FileStorage::WRITE );
	bool ret = fs2.isOpened();
	if(!ret)
	{
		cout << filename << "Can't opened !\n" << endl;
		return false;
	}
	else
	{
		workmode = (int)g_AppWorkMode;
		mtdstate = getMtdState();
		fs2<<"workmode"<<workmode;
		fs2<<"mtdstate" << mtdstate;
		//printf("\r\n[%s]:========-----======----======== Save Work Mode : <%d>\r\n",__func__, workmode);
		//printf("\r\n[%s]:========-----======----======== Save Mtd State : <%d>\r\n",__func__, mtdstate);
		return ret;
	}
	fs2.release();
    return true;
}

	
void CProcess::OnDestroy(){};
void CProcess::OnInit()
{
	extInCtrl->SysMode = 1;
}
void CProcess::OnConfig(){};
void CProcess::OnRun()
{
	update_param_alg();
};
void CProcess::OnStop(){};
void CProcess::Ontimer(){

	//msgdriv_event(MSGID_EXT_INPUT_VIDEOEN,NULL);
};
bool CProcess::OnPreProcess(int chId, Mat &frame)
{
	if(m_bCast){
		Uint32 curTm = OSA_getCurTimeInMsec();
		Uint32 elapse = curTm - m_castTm;

		if(elapse < 2000){
			return false;
		}
		else
		{
			m_bCast=false;
		}
	}
	return true;
}


int onece=0;

void CProcess::osd_mtd_show(TARGET tg[], bool bShow)
{
	int i;
	
	int frcolor=extInCtrl->DispColor[extInCtrl->SensorStat];
	unsigned char Alpha = (bShow) ? frcolor : 0;
	CvScalar colour=GetcvColour(Alpha);

	for(i=0;i<MAX_TARGET_NUMBER;i++)
	{
		if(tg[i].valid)
		{
			cv::Rect result;
			result.width = 32;
			result.height = 32;
			result.x = ((int)tg[i].cur_x) % vdisWH[extInCtrl->SensorStat][0];
			result.y = ((int)tg[i].cur_y ) % vdisWH[extInCtrl->SensorStat][1];
			result.x = result.x - result.width/2;
			result.y = result.y - result.height/2;
			rectangle(m_display.m_imgOsd[extInCtrl->SensorStat],
				Point( result.x, result.y ),
				Point( result.x+result.width, result.y+result.height),
				colour, 1, 8);
		}
	}
}
void CProcess::GB_DrawCross(Mat &textureImg,cv::Point center, bool needShow)
{
	int space = 10;
	int cross_width = 60;
	int linewidth = 1;
	int x = center.x;
	int y = center.y;
	CvScalar color = cvScalar(0,0,0,0);
	if(needShow == true){
		color =  cvScalar(0,255,0,255);
	}
	
	line(textureImg, cv::Point(x+space, y), cv::Point(x+cross_width, y),color, linewidth, 8, 0 );
	line(textureImg, cv::Point(x+space, y+1), cv::Point(x+cross_width, y+1),color, linewidth, 8, 0 );
	
	line(textureImg, cv::Point(x-space, y), cv::Point(x-cross_width, y),color, linewidth, 8, 0 );
	line(textureImg, cv::Point(x-space, y+1), cv::Point(x-cross_width, y+1),color, linewidth, 8, 0 );

	line(textureImg, cv::Point(x, y+space), cv::Point(x, y+cross_width),color, linewidth, 8, 0 );
	line(textureImg, cv::Point(x+1, y+space), cv::Point(x+1, y+cross_width),color, linewidth, 8, 0 );
	
	line(textureImg, cv::Point(x, y-space), cv::Point(x, y-cross_width),color, linewidth, 8, 0 );
	line(textureImg, cv::Point(x+1, y+space), cv::Point(x+1, y+cross_width),color, linewidth, 8, 0 );

	line(textureImg, cv::Point(x-1, y), cv::Point(x+1, y),color, linewidth, 8, 0 );
	line(textureImg, cv::Point(x, y-1), cv::Point(x, y+1),color, linewidth, 8, 0 );

}

void CProcess::DrawCross(cv::Rect rec,int fcolour ,int sensor,bool bShow /*= true*/)
{
	unsigned char colour = (bShow) ?fcolour : 0;
	Line_Param_fb lineparm;
	lineparm.x		=	rec.x;
	lineparm.y		=	rec.y;
	lineparm.width	=	rec.width;
	lineparm.height	=	rec.height;
	lineparm.frcolor	=	colour;
	if(sensor>=MAX_CHAN)
		sensor = 1;
	Drawcvcrossaim(m_display.m_imgOsd[sensor],&lineparm);
}

void CProcess::DrawAcqRect(cv::Mat frame,cv::Rect rec,int frcolor,bool bshow)
{
	int color = (bshow)?frcolor:0;
	int leftBottomx 	= rec.x;
	int leftBottomy 	= rec.y;
	int leftTopx 		= leftBottomx ;
	int leftTopy 		= leftBottomy - rec.height;
	int rightTopx 	= leftBottomx + rec.width;
	int rightTopy 		= leftTopy;
	int rightBottomx 	= rightTopx;
	int rightBottomy 	= leftBottomy;

	int cornorx = rec.width/4;
	int cornory = rec.height/4;
	
	Osd_cvPoint start;
	Osd_cvPoint end;

	//leftBottom
	start.x 	= leftBottomx;
	start.y 	= leftBottomy;
	end.x	= leftBottomx + cornorx;
	end.y 	= leftBottomy;
	DrawcvLine(frame,&start,&end,color,1);
	start.x 	= leftBottomx;
	start.y 	= leftBottomy;
	end.x	= leftBottomx;
	end.y 	= leftBottomy - cornory;
	DrawcvLine(frame,&start,&end,color,1);	
	//leftTop
	start.x 	= leftTopx;
	start.y 	= leftTopy;
	end.x	= leftTopx + cornorx;
	end.y 	= leftTopy;
	DrawcvLine(frame,&start,&end,color,1);
	start.x 	= leftTopx;
	start.y 	= leftTopy;
	end.x	= leftTopx;
	end.y 	= leftTopy + cornory;
	DrawcvLine(frame,&start,&end,color,1);	
	//rightTop
	start.x 	= rightTopx;
	start.y 	= rightTopy;
	end.x	= rightTopx - cornorx;
	end.y 	= rightTopy;
	DrawcvLine(frame,&start,&end,color,1);
	start.x 	= rightTopx;
	start.y 	= rightTopy;
	end.x	= rightTopx;
	end.y 	= rightTopy + cornory;
	DrawcvLine(frame,&start,&end,color,1);
	//rightBottom
	start.x 	= rightBottomx;
	start.y 	= rightBottomy;
	end.x	= rightBottomx - cornorx;
	end.y 	= rightBottomy;
	DrawcvLine(frame,&start,&end,color,1);
	start.x 	= rightBottomx;
	start.y 	= rightBottomy;
	end.x	= rightBottomx;
	end.y 	= rightBottomy - cornory;
	DrawcvLine(frame,&start,&end,color,1);	

	return ;
}

void CProcess::DrawRect(Mat frame,cv::Rect rec,int frcolor)
{
	int x = rec.x,y = rec.y;
	int width = rec.width;
	int height = rec.height;
	drawcvrect(frame,x,y,width,height,frcolor);
	return ;
}



int majormmtid=0;
int primajormmtid=0;

void CProcess::erassdrawmmt(TARGET tg[],bool bShow)
{
			int startx=0;
			int starty=0;
			int endx=0;
			int endy=0;
			Mat frame=m_display.m_imgOsd[extInCtrl->SensorStat];
			int i=0,j=0;
			cv::Rect result;
			short tempmmtx=0;
			short tempmmty=0;
			int tempdata=0;
			int testid=0;
			extInCtrl->Mmttargetnum=0;
			char numbuf[3];
			int frcolor=extInCtrl->DispColor[extInCtrl->SensorStat];
			unsigned char Alpha = (bShow) ? frcolor : 0;
			CvScalar colour=GetcvColour(Alpha);

			tempdata=primajormmtid;
			for(i=0;i<MAX_TARGET_NUMBER;i++)
				{

						//if(m_mtd[chId]->tg[i].valid)
						
						if((tg[primajormmtid].valid)&&(i==0))
						{
							//majormmtid=i;
							result.width = 32;
							result.height = 32;
							tempmmtx=result.x = ((int)tg[primajormmtid].cur_x) % vdisWH[extInCtrl->SensorStat][0];
							tempmmty=result.y = ((int)tg[primajormmtid].cur_y ) % vdisWH[extInCtrl->SensorStat][1];


							extInCtrl->MmtPixelX=result.x;
							extInCtrl->MmtPixelY=result.y;
							extInCtrl->MmtValid=1;
							result.x = result.x - result.width/2;
							result.y = result.y - result.height/2;

							
							 startx=PiexltoWindowsx(result.x,prisensorstatus);
							 starty=PiexltoWindowsy(result.y,prisensorstatus);
							 endx=PiexltoWindowsx(result.x+result.width,prisensorstatus);
						 	 endy=PiexltoWindowsy(result.y+result.height,prisensorstatus);

							rectangle( frame,
								Point( startx, starty ),
								Point( endx, endy),
								colour, 1, 8);
							
						}
						
						else if(tg[tempdata].valid)
							{
								testid++;
								result.width = 32;
								result.height = 32;
								tempmmtx=result.x = ((int)tg[tempdata].cur_x) % vdisWH[extInCtrl->SensorStat][0];
								tempmmty=result.y = ((int)tg[tempdata].cur_y ) % vdisWH[extInCtrl->SensorStat][1];

								 startx=PiexltoWindowsx(result.x,prisensorstatus);
								 starty=PiexltoWindowsy(result.y,prisensorstatus);
								line(frame, cvPoint(startx-16,starty), cvPoint(startx+16,starty), colour, 1, 8, 0 ); 
								line(frame, cvPoint(startx,starty-16), cvPoint(startx,starty+16), colour, 1, 8, 0 ); 
								//OSA_printf("******************the num  majormmtid=%d\n",majormmtid);
								sprintf(numbuf,"%d",(tempdata+MAX_TARGET_NUMBER-primajormmtid)%MAX_TARGET_NUMBER);
								putText(frame,numbuf,cvPoint(startx+14,starty+14),CV_FONT_HERSHEY_SIMPLEX,1,colour);
								
								
							}
				
				
						tempdata=(tempdata+1)%MAX_TARGET_NUMBER;

					}


}


void CProcess::drawmmt(TARGET tg[],bool bShow)
{
	int startx=0;
	int starty=0;
	int endx=0;
	int endy=0;
	Mat frame=m_display.m_imgOsd[extInCtrl->SensorStat];
	int i=0,j=0;
	cv::Rect result;
	short tempmmtx=0;
	short tempmmty=0;
	int tempdata=0;
	int testid=0;
	extInCtrl->Mmttargetnum=0;
	char numbuf[3];
	int frcolor=extInCtrl->DispColor[extInCtrl->SensorStat];
	unsigned char Alpha = (bShow) ? frcolor : 0;
	CvScalar colour=GetcvColour(Alpha);
	
	for(i=0;i<20;i++)
	{
		extInCtrl->MmtOffsetXY[i]=0;
	}
	for(i=0;i<MAX_TARGET_NUMBER;i++)
	{

		if(tg[majormmtid].valid==0)
		{
			//majormmtid++;
			majormmtid=(majormmtid+1)%MAX_TARGET_NUMBER;
		}
		if(tg[i].valid==1)
		{
			extInCtrl->Mmttargetnum++;
		}
	}

	primajormmtid=tempdata=majormmtid;
	for(i=0;i<MAX_TARGET_NUMBER;i++)
	{
		if((tg[majormmtid].valid)&&(i==0))
		{
			//majormmtid=i;
			result.width = 32;
			result.height = 32;
			tempmmtx=result.x = ((int)tg[majormmtid].cur_x) % vdisWH[extInCtrl->SensorStat][0];
			tempmmty=result.y = ((int)tg[majormmtid].cur_y ) % vdisWH[extInCtrl->SensorStat][1];


			extInCtrl->MmtPixelX=result.x;
			extInCtrl->MmtPixelY=result.y;
			extInCtrl->MmtValid=1;
			
			//OSA_printf("the num  majormmtid=%d\n",majormmtid);
			result.x = result.x - result.width/2;
			result.y = result.y - result.height/2;

			 startx=PiexltoWindowsx(result.x,extInCtrl->SensorStat);
			 starty=PiexltoWindowsy(result.y,extInCtrl->SensorStat);
			 endx=PiexltoWindowsx(result.x+result.width,extInCtrl->SensorStat);
		 	 endy=PiexltoWindowsy(result.y+result.height,extInCtrl->SensorStat);


			if((((extInCtrl->AvtTrkStat == eTrk_mode_mtd)||(extInCtrl->AvtTrkStat == eTrk_mode_acq)))&&(extInCtrl->DispGrp[extInCtrl->SensorStat]<3))
			{
				rectangle( frame,
					Point( startx, starty ),
					Point( endx, endy),
					colour, 1, 8);
			}
			//OSA_printf("******************the num  majormmtid=%d x=%d y=%d w=%d h=%d\n",majormmtid,
			//	result.x,result.y,result.width,result.height);
			extInCtrl->MmtOffsetXY[j]		=	tempmmtx&0xff;
			extInCtrl->MmtOffsetXY[j+1]	=	(tempmmtx>>8)&0xff;
			extInCtrl->MmtOffsetXY[j+2]	=	tempmmty&0xff;
			extInCtrl->MmtOffsetXY[j+3]	=	(tempmmty>>8)&0xff;
		}	
		else if(tg[tempdata].valid)
		{
			testid++;
			result.width = 32;
			result.height = 32;
			tempmmtx=result.x = ((int)tg[tempdata].cur_x) % vdisWH[extInCtrl->SensorStat][0];
			tempmmty=result.y = ((int)tg[tempdata].cur_y ) % vdisWH[extInCtrl->SensorStat][1];

			 startx=PiexltoWindowsx(result.x,extInCtrl->SensorStat);
			 starty=PiexltoWindowsy(result.y,extInCtrl->SensorStat);
			if((((extInCtrl->AvtTrkStat == eTrk_mode_mtd)||(extInCtrl->AvtTrkStat == eTrk_mode_acq)))&&(extInCtrl->DispGrp[extInCtrl->SensorStat]<3))
			{
				line(frame, cvPoint(startx-16,starty), cvPoint(startx+16,starty), colour, 1, 8, 0 ); 
				line(frame, cvPoint(startx,starty-16), cvPoint(startx,starty+16), colour, 1, 8, 0 ); 
				//OSA_printf("******************the num  majormmtid=%d\n",majormmtid);
				sprintf(numbuf,"%d",(tempdata+MAX_TARGET_NUMBER-majormmtid)%MAX_TARGET_NUMBER);
				putText(frame,numbuf,cvPoint(startx+14,starty+14),CV_FONT_HERSHEY_SIMPLEX,1,colour);
			}
			extInCtrl->MmtOffsetXY[j+testid*4]=tempmmtx&0xff;
			extInCtrl->MmtOffsetXY[j+1+testid*4]=(tempmmtx>>8)&0xff;
			extInCtrl->MmtOffsetXY[j+2+testid*4]=tempmmty&0xff;
			extInCtrl->MmtOffsetXY[j+3+testid*4]=(tempmmty>>8)&0xff;	
		}
		tempdata=(tempdata+1)%MAX_TARGET_NUMBER;
	}

	if(Mmtsendtime==0)
		;
	Mmtsendtime++;
	if(Mmtsendtime==1)
	{
		Mmtsendtime=0;
	}
}


void CProcess::erassdrawmmtnew(TARGETDRAW tg[],bool bShow)
{
	int startx=0;
	int starty=0;
	int endx=0;
	int endy=0;
	Mat frame=m_display.m_imgOsd[extInCtrl->SensorStat];
	int i=0,j=0;
	cv::Rect result;
	short tempmmtx=0;
	short tempmmty=0;
	int tempdata=0;
	int testid=0;
	extInCtrl->Mmttargetnum=0;
	char numbuf[3];
	int frcolor=extInCtrl->DispColor[extInCtrl->SensorStat];
	unsigned char Alpha = (bShow) ? frcolor : 0;
	CvScalar colour=GetcvColour(Alpha);

	primajormmtid;
	for(i=0;i<MAX_TARGET_NUMBER;i++)
	{
		if((tg[primajormmtid].valid)&&(i==primajormmtid))
		{	
			 startx=tg[primajormmtid].startx;//PiexltoWindowsx(result.x,prisensorstatus);
			 starty=tg[primajormmtid].starty;//PiexltoWindowsy(result.y,prisensorstatus);
			 endx=tg[primajormmtid].endx;//PiexltoWindowsx(result.x+result.width,prisensorstatus);
		 	 endy=tg[primajormmtid].endy;//PiexltoWindowsy(result.y+result.height,prisensorstatus);

			rectangle( frame,
				Point( startx, starty ),
				Point( endx, endy),
				colour, 1, 8);
			rectangle( frame,
				Point( startx-1, starty-1 ),
				Point( endx+1, endy+1),
				colour, 1, 8);
			sprintf(numbuf,"%d",primajormmtid+1);
			putText(frame,numbuf,cvPoint(startx,starty-2),CV_FONT_HERSHEY_SIMPLEX,0.8,colour);	
		}

		if((tg[i].valid)&&(i!=primajormmtid))
		{
			 startx=tg[i].startx;//PiexltoWindowsx(result.x,prisensorstatus);
			 starty=tg[i].starty;//PiexltoWindowsy(result.y,prisensorstatus);
			 endx=tg[i].endx;
			 endy=tg[i].endy;

			rectangle( frame,
			Point( startx, starty ),
			Point( endx, endy),
			colour, 1, 8);

			//OSA_printf("******************the num  majormmtid=%d\n",majormmtid);
			sprintf(numbuf,"%d",i+1);
			putText(frame,numbuf,cvPoint(startx,starty-2),CV_FONT_HERSHEY_SIMPLEX,0.8,colour);
		}
	}
}


void CProcess::drawmmtnew(TARGET tg[],bool bShow)
{
	int startx=0;
	int starty=0;
	int endx=0;
	int endy=0;
	Mat frame=m_display.m_imgOsd[extInCtrl->SensorStat];
	int i=0,j=0;
	cv::Rect result;
	short tempmmtx=0;
	short tempmmty=0;
	int tempdata=0;
	int testid=0;
	extInCtrl->Mmttargetnum=0;
	char numbuf[3];
	int frcolor=extInCtrl->DispColor[extInCtrl->SensorStat];
	unsigned char Alpha = (bShow) ? frcolor : 0;
	CvScalar colour=GetcvColour(Alpha);
	
	//memset(extInCtrl->MmtOffsetXY,0,20);
	for(i=0;i<20;i++)
	{
		extInCtrl->MmtOffsetXY[i]=0;
	}
	for(i=0;i<MAX_TARGET_NUMBER;i++)
	{

		if(tg[majormmtid].valid==0)
		{
			//majormmtid++;
			//find mmt major target;
			if(extInCtrl->MMTTempStat==3)
				majormmtid=(majormmtid+1)%MAX_TARGET_NUMBER;
			else if(extInCtrl->MMTTempStat==4)
				majormmtid=(majormmtid-1+MAX_TARGET_NUMBER)%MAX_TARGET_NUMBER;
			else
				majormmtid=(majormmtid+1)%MAX_TARGET_NUMBER;

		}
		if(tg[i].valid==1)
		{
			//valid mmt num;
			extInCtrl->Mmttargetnum++;
		}
		Mdrawbak[i].valid=0;//reset

	}
	
	primajormmtid=tempdata=majormmtid;
	for(i=0;i<MAX_TARGET_NUMBER;i++)
	{
		if((tg[majormmtid].valid)&&(i==majormmtid))
		{

			if(extInCtrl->SensorStat==0)
			{
				if(extInCtrl->FovCtrl!=5)
				{
					result.width 	= 32;
					result.height 	= 32;
				}
				else
				{
					result.width 	= 16;
					result.height 	= 16;
				}
			}
			else
			{
				result.width 	= 16;
				result.height 	= 16;
			}
			tempmmtx=result.x = ((int)tg[majormmtid].cur_x) % vdisWH[extInCtrl->SensorStat][0];
			tempmmty=result.y = ((int)tg[majormmtid].cur_y ) % vdisWH[extInCtrl->SensorStat][1];

			
			//mmt track target set
			extInCtrl->MmtPixelX=result.x;
			extInCtrl->MmtPixelY=result.y;
			extInCtrl->MmtValid=1;

			
		
			result.x = result.x - result.width/2;
			result.y = result.y - result.height/2;
			
			 startx=PiexltoWindowsxzoom(result.x,extInCtrl->SensorStat);
			 starty=PiexltoWindowsyzoom(result.y,extInCtrl->SensorStat);
			 endx=PiexltoWindowsxzoom(result.x+result.width,extInCtrl->SensorStat);
		 	 endy=PiexltoWindowsyzoom(result.y+result.height,extInCtrl->SensorStat);
			 //erase param
			 Mdrawbak[i].startx=startx;
			 Mdrawbak[i].starty=starty;
			 Mdrawbak[i].endx=endx;
			 Mdrawbak[i].endy=endy;
			 Mdrawbak[i].valid=1;

			if((((extInCtrl->AvtTrkStat == eTrk_mode_mtd)||(extInCtrl->AvtTrkStat == eTrk_mode_acq)))&&(extInCtrl->DispGrp[extInCtrl->SensorStat]<=3))
			{
				rectangle( frame,
				Point( startx, starty ),
				Point( endx, endy),
				colour, 1, 8);
				Osdflag[osdindex]=1;

				rectangle( frame,
				Point( startx-1, starty-1 ),
				Point( endx+1, endy+1),
				colour, 1, 8);

				sprintf(numbuf,"%d",majormmtid+1);
				putText(frame,numbuf,cvPoint(startx,starty-2),CV_FONT_HERSHEY_SIMPLEX,0.8,colour);

			}
			//OSA_printf("******************the num  majormmtid=%d x=%d y=%d w=%d h=%d\n",majormmtid,
			//	result.x,result.y,result.width,result.height);
			tempmmtx  =PiexltoWindowsx(tempmmtx,extInCtrl->SensorStat);
			tempmmty  =PiexltoWindowsy(tempmmty,extInCtrl->SensorStat);
			extInCtrl->MmtOffsetXY[j]=tempmmtx&0xff;
			extInCtrl->MmtOffsetXY[j+1]=(tempmmtx>>8)&0xff;
			extInCtrl->MmtOffsetXY[j+2]=tempmmty&0xff;
			extInCtrl->MmtOffsetXY[j+3]=(tempmmty>>8)&0xff;
			
		}
		
		if((tg[i].valid)&&(i!=majormmtid))
		{
			testid++;
			if(extInCtrl->SensorStat==0)
			{
				if(extInCtrl->FovCtrl!=5)
				{
					result.width = 32;
					result.height = 32;
				}
				else
				{
					result.width = 16;
					result.height = 16;
				}
			}
			else
			{
				result.width = 16;
				result.height = 16;

			}
			
			tempmmtx=result.x = ((int)tg[i].cur_x) % vdisWH[extInCtrl->SensorStat][0];
			tempmmty=result.y = ((int)tg[i].cur_y ) % vdisWH[extInCtrl->SensorStat][1];		

			//OSA_printf("+++++++++++++++the num  majormmtid=%d x=%d y=%d w=%d h=%d\n",majormmtid,
			//result.x,result.y,result.width,result.height);
			result.x = result.x - result.width/2;
			result.y = result.y - result.height/2;
			//OSA_printf("the num  majormmtid=%d\n",tempdata);

			startx=PiexltoWindowsxzoom(result.x,extInCtrl->SensorStat);
			starty=PiexltoWindowsyzoom(result.y,extInCtrl->SensorStat);
			endx=PiexltoWindowsxzoom(result.x+result.width,extInCtrl->SensorStat);
			endy=PiexltoWindowsyzoom(result.y+result.height,extInCtrl->SensorStat);

			Mdrawbak[i].startx=startx;
			Mdrawbak[i].starty=starty;
			Mdrawbak[i].endx=endx;
			Mdrawbak[i].endy=endy;
			Mdrawbak[i].valid=1;
			if((((extInCtrl->AvtTrkStat == eTrk_mode_mtd)||(extInCtrl->AvtTrkStat == eTrk_mode_acq)))&&(extInCtrl->DispGrp[extInCtrl->SensorStat]<=3))
			{
				//DrawCross(result.x,result.y,frcolor,bShow);
				//trkimgcross(frame,result.x,result.y,16);
				#if 1
				rectangle( frame,
				Point( startx, starty ),
				Point( endx, endy),
				colour, 1, 8);
				#endif
				//OSA_printf("******************the num  majormmtid=%d\n",majormmtid);
				sprintf(numbuf,"%d",i+1);
				putText(frame,numbuf,cvPoint(startx,starty-2),CV_FONT_HERSHEY_SIMPLEX,0.8,colour);
			}
			
			extInCtrl->MmtOffsetXY[j+testid*4]=tempmmtx&0xff;
			extInCtrl->MmtOffsetXY[j+1+testid*4]=(tempmmtx>>8)&0xff;
			extInCtrl->MmtOffsetXY[j+2+testid*4]=tempmmty&0xff;
			extInCtrl->MmtOffsetXY[j+3+testid*4]=(tempmmty>>8)&0xff;

			extInCtrl->MmtOffsetXY[j+testid*4]    =PiexltoWindowsx(extInCtrl->MmtOffsetXY[j+testid*4],extInCtrl->SensorStat);
			extInCtrl->MmtOffsetXY[j+1+testid*4]=PiexltoWindowsx(extInCtrl->MmtOffsetXY[j+1+testid*4],extInCtrl->SensorStat);
			extInCtrl->MmtOffsetXY[j+2+testid*4]=PiexltoWindowsy(extInCtrl->MmtOffsetXY[j+2+testid*4],extInCtrl->SensorStat);
			extInCtrl->MmtOffsetXY[j+3+testid*4]=PiexltoWindowsy(extInCtrl->MmtOffsetXY[j+3+testid*4],extInCtrl->SensorStat);
			//j++;
			
		}

		//mmt show
		tempmmtx=result.x = ((int)tg[i].cur_x) % vdisWH[extInCtrl->SensorStat][0];
		tempmmty=result.y = ((int)tg[i].cur_y ) % vdisWH[extInCtrl->SensorStat][1];
		Mmtpos[i].x=tempmmtx-result.width/2;
		Mmtpos[i].y=tempmmty-result.height/2;
		Mmtpos[i].w=result.width;
		Mmtpos[i].h=result.height;
		Mmtpos[i].valid=tg[i].valid;

	}	

	if(Mmtsendtime==0)
		;//MSGAPI_AckSnd( AckMtdInfo);
	Mmtsendtime++;
	if(Mmtsendtime==1)
	{
		Mmtsendtime=0;
	}
	
	msgdriv_event(MSGID_EXT_INPUT_MMTSHOWUPDATE, NULL);

}



void CProcess::DrawMeanuCross(int lenx,int leny,int fcolour , bool bShow ,int centerx,int centery)
{
	int templenx=lenx;
	int templeny=leny;
	int lenw=35;
	unsigned char colour = (bShow) ?fcolour : 0;
	Osd_cvPoint start;
	Osd_cvPoint end;

	////v
	start.x=centerx-templenx;
	start.y=centery-templeny;
	end.x=centerx-templenx+lenw;
	end.y=centery-templeny;
	DrawcvLine(m_display.m_imgOsd[extInCtrl->SensorStat],&start,&end,colour,1);

	start.x=centerx+templenx-lenw;
	start.y=centery-templeny;
	end.x=centerx+templenx;
	end.y=centery-templeny;
	DrawcvLine(m_display.m_imgOsd[extInCtrl->SensorStat],&start,&end,colour,1);


	start.x=centerx-templenx;
	start.y=centery+templeny;
	end.x=centerx-templenx+lenw;
	end.y=centery+templeny;
	DrawcvLine(m_display.m_imgOsd[extInCtrl->SensorStat],&start,&end,colour,1);

	start.x=centerx+templenx-lenw;
	start.y=centery+templeny;
	end.x=centerx+templenx;
	end.y=centery+templeny;
	DrawcvLine(m_display.m_imgOsd[extInCtrl->SensorStat],&start,&end,colour,1);

	//h
	start.x=centerx-templenx;
	start.y=centery-templeny;
	end.x=centerx-templenx;
	end.y=centery-templeny+lenw;
	DrawcvLine(m_display.m_imgOsd[extInCtrl->SensorStat],&start,&end,colour,1);

	start.x=centerx+templenx;
	start.y=centery-templeny;
	end.x=centerx+templenx;
	end.y=centery-templeny+lenw;
	DrawcvLine(m_display.m_imgOsd[extInCtrl->SensorStat],&start,&end,colour,1);


	start.x=centerx-templenx;
	start.y=centery+templeny-lenw;
	end.x=centerx-templenx;
	end.y=centery+templeny;
	DrawcvLine(m_display.m_imgOsd[extInCtrl->SensorStat],&start,&end,colour,1);

	start.x=centerx+templenx;
	start.y=centery+templeny-lenw;
	end.x=centerx+templenx;
	end.y=centery+templeny;
	DrawcvLine(m_display.m_imgOsd[extInCtrl->SensorStat],&start,&end,colour,1);

}

void CProcess::DrawdashCross(int x,int y,int fcolour ,bool bShow /*= true*/)
{

	int startx=0;
	int starty=0;
	int endx=0;
	int endy=0;
	unsigned char colour = (bShow) ?fcolour : 0;
	Line_Param_fb lineparm;

	startx=WindowstoPiexlx(extInCtrl->AvtPosX[extInCtrl->SensorStat],extInCtrl->SensorStat);
	starty=WindowstoPiexly(extInCtrl->AvtPosY[extInCtrl->SensorStat],extInCtrl->SensorStat);
	
	lineparm.x=startx;
	lineparm.y=starty;
	lineparm.width=50;
	lineparm.height=50;
	lineparm.frcolor=colour;

	int dashlen=2;

	Point start,end;

	if(!bShow)
	{
		lineparm.x=secBak[1].x;
		lineparm.y=secBak[1].y;
		DrawcvDashcross(m_display.m_imgOsd[extInCtrl->SensorStat],&lineparm,dashlen,dashlen);
		startx=secBak[0].x;
		starty=secBak[0].y;
		endx=secBak[1].x;
		endy=secBak[1].y;
		
		drawdashlinepri(m_display.m_imgOsd[extInCtrl->SensorStat],startx,starty,endx,endy,dashlen,dashlen,colour);
	}

	else if(extInCtrl->DispGrp[extInCtrl->SensorStat]<3)
	{
		DrawcvDashcross(m_display.m_imgOsd[extInCtrl->SensorStat],&lineparm,dashlen,dashlen);
		startx=PiexltoWindowsxzoom(extInCtrl->AvtPosX[extInCtrl->SensorStat ],extInCtrl->SensorStat);
		starty=PiexltoWindowsyzoom(extInCtrl->AvtPosY[extInCtrl->SensorStat ],extInCtrl->SensorStat);
		endx=lineparm.x;
		endy=lineparm.y;
		drawdashlinepri(m_display.m_imgOsd[extInCtrl->SensorStat],startx,starty,endx,endy,dashlen,dashlen,colour);

		secBak[0].x=startx;
		secBak[0].y=starty;
		secBak[1].x=endx;
		secBak[1].y=endy;
		
		Osdflag[osdindex]=1;	
	}
}


void CProcess::DrawdashRect(int startx,int starty,int endx,int endy,int colour)
{
	int dashlen=3;
	drawdashlinepri(m_display.m_imgOsd[extInCtrl->SensorStat],startx,starty,endx,starty,dashlen,dashlen,colour);
	drawdashlinepri(m_display.m_imgOsd[extInCtrl->SensorStat],startx,endy,endx,endy,dashlen,dashlen,colour);
	drawdashlinepri(m_display.m_imgOsd[extInCtrl->SensorStat],endx,starty,endx,endy,dashlen,dashlen,colour);
	drawdashlinepri(m_display.m_imgOsd[extInCtrl->SensorStat],startx,starty,startx,endy,dashlen,dashlen,colour);
}

#if __MOVE_DETECT__

char CProcess::getMvListValidNum()
{
	char tmp = 0;	
	for(int i =0 ;i < 10 ; i++)
	{
		if(validMtdRecord[i])
			tmp++;
	}
	return tmp;
}

void CProcess::switchMvTargetForwad()
{
	unsigned int distance = 4000*4000;
	unsigned int tmp=0 ;
	unsigned int x,y;
	int index = -1;
	
	for(int i = 0; i < mvListsum.size(); i++)
	{
		if(chooseDetect == mvListsum[i].number)
			continue;
		
		x = abs(mvListsum[i].trkobj.targetRect.x - cur_targetRect_bak.x);	
		y = abs(mvListsum[i].trkobj.targetRect.y - cur_targetRect_bak.y);
		tmp = (x*x + y*y);
		if(tmp < distance)
		{
			distance = tmp;
			index = i;
		}
	}
	if(index != -1)
	{
		chooseDetect = mvListsum[index].number;
		cur_targetRect = mvListsum[index].trkobj.targetRect;
		losenumber = -1;
	}
	else
	{
		chooseDetect = 10;
		memset( &cur_targetRect,0,sizeof(cur_targetRect));
	}
	lose_timer_flag = 0;

}

void CProcess::addMvListValidNum(char num)
{
	if(num < 10)
		validMtdRecord[num] = true;
	return ;
}

char CProcess::getMvListFirstUnusedNum()
{	
	for(int i =0 ;i < 10 ; i++)
	{
		if(!validMtdRecord[i])
			return i;
	}
	return 10;
}

void CProcess::removeMvListValidNum(char num)
{
	if(num < 10)
		validMtdRecord[num] = false;
	return ;
}

char CProcess::getMvListNextValidNum(char index)
{
	for(int i = index+1 ;i < 10 ; i++)
	{
		if(validMtdRecord[i])
			return i;
	}

	for(int i = 0 ; i<index ; i++)
	{
		if(validMtdRecord[i])
			return i;
	}
	return 10;
}

bool comp(const TRK_RECT_INFO &a,const TRK_RECT_INFO &b)
{
	unsigned int tmpa ,tmpb;
	unsigned int mx,my;
	mx = abs(a.targetRect.x - plat->cur_targetRect_bak.x);
	my = abs(a.targetRect.y - plat->cur_targetRect_bak.y);
	tmpa = mx*mx + my*my;
	mx = abs(b.targetRect.x - plat->cur_targetRect_bak.x);
	my = abs(b.targetRect.y - plat->cur_targetRect_bak.y);
	tmpb = mx*mx + my*my;
	return tmpa<tmpb;
}

void CProcess::getTargetNearToCenter(std::vector<TRK_RECT_INFO> *pVec)
{
	int sizeNum = pVec->size();
	if(sizeNum)
		sort(pVec->begin(),pVec->end(),comp);
}

void CProcess::mvIndexHandle(std::vector<TRK_INFO_APP> *mvList,std::vector<TRK_RECT_INFO> &detect,int detectNum)
{	
	int tmpIndex , i ;
	bool flag;
	TRK_INFO_APP pTmpMv;
	
	if(!mvList->empty())
	{	
		i = 0;
		std::vector<TRK_INFO_APP>::iterator pMvList = mvList->begin();
		
		for( ; pMvList !=  mvList->end(); )
		{
			tmpIndex = (*pMvList).trkobj.index;

			flag = 0;
			std::vector<TRK_RECT_INFO>::iterator pDetect = detect.begin();
			for( ; pDetect != detect.end(); )
			{
				if( tmpIndex == (*pDetect).index )
				{
					memcpy((void*)&((*pMvList).trkobj.targetRect),(void*)&((*pDetect).targetRect),sizeof(TRK_RECT_INFO));				
					if((chooseDetect == (*pMvList).number) && (losenumber != (*pMvList).number))
					{
						cur_targetRect = (*pMvList).trkobj.targetRect;
					}
					detect.erase(pDetect);
					flag = 1;
					break;
				}
				else
					++pDetect;
			}	

			if(!flag)
			{
				if((chooseDetect == (*pMvList).number) && (0 == lose_timer_flag))
				{
					losenumber = (*pMvList).number;
					cur_targetRect = (*pMvList).trkobj.targetRect;
					lose_timer_flag = 1;
					chooseDetect = 10;
				}
				removeMvListValidNum((*pMvList).number);
				mvList->erase(pMvList);
			}
			else
				++pMvList;
		
		}

		i = 0;
		while(detect.size() > 0)
		{	
			if(mvList->size() >= detectNum)
				break ;
			pTmpMv.number = getMvListFirstUnusedNum();
			if(pTmpMv.number < 10)
			{
				addMvListValidNum(pTmpMv.number);
				memcpy((void*)&(pTmpMv.trkobj),(void *)&(detect[i++].targetRect),sizeof(TRK_RECT_INFO));
				mvList->push_back(pTmpMv);	
			}
			if( chooseDetect == 10 )
				chooseDetect = pTmpMv.number;
		}	
	}
	else
	{
		int tmpnum = detect.size() < detectNum ? detect.size() : detectNum ;
		for(i =0 ; i < tmpnum ; i++)
		{
			pTmpMv.number = getMvListFirstUnusedNum();
			if(pTmpMv.number < 10)
			{
				addMvListValidNum(pTmpMv.number);
				memcpy((void*)&(pTmpMv.trkobj),(void *)&(detect[i++].targetRect),sizeof(TRK_RECT_INFO));
				mvList->push_back(pTmpMv);
			}
		}
	}
	
}
#endif

bool CProcess::OnProcess(int chId, Mat &frame)
{				
	int frcolor= extInCtrl->osdDrawColor;
	int startx=0;
	int starty=0;
	int endx=0;
	int endy=0;
	int i;
	cv::Rect recIn;
	static int coastCnt = 1;
	static int bDraw = 0;
	int color = 0;
	int grid_height = (int)((float)(outputWHF[1]/12));
	int grid_width = (int)((float)(outputWHF[0]/16));
	static int changesensorCnt = 0;
	
	if(extInCtrl->changeSensorFlag == 1)
		++changesensorCnt;
	if(changesensorCnt == 3){
		extInCtrl->changeSensorFlag =  0; 
		changesensorCnt = 0;
	}	
	
	if(((++coastCnt)%10) == 0)
	{
		bDraw = !bDraw;	
		coastCnt = 0;
	}
	
	CvScalar colour=GetcvColour(frcolor);
	static unsigned int countnofresh = 0;
	if((countnofresh ) >= 5)
	{
		countnofresh=0;
	}
	
	countnofresh++;

	osdindex=0;	
	osd_index = 0;

	Point center;
	Point start,end;
	Osd_cvPoint start1,end1;	

#if __TRACK__
	osdindex++;
	{
		 UTC_RECT_float rcResult = m_rcTrack;
		 UTC_RECT_float rcResult_algRect = m_rcTrack;

		 trackinfo_obj->trackrect=m_rcTrack;
		 trackinfo_obj->TrkStat = extInCtrl->AvtTrkStat;
		 extInCtrl->TrkStat = extInCtrl->AvtTrkStat;
		 m_SensorStat = extInCtrl->SensorStat;

	  
		 int aimw = extInCtrl->AimW[extInCtrl->SensorStat];
		 int aimh = extInCtrl->AimH[extInCtrl->SensorStat];

		if(changesensorCnt){
			rectangle( m_display.m_imgOsd[extInCtrl->SensorStat],
				Point( rcTrackBak[extInCtrl->SensorStatpri].x, rcTrackBak[extInCtrl->SensorStatpri].y ),
				Point( rcTrackBak[extInCtrl->SensorStatpri].x+rcTrackBak[extInCtrl->SensorStatpri].width, 
					rcTrackBak[extInCtrl->SensorStatpri].y+rcTrackBak[extInCtrl->SensorStatpri].height),
				cvScalar(0,0,0, 0), 1, 8 );
		}

		if(Osdflag[osdindex]==1)
 		{			
			rectangle( m_display.m_imgOsd[extInCtrl->SensorStat],
				Point( resultTrackBak.x, resultTrackBak.y ),
				Point( resultTrackBak.x+resultTrackBak.width, resultTrackBak.y+resultTrackBak.height),
				cvScalar(0,0,0,0), 1, 8 );
			rectangle( m_display.m_imgOsd[extInCtrl->SensorStat],
				Point( rcTrackBak[extInCtrl->SensorStat].x, rcTrackBak[extInCtrl->SensorStat].y ),
				Point( rcTrackBak[extInCtrl->SensorStat].x+rcTrackBak[extInCtrl->SensorStat].width, 
					rcTrackBak[extInCtrl->SensorStat].y+rcTrackBak[extInCtrl->SensorStat].height),
				cvScalar(0,0,0, 0), 1, 8 );
			Osdflag[osdindex]=0;
		}
		 if(m_bTrack)
		 {
			extInCtrl->TrkXtmp = rcResult.x + rcResult.width/2;
			extInCtrl->TrkYtmp = rcResult.y + rcResult.height/2;
			
			startx=PiexltoWindowsxzoom_TrkRect(rcResult.x+rcResult.width/2-aimw/2,extInCtrl->SensorStat);			
			starty=PiexltoWindowsyzoom_TrkRect(rcResult.y+rcResult.height/2-aimh/2 ,extInCtrl->SensorStat);
			endx  =PiexltoWindowsxzoom_TrkRect(rcResult.x+rcResult.width/2+aimw/2,extInCtrl->SensorStat);
		 	endy  =PiexltoWindowsyzoom_TrkRect(rcResult.y+rcResult.height/2+aimh/2 ,extInCtrl->SensorStat);

			if(algOsdRect == true)
			{
				rcResult_algRect.x = PiexltoWindowsx(rcResult_algRect.x,extInCtrl->SensorStat);
				rcResult_algRect.y = PiexltoWindowsy(rcResult_algRect.y,extInCtrl->SensorStat);
				rcResult_algRect.width = PiexltoWindowsx(rcResult_algRect.width,extInCtrl->SensorStat);
				rcResult_algRect.height = PiexltoWindowsy(rcResult_algRect.height,extInCtrl->SensorStat);
			}

			if( m_iTrackStat == 1 && !changesensorCnt)
			{		
				if(algOsdRect == true)
				{
					rectangle( m_display.m_imgOsd[extInCtrl->SensorStat],
						Point( rcResult_algRect.x, rcResult_algRect.y ),
						Point( rcResult_algRect.x+rcResult_algRect.width, rcResult_algRect.y+rcResult_algRect.height),
						cvScalar(0,255,0,255), 1, 8 );
				}
				else
				{
					rectangle( m_display.m_imgOsd[extInCtrl->SensorStat],
						Point( startx, starty ),
						Point( endx, endy),
						colour, 1, 8 );
				}
			}
			else
			{
				startx=PiexltoWindowsxzoom(extInCtrl->TrkXtmp-aimw/2,extInCtrl->SensorStat);			
				starty=PiexltoWindowsyzoom(extInCtrl->TrkYtmp-aimh/2 ,extInCtrl->SensorStat);
				endx=PiexltoWindowsxzoom(extInCtrl->TrkXtmp+aimw/2,extInCtrl->SensorStat);
				endy=PiexltoWindowsyzoom(extInCtrl->TrkYtmp+aimh/2 ,extInCtrl->SensorStat);			

				if(bDraw != 0 && !changesensorCnt)
				{
					if(algOsdRect == true)
						DrawdashRect(rcResult_algRect.x,rcResult_algRect.y,rcResult_algRect.x+rcResult_algRect.width,rcResult_algRect.y+rcResult_algRect.height,frcolor);
					else
						DrawdashRect(startx,starty,endx,endy,frcolor);	// track lost DashRect				
				}
			}
			if(!changesensorCnt){
				Osdflag[osdindex]=1;
				rcTrackBak[extInCtrl->SensorStat].x=startx;
				rcTrackBak[extInCtrl->SensorStat].y=starty;
				rcTrackBak[extInCtrl->SensorStat].width=endx-startx;
				rcTrackBak[extInCtrl->SensorStat].height=endy-starty;
			}
			if(algOsdRect == true)
			{
				resultTrackBak.x = rcResult_algRect.x;
				resultTrackBak.y = rcResult_algRect.y;
				resultTrackBak.width = rcResult_algRect.width;
				resultTrackBak.height = rcResult_algRect.height;
			}
			
			extInCtrl->unitAimX=rcResult.x+rcResult.width/2;
			extInCtrl->unitAimY=rcResult.y+rcResult.height/2;
			extInCtrl->unitAimW=rcResult.width;
			extInCtrl->unitAimH=rcResult.height;
		 }
		 
		 if(m_bTrack && !changesensorCnt)
		 {
			set_trktype(extInCtrl,m_iTrackStat);
			if(m_iTrackStat == 1)
			{
				rememflag=false;
			}
			else if(m_iTrackStat == 2)
			{
				if(!rememflag)
				{
					rememflag=true;
					rememtime=OSA_getCurTimeInMsec();
				}
				
				if((OSA_getCurTimeInMsec()-rememtime) > glosttime)
				{		
					set_trktype(extInCtrl,3);
				}
				else
				{
					set_trktype(extInCtrl,2);
				}
			}
		 	 if((extInCtrl->TrkStat == 1)||(extInCtrl->TrkStat == 2))
		 	 {
				extInCtrl->TrkX =rcResult.x+rcResult.width/2;
				extInCtrl->TrkY = rcResult.y+rcResult.height/2;
						
				extInCtrl->trkerrx=(PiexltoWindowsxf(extInCtrl->TrkX ,extInCtrl->SensorStat));
				extInCtrl->trkerry=(PiexltoWindowsyf(extInCtrl->TrkY ,extInCtrl->SensorStat));
				
				if(extInCtrl->TrkStat == 2)
				{
					extInCtrl->trkerrx=(PiexltoWindowsx(extInCtrl->AxisPosX[extInCtrl->SensorStat] ,extInCtrl->SensorStat));
					extInCtrl->trkerry=(PiexltoWindowsy(extInCtrl->AxisPosY[extInCtrl->SensorStat] ,extInCtrl->SensorStat));
				}

				extInCtrl->TrkErrFeedback = 1;
		 	 }
			 else
			 	extInCtrl->TrkErrFeedback = 0;
			 
			if(extInCtrl->TrkStat!=extInCtrl->TrkStatpri)
			{
				extInCtrl->TrkStatpri=extInCtrl->TrkStat;
			}

		#if __IPC__
			if(extInCtrl->TrkStat != 3)
			{
				extInCtrl->trkerrx = extInCtrl->trkerrx - extInCtrl->opticAxisPosX[extInCtrl->SensorStat];
				extInCtrl->trkerry = extInCtrl->trkerry - extInCtrl->opticAxisPosY[extInCtrl->SensorStat];
			}
			else
			{
				extInCtrl->trkerrx = 0;
				extInCtrl->trkerry = 0;
			}
			ipc_settrack(extInCtrl->TrkStat, extInCtrl->trkerrx, extInCtrl->trkerry);
			trkmsg.cmd_ID = read_shm_trkpos;
			//printf("ack the trackerr to mainThr\n");
			ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);

			if(m_display.disptimeEnable == 1){
				//test zhou qi  time
				int64 disptime = 0;
				disptime = getTickCount();
				double curtime = (disptime/getTickFrequency())*1000;
				static double pretime = 0.0;
				double time = curtime - pretime;
				pretime = curtime;

				if(m_display.disptimeEnable == 1)
				{
					putText(m_display.m_imgOsd[1],trkFPSDisplay,
						Point( m_display.m_imgOsd[1].cols-350, 25),
						FONT_HERSHEY_TRIPLEX,0.8,
						cvScalar(0,0,0,0), 1
						);
					sprintf(trkFPSDisplay, "trkerr time = %0.3fFPS", 1000.0/time);
					putText(m_display.m_imgOsd[1],trkFPSDisplay,
						Point(m_display.m_imgOsd[1].cols-350, 25),
						FONT_HERSHEY_TRIPLEX,0.8,
						cvScalar(255,255,0,255), 1
						);
				}

			}
			#endif	
				
		 }
		 else
	 	{
			rememflag=false;
			extInCtrl->TrkErrFeedback = 0;
	 	}
	}
#endif

/*
	//mtd
osdindex++;
	{
		if(Osdflag[osdindex]==1)
		{
			erassdrawmmtnew(Mdrawbak, false);
			Osdflag[osdindex]=0;
		}
		if(m_bMtd && changesensorCnt)
		{
			drawmmtnew(m_mtd[chId]->tg, true);		
		}
	}
*/

osdindex++;	//cross aim
	{
		if(changesensorCnt){
			recIn.x=crossBak.x;
	 		recIn.y=crossBak.y;
			recIn.width = crossWHBak.x;
			recIn.height = crossWHBak.y;
			DrawCross(recIn,frcolor,extInCtrl->SensorStatpri,false);
		}
		
	 	if(Osdflag[osdindex]==1){
			recIn.x=crossBak.x;
	 		recIn.y=crossBak.y;
			recIn.width = crossWHBak.x;
			recIn.height = crossWHBak.y;
			DrawCross(recIn,frcolor,extInCtrl->SensorStat,false);
			Osdflag[osdindex]=0;
 		}

	}


	osdindex++;	//acqRect
	{
		if(changesensorCnt){
			recIn = acqRectBak;
			if(extInCtrl->SensorStatpri>MAX_CHAN)
				extInCtrl->SensorStatpri = 1;
			DrawAcqRect(m_display.m_imgOsd[extInCtrl->SensorStatpri],recIn,frcolor,false);
		}

		if(Osdflag[osdindex]==1){
			recIn = acqRectBak;
			if(extInCtrl->SensorStat>=MAX_CHAN)
				extInCtrl->SensorStat = 1;
			DrawAcqRect(m_display.m_imgOsd[extInCtrl->SensorStat],recIn,frcolor,false);
			Osdflag[osdindex]=0;
 		}
	}

	
#if __MOVE_DETECT__
//mtd grid
	DrawMtdYellowGrid();
	DrawMtdRedGrid();
	DrawMtd_Rigion_Target();

#endif

	osdindex++;
	{		
		if( open_handleCalibra == true){  
			sprintf(show_key[string_cnt1], "%d", string_cnt1);	
			putText(m_display.m_imgOsd[1],show_key[string_cnt1],key1_pos,FONT_HERSHEY_TRIPLEX,0.8, cvScalar(255,0,0,255), 1);	
			cv::circle( m_display.m_imgOsd[1], key1_pos, 3 , cvScalar(255,0,255,255), 2, 8, 0);
			
			sprintf(show_key2[string_cnt2], "%d", string_cnt2);
			putText(m_display.m_imgOsd[1],show_key2[string_cnt2],key2_pos,FONT_HERSHEY_TRIPLEX,0.8, cvScalar(0,255,0,255), 1);	
			cv::circle(m_display.m_imgOsd[1],key2_pos,3 ,cvScalar(0,255,255,255),2,8,0);
			Osdflag[osdindex]=1;
		}	
		else{	
			if(Osdflag[osdindex])
			{
				
				if(key_point1_cnt!=0 || (key_point2_cnt!=0))
				{
					textPos1_backup[0] = key1_backup;
					textPos2_backup[0] = key2_backup;
					for(int m=0;m<=key_point1_cnt; m++){
						textPos1_backup[m+1] = textPos1_record[m];
					}
					for(int m=0;m<=key_point2_cnt; m++){
						textPos2_backup[m+1] = textPos2_record[m];
					}
					
					for(int i=0; i<=key_point1_cnt; i++)
					{
						putText(m_display.m_imgOsd[1],show_key[i],textPos1_backup[i],FONT_HERSHEY_TRIPLEX,0.8, cvScalar(0,0,0,0), 1);	
						cv::circle(m_display.m_imgOsd[1],textPos1_backup[i],3 ,cvScalar(0,0,0,0),2,8,0);			
					}

					for(int i=0; i<=key_point2_cnt; i++)
					{
						putText(m_display.m_imgOsd[1],show_key2[i],textPos2_backup[i],FONT_HERSHEY_TRIPLEX,0.8, cvScalar(0,0,0,0), 1);	
						cv::circle(m_display.m_imgOsd[1],textPos2_backup[i],3 ,cvScalar(0,0,0,0),2,8,0);
					}
				}
				Osdflag[osdindex] = 0;			
			}			
		}	
	}
//=========================Draw A Rectangle On Selected Picture ===============================
{

	/* Draw a rectangle  on selected picture  of saved calibration  pictures */
#if 0
	if(0){
		int leftStartX = (m_display.getSelectPicIndex() % 10)*192;
		int leftStartY = 540 - (m_display.getSelectPicIndex() /10)*108;
		m_rectSelectPic = Rect(leftStartX,leftStartY,192,108);
		rectangle (m_display.m_imgOsd[1],  m_rectSelectPic,cvScalar(0,0,255,255), 1, 8);
	}
#endif
	/* Show how many pictures have been detected Corners when change diffrent ChessBoard Poses */	
	if( (m_display.displayMode == CALIBRATE_CAPTURE) && (showDetectCorners == true)){		
		putText(m_display.m_imgOsd[1],Bak_CString,Point(245,423),FONT_HERSHEY_TRIPLEX,0.8, cvScalar(0,0,0,0), 1);
		sprintf(Bak_CString,"%d",captureCount);						
		putText(m_display.m_imgOsd[1],Bak_CString,Point(245,423),FONT_HERSHEY_TRIPLEX,0.8, cvScalar(0,255,255,255), 1);
	}else {
		putText(m_display.m_imgOsd[1],Bak_CString,Point(245,423),FONT_HERSHEY_TRIPLEX,0.8, cvScalar(0,0,0,0), 1);
	}	
}

//=============================================================================================

	if(g_displayMode == MENU_GRID_MAP_VIEW)
	{
		DrawGridMap_16X12(1);
		DrawGridMapNodeCircles_16X12(true);
		DrawSelectedCircle(true, m_curNodeIndex);
		
		{
			recIn.x=back_center.x;
			recIn.y=back_center.y;
			recIn.width = grid_width; // when resolution 1920X1080, grid_width = 120
			recIn.height = grid_height;// when resolution 1920X1080, grid_height = 90
			DrawCross(recIn,frcolor,1,false);

			back_center =  m_display.getGridViewBallImgCenter();
			recIn.x=back_center.x;
			recIn.y=back_center.y;
			recIn.width =grid_width;
			recIn.height =grid_height;
			DrawCross(recIn,frcolor,1,true);
		}
		renderText(CProcess::DRAW_NOTICE_TEXTS);

		renderflag[0]=1;
	}
	else
	{
		
	if(renderflag[0] == 1)
	{
		DrawGridMap_16X12(0);
		DrawSelectedCircle(false, m_curNodeIndex);
		DrawGridMapNodeCircles_16X12(false);
		
		renderText(CProcess::ERASE_TEXTS);
		recIn.x=back_center.x;
		recIn.y=back_center.y;
		recIn.width = grid_width;
		recIn.height = grid_height;
		DrawCross(recIn,frcolor,1,false);
	}
	renderflag[0] = 0;
}


/***********************************************************************************/
#if 0
if(g_GridMapMode == 1)
{
	sprintf(str_mode, "GridMode");	
	putText(m_display.m_imgOsd[1],str_mode,cv::Point(10,535),FONT_HERSHEY_TRIPLEX,0.4, cvScalar(255,255,0,255), 1);	


}else
{
	putText(m_display.m_imgOsd[1],str_mode,cv::Point(10,535),FONT_HERSHEY_TRIPLEX,0.4, cvScalar(0,0,0,255), 1);	
}
#endif

	if(m_bIsClickMode == true&&(g_displayMode == MENU_MAIN_VIEW))
	{
		#if 1
			recIn.x=m_bakClickPoint.x;
	 		recIn.y=m_bakClickPoint.y;
			recIn.width = 60;
			recIn.height = 60;
			DrawCross(recIn,frcolor,1,false);
			Osdflag[osdindex]=1;	
			osdindex++;
				
			m_bakClickPoint = getCurrentMouseClickPoint();
			recIn.x=m_bakClickPoint.x;
	 		recIn.y=m_bakClickPoint.y;
			recIn.width = 60;
			recIn.height = 60;
			DrawCross(recIn,frcolor,1,true);
			Osdflag[osdindex]=1;	
		#else
			//GB_DrawCross(m_display.m_imgOsd[1],cv::Point(m_bakClickPoint.x, m_bakClickPoint.y), true);

			cv::circle(m_display.m_imgOsd[1],m_bakClickPoint,3 ,cvScalar(0,0,0,0),2,8,0);
			m_bakClickPoint = getCurrentMouseClickPoint();
			cv::circle(m_display.m_imgOsd[1],m_bakballDestPoint,3 ,cvScalar(255,0,0,255),2,8,0);
		#endif
		
	}
	else{
		#if 1
			recIn.x=m_bakClickPoint.x;
	 		recIn.y=m_bakClickPoint.y;
			recIn.width = 60;
			recIn.height = 60;
			DrawCross(recIn,frcolor,1,false);
			Osdflag[osdindex]=1;	
			osdindex++;
		#else
			//GB_DrawCross(m_display.m_imgOsd[1],cv::Point(m_bakClickPoint.x, m_bakClickPoint.y), false);
			cv::circle(m_display.m_imgOsd[1],m_bakClickPoint,3 ,cvScalar(0,0,0,0),2,8,0);
		#endif
	}
//--------------------------------------------------------

	if( m_display.g_CurDisplayMode == TEST_RESULT_VIEW)
	{
		cv::circle(m_display.m_imgOsd[1],m_bakballDestPoint,3 ,cvScalar(0,0,0,0),2,8,0);
		m_bakballDestPoint = getBallImagePoint();
		cv::circle(m_display.m_imgOsd[1],m_bakballDestPoint,3 ,cvScalar(255,0,0,255),2,8,0);		
	}
	else
	{		
		cv::circle(m_display.m_imgOsd[1],m_bakballDestPoint,3 ,cvScalar(0,0,0,0),2,8,0);
	}
	
//--------------------------------------------------------
{
#if 0
	sprintf(Bak_CString,"%s","=>");
	putText(m_display.m_imgOsd[1],Bak_CString,Point(backMenuposX,backMenuposY),FONT_HERSHEY_TRIPLEX,0.8, cvScalar(0,0,0,0), 1);
	backMenuposX = 1460;	
	backMenuposY = m_display.m_currentMenuPos[m_display.m_currentFirstMenuIndex][m_display.m_currentSecondMenuIndex].posY +15;
	putText(m_display.m_imgOsd[1],Bak_CString,Point(backMenuposX,backMenuposY),FONT_HERSHEY_TRIPLEX,0.8, cvScalar(0,255,255,255), 1);
#else
if(show_circle_pointer &&
	m_display.m_currentMenuPos[m_display.m_currentFirstMenuIndex][m_display.m_currentSecondMenuIndex].isShow)
{	cv::circle(m_display.m_imgOsd[1],Point(backMenuposX,backMenuposY),8 ,cvScalar(0,0,0,0),-1,8,0);
	backMenuposX = 1460;	
	backMenuposY = m_display.m_currentMenuPos[m_display.m_currentFirstMenuIndex][m_display.m_currentSecondMenuIndex].posY +15;
	cv::circle(m_display.m_imgOsd[1],Point(backMenuposX,backMenuposY),8 ,cvScalar(0,0,255,255),-1,8,0);
}
else{	
		//backMenuposX = 1460;	
		//backMenuposY = m_display.m_currentMenuPos[m_display.m_currentFirstMenuIndex][m_display.m_currentSecondMenuIndex].posY +15;
	cv::circle(m_display.m_imgOsd[1],Point(backMenuposX,backMenuposY),8 ,cvScalar(0,0,0,0),-1,8,0);
}
#endif

}
/* 
*   Here draw circle to Mark the point after remap on the ball image 
*/

	if( m_bMarkCircle == true) {
		cv::circle(m_display.m_imgOsd[1],dest_ballPoint,3 ,cvScalar(0,255,255,255),2,8,0);
	}
	else {
		cv::circle(m_display.m_imgOsd[1],dest_ballPoint,3 ,cvScalar(0,0,0,0),2,8,0);
	}


	//center.x = vdisWH[extInCtrl->SensorStat][0]/2;
	//center.y = vdisWH[extInCtrl->SensorStat][1]/2;
	//int radius = 4;
	//cv::circle(m_display.m_imgOsd[extInCtrl->SensorStat],center,radius ,cvScalar(255,0,255,255),8,8,0);

	prisensorstatus=extInCtrl->SensorStat;


//mouse rect
	DrawDragRect();
	
//time
	if(m_time_flag)
	{
		if(m_time_show)
		{
			time_t t = time(NULL);
			
			putText(m_display.m_imgOsd[extInCtrl->SensorStat],timearrbak,Point(timexbak,timeybak),FONT_HERSHEY_TRIPLEX,0.8, cvScalar(0,0,0,0), 1);
			strftime(timearr, sizeof(timearr)-1, "%Y-%m-%d %H:%M:%S", localtime(&t));
			memcpy(timearrbak, timearr, sizeof(timearr));
			timexbak = m_display.timex;
			timeybak = m_display.timey;
			putText(m_display.m_imgOsd[extInCtrl->SensorStat],timearrbak,Point(timexbak,timeybak),FONT_HERSHEY_TRIPLEX,0.8, cvScalar(255,255,255,255), 1);
		}
		else
		{
			putText(m_display.m_imgOsd[extInCtrl->SensorStat],timearrbak,Point(timexbak,timeybak),FONT_HERSHEY_TRIPLEX,0.8, cvScalar(0,0,0,0), 1);
			m_time_flag = 0;
		}
	}

//virtual joystick
//	DrawJoys();
	DrawMouse();
	DrawTrigInter();
	Drawfeaturepoints();
	Draw_point_triangle();
	Drawsubdiv();

	static unsigned int count = 0;
	if((count & 1) == 1)
		OSA_semSignal(&(sThis->m_display.tskdisSemmain));
	count++;
	return true;
}


void CProcess::Drawfeaturepoints()
{
	static int drawpoint_falg = 0;
	if(drawpoint_falg)
	{
		m_autofr.drawPoints(m_display.m_imgOsd[extInCtrl->SensorStat], app_recommendPoints_bak, 0);
		drawpoint_falg = 0;
	}
	if(get_drawpoints_stat())
	{
		app_recommendPoints_bak = app_recommendPoints;
		//printf("%s, %d, app_recommendPoints.size=%d\n", __FILE__, __LINE__,app_recommendPoints.size());
		m_autofr.drawPoints(m_display.m_imgOsd[extInCtrl->SensorStat], app_recommendPoints_bak, 1);
		drawpoint_falg = 1;
	}
}

void CProcess::Drawsubdiv()
{
	static int drawsubidv_falg = 0;
	static vector<Vec6f> triangleList;
	
	if(drawsubidv_falg)
	{
		m_autofr.draw_subdiv(m_display.m_imgOsd[extInCtrl->SensorStat],  triangleList, 0);
		drawsubidv_falg = 0;
	}
	if(get_drawsubdiv_stat())
	{
		//printf("%s,%d, draw_subdiv\n",__FILE__,__LINE__);
		m_autofr.getTriangleList(triangleList);
		m_autofr.draw_subdiv(m_display.m_imgOsd[extInCtrl->SensorStat],  triangleList, 1);
		drawsubidv_falg = 1;
	}
}

void CProcess::Draw_point_triangle()
{
	static int draw_point_triangle_falg = 0;
	
	Point2i pos;
	
	if(draw_point_triangle_falg)
	{
		m_autofr.draw_point_triangle(m_display.m_imgOsd[extInCtrl->SensorStat],   point_triangle_bak, m_back, pos, 0);
		draw_point_triangle_falg = 0;
	}
	if(get_draw_point_triangle_stat())
	{
		point_triangle_bak = point_triangle;
		m_autofr.draw_point_triangle(m_display.m_imgOsd[extInCtrl->SensorStat],  point_triangle_bak, m_back, pos, 1);

		if(get_print_stat())
		{
			printf("\n\n%s,%d, input pixel(%d,%d), pos(%d,%d)\n",__FILE__,__LINE__,  point_triangle_bak.x,point_triangle_bak.y,pos.x,pos.y);
			for(int i = 0; i < m_back.size(); i++)
			{
				printf("%s,%d, i=%d, pixel(%d,%d), pos(%d,%d)\n",__FILE__,__LINE__, i, m_back[i].pixel.x,m_back[i].pixel.y,m_back[i].pos.x,m_back[i].pos.y);
			}
			set_print_stat(false);
		}

		draw_point_triangle_falg = 1;
	}
}
	
void CProcess::DrawDragRect()
{
	unsigned int drawRectId;
	static int drawRectId_bak;
	static int flag = 0;
	int j = 0;

	if(m_display.g_CurDisplayMode == MAIN_VIEW)
	{			
		drawRectId = 1;
	}
	else
	{
		drawRectId = extInCtrl->SensorStat;
	}

	if(flag)
	{	
		rectangle(m_display.m_imgOsd[drawRectId_bak],
				Point(mRectbak[drawRectId_bak][j].x1, mRectbak[drawRectId_bak][j].y1),
				Point(mRectbak[drawRectId_bak][j].x2, mRectbak[drawRectId_bak][j].y2),
				cvScalar(0,0,0,0), 1, 8);

		flag = 0;
	}

	
	if(m_draw)
	{	
		memcpy(mRectbak, mRect, sizeof(mRectbak));
		memcpy(m_rectnbak, m_rectn, sizeof(m_rectnbak));
		drawRectId_bak = drawRectId;
		
		mRectbak[drawRectId_bak][j].x1 = mRect[drawRectId_bak][j].x1;
		mRectbak[drawRectId_bak][j].y1 = mRect[drawRectId_bak][j].y1;
		mRectbak[drawRectId_bak][j].x2 = m_tempX;
		mRectbak[drawRectId_bak][j].y2 = m_tempY;
		rectangle(m_display.m_imgOsd[drawRectId_bak],
				Point(mRectbak[drawRectId_bak][j].x1, mRectbak[drawRectId_bak][j].y1),
				Point(mRectbak[drawRectId_bak][j].x2, mRectbak[drawRectId_bak][j].y2),
				cvScalar(0,0,255,255), 1, 8);

		flag = 1;
	}
}

void CProcess::DrawJoys()
{
	Mat frame = m_display.m_imgOsd[1];
	int jradius = get_joyradius();
	int jradius_s = 10;
	cv::Point jcenter = get_joycenter();
	int thickness = 2;
	static int flag = 0;
	
	Line_Param_fb lineparm;
	lineparm.x = jcenter.x;
	lineparm.y = jcenter.y;
	lineparm.width = jradius*2;
	int dashlen = 2;

	static cv::Point jcenter_s_bak;
	static int jradius_s_bak;

	if(flag)
	{
		lineparm.frcolor = 0;
		DrawCircle(frame, jcenter_s_bak, jradius_s_bak, lineparm.frcolor, thickness);
		DrawCircle(frame, jcenter, jradius, lineparm.frcolor, thickness);
		DrawcvDashcross(frame, &lineparm, dashlen, dashlen);
		flag = 0;
	}
	
	if((m_display.displayMode == MAIN_VIEW)&&(m_display.m_menuindex == -1))
	{		
		jcenter_s_bak = jcenter_s;
		jradius_s_bak= jradius_s;
		lineparm.frcolor = 7;
		DrawCircle(frame, jcenter_s_bak, jradius_s_bak, lineparm.frcolor, thickness);
		DrawCircle(frame, jcenter, jradius, lineparm.frcolor, thickness);
		DrawcvDashcross(frame, &lineparm, dashlen, dashlen);
		flag = 1;
	}
}

void CProcess::DrawMouse()
{
	Mat frame = m_display.m_imgOsd[1];
	int linecolor, color;
	static cv::Point jos_mouse_bak;
	static int flag = 0; 
	
	if(flag)
	{
		linecolor = 0;
		color = 0;
		DrawArrow(frame, jos_mouse_bak, linecolor, color);
		flag = 0;
	}
	
	if(mouse_show)
	{
		jos_mouse_bak = jos_mouse;
		linecolor = 1;
		color = 2;
		DrawArrow(frame, jos_mouse_bak, linecolor, color);
		flag = 1;
	}
}

void CProcess::DrawTrigInter()
{
	int Chid = 1;
	Mat frame = m_display.m_imgOsd[Chid];
	cv::Rect recIn;
	static int osd_flag = 0;
	static int mode = 0;
	int color = 0;

	if(osd_flag)
	{
	
		color = 0;
		if(0 == mode)
		{
			recIn.x = 0 + outputWHF[0] / 8;
		 	recIn.y = 0 + outputWHF[1] / 8;
		}			
		else if(1 == mode)
		{
			recIn.x = outputWHF[0] / 4 * 3 + outputWHF[0] / 8;
		 	recIn.y = 0 + outputWHF[1] / 8;
		}			
		else if(2 == mode)
		{
			recIn.x = outputWHF[0] / 4 * 3 + outputWHF[0] / 8;
		 	recIn.y = outputWHF[1] /4 * 3 + outputWHF[1] / 8;
		}
		else if(3 == mode)
		{
			recIn.x = 0 + outputWHF[0] / 8;
		 	recIn.y = outputWHF[1] /4 * 3 + outputWHF[1] / 8;
		}
		recIn.width = 15;
		recIn.height = 15;
		DrawCross(recIn,color,Chid,false);
		
		osd_flag = 0;
	}


	if((MENU_TRIG_INTER_MODE == g_displayMode) && get_showpip_stat())
	{	

		color = 6;
		mode = m_display.gettrig_pip_mode();
		if(0 == mode)
		{
			recIn.x = 0 + outputWHF[0] / 8;
		 	recIn.y = 0 + outputWHF[1] / 8;
		}			
		else if(1 == mode)
		{
			recIn.x = outputWHF[0] / 4 * 3 + outputWHF[0] / 8;
		 	recIn.y = 0 + outputWHF[1] / 8;
		}			
		else if(2 == mode)
		{
			recIn.x = outputWHF[0] / 4 * 3 + outputWHF[0] / 8;
		 	recIn.y = outputWHF[1] /4 * 3 + outputWHF[1] / 8;
		}
		else if(3 == mode)
		{
			recIn.x = 0 + outputWHF[0] / 8;
		 	recIn.y = outputWHF[1] /4 * 3 + outputWHF[1] / 8;
		}
		recIn.width = 15;
		recIn.height = 15;
		DrawCross(recIn,color,Chid,true);
		osd_flag = 1;
	}

}

void CProcess::DrawCircle(Mat frame, cv::Point center, int radius, int colour, int thickness)
{
	CvScalar colour1=GetcvColour(colour);
	cv::circle(frame, center, radius ,colour1, thickness, 8, 0);
}

void CProcess::DrawMtdYellowGrid()
{
	unsigned int drawmtdgridId; 
	static int flag = 0;
	
	if(m_display.g_CurDisplayMode == MAIN_VIEW)
	{			
		drawmtdgridId = 1;
	}
	else
	{
		drawmtdgridId = extInCtrl->SensorStat;
	}
	
	Osd_cvPoint start, end;
	int interval_w = gun_resolu[0] / GRID_CNT_X;
	int interval_h = gun_resolu[1] / GRID_CNT_Y;


	if(flag)
	{
		for(int i = 1; i < GRID_CNT_X; i++)
		{
			start.x = interval_w * i;
			start.y = 0;
			end.x = interval_w * i;
			end.y = gun_resolu[1];
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,0,1);
		}
		for(int j = 1; j < GRID_CNT_Y; j++)
		{
			start.x = 0;
			start.y = interval_h * j;
			end.x = gun_resolu[0];
			end.y = interval_h * j;
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,0,1);
		}
		flag = 0;
	}


	if(setrigion_flagv20)
	{
		for(int i = 1; i < GRID_CNT_X; i++)
		{
			start.x = interval_w * i;
			start.y = 0;
			end.x = interval_w * i;
			end.y = gun_resolu[1];
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,4,1);
		}
		for(int j = 1; j < GRID_CNT_Y; j++)
		{
			start.x = 0;
			start.y = interval_h * j;
			end.x = gun_resolu[0];
			end.y = interval_h * j;
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,4,1);
		}
		flag = 1;
	}
}
void CProcess::DrawGridMap(int flag)
{
	int Image_Width = outputWHF[0];
	int Image_Height = outputWHF[1];

	unsigned int drawmtdgridId; 
	if(m_display.g_CurDisplayMode == MAIN_VIEW)
	{			
		drawmtdgridId = 1;
	}
	else
	{
		drawmtdgridId = extInCtrl->SensorStat;
	}
	
	Osd_cvPoint start, end;
	int interval_w =GRID_WIDTH;		
	int interval_h = GRID_HEIGHT;	
		
	for(int i = 0; i <= GRID_COLS; i++)
	{
		start.x = interval_w * i + GRID_WIDTH/2;
		start.y = GRID_HEIGHT/2;	
		end.x = interval_w * i + GRID_WIDTH/2;
		end.y = gun_resolu[1] - GRID_HEIGHT/2;
		if(flag)
		{
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,4,1);
		}
		else
		{
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,0,1);
		}
	}
	for(int j = 0; j <= GRID_ROWS; j++)
	{
		start.x = GRID_WIDTH/2;	
		start.y = interval_h * j + GRID_HEIGHT/2;
		end.x = gun_resolu[0] - GRID_WIDTH/2;
		end.y = interval_h * j + GRID_HEIGHT/2;
		if(flag)
		{
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,4,1);
		}
		else
		{
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,0,1);
		}
	}
}

void CProcess::DrawGridMap_16X12(int flag)
{
	int Image_Width = outputWHF[0];
	int Image_Height = outputWHF[1];
	int grid16X12_width = (int)((float)(Image_Width/16)); // when resolution==1920 X 1080,  grid16X12_width = 120 pixels
	int grid16X12_height = (int)((float)(Image_Height/12));
	unsigned int drawmtdgridId; 
	int col_offset = (Image_Width - (GRID_COLS_15*grid16X12_width) ) / 2;
	int row_offset =  (Image_Height - (GRID_ROWS_11*grid16X12_height)) / 2;
	
	int temp_col = col_offset;

	if(m_display.g_CurDisplayMode == MAIN_VIEW)
	{			
		drawmtdgridId = 1;
	}
	else
	{
		drawmtdgridId = extInCtrl->SensorStat;
	}
	
	Osd_cvPoint start, end;
	int interval_w =grid16X12_width;		
	int interval_h = grid16X12_height;	
		
	for(int i = 0; i <= GRID_COLS_15+2; i++)
	{
		start.x = m_gridNodes[0][i].coord_x;
		start.y = grid16X12_height/2;	
		end.x =  m_gridNodes[0][i].coord_x;
		end.y = gun_resolu[1] - grid16X12_height/2;

		if(flag)
		{
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,4,1);
		}
		else
		{
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,0,1);
		}
	}
	for(int j = 0; j <= GRID_ROWS_11; j++)
	{
		start.x = grid16X12_width/2;	
		start.y = interval_h * j + grid16X12_height/2;
		end.x = gun_resolu[0] - grid16X12_width/2;
		end.y = interval_h * j + grid16X12_height/2;
		if(flag)
		{
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,4,1);
		}
		else
		{
			DrawcvLine(m_display.m_imgOsd[drawmtdgridId],&start,&end,0,1);
		}
	}
}

void CProcess::DrawGridMapNodeCircles(bool drawFlag)
{
	int radius = 8;
	int thickness = 1;
	int mark_thickness = 3;
	cv::Point tmp_mark = cv::Point(60,45);
	if(drawFlag)
	{
		for(int i=0;i<=GRID_ROWS;i++)
		{
			for(int j=0;j<=GRID_COLS;j++)
			{				
				cv::circle(m_display.m_imgOsd[1],m_nodePos[i][j],radius ,cvScalar(0,0,0,0),thickness,8,0);
				//m_backNodePos = m_nodePos[i][j];
				m_nodePos[i][j].x = m_gridNodes[i][j].coord_x;
				m_nodePos[i][j].y = m_gridNodes[i][j].coord_y;
				cv::circle(m_display.m_imgOsd[1],m_nodePos[i][j],radius ,cvScalar(255,0,0,255),thickness,8,0);

				if(m_calibratedNodes[i][j].isShow == true)
				{
					tmp_mark.x = m_calibratedNodes[i][j].x;
					tmp_mark.y = m_calibratedNodes[i][j].y;
					cv::circle(m_display.m_imgOsd[1],tmp_mark,radius ,cvScalar(0,0,0,0),mark_thickness,8,0);
					
					//m_calibratedNodes[i][j].x = m_gridNodes[i][j].coord_x;
					//m_nodePos[i][j].y = m_gridNodes[i][j].coord_y;
					cv::circle(m_display.m_imgOsd[1],tmp_mark,radius ,cvScalar(0,232,160,255),mark_thickness,8,0);
					addMarkNum();
				}
			}
		}
	}
	else
	{
		for(int i=0;i<=GRID_ROWS;i++)
		{
			for(int j=0;j<=GRID_COLS;j++)
			{				
				tmp_mark.x = m_calibratedNodes[i][j].x;
				tmp_mark.y = m_calibratedNodes[i][j].y;
				cv::circle(m_display.m_imgOsd[1],m_nodePos[i][j],radius ,cvScalar(0,0,0,0),thickness,8,0);	
				cv::circle(m_display.m_imgOsd[1],tmp_mark,radius ,cvScalar(0,0,0,0),mark_thickness,8,0);
			}
		}

	}
}

void CProcess::DrawGridMapNodeCircles_16X12(bool drawFlag)
{
	int radius = 8;
	int thickness = 1;
	int mark_thickness = 2;
	int mark_radius = 14;
	cv::Point tmp_mark=cv::Point(60,45);
	if(drawFlag)
	{
		for(int i=0;i<=GRID_ROWS_11;i++)
		{
			for(int j=0;j<=GRID_COLS_15+1;j++)
			{		
			#if 0
				cv::circle(m_display.m_imgOsd[1],m_nodePos[i][j],radius ,cvScalar(0,0,0,0),thickness,8,0);
				
				m_nodePos[i][j].x = m_gridNodes[i][j].coord_x;
				m_nodePos[i][j].y = m_gridNodes[i][j].coord_y;
				cv::circle(m_display.m_imgOsd[1],m_nodePos[i][j],radius ,cvScalar(255,0,0,255),thickness,8,0);
			#endif
			
				if(m_calibratedNodes[i][j].isShow == true)
				{	
					if(m_gridNodes[i][j].renderFlag== true){
						cv::circle(m_display.m_imgOsd[1],m_nodePos[i][j],radius ,cvScalar(0,0,0,0),thickness,8,0);
						m_gridNodes[i][j].renderFlag= false;
					}
					tmp_mark.x = m_calibratedNodes[i][j].x;
					tmp_mark.y = m_calibratedNodes[i][j].y;

					cv::circle(m_display.m_imgOsd[1],tmp_mark,mark_radius ,cvScalar(0,0,0,0),mark_thickness,8,0);
					cv::circle(m_display.m_imgOsd[1],tmp_mark,mark_radius ,cvScalar(0,232,160,255),mark_thickness,8,0);

				}
				else
				{
					cv::circle(m_display.m_imgOsd[1],m_nodePos[i][j],radius ,cvScalar(0,0,0,0),thickness,8,0);
				
					m_nodePos[i][j].x = m_gridNodes[i][j].coord_x;
					m_nodePos[i][j].y = m_gridNodes[i][j].coord_y;
					cv::circle(m_display.m_imgOsd[1],m_nodePos[i][j],radius ,cvScalar(255,0,0,255),thickness,8,0);
					renderflag[2] = 0;

				}
			
			
			}
		}
	}
	else
	{
		for(int i=0;i<=GRID_ROWS_11;i++)
		{
			for(int j=0;j<=GRID_COLS_15+1;j++)
			{				
				tmp_mark.x = m_calibratedNodes[i][j].x;
				tmp_mark.y = m_calibratedNodes[i][j].y;
				cv::circle(m_display.m_imgOsd[1],m_nodePos[i][j],radius ,cvScalar(0,0,0,0),thickness,8,0);	
				cv::circle(m_display.m_imgOsd[1],tmp_mark,mark_radius ,cvScalar(0,0,0,0),mark_thickness,8,0);
			}
		}

	}
}

void CProcess::DrawGridMapNodeCircles(bool drawFlag, int drawNodesCount)
{
	int radius = 8;
	int thickness = -1;
	int tmp_row = drawNodesCount / (GRID_COLS+1);
	int tmp_col = drawNodesCount % (GRID_COLS+1);
	
	if(drawFlag)
	{						
		cv::circle(m_display.m_imgOsd[1],m_backNodePos,radius ,cvScalar(0,0,0,0),thickness,8,0);
		m_backNodePos = m_nodePos[tmp_row][tmp_col];
		if(exposure_star == 1){
			cv::circle(m_display.m_imgOsd[1],m_backNodePos,radius ,cvScalar(255,0,0,255),thickness,8,0);	
		}
	}
	else
	{		
		cv::circle(m_display.m_imgOsd[1],m_backNodePos,radius ,cvScalar(0,0,0,0),thickness,8,0);			
	}
}

void CProcess::DrawSelectedCircle(bool drawFlag, int drawNodesCount)
{
	int radius =18;// 8;
	int thickness = 3;		// circle Fan==  -1, 
	int tmp_row = drawNodesCount / (sizeof(m_gridNodes)-1);
	int tmp_col = drawNodesCount % (sizeof(m_gridNodes)-1);
	
	if(drawFlag)
	{						
		cv::circle(m_display.m_imgOsd[1],m_backNodePos,radius ,cvScalar(0,0,0,0),thickness,8,0);
		m_backNodePos = m_nodePos[tmp_row][tmp_col];
		if(exposure_star == 1){
			cv::circle(m_display.m_imgOsd[1],m_backNodePos,radius ,cvScalar(0,0,255,255),thickness,8,0);	
		}
		m_gridNodes[tmp_row][tmp_col].isCircle = false;
		//m_gridNodes[tmp_row][tmp_col].renderFlag= false;
	}
	else
	{		
		cv::circle(m_display.m_imgOsd[1],m_backNodePos,radius ,cvScalar(0,0,0,0),thickness,8,0);			
	}
}


void CProcess::DrawMtdRedGrid()
{
	unsigned int drawmtdgridRectId; 
	cv::Rect tmp;
	static int flag = 0;

	int interval_w = gun_resolu[0] / GRID_CNT_X;
	int interval_h = gun_resolu[1] / GRID_CNT_Y;
	
	if(m_display.g_CurDisplayMode == MAIN_VIEW)
	{			
		drawmtdgridRectId = 1;
	}
	else
	{
		drawmtdgridRectId = extInCtrl->SensorStat;
	}

	if(flag)
	{
		for(int i = 0; i < GRID_CNT_X; i++)
			for(int j = 0; j < GRID_CNT_Y; j++)
			{
				if(grid19x10_bak[i][j].state)
				{
					tmp.x = (i) * interval_w;
					tmp.y = (j) * interval_h;
					tmp.width = interval_w;
					tmp.height = interval_h;
					
					rectangle(m_display.m_imgOsd[drawmtdgridRectId],Point(tmp.x,tmp.y),Point(tmp.x+tmp.width,tmp.y+tmp.height),cvScalar(0,0,0,0),3,8);
				}
			}
			
		flag = 0;
	}

	if(setrigion_flagv20)
	{
		memcpy(grid19x10_bak, grid19x10, sizeof(grid19x10_bak));
		
		for(int i = 0; i < GRID_CNT_X; i++)
			for(int j = 0; j < GRID_CNT_Y; j++)
			{
				if(grid19x10_bak[i][j].state)
				{
					tmp.x = (i) * interval_w;
					tmp.y = (j) * interval_h;
					tmp.width = interval_w;
					tmp.height = interval_h;
					rectangle(m_display.m_imgOsd[drawmtdgridRectId],Point(tmp.x,tmp.y),Point(tmp.x+tmp.width,tmp.y+tmp.height),cvScalar(0,0,255,255),3,8);
				}
			}

		flag = 1;
	}
}

void CProcess::DrawMtd_Rigion_Target()
{
	unsigned int mtd_warningbox_Id;
	Osd_cvPoint startwarnpoly,endwarnpoly;
	int polwarn_flag = 0;
	static int flag = 0;
	int color = 0;;
	
	if(m_display.g_CurDisplayMode == MAIN_VIEW)
	{			
		mtd_warningbox_Id = 1;
	}
	else
	{
		mtd_warningbox_Id = extInCtrl->SensorStat;
	}
			
	if(flag)
	{
		color = 0;
		int cnt = edge_contours_bak.size() > MAX_MTDRIGION_NUM ? MAX_MTDRIGION_NUM : edge_contours_bak.size();
		for(int i = 0; i < cnt; i++)
			for(int j = 0; j < edge_contours_bak[i].size(); j++)
			{
				polwarn_flag = (j+1)%edge_contours_bak[i].size();
				startwarnpoly.x = edge_contours_bak[i][j].x;
				startwarnpoly.y = edge_contours_bak[i][j].y;
				endwarnpoly.x = edge_contours_bak[i][polwarn_flag].x;
				endwarnpoly.y = edge_contours_bak[i][polwarn_flag].y;
				DrawcvLine(m_display.m_imgOsd[mtd_warningbox_Id],&startwarnpoly,&endwarnpoly,color,3);
			}

		cv::Rect tmp;
		mouserect recttmp;
		for(std::vector<TRK_INFO_APP>::iterator plist = mvListsum.begin(); plist != mvListsum.end(); ++plist)
		{					
				recttmp.x = (*plist).trkobj.targetRect.x;
				recttmp.y = (*plist).trkobj.targetRect.y;
				recttmp.w = (*plist).trkobj.targetRect.width;
				recttmp.h = (*plist).trkobj.targetRect.height;
				recttmp = mapfullscreen2gunv20(recttmp);
				tmp.x = recttmp.x;
				tmp.y = recttmp.y;
				tmp.width = recttmp.w;
				tmp.height = recttmp.h;
				DrawRect(m_display.m_imgOsd[mtd_warningbox_Id], tmp ,color);
		}
		
		{
			recttmp.x = cur_targetRect_bak.x;
			recttmp.y = cur_targetRect_bak.y;
			recttmp.w = cur_targetRect_bak.width;
			recttmp.h = cur_targetRect_bak.height;
			recttmp = mapfullscreen2gunv20(recttmp);
			tmp.x = recttmp.x;
			tmp.y = recttmp.y;
			tmp.width = recttmp.w;
			tmp.height = recttmp.h;
			DrawRect(m_display.m_imgOsd[mtd_warningbox_Id], tmp ,color);	
		}		
		flag = 0;
	}

	if(m_bMoveDetect)
	{
		color = 5;
		edge_contours_bak = edge_contours;
		int cnt = edge_contours_bak.size() > MAX_MTDRIGION_NUM ? MAX_MTDRIGION_NUM : edge_contours_bak.size();
		for(int i = 0; i < cnt; i++)
			for(int j = 0; j < edge_contours_bak[i].size(); j++)
			{
				polwarn_flag = (j+1)%edge_contours_bak[i].size();
				startwarnpoly.x = edge_contours_bak[i][j].x;
				startwarnpoly.y = edge_contours_bak[i][j].y;
				endwarnpoly.x = edge_contours_bak[i][polwarn_flag].x;
				endwarnpoly.y = edge_contours_bak[i][polwarn_flag].y;
				DrawcvLine(m_display.m_imgOsd[mtd_warningbox_Id],&startwarnpoly,&endwarnpoly,color,3);
			}

		detect_vect_arr_bak = detect_vect_arr;
		mvListsum.clear();
		int num = 0;
		for(int i = 0; i < detect_vect_arr_bak.size(); i++)
		{
			getTargetNearToCenter(&detect_vect_arr_bak[i]);
			mvIndexHandle(&mvList_arr[i],detect_vect_arr_bak[i],detectNum);
			/*
			for(int j = 0; j < mvList_arr[i].size(); j++)
			{
				if(num < detectNum)
					mvListsum.push_back(mvList_arr[i][j]);
				num++;
			}*/
			if(i == 0)
				mvListsum = mvList_arr[i];
		}

		if(forwardflag)
		{
			switchMvTargetForwad();
			forwardflag = 0;
		}
		else if(backflag)
		{
			switchMvTargetForwad();
			backflag = 0;
		}
		char tmpNum = 0;
		cv::Rect tmp;
		mouserect recttmp;
		tmpNum = 0;
		for(std::vector<TRK_INFO_APP>::iterator plist = mvListsum.begin(); plist != mvListsum.end(); ++plist)
		{	
			color = 3;

			recttmp.x = (*plist).trkobj.targetRect.x;
			recttmp.y = (*plist).trkobj.targetRect.y;
			recttmp.w = (*plist).trkobj.targetRect.width;
			recttmp.h = (*plist).trkobj.targetRect.height;
			recttmp = mapfullscreen2gunv20(recttmp);
			tmp.x = recttmp.x;
			tmp.y = recttmp.y;
			tmp.width = recttmp.w;
			tmp.height = recttmp.h;

			DrawRect(m_display.m_imgOsd[mtd_warningbox_Id], tmp ,color);
		}
		if((0 == lose_timer_flag) && (mvListsum.size() > 0) && cur_targetRect.width && cur_targetRect.height )
		{
			Point2i inPoint, outPoint;
			color = 6;
			cur_targetRect_bak = cur_targetRect;
			recttmp.x = cur_targetRect_bak.x;
			recttmp.y = cur_targetRect_bak.y;
			recttmp.w = cur_targetRect_bak.width;
			recttmp.h = cur_targetRect_bak.height;
			inPoint.x = recttmp.x + recttmp.w / 2;
			inPoint.y = recttmp.y + recttmp.h / 2;
			recttmp = mapfullscreen2gunv20(recttmp);
			tmp.x = recttmp.x;
			tmp.y = recttmp.y;
			tmp.width = recttmp.w;
			tmp.height = recttmp.h;

			if(1 == g_GridMapMode)
			{
				//printf("\r\nImage Points:<%d, %d>\r\n", tmp.x+tmp.width/2, tmp.y+tmp.height/2);
				pThis->MvBallCamUseLinearDeviationSelectRect(tmp.x+tmp.width/2, tmp.y + tmp.height/2, false);				
				//pThis->AutoLinkMoveBallCamera(tmp.x+tmp.width/2,tmp.y + tmp.height/2, GRID_WIDTH_120,GRID_HEIGHT_90,false);

			}
			else if(2 == g_GridMapMode)
			{
				m_autofr.Point2getPos(inPoint, outPoint);
				printf("%s, %d,aurolinkage inPoint(%d,%d),outPos(%d,%d)\n", __FILE__,__LINE__,inPoint.x,inPoint.y,outPoint.x,outPoint.y);
				trkmsg.cmd_ID = speedloop;
				memcpy(&trkmsg.param[0],&(outPoint.x), sizeof(int));
				memcpy(&trkmsg.param[4],&(outPoint.y), sizeof(int)); 
				ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
			}
				
			DrawRect(m_display.m_imgOsd[mtd_warningbox_Id], tmp ,color);
		}
		
		flag = 1;
	}
}
 
static inline void my_rotate(GLfloat result[16], float theta)
{
	float rads = float(theta/180.0f) * CV_PI;
	const float c = cosf(rads);
	const float s = sinf(rads);

	memset(result, 0, sizeof(GLfloat)*16);

	result[0] = c;
	result[1] = -s;
	result[4] = s;
	result[5] = c;
	result[10] = 1.0f;
	result[15] = 1.0f;
}



void CProcess::manualHandleKeyPoints(int &x,int &y)
{
	int offset_x = 0;
	int point_X , point_Y;
	float f_x = x;//(float)x;
	float f_y = y;//(float)y;
	
	switch(m_display.g_CurDisplayMode) {
		case PREVIEW_MODE:
			offset_x = 960;
			break;
		/*
		case LEFT_BALL_RIGHT_GUN:
			offset_x = 480;	
			break;
		*/
		default:
			break;
	}
	
	circle_point = Point(x,y);
	
	if( x <= offset_x ){
		m_camCalibra->key_points1.push_back(cv::Point2f(f_x,f_y));
		key1_pos = Point(x,y);
		textPos1_record[key_point1_cnt] = key1_pos;
		key_point1_cnt++;	
		string_cnt1 ++;
	}
	else{
		m_camCalibra->key_points2.push_back( cv::Point2f( f_x -offset_x, f_y) );
		key2_pos = Point(x,y);
		textPos2_record[key_point2_cnt] = key2_pos;
		key_point2_cnt ++;	
		string_cnt2++;
	}
	
	for (std::vector<cv::Point2f>::const_iterator itPnt = m_camCalibra->key_points1.begin();
			itPnt != m_camCalibra->key_points1.end(); ++itPnt){
		cout<< "*itPnt.x = " <<(*itPnt).x<< "\t*itPnt.y = " << (*itPnt).y << endl;
	}

	for (std::vector<cv::Point2f>::const_iterator itPnt2 = m_camCalibra->key_points2.begin();
			itPnt2 != m_camCalibra->key_points2.end(); ++itPnt2){
		cout<< "*itPnt2.x = " <<(*itPnt2).x<< "\t*itPnt2.y = " << (*itPnt2).y << endl;
	}
	
}

Point CProcess::replaceClickPoints(int pointX, int pointY)
{
	int inputX = pointX;
	int inputY = pointY;
	Point replacePoint = Point(0,0);

	if(  350<= inputX && inputX <= 400) {
		inputX = inputX - inputX*0.07 ;
	}
	else if( 400< inputX && inputX <= 450) {
		inputX = inputX - inputX*0.04 ;
	}
	else if( 450< inputX && inputX <= 500) {
		inputX = inputX - inputX*0.02 ;
	}
	else{

	}

	if( 800 < inputY && inputY <= 850) {
		inputY =  inputY - inputY * 0.037;
	}
	else if( 850 < inputY && inputY <= 900) {
		inputY =  inputY - inputY * 0.066;
	}
	else if( 950 < inputY && inputY <= 1000) {
		inputY =  inputY - inputY * 0.085;
	}
	else if( 1050 < inputY && inputY <= 1100) {
		inputY =  inputY - inputY * 0.091;
	}
	else{

	}

	replacePoint = Point(inputX, inputY);
	return replacePoint;
	
}
int CProcess::checkZoomPosNewTable(int delta)
{
	int Delta_X = delta;
	int setZoom = 2849 ;
	
	if(Delta_X >= 960){
		setZoom = 2849;
	}
	else if( 420 <= Delta_X && Delta_X<960){		
		setZoom = 2849;
	}
	else if(320 <= Delta_X && Delta_X < 420){ 
		setZoom = 2849;//6268;
	}
	else if(240 <= Delta_X && Delta_X <320){
		setZoom = 2849;//9117;
	}
	else if(200 <= Delta_X && Delta_X <240){
		setZoom = 2849;//11967;
	}
	else if(170 <= Delta_X && Delta_X <200){
		setZoom = 2849;//15101;
	}
	else  if(145 <= Delta_X && Delta_X <170){
		setZoom = 2849;//18520;
	}
	else  if(140 <= Delta_X && Delta_X <145){
		setZoom = 6268;//21058;
	}
	else  if(112 <= Delta_X && Delta_X <140){
		setZoom = 9117;//24504;
	}
	else  if(104 <= Delta_X && Delta_X <112){
		setZoom = 11967;//28208;
	}
	else  if(96 <= Delta_X && Delta_X <104){
		setZoom = 15101;//33330;
	}
	else  if(90 <= Delta_X && Delta_X <96){
		setZoom = 18520;//36750;
	}
	else  if(84 <= Delta_X && Delta_X <90){
		setZoom = 21058;//39320;
	}
	else  if(76 <= Delta_X && Delta_X <84){
		setZoom = 24504;//43870;
	}
	else  if(68 <= Delta_X && Delta_X <76){
		setZoom = 28208;//46440;
	}
	else  if(62 <= Delta_X && Delta_X <68){
		setZoom = 33330;//49230;
	}
	else  if(56<= Delta_X && Delta_X <62 ){
		setZoom = 36750;//52265;
	}
	else  if(50 <= Delta_X && Delta_X < 56){
		setZoom = 39320;//55560;
	}
	else  if(44 <= Delta_X && Delta_X <50){
		setZoom = 43870;//58520;
	}
	else  if(38 <= Delta_X && Delta_X < 44){
		setZoom = 46440;//61240;
	}
	else  if(32 <= Delta_X && Delta_X < 38){
		setZoom = 49230;//63890;
	}
	else  if(0 <= Delta_X && Delta_X <32){
		setZoom = 52265;//65535;
	}
	return setZoom;


}
int CProcess::checkZoomPosTable(int delta)
{
	int Delta_X = delta;
	int setZoom = 2849 ;
	#if 0
	if( 420 < Delta_X && Delta_X<960){		
		setZoom = 6800;
	}
	else if(320 < Delta_X ){ 
		setZoom = 9400;
	}
	else if(240 < Delta_X ){
		setZoom = 12530;
	}
	else if(200 < Delta_X ){
		setZoom = 15100;
	}
	else if(170 < Delta_X){
		setZoom = 19370;
	}
	else  if(145 < Delta_X ){
		setZoom = 20800;
	}
	else  if(140 < Delta_X ){
		setZoom = 23336;
	}
	else  if(112 < Delta_X ){
		setZoom = 26780;
	}
	else  if(104 < Delta_X ){
		setZoom = 29916;
	}
	else  if(96 < Delta_X ){
		setZoom = 33330;
	}
	else  if(90 < Delta_X ){
		setZoom = 36750;
	}
	else  if(84 < Delta_X){
		setZoom = 39320;
	}
	else  if(76 < Delta_X ){
		setZoom = 43870;
	}
	else  if(68 < Delta_X ){
		setZoom = 46440;
	}
	else  if(62 < Delta_X ){
		setZoom = 49230;
	}
	else  if(56< Delta_X ){
		setZoom = 52265;
	}
	else  if(50 < Delta_X ){
		setZoom = 55560;
	}
	else  if(44 < Delta_X){
		setZoom = 58520;
	}
	else  if(38 < Delta_X ){
		setZoom = 61240;
	}
	else  if(32 < Delta_X){
		setZoom = 63890;
	}
	else  if(26 < Delta_X ){
		setZoom = 65535;
	}
#endif
	if(Delta_X >= 960){
		setZoom = 2849;
	}
	else if( 420 <= Delta_X && Delta_X<960){		
		setZoom = 2849;
	}
	else if(320 <= Delta_X && Delta_X < 420){ 
		setZoom = 6268;
	}
	else if(240 <= Delta_X && Delta_X <320){
		setZoom = 9117;
	}
	else if(200 <= Delta_X && Delta_X <240){
		setZoom = 11967;
	}
	else if(170 <= Delta_X && Delta_X <200){
		setZoom = 15101;
	}
	else  if(145 <= Delta_X && Delta_X <170){
		setZoom = 18520;
	}
	else  if(140 <= Delta_X && Delta_X <145){
		setZoom = 21058;
	}
	else  if(112 <= Delta_X && Delta_X <140){
		setZoom = 24504;
	}
	else  if(104 <= Delta_X && Delta_X <112){
		setZoom = 28208;
	}
	else  if(96 <= Delta_X && Delta_X <104){
		setZoom = 33330;
	}
	else  if(90 <= Delta_X && Delta_X <96){
		setZoom = 36750;
	}
	else  if(84 <= Delta_X && Delta_X <90){
		setZoom = 39320;
	}
	else  if(76 <= Delta_X && Delta_X <84){
		setZoom = 43870;
	}
	else  if(68 <= Delta_X && Delta_X <76){
		setZoom = 46440;
	}
	else  if(62 <= Delta_X && Delta_X <68){
		setZoom = 49230;
	}
	else  if(56<= Delta_X && Delta_X <62 ){
		setZoom = 52265;
	}
	else  if(50 <= Delta_X && Delta_X < 56){
		setZoom = 55560;
	}
	else  if(44 <= Delta_X && Delta_X <50){
		setZoom = 58520;
	}
	else  if(38 <= Delta_X && Delta_X < 44){
		setZoom = 61240;
	}
	else  if(32 <= Delta_X && Delta_X < 38){
		setZoom = 63890;
	}
	else  if(0 <= Delta_X && Delta_X <32){
		setZoom = 65535;
	}	
	return setZoom;
}

void CProcess::Set_K_ByNewDeltaX(int delta_x)
{
	int  tmpcofx   ;		
	int  tmpcofy    ;
	int Delta_X = delta_x;
	
	if(Delta_X >= 960){
		 tmpcofx =6320 ;
		 tmpcofy =6200 ;
	}
	else if(  Delta_X >= 420  && Delta_X < 960){
		 tmpcofx =6320 ;
		 tmpcofy =6200 ;
	}
	else if(Delta_X >= 320 && Delta_X <420) { 
		 tmpcofx =6320 ;
		 tmpcofy =6200 ;
	}
	else if(240 <= Delta_X && Delta_X <320){
		 tmpcofx =6320 ;
		 tmpcofy =6200 ;
	}
	else if(200 <= Delta_X && Delta_X <240){
		 tmpcofx =6320 ;
		 tmpcofy =6200 ;
	}
	else if(170 <= Delta_X && Delta_X <200 ){
		 tmpcofx =6320 ;
		 tmpcofy =6200 ;
	}
	else  if(145 <= Delta_X && Delta_X < 170){
		 tmpcofx =6320 ;
		 tmpcofy =6200 ;
	}
	else  if(140 <= Delta_X && Delta_X <145){
		 tmpcofx =3300 ;
		 tmpcofy =3400 ;
	}
	else  if(112 < Delta_X && Delta_X <140){
		 tmpcofx =2400 ;
		 tmpcofy =2400 ;
	}
	else  if(104 <= Delta_X && Delta_X <112){
		 tmpcofx =1850 ;
		 tmpcofy =1850 ;
	}
	else  if(96 <= Delta_X && Delta_X <104){
		 tmpcofx =1500 ;
		 tmpcofy =1500 ;
	}
	else  if(90 <= Delta_X && Delta_X <96){
		 tmpcofx =1350 ;
		 tmpcofy =1360 ;
	}
	else  if(84 <= Delta_X && Delta_X <90){
		 tmpcofx =1160 ;
		 tmpcofy =1230 ;
	}
	else  if(76 <= Delta_X && Delta_X <84){
		 tmpcofx =1000 ;
		 tmpcofy =1020 ;
	}
	else  if(68 <= Delta_X && Delta_X <76){
		 tmpcofx =900 ;
		 tmpcofy =920 ;
	}
	else  if(62 <= Delta_X && Delta_X < 68){
		 tmpcofx =820 ;
		 tmpcofy =830 ;
	}
	else  if(56<= Delta_X && Delta_X < 62){
		 tmpcofx =760 ;
		 tmpcofy =750 ;
	}
	else  if(0 <= Delta_X && Delta_X < 56){
		 tmpcofx =700 ;
		 tmpcofy =710 ;
	}	
	m_cofx = tmpcofx;
	m_cofy = tmpcofy;
	return ;
}

void CProcess::Set_K_ByZoom(int Current_Zoom)
{
	int  tmpcofx   ;		
	int  tmpcofy    ;
	int zoom = Current_Zoom;
	if(2849<=zoom && zoom <6268) {
		tmpcofx = 6320;
		tmpcofy = 6200;
	}
	else if(6268 <=zoom && zoom <9117 ) {
		tmpcofx = 3300;
		tmpcofy = 3400;
	}
	else if(9117 <=zoom && zoom <11967 ) {
		tmpcofx = 2400;
		tmpcofy = 2400;
	}
	else if(11967 <=zoom && zoom <15101 ) {
		tmpcofx = 1850;
		tmpcofy = 1880;
	}
	else if(15101 <=zoom && zoom <18520 ) {
		tmpcofx = 1500;
		tmpcofy = 1540;
	}
	else if(18520 <=zoom && zoom <21058 ) {
		tmpcofx = 1350;
		tmpcofy = 1360;
	}
	else if(21058 <=zoom && zoom <24504 ) {
		tmpcofx = 1160;
		tmpcofy = 1230;
	}
	else if(24504 <=zoom && zoom <28208 ) {
		tmpcofx = 1000;
		tmpcofy = 1020;
	}
	else if(28208 <=zoom && zoom <33330 ) {
		tmpcofx = 900;
		tmpcofy = 920;
	}
	else if(33330 <=zoom && zoom <36750 ) {
		tmpcofx = 820;
		tmpcofy = 830;
	}
	else if(36750 <=zoom && zoom <39320 ) {
		tmpcofx = 760;
		tmpcofy = 750;
	}
	else if(39320 <=zoom && zoom <43870 ) {
		tmpcofx = 700;
		tmpcofy = 710;
	}
	else if(43870 <=zoom && zoom <46440 ) {
		tmpcofx = 670;
		tmpcofy = 680;
	}
	else if(46440 <=zoom && zoom <49230 ) {
		tmpcofx = 650;
		tmpcofy = 660;
	}
	else if(49230 <=zoom && zoom <52265 ) {
		tmpcofx = 630;
		tmpcofy = 635;
	}
	else if(52265 <=zoom && zoom <55560 ) {
		tmpcofx = 620;
		tmpcofy = 620;
	}
	else if(55560 <=zoom && zoom < 58535 ) {
		tmpcofx = 600;
		tmpcofy = 610;
	}
	else if(58535 <=zoom && zoom < 61535) {
		tmpcofx = 580;
		tmpcofy = 590;
	}
	else if(61535 <=zoom && zoom <= 65535) {
		tmpcofx = 560;
		tmpcofy = 560;
	}	
	else{
	}
	m_cofx = tmpcofx;
	m_cofy = tmpcofy;	
	return ;
}
void CProcess::Set_K_ByDeltaX( int delta_x)
{
	int  tmpcofx   ;		
	int  tmpcofy    ;
	int Delta_X = delta_x;
	if(Delta_X >= 960){
		 tmpcofx =6320 ;
		 tmpcofy =6200 ;
	}
	else if(  Delta_X >= 420  && Delta_X < 960){
		 tmpcofx =6320 ;
		 tmpcofy =6200 ;
	}
	else if(Delta_X >= 320 && Delta_X <420) { 
		 tmpcofx =3300 ;
		 tmpcofy =3400 ;
	}
	else if(240 <= Delta_X && Delta_X <320){
		 tmpcofx =2400 ;
		 tmpcofy =2400 ;
	}
	else if(200 <= Delta_X && Delta_X <240){
		 tmpcofx =1850 ;
		 tmpcofy =1880 ;
	}
	else if(170 <= Delta_X && Delta_X <200 ){
		 tmpcofx =1500 ;
		 tmpcofy =1540 ;
	}
	else  if(145 <= Delta_X && Delta_X < 170){
		 tmpcofx =1350 ;
		 tmpcofy =1360 ;
	}
	else  if(140 <= Delta_X && Delta_X <145){
		 tmpcofx =1160 ;
		 tmpcofy =1230 ;
	}
	else  if(112 < Delta_X && Delta_X <140){
		 tmpcofx =1000 ;
		 tmpcofy =1020 ;
	}
	else  if(104 <= Delta_X && Delta_X <112){
		 tmpcofx =900 ;
		 tmpcofy =920 ;
	}
	else  if(96 <= Delta_X && Delta_X <104){
		 tmpcofx =820 ;
		 tmpcofy =830 ;
	}
	else  if(90 <= Delta_X && Delta_X <96){
		 tmpcofx =760 ;
		 tmpcofy =750 ;
	}
	else  if(84 <= Delta_X && Delta_X <90){
		 tmpcofx =700 ;
		 tmpcofy =710 ;
	}
	else  if(76 <= Delta_X && Delta_X <84){
		 tmpcofx =670 ;
		 tmpcofy =680 ;
	}
	else  if(68 <= Delta_X && Delta_X <76){
		 tmpcofx =650 ;
		 tmpcofy =660 ;
	}
	else  if(62 <= Delta_X && Delta_X < 68){
		 tmpcofx =630 ;
		 tmpcofy =635 ;
	}
	else  if(56<= Delta_X && Delta_X < 62){
		 tmpcofx =620 ;
		 tmpcofy =620 ;
	}
	else  if(0 <= Delta_X && Delta_X < 56){
		 tmpcofx =600 ;
		 tmpcofy =610 ;
	}
	
	m_cofx = tmpcofx;
	m_cofy = tmpcofy;
	return ;
}

void CProcess::RefreshBallPTZ(int in_panPos, int in_tilPos, int in_zoom)
{
	panPos = in_panPos ;
	tiltPos = in_tilPos ;
	zoomPos = in_zoom ;
	return;
}

void CProcess::setBallPos(int in_panPos, int in_tilPos, int in_zoom)
{
	panPos = in_panPos ;
	tiltPos = in_tilPos ;
	zoomPos = in_zoom ;
	OSA_semSignal(&g_linkage_getPos);
	return;
}

void CProcess::refreshClickPoint(int x, int y)
{
	m_capX = x;
	m_capY = y;
	return ;
}

void CProcess::Test_Match_result(int x, int y)
{
	int offsetX = m_winWidth/2;
	int offsetY = 0;
	int gunImage_pointX = x- offsetX;
	int gunImage_pointY = y- offsetY;
	cv::Point2d gunImagePoint = cv::Point2d(gunImage_pointX*2, gunImage_pointY*2);
	cv::Point2d ballImagePoint;	
	cv::Point2d temp;	
	CvtImgPoint2Camera(gunImagePoint, ballImagePoint);
	int destX = ballImagePoint.x;
	int destY = ballImagePoint.y;
	setBallImagePoint(destX, destY);	
	return ;
}

void CProcess::MoveBall()
{
	static int static_cofx = 6320;
	static int static_cofy = 6200;
	int  offset_x , offset_y,ZoomPos; 
	int delta_X ;	
	int cur_Kx = 6320;
	int cur_Ky = 6200;
	 int DesPanPos = 0;
	 int DesTilPos =0;	

	int Origin_PanPos = panPos;
	int Origin_TilPos = tiltPos;

	int curPanPos = panPos;	
	int curTilPos = tiltPos;		

	//Set_K_ByDeltaX(m_iDelta_X);	
	//Set_K_ByNewDeltaX( m_iDelta_X );
	
	Set_K_ByZoom(zoomPos);
	
	cur_Kx = m_cofx;
	cur_Ky = m_cofy;

	int  inputX = m_capX;	
	int  inputY = m_capY;	
	int  tmpcofx = cur_Kx;		
	int  tmpcofy = cur_Ky;		

	inputX -= m_winWidth/4;   //480;
	inputY -= m_winHeight/4;   //270;	

	float coefficientx = (float)tmpcofx*0.001f;
	float coefficienty = (float)tmpcofy*0.001f;
	float tmpficientx = 1.0;
	inputX = (int)((float)inputX * coefficientx * tmpficientx);
	inputY = (int)((float)inputY * coefficienty);		
	
	SetDestPosScope(inputX, inputY, Origin_PanPos,Origin_TilPos,DesPanPos, DesTilPos);
	
	ZoomPos =m_iZoom; 	// zoomPos;

#if 0
	printf("\r\n===============Destination====================(2)\r\n");
	printf("DesPanPos = %d\r\nDesTilPos = %d\r\nZoomPos  = %d \r\n",DesPanPos,DesTilPos,ZoomPos);
	printf("\r\n==========================================\r\n");
#endif	
	trkmsg.cmd_ID = acqPosAndZoom;

	memcpy(&trkmsg.param[0],&DesPanPos, 4);
	memcpy(&trkmsg.param[4],&DesTilPos, 4); 	
	memcpy(&trkmsg.param[8],&zoomPos, 4);
	ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);

}
void CProcess::MvBallCamByClickBallIMg(int x, int y)
{
	static int static_cofx = 6320;
	static int static_cofy = 6200;
	int point_X , point_Y , offset_x , offset_y,ZoomPos; 
	int delta_X ;	
	int cur_Kx = 6320;
	int cur_Ky = 6200;	
	switch(m_display.g_CurDisplayMode) 
	{
		case PREVIEW_MODE:
			offset_x = 0;	
			offset_y = 0;
			break;
		case MAIN_VIEW:
			offset_x = m_winWidth/4;
			offset_y = 0;
			break;		
		default:
			break;
	}		
	point_X  = x - offset_x;
	point_Y  = y - offset_y;		
	int flag = 0;		
//----------Query Current Position -------------------
	refreshClickPoint(point_X, point_Y);
	setPTZflag(true);
	QueryCurBallCamPosition();		
}

void CProcess::SetDestPosScope(int &inputX, int &inputY, int &Origin_PanPos, int &Origin_TilPos,int &DesPanPos, int &DesTilPos)
{	
	if(inputX + Origin_PanPos < 0)
	{
		DesPanPos = 36000 + (inputX + Origin_PanPos);
	}
	else if(inputX + Origin_PanPos > 35999)
	{
		DesPanPos = inputX - (36000 - Origin_PanPos);
	}
	else{
		DesPanPos = Origin_PanPos + inputX;
	}

	if(Origin_TilPos > 32768)
	{
		if(inputY < 0)
		{			
			DesTilPos = Origin_TilPos - inputY ;
		}
		else
		{
			if(Origin_TilPos - inputY < 32769)
				DesTilPos = inputY - (Origin_TilPos - 32768);
			else
				DesTilPos = Origin_TilPos - inputY;
		}
	}
	else
	{
		if(inputY < 0)
		{
			if(Origin_TilPos + inputY < 0)
			{
				DesTilPos = -inputY - Origin_TilPos + 32768; 
			}
			else
				DesTilPos = Origin_TilPos + inputY;
		}
		else
		{
			DesTilPos = Origin_TilPos + inputY;
		}
	}

	if(DesTilPos > 20000)
	{
		if(DesTilPos > 32768 + 1900)
			DesTilPos = 32768 + 1900;
	}
	else
	{
		if(DesTilPos > 8900)
			DesTilPos = 8900;
	}
}
void CProcess::TransformPixByOriginPoints(int &X, int &Y)
{
	int DesPanPos, DesTilPos , tmp_zoomPos;	
	int  inputX = X ;
	int  inputY = Y ;
	
	int  tmpcofx = 6300;
	int  tmpcofy = 6200;
	float coefficientx = (float)tmpcofx*0.001f;
	float coefficienty = (float)tmpcofy*0.001f;

	inputX -= m_winWidth/4;	
	inputY -= m_winHeight/4;		

	inputX = (int)((float)inputX * coefficientx );
	inputY = (int)((float)inputY * coefficienty);
	int Origin_PanPos = g_camParams.panPos;
	int Origin_TilPos = g_camParams.tiltPos;

	SetDestPosScope(inputX, inputY, Origin_PanPos,Origin_TilPos,DesPanPos, DesTilPos);

	tmp_zoomPos = zoomPos;	//m_iZoom;

	//trkmsg.cmd_ID = speedloop;
	trkmsg.cmd_ID = acqPosAndZoom;
	memcpy(&trkmsg.param[0],&DesPanPos, 4);
	memcpy(&trkmsg.param[4],&DesTilPos, 4); 
	memcpy(&trkmsg.param[8],&tmp_zoomPos,4); 	
	
	ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);	
}

void CProcess::CvtImgPoint2Camera(cv::Point2d &imgCoords, cv::Point2d &camCoords)
{
	cv::Point2d opt = imgCoords;
	std::vector<cv::Point2d> distorted, normalizedUndistorted;
	distorted.push_back(cv::Point2d(opt.x, opt.y));
	undistortPoints(distorted,normalizedUndistorted,g_camParams.cameraMatrix_gun,g_camParams.distCoeffs_gun);
	std::vector<cv::Point3d> objectPoints;

	for (std::vector<cv::Point2d>::const_iterator itPnt = normalizedUndistorted.begin();
	itPnt != normalizedUndistorted.end(); ++itPnt)
	{
		objectPoints.push_back(cv::Point3d(itPnt->x, itPnt->y, 1));
	}
	std::vector<cv::Point2d> imagePoints(objectPoints.size());
	projectPoints(objectPoints, cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),g_camParams.cameraMatrix_ball,cv::Mat(),imagePoints);
	std::vector<cv::Point2d> ballImagePoints(imagePoints.size());
	perspectiveTransform(imagePoints, ballImagePoints, g_camParams.homography);
	std::vector<cv::Point2d>::iterator itp = imagePoints.begin();
	cv::Point2d pt = *itp;
	cv::Point2d upt( pt.x, pt.y );			
	itp = ballImagePoints.begin();
	pt = *itp;
	pt.x /= 2.0;	
 	pt.y /= 2.0;	 
	camCoords = cv::Point2d( pt.x, pt.y );
	return ;
}

void CProcess::CvtImgCoords2CamCoords(Point &imgCoords, Point &camCoords)
{
	Point opt = imgCoords;
	std::vector<cv::Point2d> distorted, normalizedUndistorted;
	distorted.push_back(cv::Point2d(opt.x, opt.y));
	undistortPoints(distorted,normalizedUndistorted,g_camParams.cameraMatrix_gun,g_camParams.distCoeffs_gun);
	std::vector<cv::Point3d> objectPoints;
	for (std::vector<cv::Point2d>::const_iterator itPnt = normalizedUndistorted.begin();
	itPnt != normalizedUndistorted.end(); ++itPnt)
	{
		objectPoints.push_back(cv::Point3d(itPnt->x, itPnt->y, 1));
	}
	std::vector<cv::Point2d> imagePoints(objectPoints.size());
	projectPoints(objectPoints, cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),g_camParams.cameraMatrix_ball,cv::Mat(),imagePoints);
	std::vector<cv::Point2d> ballImagePoints(imagePoints.size());
	perspectiveTransform(imagePoints, ballImagePoints, g_camParams.homography);
	std::vector<cv::Point2d>::iterator itp = imagePoints.begin();
	cv::Point2d pt = *itp;
	Point upt( pt.x, pt.y );			
	itp = ballImagePoints.begin();
	pt = *itp;
	pt.x /= 2.0;	
 	pt.y /= 2.0;	 
	camCoords = Point( pt.x, pt.y );
	return ;
}


void CProcess::MvBallCamByClickGunImg(int x, int y,bool needChangeZoom)
{
	int delta_X ;
	int offset_x = 0;
	int offset_y = m_winHeight/2;	
	int point_X = ( x-offset_x ) ;
	int point_Y = ( y-offset_y ) *2;	
	Point imgCoords = Point( point_X ,  point_Y );  
	//Point imgCoords = replaceClickPoints(point_X, point_Y);	
	Point camCoords;
	CvtImgCoords2CamCoords(imgCoords, camCoords);
	TransformPixByOriginPoints(camCoords.x, camCoords.y );
}

void CProcess::MvBallCamUseLinearDeviationSelectRect(int x, int y,bool needChangeZoom)
{
	int point_X , point_Y , offset_x , offset_y,tmp_zoomPos; 
	int delta_X ;
	bool isDeltaValid = false;
	Point opt;	
	switch(m_display.g_CurDisplayMode) 
	{
		case PREVIEW_MODE:	
			offset_x = m_winWidth/2;  
			offset_y = 0;
			break;
		case MAIN_VIEW:
			offset_x =0;
			offset_y = m_winHeight/2;   
			break;			
		default:
			break;
	}
	LeftPoint.x -= offset_x;
	RightPoint.x -=offset_x;
	LeftPoint.y -= offset_y;
	RightPoint.y -=offset_y;	
	delta_X = abs(LeftPoint.x - RightPoint.x) ;
	if(needChangeZoom == true ) 
	{
		if(delta_X < MIN_VALID_RECT_WIDTH_IN_PIXEL) 
		{
			;  // Do Nothing
		}
		else
		{
			isDeltaValid = true;
			tmp_zoomPos = checkZoomPosTable( delta_X );		
			m_iZoom = tmp_zoomPos;
			zoomPos = tmp_zoomPos;
		}
	}

	if(needChangeZoom == true)
	{
		if((LeftPoint.x < RightPoint.x)&&(LeftPoint.y < RightPoint.y)) 
		{
			point_X = abs(LeftPoint.x - RightPoint.x) /2 + LeftPoint.x;
			point_Y = abs(LeftPoint.y - RightPoint.y) /2 + LeftPoint.y;	
		}
		else if((LeftPoint.x < RightPoint.x)&&(LeftPoint.y > RightPoint.y)) 
		{
			point_X = abs(LeftPoint.x - RightPoint.x) /2 + LeftPoint.x;
			point_Y = abs(LeftPoint.y - RightPoint.y) /2 + RightPoint.y;	
		}
		else if((LeftPoint.x > RightPoint.x)&&(LeftPoint.y > RightPoint.y)) 
		{
			point_X = abs(LeftPoint.x - RightPoint.x) /2 + RightPoint.x;
			point_Y = abs(LeftPoint.y - RightPoint.y) /2 + RightPoint.y;	
		}
		else
		{
			point_X = abs(LeftPoint.x - RightPoint.x) /2 + RightPoint.x;
			point_Y = abs(LeftPoint.y - RightPoint.y) /2 + LeftPoint.y;	
		}		
	}
	else  {
		point_X = (x - offset_x);
		point_Y = (y- offset_y);		
	}
	switch(m_display.g_CurDisplayMode) 
	{
		case PREVIEW_MODE:
			opt = cv::Point( point_X*2, point_Y*2 );	
			break;
		case MAIN_VIEW:
			opt = cv::Point( point_X, point_Y * 2 );
			break;		
		default:
			break;
	}

	getLinearDeviationForSelectRect(opt.x, opt.y, GRID_WIDTH_120,GRID_HEIGHT_90,needChangeZoom);


}
void CProcess::MvBallCamBySelectRectangle(int x, int y,bool needChangeZoom)
{	
	int point_X , point_Y , offset_x , offset_y,tmp_zoomPos; 
	int delta_X ;
	bool isDeltaValid = false;
	Point opt;
	
	switch(m_display.g_CurDisplayMode) 
	{
		case PREVIEW_MODE:	
			offset_x = m_winWidth/2;  
			offset_y = 0;
			break;
		case MAIN_VIEW:
			offset_x =0;
			offset_y = m_winHeight/2;   
			break;			
		default:
			break;
	}

	LeftPoint.x -= offset_x;
	RightPoint.x -=offset_x;
	LeftPoint.y -= offset_y;
	RightPoint.y -=offset_y;
	
	delta_X = abs(LeftPoint.x - RightPoint.x) ;
	m_iDelta_X = delta_X;
	
	if(needChangeZoom == true ) {
		if(delta_X < MIN_VALID_RECT_WIDTH_IN_PIXEL) {
			;  // Do Nothing
		}
		else
		{
			isDeltaValid = true;
			tmp_zoomPos = checkZoomPosTable( delta_X );		
			m_iZoom = tmp_zoomPos;
			zoomPos = tmp_zoomPos;
			//tmp_zoomPos = checkZoomPosTable(delta_X);			
		}
	}
	if(needChangeZoom == true)
	{
		if(LeftPoint.x < RightPoint.x) {
			point_X = abs(LeftPoint.x - RightPoint.x) /2 + LeftPoint.x;
			point_Y = abs(LeftPoint.y - RightPoint.y) /2 + LeftPoint.y;	
		}else{
			point_X = abs(LeftPoint.x - RightPoint.x) /2 + RightPoint.x;
			point_Y = abs(LeftPoint.y - RightPoint.y) /2 + RightPoint.y;	
		}		
	}
	else  {
		point_X = (x - offset_x);
		point_Y = (y- offset_y);		
	}
	switch(m_display.g_CurDisplayMode) {
		case PREVIEW_MODE:
			opt = Point( point_X*2, point_Y*2 );	
			break;
		case MAIN_VIEW:
			opt = Point( point_X, point_Y * 2 );
			break;		
		default:
			break;
	}
	
	Point bpt;
	CvtImgCoords2CamCoords(opt, bpt);
	
	int DesPanPos, DesTilPos ;
		
	int  inputX = bpt.x;
	int  inputY = bpt.y;
#if 1
	int  tmpcofx = 6300;
	int  tmpcofy = 6200;

	inputX -= m_winWidth/4;
	inputY -= m_winHeight/4;

	float coefficientx = (float)tmpcofx*0.001f;
	float coefficienty = (float)tmpcofy*0.001f;

	float tmpficientx = 1.0;

	inputX = (int)((float)inputX * coefficientx * tmpficientx);
	inputY = (int)((float)inputY * coefficienty);

	float kx1 = 35.0/600.0;
	float ky1 = 9.0/600.0;
	float kx2 = 7.0/600.0;
	float ky2 = 4.0/600.0;

	inputX	 += (int)(inputX * kx1 + inputY * kx2);
	inputY	 -= (int)(inputX * ky1 + inputY * ky2); 	
#else
	inputX -= 474;//480;   
	inputY -= 276;//270; 	

	float coefficientx ;//= (float)tmpcofx*0.001f;
	float coefficienty ;//= (float)tmpcofy*0.001f;
	float fx = 1796.2317019134 + 10;
	float fy = 1795.8556284573 +55+20;;
	float degperpixX = 36000/(2*CV_PI*fx);
	float degperpixY = 36000/(2*CV_PI*fy);
	coefficientx = degperpixX*2;
	coefficienty = degperpixY*2;
	
	inputX = (int)((float)inputX * coefficientx );
	inputY = (int)((float)inputY * coefficienty);
#endif

	int Origin_PanPos = g_camParams.panPos;
	int Origin_TilPos = g_camParams.tiltPos;
	SetDestPosScope(inputX, inputY, Origin_PanPos,Origin_TilPos,DesPanPos, DesTilPos);
	if(  needChangeZoom == true ) 
	{
		trkmsg.cmd_ID = acqPosAndZoom;
		memcpy(&trkmsg.param[0],&DesPanPos, 4);
		memcpy(&trkmsg.param[4],&DesTilPos, 4); 
		if(isDeltaValid == true)
		{
			memcpy(&trkmsg.param[8],&(tmp_zoomPos)  , 4); 
		}
	}
	else
	{
		trkmsg.cmd_ID = speedloop;
		memcpy(&trkmsg.param[0],&DesPanPos, 4);
		memcpy(&trkmsg.param[4],&DesTilPos, 4); 	
	}
	
	ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
	
}
void CProcess::OnMouseLeftDwn(int x, int y)
{
	if( open_handleCalibra == true) {
		manualHandleKeyPoints(x,y);		
	}	
}
void CProcess::SetMtdConfig(MTD_Config mtdconfig)
{
	MTD_Config tempConfig = {0};	
	tempConfig = mtdconfig;
/*	
	CMD_Mtd_Frame sendconfig;
	sendconfig.detectNum = mtdconfig.targetNum;
	sendconfig.tmpMaxPixel =mtdconfig.maxArea;
	sendconfig.tmpMinPixel = mtdconfig.minArea;
	sendconfig.sensitivityThreshold = mtdconfig.sensitivity;
	sendconfig.TrkMaxTime = mtdconfig.trackTime;
*/
	SENDST	message = {0};
	message.cmd_ID = storeMtdConfig;
	memcpy(&message.param[0],&tempConfig, sizeof(MTD_Config));
	ipc_sendmsg(&message, IPC_FRIMG_MSG);


	//MTD_Config tempConfig = {0};	
	//tempConfig = mtdconfig;	
	//int datelength = sizeof(int);
	/*
	memcpy(&message.param[0*datelength],&tempConfig.targetNum, datelength);
	memcpy(&message.param[1*datelength],&tempConfig.trackTime, datelength); 	
	memcpy(&message.param[2*datelength],&tempConfig.maxArea, datelength); 	
	memcpy(&message.param[3*datelength],&tempConfig.minArea, datelength); 	
	memcpy(&message.param[4*datelength],&tempConfig.sensitivity, datelength); 
	*/	
	printf("\r\n[%s]:>>>>>>>>>>>>>>>  Send Set Mtd Config  to Linkage ctrl process !!!\r\n",__func__ );

	printf("\r\n[%s]:MTD Config:\r\nTarget Number:%d\r\nTrack Time:%d\r\nMax Area:%d\r\nMin Area:%d\r\nSensitivity:%d\r\n",
	__func__,tempConfig.targetNum,tempConfig.trackTime, tempConfig.maxArea,tempConfig.minArea,tempConfig.sensitivity);


}
void CProcess::SetDefaultWorkMode( GB_WorkMode workmode )
{
	int workModeType = 0;
	switch(workmode)
	{
		case HANDLE_LINK_MODE:
			workModeType = 0;
			break;
		case AUTO_LINK_MODE:
			workModeType = 1;
			break;
		case ONLY_BALL_MODE:
			workModeType = 2;
			break;
		default:
			workModeType = 0;
			break;			
	}
	SENDST	tmp = {0};
	tmp.cmd_ID = storeDefaultWorkMode;
	tmp.param[0] = workModeType+1;
	ipc_sendmsg(&tmp, IPC_FRIMG_MSG);
	printf("\r\n[%s]:Send Set Default Work Mode <%d> to Linkage ctrl process !!!\r\n",__func__, workModeType );
}
void CProcess::setWorkMode(GB_WorkMode workmode)
{
	g_AppWorkMode = workmode;
	int value = 0;
	if( g_AppWorkMode == AUTO_LINK_MODE )
	{
		value = 1;
		//printf("\r\n[%s]:&&&&&&&&&&&&&&&&& (AUTO_LINK_MODE )\r\n",__func__);
	}
	else
	{
		value = 0;
	}

	if(g_AppWorkMode == AUTO_LINK_MODE)
	{
		set_mouse_show(0);
	}
	else if(g_AppWorkMode == ONLY_BALL_MODE)
	{
		set_mouse_show(0);
	}
	else if(g_AppWorkMode == HANDLE_LINK_MODE)
	{
		
	}			
	SENDST	tmp;
	tmp.cmd_ID = mtdmode;
	tmp.param[0] = value ;
	ipc_sendmsg(&tmp, IPC_FRIMG_MSG);	
}

void CProcess::OnJosCtrl(int key, int param)
{
	switch(key)
	{
		case JOSF1_OPEN_AUTOLINKMODE:
		{
			if(MENU_TRIG_INTER_MODE == g_displayMode)
			{
				int num = m_autofr.getcalibnum();
				//if(num >= 4)
				{
					if(get_twinkle_flag())
						set_jos_mouse_mode(jos_mode);
					else
						set_jos_mouse_mode(mouse_mode);
				}
			}
			else
			{
				GB_WorkMode nextMode = (GB_WorkMode)(param - 1);
				setWorkMode(nextMode);
			}
		}
			break;
		case JOSF2_ENTER_MENU:			
			if(g_displayMode == MENU_GRID_MAP_VIEW)
			{
				#if 0
				if(msgextMenuCtrl!=NULL)
				{
					menu_param_t *pMenuStatus = msgextMenuCtrl;				
					pMenuStatus->menuarray[submenu_gridMapCalibrate].pointer = 0; 
				}
				#endif
				SENDST trkmsg2={0};
				trkmsg2.cmd_ID = enter_gridmap_view;
				trkmsg2.param[0] = 0;
				ipc_sendmsg(&trkmsg2, IPC_FRIMG_MSG);
				app_ctrl_setMenuStat(mainmenu2);			
				g_displayMode = MENU_MAIN_VIEW;
			}
			else
			{
				app_ctrl_setMenu_jos(param);
			}
			break;
		default:
			break;
	}
}

void CProcess::OnSpecialKeyDwn(int key,int x, int y)
{
	switch( key ) 
	{
		case 1:
			{
				GB_WorkMode nextMode = GB_WorkMode(((int)g_AppWorkMode+1)% MODE_COUNT);
				setWorkMode(nextMode);
			}
			break;
		case 2:
			app_ctrl_setMenu();  // Open Menu information
			break;
		case 3:
			start_calibrate = true;
			break;
		case 10:
			printf(" Move ball to destionation  1 Pixel  (%d , %d )    to   Pos (%d,%d ) \n" ,
					m_back[0].pixel.x,m_back[0].pixel.y,m_back[0].pos.x,m_back[0].pos.y);
			
			trkmsg.cmd_ID = acqPosAndZoom;
			memcpy(&trkmsg.param[0],&m_back[0].pos.x,4);
			memcpy(&trkmsg.param[4],&m_back[0].pos.y,4);
			ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
			break;
		case 11:
			printf(" Move ball to destionation 2  Pixel  (%d , %d )   to  Pos (%d,%d ) \n" ,
					m_back[1].pixel.x,m_back[1].pixel.y,m_back[1].pos.x,m_back[1].pos.y);

			trkmsg.cmd_ID = acqPosAndZoom;
			memcpy(&trkmsg.param[0],&m_back[1].pos.x,4);
			memcpy(&trkmsg.param[4],&m_back[1].pos.y,4);
			ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
			break;
		case 12:
			printf(" Move ball to destionation 3   Pixel  (%d , %d )    to  Pos (%d,%d ) \n" ,
					m_back[2].pixel.x,m_back[2].pixel.y,m_back[2].pos.x,m_back[2].pos.y);

			trkmsg.cmd_ID = acqPosAndZoom;
			memcpy(&trkmsg.param[0],&m_back[2].pos.x,4);
			memcpy(&trkmsg.param[4],&m_back[2].pos.y,4);
			ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
			break;
		//case 4:	
			//g_displayMode = MENU_GRID_MAP_VIEW;
			//break;
		case 5:	
			show_TrigonometricMat = true;
			#if 0
			{
				setGridMapCalibrate(true);
				QueryCurBallCamPosition();
				int row = m_curNodeIndex/(GRID_COLS_15+2);
				int col = m_curNodeIndex%(GRID_COLS_15+2);
				m_calibratedNodes[row][col].isShow = true;
				m_gridNodes[row][col].renderFlag== true;
				m_gridNodes[row][col].has_mark = 1;
				m_calibratedNodes[0][0].x = 60;
				m_calibratedNodes[0][0].y = 45;

				printf("\r\n[%s]: Mark Node Position:<%d,%d><%d, %d>\r\n",__func__,row,col,m_calibratedNodes[row][col].x,m_calibratedNodes[row][col].y);
				if(row!=m_lastRow || col!=m_lastCol)
				{
					m_successCalibraNum +=1;
					m_lastRow = row;
					m_lastCol = col;
					printf("\r\n[%s]: has_calibrated_num = %d\r\n",__func__,m_successCalibraNum);
				}
			}
			#endif
			break;
		case 6:
			{
				imageListForCalibra.clear();
				captureCount = 0;
			}
			break;
		case 7:
			capIndex = (capIndex+1) %2;
			break;
		case 8:
			break;
		case 9:
			g_GridMapMode  =  (g_GridMapMode + 1) % 3;
			break;
		/*
		case 10:
			m_intrMatObj->setCalibrateSwitch(true);			
			break;
		case 11:			
			if(g_displayMode == MENU_GRID_MAP_VIEW)
			{
				for(int i=0;i<=GRID_ROWS_11;i++)
				{
					for(int j=0;j<=GRID_COLS_15+1;j++)
					{
						m_calibratedNodes[i][j].isShow = false;
					}
				}
			}
			else
			{
				ImageList.clear();
			}
			break;	
		case 12:
			pThis->writeParams("SaveGridMap.yml");			
			break;
			*/
		case SPECIAL_KEY_DOWN:
			app_ctrl_downMenu();
			if(g_displayMode == MENU_GRID_MAP_VIEW)
			{
				m_curNodeIndex = (m_curNodeIndex+GRID_COLS_15+2)%((GRID_COLS_15+2)*(GRID_ROWS_11+1));
				if(m_curNodeIndex > 112) // total 204 nodes
				{
					m_display.setGridViewPortPosition(m_gridNodes[4][0].coord_x, 1080-m_gridNodes[4][0].coord_y);
				}
				else
				{
					m_display.setGridViewPortPosition(m_gridNodes[11][10].coord_x, 1080-m_gridNodes[11][10].coord_y);
				}
			}
			break;
		case SPECIAL_KEY_UP:
			app_ctrl_upMenu();
			if(g_displayMode == MENU_GRID_MAP_VIEW){
				int total = (GRID_COLS_15+2)*(GRID_ROWS_11+1);
				int cols = GRID_COLS_15+2;
				m_curNodeIndex = (m_curNodeIndex -cols + total)%total;
				if(m_curNodeIndex > 112)// total 204 nodes
				{
					m_display.setGridViewPortPosition(m_gridNodes[4][0].coord_x, 1080-m_gridNodes[4][0].coord_y);
				}
				else
				{
					m_display.setGridViewPortPosition(m_gridNodes[11][10].coord_x, 1080-m_gridNodes[11][10].coord_y);
				}
			}
			break;
		case SPECIAL_KEY_PAGEUP:
			saveOnePicture = true;
			break;
		case SPECIAL_KEY_RIGHT:
			{
				#if 0
				m_curNodeIndex = (m_curNodeIndex+1)%((GRID_COLS+1)*(GRID_ROWS+1));
				#else
				m_curNodeIndex = (m_curNodeIndex+1)%((GRID_COLS_15+2)*(GRID_ROWS_11+1));
				#endif
				
				if(m_curNodeIndex > 112)// total 204 nodes
				{
					m_display.setGridViewPortPosition(m_gridNodes[4][0].coord_x, 1080-m_gridNodes[4][0].coord_y);
				}
				else
				{
					m_display.setGridViewPortPosition(m_gridNodes[11][10].coord_x, 1080-m_gridNodes[11][10].coord_y);
				}				
			}
			break;
		case SPECIAL_KEY_LEFT:
			{	
				#if 0
				m_curNodeIndex = (m_curNodeIndex+  (GRID_COLS+1)*(GRID_ROWS+1) -1 )%((GRID_COLS+1)*(GRID_ROWS+1));
				#else
				m_curNodeIndex = (m_curNodeIndex+  (GRID_COLS_15+2)*(GRID_ROWS_11+1) -1 )%((GRID_COLS_15+2)*(GRID_ROWS_11+1));
				#endif
				if(m_curNodeIndex < 112) //otal 204 nodes
				{
					m_display.setGridViewPortPosition(m_gridNodes[11][10].coord_x, 1080-m_gridNodes[11][10].coord_y);
				}
				else
				{
					m_display.setGridViewPortPosition(m_gridNodes[4][0].coord_x, 1080-m_gridNodes[4][0].coord_y);
				}
			}			
			break;
		default:
			break;
	}
}

void CProcess::OnKeyDwn(unsigned char key)
{
	char flag = 0;
	CMD_EXT *pIStuts = extInCtrl;
	CMD_EXT tmpCmd = {0};

	menu_param_t *pMenuStatus = &extMenuCtrl;
	menu_param_t tmpMenuCmd = {0};

	if(key == 'a' || key == 'A')
	{
		tmpCmd.SensorStat = (pIStuts->SensorStat + 1)%MAX_CHAN;
		app_ctrl_setSensor(&tmpCmd);		
	}

	if(key == 'b' || key == 'B')
	{
		//pIStuts->PicpSensorStat = (pIStuts->PicpSensorStat + 1) % (eSen_Max+1);
		if(pIStuts->PicpSensorStat==0xff)
			pIStuts->PicpSensorStat=1;
		else 
			pIStuts->PicpSensorStat=0xff;		
		msgdriv_event(MSGID_EXT_INPUT_ENPICP, NULL);
	}

	if(key == 'c'|| key == 'C')
	{




				
	}

	if(key == 'd'|| key == 'D')
	{
	
		if(pIStuts->MmtStat[pIStuts->SensorStat])
			pIStuts->MmtStat[pIStuts->SensorStat] = eImgAlg_Disable;
		else
			pIStuts->MmtStat[pIStuts->SensorStat] = eImgAlg_Enable;
		msgdriv_event(MSGID_EXT_INPUT_ENMTD, NULL);
	}

	if (key == 'e' || key == 'E')
	{
		forwardflag = true;
	}

	if (key == 'f' || key == 'F')
	{
		backflag = true;
	}

	if (key == 'f' || key == 'F')
	{
		if(pIStuts->ImgFrezzStat[pIStuts->SensorStat])
			pIStuts->ImgFrezzStat[pIStuts->SensorStat] = eImgAlg_Disable;
		else
			pIStuts->ImgFrezzStat[pIStuts->SensorStat] = eImgAlg_Enable;
			
		msgdriv_event(MSGID_EXT_INPUT_ENFREZZ, NULL);
	}

	if (key == 'g')
	{
		tmpMenuCmd.Trig_Inter_Mode = (pMenuStatus->Trig_Inter_Mode + 1) % 2;
		app_ctrl_settrig_inter(&tmpMenuCmd);
	}
	if (key == 'h' || key == 'H')
	{
		app_getPT();
	}

	if (key == 'i')
	{
		if(pMenuStatus->Trig_Inter_Mode)
		{
			set_send_mat_stat(true);
		}
	}

	if (key == 'I')
	{
		if((pMenuStatus->Trig_Inter_Mode) && (get_cloneSrcImage_stat()))
		{
			printf("%s,%d, autoFindPoints\n",__FILE__,__LINE__);
			m_autofr.autoFindPoints();
		}
	}
	
	if (key == 'j' || key == 'J')
	{
		app_ctrl_save_trig_inter();
	}

	if (key == 'k' || key == 'K')
	{
		if(pIStuts->MtdState[pIStuts->SensorStat])
			pIStuts->MtdState[pIStuts->SensorStat] = eImgAlg_Disable;
		else
		{
			pIStuts->MtdState[pIStuts->SensorStat] = eImgAlg_Enable;
#if __MOVE_DETECT__
			chooseDetect = 10;
#endif
		}
		msgdriv_event(MSGID_EXT_MVDETECT, NULL);

		//printf("pIStuts->MtdState[pIStuts->SensorStat]  = %d\n",pIStuts->MtdState[pIStuts->SensorStat] );
	}

	if(key == 'l') {
		m_display.changeDisplayMode(SIDE_BY_SIDE);
	}
		
	if(key == 'M' || key == 'm' ) {
		m_camCalibra->bool_Calibrate = true;
		printf("\r\n [%s] ================ Press Key ' m ' : Start to Calibrate!\r\n",__FUNCTION__);
	}

	if(key =='n') {
		m_display.savePic_once = true;
	}
	
	if( key == 'N')
	{
		bool stat = get_draw_point_triangle_stat();
		set_draw_point_triangle_stat(!stat);
	}
	
	if(key == 'o')
	{
		int mode = plat->m_display.gettrig_pip_mode();
		mode = (mode + 1) % 4;
		plat->m_display.settrig_pip_mode(mode);
	}

	if(key == 'O')
	{
		bool stat = plat->get_showpip_stat();
		plat->set_showpip_stat(!stat);
	}
	
	if (key == 'p'|| key == 'P')
	{
		msgdriv_event(MSGID_EXT_INPUT_PICPCROP, NULL);
	}
		
	if(key == 'Q') {
		//m_display.switchDisplayMode();
		MenuDisplay nextMode = MenuDisplay((int)(g_displayMode+1) % MENU_DISPLAY_COUNT);
		g_displayMode = nextMode;
	}

	if(key == 'r'|| key == 'R')
	{
	}

	if (key == 's')
	{
		jos_mouse_Mode stat = get_gridinter_mode();
		stat = (jos_mouse_Mode)((stat + 1) % exit_calibrate_mode);
		if(pMenuStatus->Trig_Inter_Mode)
		{
			set_gridinter_mode(stat);
		}
	}

	if(key == 'S')
	{
		bool stat = get_drawpoints_stat();
		if(pMenuStatus->Trig_Inter_Mode)
			set_drawpoints_stat(!stat);
	}
	
	if (key == 't')
	{
		bool stat = get_drawsubdiv_stat();
		if(pMenuStatus->Trig_Inter_Mode)
			set_drawsubdiv_stat(!stat);
	}

	if (key == 'T')
	{
	
	}
			
	if(key == 'U' || key == 'u' ) {
		m_camCalibra->writeParam_flag = true;
	}
		
	if(key == 'v' || key == 'V') {
		m_camCalibra->start_cloneVideoSrc = true;
		printf("\r\n [%s] ================ Press Key ' v ' : Start Clone One Frame Image !\r\n",__FUNCTION__);
	}

	if(key == 'w'|| key == 'W')
	{
		if(pIStuts->ImgMmtshow[pIStuts->SensorStat])
			pIStuts->ImgMmtshow[pIStuts->SensorStat] = eImgAlg_Disable;
		else
			pIStuts->ImgMmtshow[pIStuts->SensorStat] = eImgAlg_Enable;
		
		msgdriv_event(MSGID_EXT_INPUT_MMTSHOW, NULL);
		OSA_printf("MSGID_EXT_INPUT_MMTSHOW\n");
	}

	if (key == 'x'|| key == 'X') {
		open_handleCalibra = false ; 
		m_camCalibra->Set_Handler_Calibra = false ;
		m_camCalibra->start_cloneVideoSrc = false;
	}
	
	if (key == 'y'|| key == 'Y') {		
		open_handleCalibra = true ;
		m_camCalibra->Set_Handler_Calibra = true ;
	}
			

	if (key == 'z')
	{
		if(Set_SelectByRect)
			Set_SelectByRect = false ;
		else
			Set_SelectByRect = true ;
		//pIStuts->ImgZoomStat[0]=(pIStuts->ImgZoomStat[0]+1)%2;
		//msgdriv_event(MSGID_EXT_INPUT_ENZOOM, NULL);
	}


	if((key >= '0') && (key <= '9'))
	{
		app_ctrl_setnumber(key);
		switch(key){
			case '0':	
				break;
			case '1':
				if(g_displayMode == MENU_GRID_MAP_VIEW)
				{
					setGridMapCalibrate(true);
					QueryCurBallCamPosition();
					int row = m_curNodeIndex/(GRID_COLS_15+2);
					int col = m_curNodeIndex%(GRID_COLS_15+2);
					m_calibratedNodes[row][col].isShow = true;
					m_gridNodes[row][col].has_mark = 1;
					m_calibratedNodes[0][0].x = 60;
					m_calibratedNodes[0][0].y = 45;

					if(row!=m_lastRow || col!=m_lastCol)
					{
						m_successCalibraNum +=1;
						m_lastRow = row;
						m_lastCol = col;
					}
				}

				break;
			case '2':
				if(g_displayMode == MENU_GRID_MAP_VIEW)
				{
					int total = (GRID_COLS_15+2)*(GRID_ROWS_11+1);
					int cols = GRID_COLS_15+2;
					m_curNodeIndex = (m_curNodeIndex -cols + total)%total;
					if(m_curNodeIndex > 112)	// total 192/432 nodes
					{
						m_display.setGridViewPortPosition(m_gridNodes[4][0].coord_x, 1080-m_gridNodes[4][0].coord_y);
					}
					else
					{
						m_display.setGridViewPortPosition(m_gridNodes[11][10].coord_x, 1080-m_gridNodes[11][10].coord_y);
					}
				}
				break;
			case '3':					
				break;
			case '4':
				if(g_displayMode == MENU_GRID_MAP_VIEW)
				{
					m_curNodeIndex = (m_curNodeIndex+  (GRID_COLS_15+2)*(GRID_ROWS_11+1) -1 )%((GRID_COLS_15+2)*(GRID_ROWS_11+1));
					if(m_curNodeIndex < 112)// total 192/432 nodes
					{
						m_display.setGridViewPortPosition(m_gridNodes[11][10].coord_x, 1080-m_gridNodes[11][10].coord_y);
					}
					else
					{
						m_display.setGridViewPortPosition(m_gridNodes[4][0].coord_x, 1080-m_gridNodes[4][0].coord_y);

					}
				}

				break;
			case '5':
				if(g_displayMode == MENU_GRID_MAP_VIEW)
				{
					pThis->writeParams("SaveGridMap.yml");
				}

				break;
			case '6':
				if(g_displayMode == MENU_GRID_MAP_VIEW)
				{
					m_curNodeIndex = (m_curNodeIndex+1)%((GRID_COLS_15+2)*(GRID_ROWS_11+1));

					if(m_curNodeIndex > 112)// total 192/432 nodes
					{
						m_display.setGridViewPortPosition(m_gridNodes[4][0].coord_x, 1080-m_gridNodes[4][0].coord_y);
					}
					else{
						m_display.setGridViewPortPosition(m_gridNodes[11][10].coord_x, 1080-m_gridNodes[11][10].coord_y);
					}
				}

				break;
			case '7':
				if(g_displayMode == MENU_GRID_MAP_VIEW)
				{
				}

				break;
			case '8':
				if(g_displayMode == MENU_GRID_MAP_VIEW)
				{				
					m_curNodeIndex = (m_curNodeIndex+GRID_COLS_15+2)%((GRID_COLS_15+2)*(GRID_ROWS_11+1));
					if(m_curNodeIndex > 112)// total 192/432 nodes
					{
						m_display.setGridViewPortPosition(m_gridNodes[4][0].coord_x, 1080-m_gridNodes[4][0].coord_y);
					}
					else{
						m_display.setGridViewPortPosition(m_gridNodes[11][10].coord_x, 1080-m_gridNodes[11][10].coord_y);
					}
				}

				break;
			case '9':
				if(g_displayMode == MENU_GRID_MAP_VIEW)
				{
				}
				break;
			default:
				break;
		}

	}
			
		
	if(key == 13)
	{
		app_ctrl_enter();
	}
}


void CProcess::msgdriv_event(MSG_PROC_ID msgId, void *prm)
{
	int tempvalue=0;
	CMD_EXT *pIStuts = extInCtrl;
	menu_param_t *pMenuStatus = &extMenuCtrl;
	CMD_EXT *pInCmd = NULL;
	CMD_EXT tmpCmd = {0};
	if(msgId == MSGID_EXT_INPUT_SENSOR || msgId == MSGID_EXT_INPUT_ENPICP)
	{
		if(prm != NULL)
		{
			pInCmd = (CMD_EXT *)prm;
			pIStuts->SensorStat = pInCmd->SensorStat;
			pIStuts->PicpSensorStat = pInCmd->PicpSensorStat;
		}
		int itmp;
		//chage acq;
		m_rcAcq.width		=	pIStuts->AimW[pIStuts->SensorStat];
		m_rcAcq.height	=	pIStuts->AimH[pIStuts->SensorStat];

		m_rcAcq.x=pIStuts->opticAxisPosX[pIStuts->SensorStat]-m_rcAcq.width/2;
		m_rcAcq.y=pIStuts->opticAxisPosY[pIStuts->SensorStat]-m_rcAcq.height/2;

		OSA_printf("recv   the rctrack x=%f y=%f w=%f h=%f  sensor=%d picpsensor=%d\n",m_rcAcq.x,m_rcAcq.y,
			m_rcAcq.width,m_rcAcq.height,pIStuts->SensorStat,pIStuts->PicpSensorStat);
		
		itmp = pIStuts->SensorStat;
		dynamic_config(VP_CFG_MainChId, itmp, NULL);

#if 1//change the sensor picp change too
		if((pIStuts->PicpSensorStat>=eSen_CH0)&&(pIStuts->PicpSensorStat<eSen_Max))
		{
			for(int chi=eSen_CH0;chi<eSen_Max;chi++)
			{
				if(pIStuts->ImgPicp[chi]==1)
					pIStuts->PicpSensorStatpri=pIStuts->PicpSensorStat;
			}
			
		}
#endif

		itmp = pIStuts->PicpSensorStat;//freeze change
		dynamic_config(VP_CFG_SubChId, itmp, NULL);

//sensor 1 rect

		DS_Rect lay_rect;
			lay_rect.w = vcapWH[0][0]/3;
			lay_rect.h = vcapWH[0][1]/3;
			lay_rect.x = pIStuts->AxisPosX[pIStuts->SensorStat] -lay_rect.w/2;
			lay_rect.y = pIStuts->AxisPosY[pIStuts->SensorStat] -lay_rect.h/2;

		if(pIStuts->ImgZoomStat[pIStuts->SensorStat] == 2){
			lay_rect.w = vcapWH[0][0]/12;
			lay_rect.h = vcapWH[0][1]/12;
			lay_rect.x = pIStuts->AxisPosX[pIStuts->SensorStat] -lay_rect.w/2;
			lay_rect.y = pIStuts->AxisPosY[pIStuts->SensorStat] -lay_rect.h/2;
		}
		else if(pIStuts->ImgZoomStat[pIStuts->SensorStat] == 4){
			lay_rect.w = vcapWH[0][0]/18;
			lay_rect.h = vcapWH[0][1]/18;
			lay_rect.x = pIStuts->AxisPosX[pIStuts->SensorStat] -lay_rect.w/2;
			lay_rect.y = pIStuts->AxisPosY[pIStuts->SensorStat] -lay_rect.h/2;
		}
		else if(pIStuts->ImgZoomStat[pIStuts->SensorStat] == 6){
			lay_rect.w = vcapWH[0][0]/24;
			lay_rect.h = vcapWH[0][1]/24;
			lay_rect.x = pIStuts->AxisPosX[pIStuts->SensorStat] -lay_rect.w/2;
			lay_rect.y = pIStuts->AxisPosY[pIStuts->SensorStat] -lay_rect.h/2;
		}
		else if(pIStuts->ImgZoomStat[pIStuts->SensorStat] == 8){
			lay_rect.w = vcapWH[0][0]/30;
			lay_rect.h = vcapWH[0][1]/30;
			lay_rect.x = pIStuts->AxisPosX[pIStuts->SensorStat] - lay_rect.w/2;
			lay_rect.y = pIStuts->AxisPosY[pIStuts->SensorStat] - lay_rect.h/2;
		}

		m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 1, &lay_rect);

//picp position
		lay_rect=rendpos[pIStuts->PicpPosStat];
		
		lay_rect.w = VIDEO_DIS_WIDTH/3;
		lay_rect.h =VIDEO_DIS_HEIGHT/3;
		lay_rect.x = VIDEO_DIS_WIDTH*2/3;
		lay_rect.y = VIDEO_DIS_HEIGHT*2/3;
		m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 1, &lay_rect);

///sensor zoom

		if(0)//(pIStuts->ImgZoomStat[pIStuts->SensorStat])
		{
			/*
			memset(&lay_rect, 0, sizeof(DS_Rect));
			if(pIStuts->SensorStat==0)//just tv zooom
			{
				lay_rect.w = vcapWH[pIStuts->SensorStat][0]/2;
				lay_rect.h = vcapWH[pIStuts->SensorStat][1]/2;
				lay_rect.x = vcapWH[pIStuts->SensorStat][0]/4;
				lay_rect.y = vcapWH[pIStuts->SensorStat][1]/4;
			}
			*/
			
			//m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 0, &lay_rect);
			if(pIStuts->PicpSensorStat==1)
			{
				lay_rect.w = vcapWH[1][0]/6;
				lay_rect.h = vcapWH[1][1]/6;
				lay_rect.x = vcapWH[1][0]/2-lay_rect.w/2;
				lay_rect.y = vcapWH[1][1]/2-lay_rect.h/2;
				m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 1, &lay_rect);
			}
			if(pIStuts->PicpSensorStat==0)
			{	
				lay_rect.w = vcapWH[0][0]/6;
				lay_rect.h = vcapWH[0][1]/6;
				lay_rect.x = vcapWH[0][0]/2-lay_rect.w/2;
				lay_rect.y = vcapWH[0][1]/2-lay_rect.h/2;
				m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 1, &lay_rect);
			}
		}


//mmt show change
	if(pIStuts->ImgMmtshow[pIStuts->SensorStat^1]==0x01)
		{
			
			int mmtchid=0;
			int chid=pIStuts->SensorStat;
			pIStuts->ImgMmtshow[pIStuts->SensorStat^1]=0;
			pIStuts->ImgMmtshow[pIStuts->SensorStat]=1;
			itmp = chid;
			mmtchid=2;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			//chid++;
			itmp=chid;
			mmtchid=3;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			//chid++;
			itmp=chid;
			mmtchid=4;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			//chid++;
			itmp=chid;
			mmtchid=5;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			//chid++;
			itmp=chid;
			mmtchid=6;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			lay_rect.w = vdisWH[0][0]/3*2;
			lay_rect.h = vdisWH[0][1]/3*2;
			lay_rect.x = 0;
			lay_rect.y = vdisWH[0][1]/3;
			m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 0, &lay_rect);
			//m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 7, NULL);
			



		}

		
		
	
	}

	if(msgId == MSGID_EXT_INPUT_TRACK)
	{
		if(prm != NULL)
		{
			pInCmd = (CMD_EXT *)prm;
			pIStuts->AvtTrkStat = pInCmd->AvtTrkStat;
		}
		char procStr[][10] = {"ACQ", "TARGET", "MTD", "SECTRK", "SEARCH", "ROAM", "SCENE", "IMGTRK"};
		UTC_RECT_float rc;
		if (pIStuts->AvtTrkStat == eTrk_mode_acq)
		{
			OSA_printf(" %d:%s set track to [%s]\n", OSA_getCurTimeInMsec(), __func__,
					   procStr[pIStuts->AvtTrkStat]);

			dynamic_config(VP_CFG_TrkEnable, 0);
			pIStuts->AvtPosX[extInCtrl->SensorStat] = pIStuts->AxisPosX[extInCtrl->SensorStat];
			pIStuts->AvtPosY[extInCtrl->SensorStat] = pIStuts->AxisPosY[extInCtrl->SensorStat];	
			pIStuts->AimW[pIStuts->SensorStat] = 60;
			pIStuts->AimH[pIStuts->SensorStat] = 60;
			
			pIStuts->unitAimX = pIStuts->AvtPosX[extInCtrl->SensorStat] ;
			if(pIStuts->unitAimX < 0)
			{
				pIStuts->unitAimX = 0;
			}

			pIStuts->unitAimY = pIStuts->AvtPosY[extInCtrl->SensorStat];

			if(pIStuts->unitAimY<0)
			{
				pIStuts->unitAimY=0;
			}
			rc.width	= pIStuts->AimW[pIStuts->SensorStat];
			rc.height	= pIStuts->AimH[pIStuts->SensorStat];
			rc.x=pIStuts->unitAimX-rc.width/2;
			rc.y=pIStuts->unitAimY-rc.height/2;
			dynamic_config(VP_CFG_TrkEnable, 0,&rc);
			return ;
		}

		if (pIStuts->AvtTrkStat == eTrk_mode_sectrk)
		{
			OSA_printf(" %d:%s line:%d set track to [%s]\n", OSA_getCurTimeInMsec(), __func__,
					   __LINE__,procStr[pIStuts->AvtTrkStat]);
			pIStuts->unitAimX = pIStuts->AvtPosX[extInCtrl->SensorStat];
			pIStuts->unitAimY = pIStuts->AvtPosY[extInCtrl->SensorStat] ;
		}
		else if (pIStuts->AvtTrkStat == eTrk_mode_search)
		{
			OSA_printf(" %d:%s line:%d set track to [%s]\n", OSA_getCurTimeInMsec(), __func__,
					   __LINE__,procStr[pIStuts->AvtTrkStat]);

		  	//pIStuts->AvtTrkStat = eTrk_mode_search;
		 	pIStuts->unitAimX = pIStuts->AvtPosX[extInCtrl->SensorStat];
		   	pIStuts->unitAimY = pIStuts->AvtPosY[extInCtrl->SensorStat] ;
		}
		else if (pIStuts->AvtTrkStat == eTrk_mode_mtd)
		{
			pIStuts->unitAimX = pIStuts->AvtPosX[extInCtrl->SensorStat];
		   	pIStuts->unitAimY = pIStuts->AvtPosY[extInCtrl->SensorStat] ;
			dynamic_config(VP_CFG_TrkEnable, 0,NULL);
			return ;
			
			pIStuts->AvtTrkStat = eTrk_mode_target;

			//zoom for mtdTrk change xy 	
			pIStuts->unitAimX=pIStuts->MmtPixelX;
			pIStuts->unitAimY=pIStuts->MmtPixelY;
		 	
			if(pIStuts->MmtValid)
			{
				tempvalue=pIStuts->MmtPixelX;
				
				if(tempvalue<0)
					{
						pIStuts->unitAimX=0;
					}
				else
					{
						pIStuts->unitAimX=tempvalue;

					}
				tempvalue=pIStuts->MmtPixelY ;
				//- pIStuts->unitAimH/2;
				if(tempvalue<0)
					{
						pIStuts->unitAimY=0;
					}
				else
					{
						pIStuts->unitAimY=tempvalue;

					}
				
				OSA_printf(" %d:%s set track to x =%f y=%f  mtdx=%d mtdy=%d  w=%d  h=%d\n", OSA_getCurTimeInMsec(), __func__,
						pIStuts->unitAimX,pIStuts->unitAimY, pIStuts->MmtPixelX,pIStuts->MmtPixelY,pIStuts->unitAimW/2,pIStuts->unitAimH/2);
			}
			else
			{
				pIStuts->unitAimX = pIStuts->opticAxisPosX[extInCtrl->SensorStat ] - pIStuts->unitAimW/2;
				pIStuts->unitAimY = pIStuts->opticAxisPosY[extInCtrl->SensorStat ] - pIStuts->unitAimH/2;
			}
		}

		//printf("%s,line:%d   aimx,aimy=(%d,%d)\n",__func__,__LINE__,pIStuts->AvtPosX[0],pIStuts->AvtPosY[0]);
		if((m_curChId== video_gaoqing0)||(m_curChId== video_gaoqing)||(m_curChId== video_gaoqing2)||(m_curChId== video_gaoqing3))
		{
			rc.width= pIStuts->AimW[pIStuts->SensorStat];
			rc.height=pIStuts->AimW[pIStuts->SensorStat];
			pIStuts->unitAimX = pIStuts->AvtPosX[pIStuts->SensorStat];
			pIStuts->unitAimY = pIStuts->AvtPosY[pIStuts->SensorStat];
/*
			printf("AvtPosX[%d] , AvtPosY[%d] (%d,%d) \n",pIStuts->SensorStat,
				pIStuts->SensorStat,pIStuts->AvtPosX[pIStuts->SensorStat],
				pIStuts->AvtPosY[pIStuts->SensorStat]);
*/
		}
		else if(m_curChId == video_pal)
		{
			rc.width= pIStuts->AcqRectW[pIStuts->SensorStat];
			rc.height=pIStuts->AcqRectH[pIStuts->SensorStat];
			pIStuts->unitAimX = pIStuts->AvtPosX[pIStuts->SensorStat];
			pIStuts->unitAimY = pIStuts->AvtPosY[pIStuts->SensorStat];
		}
		if(pIStuts->AvtTrkStat == eTrk_mode_sectrk || pIStuts->AvtTrkStat ==eTrk_mode_acqmove){
			pIStuts->unitAimX = pIStuts->AvtPosX[pIStuts->SensorStat];
			pIStuts->unitAimY = pIStuts->AvtPosY[pIStuts->SensorStat];
			printf("set set set   x  , y (%d , %d ) \n",pIStuts->unitAimX,pIStuts->unitAimY);
		}

		rc.x=pIStuts->unitAimX-rc.width/2;
		rc.y=pIStuts->unitAimY-rc.height/2;
		
			
		OSA_printf("%s,line:%d   rc. xy(%f,%f),wh(%f,%f)\n",__func__,__LINE__,rc.x,rc.y,rc.width,rc.height);
		dynamic_config(VP_CFG_TrkEnable, 1,&rc);
		if(pIStuts->AvtTrkStat == eTrk_mode_sectrk)
		{
			m_intervalFrame=2;
			m_rcAcq=rc;
			pIStuts->AvtTrkStat = eTrk_mode_target;
			OSA_printf("%s  line:%d		set sec track\n ",__func__,__LINE__);	
		}		
 	}

	if(msgId == MSGID_EXT_INPUT_ENMTD)
	{
		if(prm != NULL)
		{
			pInCmd = (CMD_EXT *)prm;
			pIStuts->MmtStat[0] = pInCmd->MmtStat[0];
			pIStuts->MmtStat[1] = pInCmd->MmtStat[1];
		}
		int MMTStatus = (pIStuts->MmtStat[pIStuts->SensorStat]&0x01) ;
		if(MMTStatus)
			dynamic_config(VP_CFG_MmtEnable, 1);
		else
			dynamic_config(VP_CFG_MmtEnable, 0);
		//FOR DUMP FRAME
		if(MMTStatus)
			dynamic_config(CDisplayer::DS_CFG_MMTEnable, pIStuts->SensorStat, &MMTStatus);
		else
			dynamic_config(CDisplayer::DS_CFG_MMTEnable, pIStuts->SensorStat, &MMTStatus);
	}

	if(msgId == MSGID_EXT_INPUT_ENENHAN)
	{
		if(prm != NULL)
		{
			pInCmd = (CMD_EXT *)prm;
			pIStuts->ImgEnhStat[0] = pInCmd->ImgEnhStat[0];
			pIStuts->ImgEnhStat[1] = pInCmd->ImgEnhStat[1];
		}
		int ENHStatus = (pIStuts->ImgEnhStat[pIStuts->SensorStat]&0x01) ;
		OSA_printf(" %d:%s set mtd enMask %d\n", OSA_getCurTimeInMsec(),__func__,ENHStatus);
		if(ENHStatus)
			dynamic_config(CDisplayer::DS_CFG_EnhEnable, pIStuts->SensorStat, &ENHStatus);
		else
			dynamic_config(CDisplayer::DS_CFG_EnhEnable, pIStuts->SensorStat, &ENHStatus);
	}


	if(msgId == MSGID_EXT_INPUT_AIMPOS || msgId == MSGID_EXT_INPUT_AIMSIZE)
	{
		if(prm != NULL)
		{
			pInCmd = (CMD_EXT *)prm;
			pIStuts->AvtTrkAimSize = pInCmd->AvtTrkAimSize;
			pIStuts->aimRectMoveStepX = pInCmd->aimRectMoveStepX;
			pIStuts->aimRectMoveStepY= pInCmd->aimRectMoveStepY;
		}
		
		if(pIStuts->AvtTrkStat)
		{
			UTC_RECT_float rc;
			if(msgId == MSGID_EXT_INPUT_AIMSIZE)
			{
				pIStuts->unitAimW  =  pIStuts->AimW[pIStuts->SensorStat];
				pIStuts->unitAimH	  =  pIStuts->AimW[pIStuts->SensorStat];

				rc.x	=	pIStuts->unitAimX-pIStuts->unitAimW/2;
				rc.y	=	pIStuts->unitAimY-pIStuts->unitAimH/2;
				rc.width=pIStuts->unitAimW;
				rc.height=pIStuts->unitAimH;
				//OSA_printf("***xy = (%f,%f)  WH(%f,%f)\n",rc.x,rc.y,rc.width,rc.height);
			}
			else
			{
				moveStat = true;
				//printf("----- XY(%d,%d),WH(%d,%d)\n",pIStuts->unitAimX,pIStuts->unitAimY,pIStuts->unitAimW,pIStuts->unitAimH);
				
				//printf("111W,H : (%d,%d)\n",pIStuts->unitAimW,pIStuts->unitAimH);
				rc.width=pIStuts->unitAimW;
				rc.height=pIStuts->unitAimH;
				//printf("222rc.width,rc.height : (%f,%f)\n",rc.width,rc.height);
				
				rc.x = pIStuts->unitAimX-pIStuts->unitAimW/2 + pIStuts->aimRectMoveStepX;
				rc.y = pIStuts->unitAimY-pIStuts->unitAimH/2  + pIStuts->aimRectMoveStepY;
				//printf("333rc.x,rc.y : (%d,%d)\n",rc.x,rc.y);

				pIStuts->aimRectMoveStepX = 0;
				pIStuts->aimRectMoveStepY = 0;
				
			}
			m_intervalFrame=1;
			m_rcAcq=rc;
			OSA_printf(" %d:%s refine move (%d, %d), wh(%f, %f)  aim(%d,%d) rc(%f,%f)\n", OSA_getCurTimeInMsec(), __func__,
						pIStuts->aimRectMoveStepX, pIStuts->aimRectMoveStepY, 
						rc.width, rc.height,pIStuts->unitAimX,pIStuts->unitAimY,rc.x,rc.y);
		}

		return ;
	}

	if(msgId == MSGID_EXT_INPUT_ENZOOM)
	{
		if(prm != NULL)
		{
			pInCmd = (CMD_EXT *)prm;
			pIStuts->ImgZoomStat[0] = pInCmd->ImgZoomStat[0];
			pIStuts->ImgZoomStat[1] = pInCmd->ImgZoomStat[1];
		}

		DS_Rect lay_rect;
		
		if(pIStuts->SensorStat==0)//tv
		{
			memset(&lay_rect, 0, sizeof(DS_Rect));
			if(pIStuts->ImgZoomStat[0] == 2)
			{
				lay_rect.w = vdisWH[0][0]/2;
				lay_rect.h = vdisWH[0][1]/2;
				lay_rect.x = vdisWH[0][0]/4;
				lay_rect.y = vdisWH[0][1]/4;
			}
			else if(pIStuts->ImgZoomStat[0] == 4)
			{
				lay_rect.w = vdisWH[0][0]/4;
				lay_rect.h = vdisWH[0][1]/4;
				lay_rect.x = vdisWH[0][0]/2 - lay_rect.w/2;
				lay_rect.y = vdisWH[0][1]/2 - lay_rect.h/2;
			}
			else if(pIStuts->ImgZoomStat[0] == 8)
			{
				lay_rect.w = vdisWH[0][0]/8;
				lay_rect.h = vdisWH[0][1]/8;
				lay_rect.x = vdisWH[0][0]/2 - lay_rect.w/2;
				lay_rect.y = vdisWH[0][1]/2 - lay_rect.h/2;
			}
			else
			{
				lay_rect.w = vdisWH[0][0];
				lay_rect.h = vdisWH[0][1];
				lay_rect.x = 0;
				lay_rect.y = 0;
			}		
			
			m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 0, &lay_rect);
			memset(&lay_rect, 0, sizeof(DS_Rect));
			
			lay_rect.w = vcapWH[0][0]/6;
			lay_rect.h = vcapWH[0][1]/6;
			lay_rect.x = pIStuts->AxisPosX[pIStuts->SensorStat] - lay_rect.w/2;
			lay_rect.y = pIStuts->AxisPosY[pIStuts->SensorStat] - lay_rect.h/2;
			
			if(pIStuts->ImgZoomStat[pIStuts->SensorStat] == 2){
				lay_rect.w =vdisWH[0][0]/12;
				lay_rect.h = vdisWH[0][1]/12;
				lay_rect.x = pIStuts->AxisPosX[pIStuts->SensorStat] - lay_rect.w/2;
				lay_rect.y = pIStuts->AxisPosY[pIStuts->SensorStat] - lay_rect.h/2;
			}
			if(pIStuts->ImgZoomStat[pIStuts->SensorStat] == 4){
				lay_rect.w = vcapWH[0][0]/24;
				lay_rect.h = vcapWH[0][1]/24;
				lay_rect.x = pIStuts->AxisPosX[pIStuts->SensorStat] -lay_rect.w/2;
				lay_rect.y = pIStuts->AxisPosY[pIStuts->SensorStat] -lay_rect.h/2;
			}
			else if(pIStuts->ImgZoomStat[pIStuts->SensorStat] == 6){
				lay_rect.w = vcapWH[0][0]/48;
				lay_rect.h = vcapWH[0][1]/48;
				lay_rect.x = pIStuts->AxisPosX[pIStuts->SensorStat] -lay_rect.w/2;
				lay_rect.y = pIStuts->AxisPosY[pIStuts->SensorStat] -lay_rect.h/2;
			}
			else if(pIStuts->ImgZoomStat[pIStuts->SensorStat] == 8){
				lay_rect.w = vcapWH[0][0]/64;
				lay_rect.h = vcapWH[0][1]/64;
				lay_rect.x = pIStuts->AxisPosX[pIStuts->SensorStat] - lay_rect.w/2;
				lay_rect.y = pIStuts->AxisPosY[pIStuts->SensorStat] - lay_rect.h/2;
			}
			
			
			m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 1, &lay_rect);			
		}
		else
		{
			memset(&lay_rect, 0, sizeof(DS_Rect));
			memset(&lay_rect, 0, sizeof(DS_Rect));
			if(pIStuts->ImgZoomStat[0]&&(pIStuts->PicpSensorStat==0))
			{
				lay_rect.w = vcapWH[0][0]/6;
				lay_rect.h = vcapWH[0][1]/6;
				lay_rect.x = vcapWH[0][0]/2-lay_rect.w/2;
				lay_rect.y = vcapWH[0][1]/2-lay_rect.h/2;
			}
			else 
			{
				lay_rect.w = vcapWH[0][0]/3;
				lay_rect.h = vcapWH[0][1]/3;
				lay_rect.x = vcapWH[0][0]/2-lay_rect.w/2;
				lay_rect.y = vcapWH[0][1]/2-lay_rect.h/2;		
			}
			m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 1, &lay_rect);
		}

		return ;
	}
	
	if(msgId ==MSGID_EXT_INPUT_PICPCROP)
	{		
		m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 1, &rendpos[pIStuts->PicpPosStat]);
	}

	if(msgId ==MSGID_EXT_INPUT_VIDEOEN)
	{
		int status=pIStuts->unitFaultStat&0x01;
		status^=1;
		m_display.dynamic_config(CDisplayer::DS_CFG_VideodetEnable, 0, &status);
		OSA_printf("MSGID_EXT_INPUT_VIDEOEN status0=%d\n",status);
		 status=(pIStuts->unitFaultStat>1)&0x01;
		 status^=1;
		m_display.dynamic_config(CDisplayer::DS_CFG_VideodetEnable, 1, &status);
		OSA_printf("MSGID_EXT_INPUT_VIDEOEN status1=%d\n",status);
	}
	if(msgId ==MSGID_EXT_INPUT_MMTSHOW)
	{
		int itmp=0;
		int mmtchid=0;
		DS_Rect lay_rect;
		if(pIStuts->ImgMmtshow[pIStuts->SensorStat])
		{
			int chid=pIStuts->SensorStat;
			itmp = chid;
			mmtchid=2;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			//chid++;
			itmp=chid;
			mmtchid=3;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			//chid++;
			itmp=chid;
			mmtchid=4;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			//chid++;
			itmp=chid;
			mmtchid=5;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			//chid++;
			itmp=chid;
			mmtchid=6;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			lay_rect.w = vdisWH[0][0]/3*2;
			lay_rect.h = vdisWH[0][1]/3*2;
			lay_rect.x = 0;
			lay_rect.y = vdisWH[0][1]/3;
			m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 0, &lay_rect);
			//m_display.dynamic_config(CDisplayer::DS_CFG_Rendercount, 7, NULL);

			//m_display.m_renderCount
		}
		else
		{
			itmp = 8;
			mmtchid=2;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			itmp=8;
			mmtchid=3;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			itmp=8;
			mmtchid=4;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			itmp=8;
			mmtchid=5;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			itmp=8;
			mmtchid=6;
			dynamic_config(VP_CFG_SubPicpChId, itmp, &mmtchid);
			lay_rect.w = vdisWH[0][0];
			lay_rect.h = vdisWH[0][1];
			lay_rect.x = 0;
			lay_rect.y = 0;
			m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 0, &lay_rect);
			//m_display.dynamic_config(CDisplayer::DS_CFG_Rendercount, 2, NULL);
		}
		
	#if 1	
	lay_rect.w = 30;
	lay_rect.h = 30;
	lay_rect.x = 0;
	lay_rect.y = 0;
	m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 2, &lay_rect);
	m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 3, &lay_rect);
	m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 4, &lay_rect);
	m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 5, &lay_rect);
	m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 6, &lay_rect);
	lay_rect.w = vdisWH[0][0]/3;
	lay_rect.h = vdisWH[0][1]/3;
	lay_rect.x = 0;
	lay_rect.y = 0;
	m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 2, &lay_rect);
	lay_rect.w = vdisWH[0][0]/3;
	lay_rect.h = vdisWH[0][1]/3;
	lay_rect.x = vdisWH[0][0]/3;
	lay_rect.y = 0;
	m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 3, &lay_rect);
	lay_rect.w = vdisWH[0][0]/3;
	lay_rect.h = vdisWH[0][1]/3;
	lay_rect.x = vdisWH[0][0]/3;
	lay_rect.x=lay_rect.x*2;
	lay_rect.y = 0;
	m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 4, &lay_rect);
	lay_rect.w = vdisWH[0][0]/3;
	lay_rect.h = vdisWH[0][1]/3;
	lay_rect.x = vdisWH[0][0]/3;
	lay_rect.x=lay_rect.x*2;
	lay_rect.y = vdisWH[0][1]/3;
	m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 5, &lay_rect);
	lay_rect.w = vdisWH[0][0]/3;
	lay_rect.h = vdisWH[0][1]/3;
	lay_rect.x = vdisWH[0][0]/3;
	lay_rect.x=lay_rect.x*2;
	lay_rect.y = vdisWH[0][1]/3;
	lay_rect.y=lay_rect.y*2;
	m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 6, &lay_rect);
	#endif

	}

	if(msgId ==MSGID_EXT_INPUT_MMTSHOWUPDATE)
	{	
		for(int i=0;i<5;i++)
		{
			if(Mmtpos[i].valid)
			{
				//m_display.m_renders[i+2].videodect=1;
				m_display.dynamic_config(CDisplayer::DS_CFG_VideodetEnable, i+2, &Mmtpos[i].valid);
				m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, i+2, &Mmtpos[i]);
			}
			else
			{
				//m_display.m_renders[i+2].videodect=0;
				//OSA_printf("the id=%d valid =%d\n",i+2,Mmtpos[i].valid);
				m_display.dynamic_config(CDisplayer::DS_CFG_VideodetEnable, i+2, &Mmtpos[i].valid);	
			}
		}
	}
#if __MOVE_DETECT__
	if(msgId == MSGID_EXT_MVDETECT)
	{	
		int Mtdstatus = (pIStuts->MtdState[pIStuts->SensorStat]&0x01) ;
		if(Mtdstatus)
		{
			struct timeval tv;
			for(int i = 0; i < MAX_MTDRIGION_NUM; i++)
			{
				while(!m_pMovDetector->isWait(i))
				{
					tv.tv_sec = 0;
					tv.tv_usec = (10%1000)*1000;
					select(0, NULL, NULL, NULL, &tv);
				}
				if(m_pMovDetector->isWait(i))
				{
					m_pMovDetector->mvOpen(i);	
					dynamic_config(VP_CFG_MvDetect, 1,NULL);
					chooseDetect = 10;					
				}
			}
		}
		else
		{
			for(int i = 0; i < MAX_MTDRIGION_NUM; i++)
			{
				if(m_pMovDetector->isRun(i))
				{
					dynamic_config(VP_CFG_MvDetect, 0,NULL);
					m_pMovDetector->mvClose(i);					
				}	
			}
		}
	}
	if(msgId == MSGID_EXT_MVDETECTSELECT)
	{
		int MtdSelect = (pIStuts->MtdSelect[pIStuts->SensorStat]);
		if(ipc_eMTD_Next == MtdSelect)
		{
			forwardflag = true;
		}
		else if(ipc_eMTD_Prev == MtdSelect)
		{
			backflag = true;
		}
		else if(ipc_eMTD_Select == MtdSelect)
		{

		}
	}
	if(msgId == MSGID_EXT_MVDETECT_SETRIGIONSTAT)
	{
		setrigion_flagv20 = pIStuts->MtdSetRigion;
	}
	if(msgId == MSGID_EXT_MVDETECT_SETRIGION)
	{
		memcpy(&mtdrigionv20, &pMenuStatus->Mtdmouseclick, sizeof(mtdrigionv20));
		updateredgrid();
	}
		
#endif
	if(msgId == MSGID_EXT_INPUT_ALGOSDRECT)
	{
	 	algOsdRect=pIStuts->Imgalgosdrect;
	}
	
	if(msgId == MSGID_EXT_MENUSWITCH)
	{
		m_display.m_menuindex = extMenuCtrl.MenuStat;
	}
	if(msgId == MSGID_EXT_UPMENU)
	{
		int menustate = extMenuCtrl.MenuStat;
		int pointer = m_display.dismenuarray[menustate].pointer;
		if((menustate >= mainmenu2) && (menustate <= submenu_handleMatchPoints))
		{
			if(pointer > 0)
			{
				m_display.disMenuBuf[menustate][pointer].color = 2;
				m_display.dismenuarray[menustate].pointer= extMenuCtrl.menuarray[menustate].pointer;
				if(menustate == submenu_setcom){
					m_display.m_currentMenuPos[menustate][m_display.dismenuarray[menustate].pointer].isShow = true;
					show_circle_pointer = true;
				}
				else{
					m_display.m_currentMenuPos[menustate][m_display.dismenuarray[menustate].pointer].isShow = false;
					show_circle_pointer = false;
				}
				m_display.m_currentSecondMenuIndex = m_display.dismenuarray[menustate].pointer; // add by swj
				m_display.m_currentFirstMenuIndex = menustate;
				
				m_display.disMenuBuf[menustate][m_display.dismenuarray[menustate].pointer].color = 3;
			}
		}
	}
	if(msgId == MSGID_EXT_DOWNMENU)
	{
		int menustate = extMenuCtrl.MenuStat;
		int pointer = m_display.dismenuarray[menustate].pointer;
		if((menustate >= mainmenu2) && (menustate <= submenu_handleMatchPoints))
		{
			if(pointer < extMenuCtrl.menuarray[menustate].submenu_cnt - 1)
			{
				m_display.disMenuBuf[menustate][pointer].color = 2;
				m_display.dismenuarray[menustate].pointer = extMenuCtrl.menuarray[menustate].pointer;
				m_display.disMenuBuf[menustate][m_display.dismenuarray[menustate].pointer].color = 3;

				if(menustate == submenu_setcom){
					m_display.m_currentMenuPos[menustate][m_display.dismenuarray[menustate].pointer].isShow = true;
					show_circle_pointer = true;
				}
				else{
					m_display.m_currentMenuPos[menustate][m_display.dismenuarray[menustate].pointer].isShow = false;
					show_circle_pointer = false;
				}
				m_display.m_currentSecondMenuIndex = m_display.dismenuarray[menustate].pointer; // add by swj
				m_display.m_currentFirstMenuIndex = menustate;


			}
		}
	}
	if(msgId == MSGID_EXT_SMR)
	{
		getmtdedge();// press Enter
	}
	if(msgId == MSGID_EXT_SETRESOL)
	{
        unsigned char resolbuf[maxresolid][128] = {
            "格式 1920x1080@60Hz","格式 1024x768@60Hz","格式 1280x1024@60Hz"};
		m_display.disresol_type_tmp = pMenuStatus->resol_type_tmp;
		swprintf(m_display.disMenu[submenu_setimg][1], 33, L"%s", resolbuf[m_display.disresol_type_tmp]);
	}
	if(msgId == MSGID_EXT_SAVERESOL)
	{
		m_display.disresol_type = pMenuStatus->resol_type;
		udoutputresol(m_display.disresol_type);
		writeshell(m_display.disresol_type);
	}
	if(msgId == MSGID_EXT_SETMTDNUM)
	{
		if((pMenuStatus->osd_mudnum < MIN_MTDTARGET_NUM) || (pMenuStatus->osd_mudnum > MAX_MTDTARGET_NUM))
			swprintf(m_display.disMenu[submenu_mtd][1], 33, L"目标个数     %d(超出范围%d~%d)", pMenuStatus->osd_mudnum,MIN_MTDTARGET_NUM,MAX_MTDTARGET_NUM);
		else
			swprintf(m_display.disMenu[submenu_mtd][1], 33, L"目标个数     %d", pMenuStatus->osd_mudnum);
	}
	if(msgId == MSGID_EXT_SETMTDTRKTIME)
	{
		if((pMenuStatus->osd_trktime < MIN_MTDTRKTIME) || (pMenuStatus->osd_trktime > MAX_MTDTRKTIME))
			swprintf(m_display.disMenu[submenu_mtd][2], 33, L"跟踪持续时间 %d秒(超出范围%d~%d)", pMenuStatus->osd_trktime,MIN_MTDTRKTIME,MAX_MTDTRKTIME);
		else
			swprintf(m_display.disMenu[submenu_mtd][2], 33, L"跟踪持续时间 %d秒", pMenuStatus->osd_trktime);
	}
	if(msgId == MSGID_EXT_SETMTDMAXSIZE)
	{
		if((pMenuStatus->osd_maxsize < minsize) || (pMenuStatus->osd_maxsize > MAX_MTDMAXSIZE))
			swprintf(m_display.disMenu[submenu_mtd][3], 33, L"最大目标面积 %d(超出范围)", pMenuStatus->osd_maxsize);
		else
			swprintf(m_display.disMenu[submenu_mtd][3], 33, L"最大目标面积 %d", pMenuStatus->osd_maxsize);
	}
	if(msgId == MSGID_EXT_SETMTDMINSIZE)
	{
		if((pMenuStatus->osd_minsize < MIN_MTDMINSIZE) || (pMenuStatus->osd_minsize > maxsize))
			swprintf(m_display.disMenu[submenu_mtd][4], 33, L"最小目标面积 %d(超出范围)", pMenuStatus->osd_minsize);
		else
			swprintf(m_display.disMenu[submenu_mtd][4], 33, L"最小目标面积 %d", pMenuStatus->osd_minsize);
	}
	if(msgId == MSGID_EXT_SETMTDSENSI)
	{
		if((pMenuStatus->osd_sensi < MIN_MTDSENSI) || (pMenuStatus->osd_sensi > MAX_MTDSENSI))
			swprintf(m_display.disMenu[submenu_mtd][5], 33, L"灵敏度       %d(超出范围%d~%d)", pMenuStatus->osd_sensi,MIN_MTDMINSIZE,MAX_MTDMAXSIZE);
		else
			swprintf(m_display.disMenu[submenu_mtd][5], 33, L"灵敏度       %d", pMenuStatus->osd_sensi);
	}
	
	if(msgId == MSGID_EXT_SETBAUD){

		unsigned char baudlbuf[MAX_BAUDID][128] = {
			"波特率     2400","波特率     4800","波特率     9600","波特率     115200"};

		m_display.disbaud_type = pMenuStatus->baud_type;
		swprintf(m_display.disMenu[submenu_setcom][0], 33, L"%s", baudlbuf[m_display.disbaud_type]);

		switch(m_display.disbaud_type){
			case 0:
				m_display.saveBaudrate = 2400;
				break;
			case 1:
				m_display.saveBaudrate = 4800;
				break;
			case 2:
				m_display.saveBaudrate = 9600;
				break;
			case 3:
				m_display.saveBaudrate = 115200;
				break;
			default:
				break;
		}

		if(changeComBaud == true) {
			changeComBaud = false;
			printf("\r\nOOOOOOOO%d, %dOOOOOOOOOOO Save BaudRate !!!\r\n",m_display.ballAddressID,m_display.saveBaudrate);
				trkmsg.cmd_ID = ballbaud;
				memcpy(&trkmsg.param[0],(void *)&(m_display.ballAddressID), sizeof(m_display.ballAddressID));
				memcpy(&trkmsg.param[4],(void *)&(m_display.saveBaudrate), sizeof(m_display.saveBaudrate));
						
				ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);


				bool retvalue = m_display.saveComConfigs("ctrl_config.yml");
				if(!retvalue) {
					printf("\r\nXXXXXXXXXXXXXXXXXXXXXXXXXX  Save COM config Failed !!!\r\n");		
				}

		}
		
	}
	
	
}

int CProcess::updateredgrid()
{
	int interval_w = gun_resolu[0] / GRID_CNT_X;
	int interval_h = gun_resolu[1] / GRID_CNT_Y;
	
	int x = mtdrigionv20.x / interval_w;
	int y = mtdrigionv20.y / interval_h;
	x = x >= GRID_CNT_X ? GRID_CNT_X -1 : x;
	y = y >= GRID_CNT_Y ? GRID_CNT_Y -1 : y;
	
	if((0 == mtdrigionv20.button) && (0 == mtdrigionv20.state))
	{
			grid19x10[x][y].state= 1;
			m_click_v20L = 1;
			mRectv20L.x1 = x;
			mRectv20L.y1 = y;
	}
	else if((0 == mtdrigionv20.button) && (1 == mtdrigionv20.state))
	{
			m_click_v20L = 0;
			mRectv20L.x2 = x;
			mRectv20L.y2 = y;
			updateredgridfrrectL();
	}
	if((2 == mtdrigionv20.button) && (0 == mtdrigionv20.state))
	{
			grid19x10[x][y].state = 0;
			m_click_v20R = 1;
			mRectv20R.x1 = x;
			mRectv20R.y1 = y;
	}
	if((2 == mtdrigionv20.button) && (1 == mtdrigionv20.state))
	{
			m_click_v20R = 0;
			mRectv20R.x2 = x;
			mRectv20R.y2 = y;
			updateredgridfrrectR();
	}
}

int CProcess::updateredgridfrrectL()
{
	for(int i = mRectv20L.x1; i <= mRectv20L.x2; i++)
		for(int j = mRectv20L.y1; j <= mRectv20L.y2; j++)
			grid19x10[i][j].state= 1;
}

int CProcess::updateredgridfrrectR()
{
	for(int i = mRectv20R.x1; i <= mRectv20R.x2; i++)
		for(int j = mRectv20R.y1; j <= mRectv20R.y2; j++)
			grid19x10[i][j].state= 0;
}

int CProcess::getmtdedge()
{
	usopencvapi2();
}

int CProcess::usopencvapi2()
{
	int psize = 0;
	int interval_w = gun_resolu[0] / GRID_CNT_X;
	int interval_h = gun_resolu[1] / GRID_CNT_Y;

	unsigned int curId;
	if(m_display.g_CurDisplayMode == MAIN_VIEW) {
		curId = 1;	
	}else{
		curId = m_curChId;
	}
	
	Mat mask = Mat::zeros(gun_resolu[1], gun_resolu[0], CV_8UC1);
	Rect rect;
	int flag = 0;

	for(int i = 0; i < GRID_CNT_X; i++)
		for(int j = 0; j < GRID_CNT_Y; j++)
		{
			if(grid19x10[i][j].state == 1)
			{
				rect.x = i * interval_w;
				rect.y = j * interval_h;
				rect.width = interval_w;
				rect.height = interval_h;
				mask(rect).setTo(255);
				flag = 1;
			}
		}

	if(flag == 0)
		return -1;
	
	for(int i = 1; i < GRID_CNT_X; i++)
		for(int j = 1; j < GRID_CNT_Y; j++)
			{
				int xx = i * interval_w;
				int yy = j * interval_h;
				mask.at<uchar>(yy,xx) = 0;
				mask.at<uchar>(yy,xx-1) = 0;
				mask.at<uchar>(yy,xx+1) = 0;
				mask.at<uchar>(yy-1,xx) = 0;
				mask.at<uchar>(yy+1,xx) = 0;
			}
				
		
	float floatx,floaty;
	int setx, sety = 0;
	std::vector< std::vector< cv::Point > > polyWarnRoi;
	std::vector< std::vector< cv::Point > > contours;
	findContours(mask,contours, noArray(), RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
	Mat dstImage = Mat::zeros(mask.size(),CV_8UC1);
	drawContours(dstImage,contours, -1, Scalar(255,10,10));

	edge_contours = contours;
	polyWarnRoi = contours;

	for(int i = 0; i < contours.size(); i++)
	{
		int psize = contours[i].size();
		floatx = contours[i][0].x;
		floaty = contours[i][0].y;
		map1080p2normal_point(&floatx, &floaty);
		mapnormal2curchannel_point(&floatx, &floaty, vdisWH[curId][0], vdisWH[curId][1]);
		setx = floatx;
		sety = floaty;
		polyWarnRoi[i][0] = cv::Point(setx, sety);
		mapfullscreen2gun_pointv20(&setx, &sety);
		edge_contours[i][0].x = setx;
		edge_contours[i][0].y = sety;
		for(int j = 1; j < contours[i].size(); j++)
		{
			floatx = contours[i][psize-j].x;
			floaty = contours[i][psize-j].y;
			map1080p2normal_point(&floatx, &floaty);
			mapnormal2curchannel_point(&floatx, &floaty, vdisWH[curId][0], vdisWH[curId][1]);

			setx = floatx;
			sety = floaty;
			polyWarnRoi[i][j] = cv::Point(setx, sety);

			mapfullscreen2gun_pointv20(&setx, &sety);
			edge_contours[i][j].x = setx;
			edge_contours[i][j].y = sety;
		}
	}

	if(contours.size() > 3)
	{
        swprintf(m_display.disMtd[0][4], 33, L"错误，检测区域大于3个");
	}
	else
	{
        swprintf(m_display.disMtd[0][4], 33, L"检测区域:%d个", contours.size());
	}

	if(edge_contours.size() != 0)
	{
		SaveMtdSelectArea("SaveMtdArea.yml", edge_contours);
	}


	for(int i = 0; i < contours.size(); i++)
	{
		m_pMovDetector->setWarningRoi(polyWarnRoi[i], i);
	}
}


int CProcess::setresol(int resoltype)
{
	switch(resoltype)
	{
		case r1920x1080_f60:
			system("xrandr -s 1920x1080_60.00");
			outputWHF[0] = 1920;
			outputWHF[1] = 1080;
			outputWHF[2] = 60;
			break;
		case r1024x768_f60:
			system("xrandr -s 1024x768_60.01");
			outputWHF[0] = 1024;
			outputWHF[1] = 768;
			outputWHF[2] = 60;
			break;
		case r1280x1024_f60:
			system("xrandr -s 1280x1024_60.00");
			outputWHF[0] = 1280;
			outputWHF[1] = 1024;
			outputWHF[2] = 60;
			break;
		default:
			break;	
	}
	this->setDisplayResolution(m_display, outputWHF[0],outputWHF[1]);
	this->setStaticScreenResolution(outputWHF[0], outputWHF[1]);
	glutFullScreen();
}

int CProcess::udoutputresol(int resoltype)
{
	SENDST test;
	CMD_SETCONFIG cmdsetconfig;
	int send_flag = 1;

	test.cmd_ID = setconfig;
	cmdsetconfig.block = 51;
	cmdsetconfig.field = 5;

	switch(resoltype)
	{
		case r1920x1080_f60:
			cmdsetconfig.value = 5;
			break;
		case r1024x768_f60:
			cmdsetconfig.value = 7;
			break;
		case r1280x1024_f60:
			cmdsetconfig.value = 6;
			break;
		default:
			send_flag = 0;
			break;	
	}

	if(send_flag)
	{
		memcpy(test.param, &cmdsetconfig, sizeof(cmdsetconfig));
		ipc_sendmsg(&test, IPC_FRIMG_MSG);
	}
}

int CProcess::writeshell(int resoltype)
{
	FILE *fp = NULL;
	char contents[256] = {0};

	switch(resoltype)
	{
		case r1920x1080_f60:
			sprintf(contents,"#! /bin/bash\n" \
               "xrandr -s 1920x1080_60.00\n");
			break;
		case r1024x768_f60:
			sprintf(contents,"#! /bin/bash\n" \
               "xrandr -s 1024x768_60.01\n");
			break;
		case r1280x1024_f60:
			sprintf(contents,"#! /bin/bash\n" \
               "xrandr -s 1280x1024_60.00\n");
			break;
		default:
			break;	
	}

	if(( fp = fopen("/home/ubuntu/dss_bin/setresol.sh", "w"))==NULL)
	{
		fprintf(stderr,"create shell file error");
		return -1;
	}
	fprintf(fp, "%s", contents);
	fclose(fp);
	return 0;
}

/////////////////////////////////////////////////////
//int majormmtid=0;

 int  CProcess::MSGAPI_initial()
{
   MSGDRIV_Handle handle=&g_MsgDrvObj;
    assert(handle != NULL);
    memset(handle->msgTab, 0, sizeof(MSGTAB_Class) * MAX_MSG_NUM);
//MSGID_EXT_INPUT_MTD_SELECT
    MSGDRIV_attachMsgFun(handle,    MSGID_SYS_INIT,           			MSGAPI_init_device,       		0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_SENSOR,           	MSGAPI_inputsensor,       		0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_PICPCROP,      		MSGAPI_croppicp,       		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_TRACK,          	MSGAPI_inputtrack,     		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ENMTD,              MSGAPI_inpumtd,       		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_MTD_SELECT,     	MSGAPI_inpumtdSelect,    		0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_AIMPOS,          	MSGAPI_setAimRefine,    		0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_AIMSIZE,          	MSGAPI_setAimSize,    		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ENENHAN,           	MSGAPI_inpuenhance,       	    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ENBDT,           	MSGAPI_inputbdt,         		0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ENZOOM,           	MSGAPI_inputzoom,               0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ENFREZZ,           	MSGAPI_inputfrezz,              0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_MTD_SELECT,      	MSGAPI_inputmmtselect,          0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_AXISPOS,     	  	MSGAPI_inputpositon,            0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_COAST,             	MSGAPI_inputcoast,              0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_FOVSELECT,          MSGAPI_inputfovselect,          0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_FOVSTAT,            MSGAPI_inputfovchange,          0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_SEARCHMOD,          MSGAPI_inputsearchmod,          0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_VIDEOEN,            MSGAPI_inputvideotect,          0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_MMTSHOW,            MSGAPI_mmtshow,                 0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_FOVCMD,             MSGAPI_FOVcmd,                 	0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_CFGSAVE,            MSGAPI_SaveCfgcmd,              0);	
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_MVDETECT,             	MSGAPI_setMtdState,             0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_MVDETECTSELECT,           MSGAPI_setMtdSelect,            0);	
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_MVDETECT_SETRIGIONSTAT,       MSGAPI_setMtdSetRigionStat,            0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_MVDETECT_SETRIGION,       MSGAPI_setMtdSetRigion,            0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_UPDATE_ALG,             	MSGAPI_update_alg,              0);	
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_UPDATE_OSD,             	MSGAPI_update_osd,              0);	
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_UPDATE_CAMERA,            MSGAPI_update_camera,           0);	
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ALGOSDRECT,         MSGAPI_input_algosdrect,        0);	
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_SETCURPOS,     MSGAPI_update_ballPos,        	0);	
	MSGDRIV_attachMsgFun(handle,    MSGID_EXT_MENUSWITCH,     MSGAPI_update_menuindex,        	0);
	MSGDRIV_attachMsgFun(handle,    MSGID_EXT_UPMENU,     MSGAPI_up_menu,        	0);
	MSGDRIV_attachMsgFun(handle,    MSGID_EXT_DOWNMENU,     MSGAPI_down_menu,        	0);
	MSGDRIV_attachMsgFun(handle,    MSGID_EXT_SMR,     MSGAPI_save_mtdrigion,        	0);
	MSGDRIV_attachMsgFun(handle,    MSGID_EXT_SETRESOL,     MSGAPI_set_resol,        	0);
	MSGDRIV_attachMsgFun(handle,	MSGID_EXT_SETBAUD, 	MSGAPI_set_baud,			0);
	MSGDRIV_attachMsgFun(handle,	MSGID_EXT_SAVERESOL, 	MSGAPI_save_resol,			0);
	MSGDRIV_attachMsgFun(handle,	MSGID_EXT_SETMTDNUM, 	MSGAPI_set_mtdnum,			0);
	MSGDRIV_attachMsgFun(handle,	MSGID_EXT_SETMTDTRKTIME, 	MSGAPI_set_mtdtrktime,			0);
	MSGDRIV_attachMsgFun(handle,	MSGID_EXT_SETMTDMAXSIZE, 	MSGAPI_set_mtdmaxsize,			0);
	MSGDRIV_attachMsgFun(handle,	MSGID_EXT_SETMTDMINSIZE, 	MSGAPI_set_mtdminsize,			0);
	MSGDRIV_attachMsgFun(handle,	MSGID_EXT_SETMTDSENSI, 	MSGAPI_set_mtdsensi,			0);

    return 0;
}


void CProcess::MSGAPI_init_device(long lParam )
{
	sThis->msgdriv_event(MSGID_SYS_INIT,NULL);
}

  void CProcess::MSGAPI_inputsensor(long lParam )
{
	CMD_EXT *pIStuts = sThis->extInCtrl;	
	sThis->msgdriv_event(MSGID_EXT_INPUT_SENSOR,NULL);
}

void CProcess::MSGAPI_picp(long lParam )
{
	CMD_EXT *pIStuts = sThis->extInCtrl;
		if(pIStuts->PicpSensorStat == 0xFF)
			pIStuts->PicpSensorStat = (pIStuts->SensorStat + 1)%eSen_Max;
		else
			pIStuts->PicpSensorStat = 0xFF;
	
	sThis->msgdriv_event(MSGID_EXT_INPUT_ENPICP,NULL);
}


void CProcess::MSGAPI_croppicp(long lParam )
{
	//sThis->msgdriv_event(MSGID_EXT_INPUT_PICPCROP,NULL);
	sThis->msgdriv_event(MSGID_EXT_INPUT_ENPICP,NULL);
}

void CProcess::MSGAPI_inputtrack(long lParam )
{
	CMD_EXT *pIStuts = sThis->extInCtrl;
	sThis->msgdriv_event(MSGID_EXT_INPUT_TRACK,NULL);
}


void CProcess::MSGAPI_inpumtd(long lParam )
{
	sThis->msgdriv_event(MSGID_EXT_INPUT_ENMTD,NULL);
}

void CProcess::MSGAPI_inpumtdSelect(long lParam )
{
	CMD_EXT *pIStuts = sThis->extInCtrl;
	int i;
	if(pIStuts->MMTTempStat==3)
	{
		for(i=0;i<MAX_TARGET_NUMBER;i++)
		{
			if(sThis->m_mtd[pIStuts->SensorStat]->tg[majormmtid].valid==1)
			{
				//majormmtid++;
				majormmtid=(majormmtid+1)%MAX_TARGET_NUMBER;
			}
		}	
	}
	else if(pIStuts->MMTTempStat==4)
	{
		for(i=0;i<MAX_TARGET_NUMBER;i++)
		{
			if(sThis->m_mtd[pIStuts->SensorStat]->tg[majormmtid].valid==1)
			{
				//majormmtid++;
				if(majormmtid>0)
					majormmtid=(majormmtid-1);
				else
				{
					majormmtid=MAX_TARGET_NUMBER-1;
				}
			}
		}
	}
	OSA_printf("MSGAPI_inpumtdSelect\n");
}


void CProcess::MSGAPI_inpuenhance(long lParam )
{
	sThis->msgdriv_event(MSGID_EXT_INPUT_ENENHAN,NULL);
}

void CProcess::MSGAPI_setMtdState(long lParam )
{
	sThis->msgdriv_event(MSGID_EXT_MVDETECT,NULL);
}
void CProcess::MSGAPI_setMtdSelect(long lParam )
{

	sThis->msgdriv_event(MSGID_EXT_MVDETECTSELECT,NULL);
}

void CProcess::MSGAPI_setMtdSetRigionStat(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_MVDETECT_SETRIGIONSTAT,NULL);
}

void CProcess::MSGAPI_setMtdSetRigion(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_MVDETECT_SETRIGION,NULL);
}

void CProcess::MSGAPI_setAimRefine(long lParam)
{
	CMD_EXT *pIStuts = sThis->extInCtrl;

	if(pIStuts->aimRectMoveStepX==eTrk_ref_left)
	{
		pIStuts->aimRectMoveStepX=-1;
	}
	else if(pIStuts->aimRectMoveStepX==eTrk_ref_right)
	{
		pIStuts->aimRectMoveStepX=1;
	}
	if(pIStuts->aimRectMoveStepY==eTrk_ref_up)
	{
		pIStuts->aimRectMoveStepY=-1;
	}
	else if(pIStuts->aimRectMoveStepY==eTrk_ref_down)
	{
		pIStuts->aimRectMoveStepY=1;
	}
	sThis->msgdriv_event(MSGID_EXT_INPUT_AIMPOS,NULL);
}


void CProcess::MSGAPI_setAimSize(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_INPUT_AIMSIZE,NULL);
}

void CProcess::MSGAPI_inputbdt(long lParam )
{
	CMD_EXT *pIStuts = sThis->extInCtrl;
	if(pIStuts->TvCollimation!=1)
		pIStuts->ImgBlobDetect[pIStuts->SensorStat] = eImgAlg_Disable;
	else
		pIStuts->ImgBlobDetect[pIStuts->SensorStat] = eImgAlg_Enable;
	sThis->msgdriv_event(MSGID_EXT_INPUT_ENBDT,NULL);	
	OSA_printf("fun=%s line=%d \n",__func__,__LINE__);
}


void CProcess::MSGAPI_inputzoom(long lParam )
{
	CMD_EXT *pIStuts = sThis->extInCtrl;
	sThis->msgdriv_event(MSGID_EXT_INPUT_ENZOOM,NULL);
}


void CProcess::MSGAPI_inputfrezz(long lParam )
{
	CMD_EXT *pIStuts = sThis->extInCtrl;	
	if( pIStuts->FrCollimation==1)
	{
		pIStuts->PicpSensorStat=0;//tv picp sensor
		sThis->msgdriv_event(MSGID_EXT_INPUT_ENPICP, NULL);
		//dong jie chuang kou
		pIStuts->ImgFrezzStat[pIStuts->SensorStat] = eImgAlg_Enable;
		sThis->msgdriv_event(MSGID_EXT_INPUT_ENFREZZ,NULL);
	}
	else
	{	
		if((pIStuts->PicpSensorStatpri!=0))//tui picp the sensor is tv
		{
			pIStuts->PicpSensorStatpri=pIStuts->PicpSensorStat=2;//tui chu picp
			sThis->msgdriv_event(MSGID_EXT_INPUT_ENPICP, NULL);
			OSA_printf("MSGAPI_inputfrezz*****************************************disable \n");
		}
		else
		{
			pIStuts->PicpSensorStat=0;
		}
		//tui chu dong jie chuang kou
		pIStuts->ImgFrezzStat[pIStuts->SensorStat] = eImgAlg_Disable;
		sThis->msgdriv_event(MSGID_EXT_INPUT_ENFREZZ,NULL);
		
		OSA_printf("the*****************************************disable PicpSensorStatpri=%d\n",pIStuts->PicpSensorStatpri);
	}

			
	
	OSA_printf("%s\n",__func__);
}


void CProcess::MSGAPI_inputmmtselect(long lParam )
{
	CMD_EXT *pIStuts = sThis->extInCtrl;
	if(pIStuts->MmtSelect[pIStuts->SensorStat]  ==eMMT_Next)
		majormmtid=(majormmtid+1)%MAX_TARGET_NUMBER;
	else if(pIStuts->MmtSelect[pIStuts->SensorStat]  ==  eMMT_Prev)
	{
		majormmtid=(majormmtid-1+MAX_TARGET_NUMBER)%MAX_TARGET_NUMBER;
	}
	OSA_printf("%s\n",__func__);
}



void CProcess::MSGAPI_inputpositon(long lParam )
{
	CMD_EXT *pIStuts = sThis->extInCtrl;

	if((pIStuts->AxisPosX[pIStuts->SensorStat]>=50)&&(pIStuts->AxisPosX[pIStuts->SensorStat]<=vcapWH[pIStuts->SensorStat][0]-50))
	{
		if(pIStuts->axisMoveStepX != 0)
		{
			pIStuts->AxisPosX[pIStuts->SensorStat] += pIStuts->axisMoveStepX;
			pIStuts->axisMoveStepX = 0;
		}	
		pIStuts->unitAimX = pIStuts->AxisPosX[pIStuts->SensorStat];
	}
	if((pIStuts->AxisPosY[pIStuts->SensorStat]>=50)&&(pIStuts->AxisPosY[pIStuts->SensorStat]<=vcapWH[pIStuts->SensorStat][1]-50))
	{
		if(pIStuts->axisMoveStepY != 0)
		{
			pIStuts->AxisPosY[pIStuts->SensorStat] += pIStuts->axisMoveStepY;
			pIStuts->axisMoveStepY = 0;
		}
		pIStuts->unitAimY = pIStuts->AxisPosY[pIStuts->SensorStat];
	}
	
	OSA_printf("%s   THE=unitAimX=%d unitAxisY=%d\n",__func__,pIStuts->opticAxisPosX[pIStuts->SensorStat],pIStuts->opticAxisPosY[pIStuts->SensorStat]);
}

void CProcess::MSGAPI_inputcoast(long lParam )
{

	sThis->msgdriv_event(MSGID_EXT_INPUT_COAST,NULL);
	
	//printf("%s\n",__func__);
}

void CProcess::MSGAPI_inputfovselect(long lParam )
{

	CMD_EXT *pIStuts = sThis->extInCtrl;

	if(pIStuts->changeSensorFlag == 0)
	{
		//OSA_printf("FovStat = %d SensorStat=%d\n",pIStuts->FovStat,pIStuts->SensorStat);
		if(video_pal == pIStuts->SensorStat)
		{
		#if __TRACK__
			if(pIStuts->FovStat == 1)
				sThis->Track_fovreacq( 2400,pIStuts->SensorStat,0);
			else if(pIStuts->FovStat == 3)
				sThis->Track_fovreacq( 330,pIStuts->SensorStat,0);
			else if(pIStuts->FovStat == 4)	
				sThis->Track_fovreacq( 110,pIStuts->SensorStat,0);					
			else if(pIStuts->FovStat == 5)
				sThis->Track_fovreacq( 55,pIStuts->SensorStat,0);
			
		#endif
		}
		else if((video_gaoqing0 == pIStuts->SensorStat)||(video_gaoqing == pIStuts->SensorStat)||(video_gaoqing2 == pIStuts->SensorStat)||(video_gaoqing3 == pIStuts->SensorStat)){
		#if __TRACK__
			if(pIStuts->FovStat == 1)
				sThis->Track_fovreacq( 4000,pIStuts->SensorStat,0);
			else if(pIStuts->FovStat == 4)
				sThis->Track_fovreacq( 120,pIStuts->SensorStat,0);
			else if(pIStuts->FovStat == 5)
				sThis->Track_fovreacq( 60,pIStuts->SensorStat,0);
		#endif
		}

		//OSA_printf("fovselectXY(%f,%f),WH(%f,%f)\n",sThis->trackinfo_obj->reAcqRect.x,sThis->trackinfo_obj->reAcqRect.y,sThis->trackinfo_obj->reAcqRect.width,sThis->trackinfo_obj->reAcqRect.height);
		#if __TRACK__
		if(pIStuts->AvtTrkStat){	
			sThis->Track_reacq(sThis->trackinfo_obj->reAcqRect,2);
		}
		#endif
	}
}

void CProcess::MSGAPI_inputfovchange(long lParam )
{

	CMD_EXT *pIStuts = sThis->extInCtrl;

	//OSA_printf("%s:unitFovAngle = %f\n",__func__,pIStuts->unitFovAngle[pIStuts->SensorStat]);
	#if __TRACK__
	sThis->Track_fovreacq( pIStuts->unitFovAngle[pIStuts->SensorStat],pIStuts->SensorStat,0);
	#endif
}


void CProcess::MSGAPI_inputsearchmod(long lParam )
{
}


 void CProcess::MSGAPI_inputvideotect(long lParam )
{
	OSA_printf("MSGAPI_inputvideotect*******************\n");
	sThis->msgdriv_event(MSGID_EXT_INPUT_VIDEOEN,NULL);
}

  void CProcess::MSGAPI_mmtshow(long lParam )
{
	OSA_printf("MSGAPI_mmtshow\n");
}
void CProcess::MSGAPI_FOVcmd(long lParam )
{
	CMD_EXT *pIStuts = sThis->extInCtrl;
	if((pIStuts->FovCtrl==5)&&(pIStuts->SensorStat==0))
		sThis->tvzoomStat=1;
	else
		sThis->tvzoomStat=0;
}
void CProcess::MSGAPI_SaveCfgcmd(long lParam )
{
	sThis->msgdriv_event(MSGID_EXT_INPUT_CFGSAVE,NULL);
}	

void CProcess::initAcqRect()
{	
	CMD_EXT *pIStuts = extInCtrl;
	pIStuts->AcqRectW[0] = gConfig_Osd_param.ch0_acqRect_width;
	pIStuts->AcqRectW[1] = gConfig_Osd_param.ch1_acqRect_width;
	pIStuts->AcqRectW[2] = gConfig_Osd_param.ch2_acqRect_width;
	pIStuts->AcqRectW[3] = gConfig_Osd_param.ch3_acqRect_width;
	pIStuts->AcqRectW[4] = gConfig_Osd_param.ch4_acqRect_width;
	pIStuts->AcqRectH[0]  = gConfig_Osd_param.ch0_acqRect_height;
	pIStuts->AcqRectH[1]  = gConfig_Osd_param.ch1_acqRect_height;
	pIStuts->AcqRectH[2]  = gConfig_Osd_param.ch2_acqRect_height;
	pIStuts->AcqRectH[3]  = gConfig_Osd_param.ch3_acqRect_height;
	pIStuts->AcqRectH[4]  = gConfig_Osd_param.ch4_acqRect_height;
	return ;
}

void CProcess::initAimRect()
{
	CMD_EXT *pIStuts = extInCtrl;
	
	
	return ;
}
void CProcess::MSGAPI_update_osd(long lParam)
{
	plat->update_param_osd();
}

void CProcess::update_param_osd()
{
	CMD_EXT *pIStuts = extInCtrl;
	pIStuts->SensorStatBegin 		= gConfig_Osd_param.MAIN_Sensor;
	pIStuts->osdTextShow 			= gConfig_Osd_param.OSD_text_show;
	pIStuts->osdDrawShow 			= gConfig_Osd_param.OSD_draw_show;
	//pIStuts->crossDrawShow 			= gConfig_Osd_param.CROSS_draw_show;
	//memcpy((void *)pIStuts->crossDrawShow, (void *)&gConfig_Osd_param.CROSS_draw_show, sizeof(gConfig_Osd_param.CROSS_draw_show));
	for(int i=0; i< 5; i++) {
		pIStuts->crossDrawShow[i]=gConfig_Osd_param.CROSS_draw_show[i];
		pIStuts->osdBoxShow[i] = gConfig_Osd_param.osdBoxShow[i];
		pIStuts->osdChidIDShow[i] = gConfig_Osd_param.osdChidIDShow[i];
		pIStuts->osdChidNameShow[i] = gConfig_Osd_param.osdChidNmaeShow[i];
		
	}


	pIStuts->osdTextColor 			=  gConfig_Osd_param.OSD_text_color;
	pIStuts->osdTextAlpha			=  gConfig_Osd_param.OSD_text_alpha;
	pIStuts->osdTextFont			= gConfig_Osd_param.OSD_text_font;
	pIStuts->osdTextSize			= gConfig_Osd_param.OSD_text_size;
	pIStuts->osdDrawColor 			= gConfig_Osd_param.OSD_draw_color;
	pIStuts->AcqRectW[0] 			= gConfig_Osd_param.ch0_acqRect_width;
	pIStuts->AcqRectW[1] 			= gConfig_Osd_param.ch1_acqRect_width;
	pIStuts->AcqRectW[2] 			= gConfig_Osd_param.ch2_acqRect_width;
	pIStuts->AcqRectW[3] 			= gConfig_Osd_param.ch3_acqRect_width;
	pIStuts->AcqRectW[4] 			= gConfig_Osd_param.ch4_acqRect_width;
	pIStuts->AcqRectW[5] 			= gConfig_Osd_param.ch5_acqRect_width;
	pIStuts->AcqRectH[0] 			= gConfig_Osd_param.ch0_acqRect_height;
	pIStuts->AcqRectH[1] 			= gConfig_Osd_param.ch1_acqRect_height;
	pIStuts->AcqRectH[2] 			= gConfig_Osd_param.ch2_acqRect_height;
	pIStuts->AcqRectH[3] 			= gConfig_Osd_param.ch3_acqRect_height;
	pIStuts->AcqRectH[4] 			= gConfig_Osd_param.ch4_acqRect_height;
	pIStuts->AcqRectH[5] 			= gConfig_Osd_param.ch5_acqRect_height;

	pIStuts->AimW[0] 				= gConfig_Osd_param.ch0_aim_width;
	pIStuts->AimW[1] 				= gConfig_Osd_param.ch1_aim_width;
	pIStuts->AimW[2] 				= gConfig_Osd_param.ch2_aim_width;
	pIStuts->AimW[3] 				= gConfig_Osd_param.ch3_aim_width;
	pIStuts->AimW[4] 				= gConfig_Osd_param.ch4_aim_width;
	pIStuts->AimW[5] 				= gConfig_Osd_param.ch5_aim_width;
	pIStuts->AimH[0] 				= gConfig_Osd_param.ch0_aim_height;
	pIStuts->AimH[1] 				= gConfig_Osd_param.ch1_aim_height;
	pIStuts->AimH[2] 				= gConfig_Osd_param.ch2_aim_height;
	pIStuts->AimH[3] 				= gConfig_Osd_param.ch3_aim_height;
	pIStuts->AimH[4] 				= gConfig_Osd_param.ch4_aim_height;
	pIStuts->AimH[5] 				= gConfig_Osd_param.ch5_aim_height;

	m_acqRectW 	= pIStuts->AimW[pIStuts->SensorStat];
	m_acqRectH  = pIStuts->AimH[pIStuts->SensorStat];
	
	m_display.disptimeEnable = gConfig_Osd_param.Timedisp_9;
	m_display.m_bOsd = pIStuts->osdDrawShow;
	m_display.m_crossOsd = pIStuts->crossDrawShow;
	
	//pIStuts->crossAxisWidth 		= gConfig_Osd_param.CROSS_AXIS_WIDTH;
	//pIStuts->crossAxisHeight		= gConfig_Osd_param.CROSS_AXIS_HEIGHT;
	//pIStuts->picpCrossAxisWidth		= gConfig_Osd_param.Picp_CROSS_AXIS_WIDTH;
	//pIStuts->picpCrossAxisHeight	= gConfig_Osd_param.Picp_CROSS_AXIS_HEIGHT;
	pIStuts->crossAxisWidth[video_pal] = 40;
	pIStuts->crossAxisHeight[video_pal] = 40;
	pIStuts->crossAxisWidth[video_gaoqing0] = 60;
	pIStuts->crossAxisHeight[video_gaoqing0] = 60;
	pIStuts->crossAxisWidth[video_gaoqing] = 60;
	pIStuts->crossAxisHeight[video_gaoqing] = 60;
	pIStuts->crossAxisWidth[video_gaoqing2] = 60;
	pIStuts->crossAxisHeight[video_gaoqing2] = 60;
	pIStuts->crossAxisWidth[video_gaoqing3] = 60;
	pIStuts->crossAxisHeight[video_gaoqing3] = 60;
	pIStuts->picpCrossAxisWidth = 40;
	pIStuts->picpCrossAxisHeight = 40;

	return;
}

void CProcess::MSGAPI_update_alg(long lParam)
{
	plat->update_param_alg();
}
#define UTCPARM 0
void CProcess::update_param_alg()
{
	UTC_DYN_PARAM dynamicParam;
	if(gConfig_Alg_param.occlusion_thred > 0.0001)
		dynamicParam.occlusion_thred = gConfig_Alg_param.occlusion_thred;
	else
		dynamicParam.occlusion_thred = 0.28;

	//dynamicParam.occlusion_thred = 0.30;
	
	if(gConfig_Alg_param.retry_acq_thred> 0.0001)
		dynamicParam.retry_acq_thred = gConfig_Alg_param.retry_acq_thred;
	else
		dynamicParam.retry_acq_thred = 0.38;

	//dynamicParam.retry_acq_thred = 0.40;
	
	float up_factor;
	if(gConfig_Alg_param.up_factor > 0.0001)
		up_factor = gConfig_Alg_param.up_factor;
	else
		up_factor = 0.0125;
	//up_factor = 0.025;

	TRK_SECH_RESTRAINT resTraint;
	if(gConfig_Alg_param.res_distance > 0)
		resTraint.res_distance = gConfig_Alg_param.res_distance;
	else
		resTraint.res_distance = 80;
	
	if(gConfig_Alg_param.res_area> 0)
		resTraint.res_area = gConfig_Alg_param.res_area;
	else
		resTraint.res_area = 5000;
	//printf("UtcSetRestraint: distance=%d area=%d \n", resTraint.res_distance, resTraint.res_area);

	int gapframe;
	if(gConfig_Alg_param.gapframe> 0)
		gapframe = gConfig_Alg_param.gapframe;
	else
		gapframe = 10;

   	bool enhEnable;
	enhEnable = gConfig_Alg_param.enhEnable;	

	float cliplimit;
	if(gConfig_Alg_param.cliplimit> 0)
		cliplimit = gConfig_Alg_param.cliplimit;
	else
		cliplimit = 4.0;

	bool dictEnable;

	dictEnable = gConfig_Alg_param.dictEnable;

	int moveX,moveY;
	if(gConfig_Alg_param.moveX > 0)
		moveX = gConfig_Alg_param.moveX;
	else
		moveX = 20;

	if(gConfig_Alg_param.moveY>0)
		moveY = gConfig_Alg_param.moveY;
	else
		moveY = 10;

	int moveX2,moveY2;
	if(gConfig_Alg_param.moveX2 > 0)
		moveX2 = gConfig_Alg_param.moveX2;
	else
		moveX2 = 30;

	if(gConfig_Alg_param.moveY2 > 0)
		moveY2 = gConfig_Alg_param.moveY2;
	else
		moveY2 = 20;

	int segPixelX,segPixelY;

	if(gConfig_Alg_param.segPixelX > 0)
		segPixelX = gConfig_Alg_param.segPixelX;
	else
		segPixelX = 600;
	if(gConfig_Alg_param.segPixelY > 0)
		segPixelY = gConfig_Alg_param.segPixelY;
	else
		segPixelY = 450;

	/*
	if(gConfig_Alg_param.algOsdRect_Enable == 1)
		algOsdRect = true;
	else
		algOsdRect = false;
	*/

	if(gConfig_Alg_param.ScalerLarge > 0)
		ScalerLarge = gConfig_Alg_param.ScalerLarge;
	else
		ScalerLarge = 256;
	if(gConfig_Alg_param.ScalerMid > 0)
		ScalerMid = gConfig_Alg_param.ScalerMid;
	else
		ScalerMid = 128;
	if(gConfig_Alg_param.ScalerSmall >0)
		ScalerSmall = gConfig_Alg_param.ScalerSmall;
	else
		ScalerSmall = 64;

	int Scatter;
	if(gConfig_Alg_param.Scatter > 0)
		Scatter = gConfig_Alg_param.Scatter;
	else
		Scatter = 10;

	float ratio;
	if(gConfig_Alg_param.ratio >0.1)
		ratio = gConfig_Alg_param.ratio;
	else
		ratio = 1.0;

	bool FilterEnable;

	FilterEnable = gConfig_Alg_param.FilterEnable;

	bool BigSecEnable;
	BigSecEnable = gConfig_Alg_param.BigSecEnable;

	int SalientThred;
	if(gConfig_Alg_param.SalientThred > 0)
		SalientThred = gConfig_Alg_param.SalientThred;
	else
		SalientThred = 40;
	bool ScalerEnable;
	ScalerEnable = gConfig_Alg_param.ScalerEnable;

	bool DynamicRatioEnable;
	DynamicRatioEnable = ScalerEnable = gConfig_Alg_param.DynamicRatioEnable;

	UTC_SIZE acqSize;
	if(gConfig_Alg_param.acqSize_width > 0)	
		acqSize.width = gConfig_Alg_param.acqSize_width;
	else
		acqSize.width = 8;
	if(gConfig_Alg_param.acqSize_height > 0)
		acqSize.height = gConfig_Alg_param.acqSize_height;
	else
		acqSize.height = 8;
	
	if(gConfig_Alg_param.TrkAim43_Enable == 1)
		TrkAim43 = true;
	else
		TrkAim43 = false;

	bool SceneMVEnable;
	SceneMVEnable = gConfig_Alg_param.SceneMVEnable;

	bool BackTrackEnable;
	BackTrackEnable = gConfig_Alg_param.BackTrackEnable;

	bool  bAveTrkPos;
	bAveTrkPos = gConfig_Alg_param.bAveTrkPos;

	float fTau;
	if(gConfig_Alg_param.fTau > 0.01)
		fTau = gConfig_Alg_param.fTau;
	else
		fTau = 0.5;

	int  buildFrms;
	if(gConfig_Alg_param.buildFrms > 0)
		buildFrms = gConfig_Alg_param.buildFrms;
	else
		buildFrms = 500;
	
	int  LostFrmThred;
	if(gConfig_Alg_param.LostFrmThred > 0)
		LostFrmThred = gConfig_Alg_param.LostFrmThred;
	else
		LostFrmThred = 30;

	float  histMvThred;
	if(gConfig_Alg_param.histMvThred > 0.01)
		histMvThred = gConfig_Alg_param.histMvThred;
	else
		histMvThred = 1.0;

	int  detectFrms;
	if(gConfig_Alg_param.detectFrms > 0)
		detectFrms = gConfig_Alg_param.detectFrms;
	else
		detectFrms = 30;

	int  stillFrms;
	if(gConfig_Alg_param.stillFrms > 0)
		stillFrms = gConfig_Alg_param.stillFrms;
	else
		stillFrms = 50;

	float  stillThred;
	if(gConfig_Alg_param.stillThred> 0.01)
		stillThred = gConfig_Alg_param.stillThred;
	else
		stillThred = 0.1;


	bool  bKalmanFilter;
	bKalmanFilter = gConfig_Alg_param.bKalmanFilter;

	float xMVThred, yMVThred;
	if(gConfig_Alg_param.xMVThred> 0.01)
		xMVThred = gConfig_Alg_param.xMVThred;
	else
		xMVThred = 3.0;
	if(gConfig_Alg_param.yMVThred> 0.01)
		yMVThred = gConfig_Alg_param.yMVThred;
	else
		yMVThred = 2.0;

	float xStillThred, yStillThred;
	if(gConfig_Alg_param.xStillThred> 0.01)
		xStillThred = gConfig_Alg_param.xStillThred;
	else
		xStillThred = 0.5;
	if(gConfig_Alg_param.yStillThred> 0.01)
		yStillThred= gConfig_Alg_param.yStillThred;
	else
		yStillThred = 0.3;

	float slopeThred;
	if(gConfig_Alg_param.slopeThred> 0.01)
		slopeThred = gConfig_Alg_param.slopeThred;
	else
		slopeThred = 0.08;

	float kalmanHistThred;
	if(gConfig_Alg_param.kalmanHistThred> 0.01)
		kalmanHistThred = gConfig_Alg_param.kalmanHistThred;
	else
		kalmanHistThred = 2.5;

	float kalmanCoefQ, kalmanCoefR;
	if(gConfig_Alg_param.kalmanCoefQ> 0.000001)
		kalmanCoefQ = gConfig_Alg_param.kalmanCoefQ;
	else
		kalmanCoefQ = 0.00001;
	if(gConfig_Alg_param.kalmanCoefR> 0.000001)
		kalmanCoefR = gConfig_Alg_param.kalmanCoefR;
	else
		kalmanCoefR = 0.0025;

	bool  bSceneMVRecord;
	bSceneMVRecord = 0;//gConfig_Alg_param.SceneMVEnable;
	
	if(bSceneMVRecord == true)
		wFileFlag = true;
	
	
#if __TRACK__
	UtcSetPLT_BS(m_track, tPLT_WRK, BoreSight_Mid);
#endif


	//enh
	if(gConfig_Alg_param.Enhmod_0 > 4)
		m_display.enhancemod = gConfig_Alg_param.Enhmod_0;
	else
		m_display.enhancemod = 1;
	
	if((gConfig_Alg_param.Enhparm_1>0.0)&&(gConfig_Alg_param.Enhparm_1<5.0))
		m_display.enhanceparam=gConfig_Alg_param.Enhparm_1;
	else
		m_display.enhanceparam=3.5;

	//mmt
	if((gConfig_Alg_param.Mmtdparm_2<0) || (gConfig_Alg_param.Mmtdparm_2>15))
		DetectGapparm = 10;
	else
		DetectGapparm = 3;
	
#if __MMT__
	m_MMTDObj.SetSRDetectGap(DetectGapparm);
#endif

	if(gConfig_Alg_param.Mmtdparm_3 > 0)
		MinArea = gConfig_Alg_param.Mmtdparm_3;
	else
		MinArea = 80;
	if(gConfig_Alg_param.Mmtdparm_4 > 0)
		MaxArea = gConfig_Alg_param.Mmtdparm_4;
	else
		MaxArea = 3600;

#if __MMT__
	m_MMTDObj.SetConRegMinMaxArea(MinArea, MaxArea);
#endif

	if(gConfig_Alg_param.Mmtdparm_5 > 0)
		stillPixel = gConfig_Alg_param.Mmtdparm_5;
	else
		stillPixel = 6;
	if(gConfig_Alg_param.Mmtdparm_6 > 0)
		movePixel = gConfig_Alg_param.Mmtdparm_6;
	else
		movePixel = 16;

#if __MMT__
	m_MMTDObj.SetMoveThred(stillPixel, movePixel);
#endif

	if(gConfig_Alg_param.Mmtdparm_7 > 0.001)
		lapScaler = gConfig_Alg_param.Mmtdparm_7;
	else
		lapScaler = 1.25;

#if __MMT__
	m_MMTDObj.SetLapScaler(lapScaler);
#endif

	if(gConfig_Alg_param.Mmtdparm_8 > 0)
		lumThred = gConfig_Alg_param.Mmtdparm_8;
	else
		lumThred = 50;

#if __MMT__
	m_MMTDObj.SetSRLumThred(lumThred);
#endif

#if __TRACK__
	UtcSetDynParam(m_track, dynamicParam);
	UtcSetUpFactor(m_track, up_factor);
	UtcSetUpFactor(m_track, up_factor);
	UtcSetBlurFilter(m_track,FilterEnable);
	UtcSetBigSearch(m_track, BigSecEnable);
#endif

#if UTCPARM

	UtcSetDynParam(m_track, dynamicParam);
	UtcSetUpFactor(m_track, up_factor);
	UtcSetRestraint(m_track, resTraint);
	UtcSetIntervalFrame(m_track, gapframe);
	UtcSetEnhance(m_track, enhEnable);
	UtcSetEnhfClip(m_track, cliplimit);	
	UtcSetPredict(m_track, dictEnable);
	UtcSetMvPixel(m_track,moveX,moveY);
	UtcSetMvPixel2(m_track,moveX2,moveY2);
	UtcSetSegPixelThred(m_track,segPixelX,segPixelY);
	UtcSetSalientScaler(m_track, ScalerLarge, ScalerMid, ScalerSmall);
	UtcSetSalientScatter(m_track, Scatter);
	UtcSetSRAcqRatio(m_track, ratio);
	UtcSetBlurFilter(m_track,FilterEnable);
	UtcSetBigSearch(m_track, BigSecEnable);
	UtcSetSalientThred(m_track,SalientThred);
	UtcSetMultScaler(m_track, ScalerEnable);
	UtcSetDynamicRatio(m_track, DynamicRatioEnable);
	UtcSetSRMinAcqSize(m_track,acqSize);
	UtcSetSceneMV(m_track, SceneMVEnable);
	UtcSetBackTrack(m_track, BackTrackEnable);
	UtcSetAveTrkPos(m_track, bAveTrkPos);
	UtcSetDetectftau(m_track, fTau);
	UtcSetDetectBuildFrms(m_track, buildFrms);
	UtcSetLostFrmThred(m_track, LostFrmThred);
	UtcSetHistMVThred(m_track, histMvThred);
	UtcSetDetectFrmsThred(m_track, detectFrms);
	UtcSetStillFrmsThred(m_track, stillFrms);
	UtcSetStillPixThred(m_track, stillThred);
	UtcSetKalmanFilter(m_track, bKalmanFilter);
	UtcSetKFMVThred(m_track, xMVThred, yMVThred);
	UtcSetKFStillThred(m_track, xStillThred, yStillThred);
	UtcSetKFSlopeThred(m_track, slopeThred);
	UtcSetKFHistThred(m_track, kalmanHistThred);
	UtcSetKFCoefQR(m_track, kalmanCoefQ, kalmanCoefR);
	UtcSetSceneMVRecord(m_track, bSceneMVRecord);
	UtcSetRoiMaxWidth(m_track, 400);

#endif


	
	return ;
}

void CProcess::MSGAPI_update_camera(long lParam)
{
}

void CProcess::MSGAPI_update_ballPos(long lParam)
{
	//proc->setBallPos(linkagePos.panPos, linkagePos.tilPos, linkagePos.zoom);	
	m_camCalibra->setBallPos(linkagePos.panPos, linkagePos.tilPos, linkagePos.zoom);	
}


void CProcess::MSGAPI_input_algosdrect(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_INPUT_ALGOSDRECT,NULL);
}
#if __TRACK__
void CProcess::set_trktype(CMD_EXT *p, unsigned int stat)
{
	
	static int old_stat = -1;
	int flag = 0;
	SENDST test;
	test.cmd_ID = trktype;

	p->TrkStat = stat;

	if(old_stat == eTrk_Lost && stat == eTrk_Assi)
		return;
		
	if(old_stat != stat)
	{
		old_stat = stat;
		flag = 1;
	}
	if(flag&&(extInCtrl->AvtTrkStat == eTrk_mode_target))
		ipc_sendmsg(&test,IPC_FRIMG_MSG);
}
#endif

float  CProcess::PiexltoWindowsyf(float y,int channel)
{
	 float ret=0;
	 ret= (y*1.0/vcapWH[channel][1]*vdisWH[channel][1]);

	  if(ret<0)
 	{
		ret=0;
 	}
	 else if(ret>=vdisWH[channel][1])
 	{
		ret=vdisWH[channel][1];
 	}
	
	return  ret;
}
void CProcess::MSGAPI_update_menuindex(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_MENUSWITCH,NULL);
}

void CProcess::MSGAPI_up_menu(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_UPMENU,NULL);
}

void CProcess::MSGAPI_down_menu(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_DOWNMENU,NULL);
}

void CProcess::MSGAPI_save_mtdrigion(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_SMR,NULL);
}

void CProcess::MSGAPI_set_resol(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_SETRESOL,NULL);
}

void CProcess::MSGAPI_set_baud(long lParam)
{

	sThis->msgdriv_event(MSGID_EXT_SETBAUD,NULL);
}

void CProcess::MSGAPI_save_resol(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_SAVERESOL,NULL);
}

void CProcess::MSGAPI_set_mtdnum(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_SETMTDNUM,NULL);
}

void CProcess::MSGAPI_set_mtdtrktime(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_SETMTDTRKTIME,NULL);
}

void CProcess::MSGAPI_set_mtdmaxsize(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_SETMTDMAXSIZE,NULL);
}

void CProcess::MSGAPI_set_mtdminsize(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_SETMTDMINSIZE,NULL);
}

void CProcess::MSGAPI_set_mtdsensi(long lParam)
{
	sThis->msgdriv_event(MSGID_EXT_SETMTDSENSI,NULL);
}


void CProcess::setGridMapCalibrate(bool flag)
{
	m_bGridMapCalibrate = flag;
}
const bool CProcess::getGridMapCalibrate()
{
	return m_bGridMapCalibrate;
}
void CProcess::setQueryZoomFlag(bool flag)
{
	m_queryZoom = flag;
}
const bool CProcess::getQueryZoomFlag()
{
	return m_queryZoom ;
}
int CProcess::getCurrentZoomValue()
{
	return zoomPos;
}
void CProcess::addMarkNum()
{
	m_successCalibraNum++;
	return;
}
void CProcess::setPTZflag(bool flag)
{
	m_bRefreshPTZValue = flag;
	return ;
}
bool CProcess::getPTZflag()
{
	return m_bRefreshPTZValue;
}

void CProcess::CaptureMouseClickPoint(int x, int y)
{
	m_curClickPoint = Point(x, y);
}
cv::Point CProcess::getCurrentMouseClickPoint()
{
	return m_curClickPoint;
}

void CProcess::setBallImagePoint(int &x, int &y) 
{
	m_ballDestPoint.x = x;
	m_ballDestPoint.y = y;
}

cv::Point CProcess::getBallImagePoint()
{
	return m_ballDestPoint;
}

void CProcess::renderText(enum DrawBehavior drawbehavior)
{	
	int window_width = outputWHF[0];
	int window_height = outputWHF[1];
	if(drawbehavior == CProcess::DRAW_NOTICE_TEXTS)
	{
		sprintf(tmp_str[0], "ToTal Nodes: %d", 204);	
		putText(m_display.m_imgOsd[1],tmp_str[0],cv::Point(window_width/42,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(255,255,0,255), 1);	

		putText(m_display.m_imgOsd[1],tmp_str[1],cv::Point(window_width/9,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(0,0,0,0), 1);
		sprintf(tmp_str[1], "Cur-Node:%d", (m_curNodeIndex+1));	
		putText(m_display.m_imgOsd[1],tmp_str[1],cv::Point(window_width/9,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(0,255,255,255), 1);	

		int row = (m_curNodeIndex/(GRID_COLS_15 +2));
		int col = (m_curNodeIndex%(GRID_COLS_15 +2));
		putText(m_display.m_imgOsd[1],tmp_str[3],cv::Point(window_width/5,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(0,0,0,0), 1);
		sprintf(tmp_str[3], "<row-%d, col-%d>",row,col );	
		putText(m_display.m_imgOsd[1],tmp_str[3],cv::Point(window_width/5,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(0,0,255,255), 1);	

		//putText(m_display.m_imgOsd[1],tmp_str[4],cv::Point(window_width/2,window_height/43),FONT_HERSHEY_TRIPLEX,0.6, cvScalar(0,0,0,0), 1);
		//sprintf(tmp_str[4], "Finished:%d",m_successCalibraNum);	
		//putText(m_display.m_imgOsd[1],tmp_str[4],cv::Point(window_width/2,window_height/43),FONT_HERSHEY_TRIPLEX,0.6, cvScalar(0,255,255,255), 1);	

		putText(m_display.m_imgOsd[1],m_appVersion,cv::Point(window_width-100,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(0,0,0,0), 1);
		sprintf(m_appVersion, "V%d.%d.%d",m_AppVersion>>6,(m_AppVersion>>3)&0x07,m_AppVersion&0x07);	
		putText(m_display.m_imgOsd[1],m_appVersion,cv::Point(window_width-100,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(0,255,255,255), 1);	

	}
	else if (drawbehavior == CProcess::ERASE_TEXTS)
	{
		putText(m_display.m_imgOsd[1],tmp_str[0],cv::Point(window_width/42,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(0,0,0,0), 1);
		putText(m_display.m_imgOsd[1],tmp_str[1],cv::Point(window_width/9,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(0,0,0,0), 1);
		putText(m_display.m_imgOsd[1],tmp_str[3],cv::Point(window_width/5,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(0,0,0,0), 1);	
		//putText(m_display.m_imgOsd[1],tmp_str[4],cv::Point(window_width/2,window_height/43),FONT_HERSHEY_TRIPLEX,0.6, cvScalar(0,0,0,0), 1);	
		putText(m_display.m_imgOsd[1],m_appVersion,cv::Point(window_width-100,window_height/43),FONT_HERSHEY_TRIPLEX,0.5, cvScalar(0,0,0,0), 1);
	}
	else
	{
	}
}
void CProcess::DrawCenterCross(cv::Point pos, int width,int height)
{
}
void CProcess::renderCircles(int radius, cv::Point position)
{
}
	
