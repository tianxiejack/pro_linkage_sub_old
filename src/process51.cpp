
#include <glut.h>
#include "process51.hpp"
#include "vmath.h"
#include "dx_main.h"
#include "msgDriv.h"
#include"app_ctrl.h"
#include"dx.h"
#include"osd_cv.h"
#include "app_status.h"
#include "configable.h"
#include "Ipcctl.h"


OSD_Param gConfig_Osd_param = {0};

CProcess * CProcess::sThis = NULL;
static bool DrawMoveDetect = 0;
CProcess* plat = NULL;

SENDST trkmsg={0};

void inputtmp(unsigned char cmdid)
{
	plat->OnKeyDwn(cmdid);
}

void getMmtTg(unsigned char index,int *x,int *y)
{
	int cur_x = 
	*x = (int)plat->m_mtd[0]->tg[index].cur_x%VIDEO_IMAGE_WIDTH_0;
	*y = (int)plat->m_mtd[0]->tg[index].cur_y%VIDEO_IMAGE_HEIGHT_0;
	return ;
}

CProcess::CProcess()
{
	memset(&rcTrackBak, 0, sizeof(rcTrackBak));
	memset(tgBak, 0, sizeof(tgBak));
	memset(&blob_rectBak, 0, sizeof(blob_rectBak));
	memset(&extOutAck, 0, sizeof(ACK_EXT));
	prisensorstatus=0;//tv
	m_castTm=0;
	m_bCast=false;
	rememflag=false;
	rememtime=0;
	// default cmd value
	extInCtrl = (CMD_EXT*)ipc_getimgstatus_p();
	CMD_EXT *pIStuts = extInCtrl;

	pIStuts->opticAxisPosX[0]   = VIDEO_IMAGE_WIDTH_0/2;
	pIStuts->opticAxisPosY[0]   = VIDEO_IMAGE_HEIGHT_0/2;
	pIStuts->opticAxisPosX[1]   = VIDEO_IMAGE_WIDTH_1/2;
	pIStuts->opticAxisPosY[1]   = VIDEO_IMAGE_HEIGHT_1/2;
	
	pIStuts->unitAimW 	= 	AIM_WIDTH;
	pIStuts->unitAimH 	= 	AIM_HEIGHT;
	pIStuts->unitAimX		=	VIDEO_IMAGE_WIDTH_0/2;
	pIStuts->unitAimY		=	VIDEO_IMAGE_HEIGHT_0/2;
	pIStuts->SensorStat 	= pIStuts->SensorStatBegin;
	pIStuts->PicpSensorStatpri	=	pIStuts->PicpSensorStat = 0xFF;
	
	pIStuts->changeSensorFlag = 0;
	crossBak.x = pIStuts->opticAxisPosX[pIStuts->SensorStat ];
	crossBak.y = pIStuts->opticAxisPosY[pIStuts->SensorStat ];
	pIStuts->AvtTrkAimSize= AVT_TRK_AIM_SIZE;

	pIStuts->AvtPosX[0]	=VIDEO_IMAGE_WIDTH_0/2;
	pIStuts->AvtPosY[0]	=VIDEO_IMAGE_HEIGHT_0/2;

	pIStuts->AxisPosX[0]	=VIDEO_IMAGE_WIDTH_0/2;
	pIStuts->AxisPosY[0]	=VIDEO_IMAGE_HEIGHT_0/2;
	pIStuts->AxisPosX[1]	=VIDEO_IMAGE_WIDTH_0/2;
	pIStuts->AxisPosY[1]	=VIDEO_IMAGE_HEIGHT_0/2;
	
	pIStuts->PicpPosStat = 0;
	pIStuts->validChId = pIStuts->SensorStatBegin;
	pIStuts->FovStat=1;

	pIStuts->FrCollimation=2;
	pIStuts->PicpSensorStatpri=2;
	pIStuts->axisMoveStepX = 0;
	pIStuts->axisMoveStepY = 0;

	memset(secBak,0,sizeof(secBak));
	memset(Osdflag,0,sizeof(Osdflag));
	
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

	msgextInCtrl = extInCtrl;
	sThis = this;
	plat = this;

	pIStuts->DispGrp[0] = 1;
	pIStuts->DispGrp[1] = 1;
	pIStuts->DispColor[0]=2;
	pIStuts->DispColor[1]=2;
	
}

CProcess::~CProcess()
{
	sThis=NULL;
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



int  CProcess::PiexltoWindowsx(int x,int channel)
{
	 int ret=0;
	 ret= cvRound(x*1.0/vcapWH[channel][0]*vdisWH[0][0]);
	 if(ret<0)
 	 {
		ret=0;
 	 }
	 else if(ret>=vdisWH[0][0])
 	 {
		ret=vdisWH[0][0];
 	 }
	 if(extInCtrl->ImgMmtshow[extInCtrl->SensorStat])
 	 {
		ret =ret*2/3;
 	 }

	 return ret;
}

int  CProcess::PiexltoWindowsy(int y,int channel)
{
	 int ret=0;
	 ret= cvRound(y*1.0/vcapWH[channel][1]*vdisWH[0][1]);

	 if(ret<0)
 	 {
		ret=0;
 	 }
	 else if(ret>=vdisWH[0][1])
 	 {
		ret=vdisWH[0][1];
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
	 ret= (x*1.0/vcapWH[channel][0]*vdisWH[0][0]);
	 if(ret<0)
 	{
		ret=0;
 	}
	 else if(ret>=vdisWH[0][0])
 	{
		ret=vdisWH[0][0];
 	}

	  return ret;
}

float  CProcess::PiexltoWindowsyf(float y,int channel)
{
	 float ret=0;
	 ret= (y*1.0/vcapWH[channel][1]*vdisWH[0][1]);

	  if(ret<0)
 	{
		ret=0;
 	}
	 else if(ret>=vdisWH[0][1])
 	{
		ret=vdisWH[0][1];
 	}
	
	return  ret;
}


int  CProcess::PiexltoWindowsxzoom(int x,int channel)
{
	int ret=0;
	 ret= cvRound(x*1.0/vcapWH[channel][0]*vdisWH[0][0]);
	 if(ret<0)
 	{
		ret=0;
 	}
	 else if(ret>=vdisWH[0][0])
 	{
		ret=vdisWH[0][0];
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
	 ret= cvRound(y*1.0/vcapWH[channel][1]*vdisWH[0][1]);

	  if(ret<0)
 	{
		ret=0;
 	}
	 else if(ret>=vdisWH[0][1])
 	{
		ret=vdisWH[0][1];
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

	ret= cvRound(x*1.0/vcapWH[channel][0]*vdisWH[0][0]);
	
	if(ret<0)
	{
		ret=0;
	}
	else if(ret>=vdisWH[0][0])
	{
		ret=vdisWH[0][0];
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

	 ret= cvRound(y*1.0/vcapWH[channel][1]*vdisWH[0][1]);

	if(ret<0)
 	{
		ret=0;
 	}
	 else if(ret>=vdisWH[0][1])
 	{
		ret=vdisWH[0][1];
 	}

	if((ret%2)==0)
		ret = ret;
	else
		ret = ret+1;

	return  ret;
}
void CProcess::OnCreate()
{
	MSGAPI_initial();
};
void CProcess::OnDestroy(){};
void CProcess::OnInit()
{
	extInCtrl->SysMode = 1;
}
void CProcess::OnConfig(){};
void CProcess::OnRun(){};
void CProcess::OnStop(){};
void CProcess::Ontimer(){

	//msgdriv_event(MSGID_EXT_INPUT_VIDEOEN,NULL);
};
bool CProcess::OnPreProcess(int chId, Mat &frame)
{
	set_graph();

	
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

void CProcess::process_osd1()
{
	int devId=0;
	Mat frame=sThis->m_dccv;
	CMD_EXT *pIStuts = sThis->extInCtrl;
	int winId;
	Text_Param_fb * textParam = NULL;
	Line_Param_fb * lineParam = NULL;
	Text_Param_fb * textParampri = NULL;
	Line_Param_fb * lineParampri = NULL;

	if(sThis->m_display.m_bOsd == false)
		return ;

	for(winId = 0; winId < grpxChWinPrms.chParam[devId].numWindows; winId++)
	{
		textParam = &grpxChWinPrms.chParam[devId].winPrms[winId];
		textParampri = &grpxChWinPrms.chParam[devId].winPrms_pri[winId];
		lineParam = (Line_Param_fb *)&grpxChWinPrms.chParam[devId].winPrms[winId];
		lineParampri = (Line_Param_fb *)&grpxChWinPrms.chParam[devId].winPrms_pri[winId];
		if(onece<ALG_LINK_GRPX_MAX_WINDOWS)
		{
			memcpy(textParampri,textParam,sizeof(Text_Param_fb));
			onece++;
		}
		
		if(winId==WINID_TV_FOV_CHOOSE_1/2)
		{
			//printf("textParam->enableWin=%d  objType=%d valid=%d\n",textParam->enableWin,textParam->objType,textParam->text_valid);
		}
		//printf("textParam->enableWin=%d  objType=%d valid=%d\n",textParam->enableWin,textParam->objType,textParam->text_valid);
		if(!textParam->enableWin)
			continue;

		//EraseDraw_graph_osd(frame,textParam,textParampri);
		//	Draw_graph_osd(frame,textParam,lineParam);
		
		memcpy(textParampri,textParam,sizeof(Text_Param_fb));
	}

}

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
			result.x = ((int)tg[i].cur_x) % VIDEO_IMAGE_WIDTH_0;
			result.y = ((int)tg[i].cur_y ) % VIDEO_IMAGE_HEIGHT_0;
			result.x = result.x - result.width/2;
			result.y = result.y - result.height/2;
			rectangle( m_dccv,
				Point( result.x, result.y ),
				Point( result.x+result.width, result.y+result.height),
				colour, 1, 8);
		}
	}
}

void CProcess::DrawBlob(BlobRect blobRct,  bool bShow /*= true*/)
{
	int frcolor=extInCtrl->DispColor[extInCtrl->SensorStat];
	unsigned char Alpha = (bShow) ? frcolor : 0;
	CvScalar colour=GetcvColour(Alpha);
	int startx=0;
	int starty=0;
	int endx=0;
	int endy=0;
	int sensor=0;

	if(bShow)
	{
		sensor=extInCtrl->SensorStat;

	}
	else
		{
			sensor=prisensorstatus;

		}

	if(blobRct.bvalid){

		Point pt1,pt2,center;
		center.x = blobRct.center.x;
		center.y = blobRct.center.y;

	 	startx=PiexltoWindowsx(center.x,sensor);
	 	starty=PiexltoWindowsy(center.y,sensor);
		pt1.x=startx-10;pt1.y=starty;
		pt2.x=startx+10;pt2.y=starty;
		line(m_dccv, pt1, pt2, colour, 1, 8, 0 );
		pt1.x=startx;pt1.y=starty-10;
		pt2.x=startx;pt2.y=starty+10;
		line(m_dccv, pt1, pt2, colour, 1, 8, 0 );

	}
}


void CProcess::DrawCross(cv::Rect rec,int fcolour ,bool bShow /*= true*/)
{
	
	unsigned char colour = (bShow) ?fcolour : 0;
	Line_Param_fb lineparm;
	lineparm.x		=	rec.x;
	lineparm.y		=	rec.y;
	lineparm.width	=	rec.width;
	lineparm.height	=	rec.height;
	lineparm.frcolor	=	colour;
	Drawcvcrossaim(m_dccv,&lineparm);
}

void CProcess::DrawAcqRect(cv::Mat frame,cv::Rect rec,int frcolor,bool bshow)
{
	unsigned char color = (bshow)?frcolor:0;
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
	DrawcvLine(frame,&start,&end,frcolor,1);
	start.x 	= leftBottomx;
	start.y 	= leftBottomy;
	end.x	= leftBottomx;
	end.y 	= leftBottomy - cornory;
	DrawcvLine(frame,&start,&end,frcolor,1);	
	//leftTop
	start.x 	= leftTopx;
	start.y 	= leftTopy;
	end.x	= leftTopx + cornorx;
	end.y 	= leftTopy;
	DrawcvLine(frame,&start,&end,frcolor,1);
	start.x 	= leftTopx;
	start.y 	= leftTopy;
	end.x	= leftTopx;
	end.y 	= leftTopy + cornory;
	DrawcvLine(frame,&start,&end,frcolor,1);	
	//rightTop
	start.x 	= rightTopx;
	start.y 	= rightTopy;
	end.x	= rightTopx - cornorx;
	end.y 	= rightTopy;
	DrawcvLine(frame,&start,&end,frcolor,1);
	start.x 	= rightTopx;
	start.y 	= rightTopy;
	end.x	= rightTopx;
	end.y 	= rightTopy + cornory;
	DrawcvLine(frame,&start,&end,frcolor,1);
	//rightBottom
	start.x 	= rightBottomx;
	start.y 	= rightBottomy;
	end.x	= rightBottomx - cornorx;
	end.y 	= rightBottomy;
	DrawcvLine(frame,&start,&end,frcolor,1);
	start.x 	= rightBottomx;
	start.y 	= rightBottomy;
	end.x	= rightBottomx;
	end.y 	= rightBottomy - cornory;
	DrawcvLine(frame,&start,&end,frcolor,1);	

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
			Mat frame=m_dccv;
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
							tempmmtx=result.x = ((int)tg[primajormmtid].cur_x) % VIDEO_IMAGE_WIDTH_0;
							tempmmty=result.y = ((int)tg[primajormmtid].cur_y ) % VIDEO_IMAGE_HEIGHT_0;


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
								tempmmtx=result.x = ((int)tg[tempdata].cur_x) % VIDEO_IMAGE_WIDTH_0;
								tempmmty=result.y = ((int)tg[tempdata].cur_y ) % VIDEO_IMAGE_HEIGHT_0;

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
	Mat frame=m_dccv;
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
			tempmmtx=result.x = ((int)tg[majormmtid].cur_x) % VIDEO_IMAGE_WIDTH_0;
			tempmmty=result.y = ((int)tg[majormmtid].cur_y ) % VIDEO_IMAGE_HEIGHT_0;


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
			tempmmtx=result.x = ((int)tg[tempdata].cur_x) % VIDEO_IMAGE_WIDTH_0;
			tempmmty=result.y = ((int)tg[tempdata].cur_y ) % VIDEO_IMAGE_HEIGHT_0;

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
	Mat frame=m_dccv;
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
	Mat frame=m_dccv;
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
			tempmmtx=result.x = ((int)tg[majormmtid].cur_x) % VIDEO_IMAGE_WIDTH_0;
			tempmmty=result.y = ((int)tg[majormmtid].cur_y ) % VIDEO_IMAGE_HEIGHT_0;

			
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
			
			tempmmtx=result.x = ((int)tg[i].cur_x) % VIDEO_IMAGE_WIDTH_0;
			tempmmty=result.y = ((int)tg[i].cur_y ) % VIDEO_IMAGE_HEIGHT_0;		

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
		tempmmtx=result.x = ((int)tg[i].cur_x) % VIDEO_IMAGE_WIDTH_0;
		tempmmty=result.y = ((int)tg[i].cur_y ) % VIDEO_IMAGE_HEIGHT_0;
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
	DrawcvLine(m_dccv,&start,&end,colour,1);

	start.x=centerx+templenx-lenw;
	start.y=centery-templeny;
	end.x=centerx+templenx;
	end.y=centery-templeny;
	DrawcvLine(m_dccv,&start,&end,colour,1);


	start.x=centerx-templenx;
	start.y=centery+templeny;
	end.x=centerx-templenx+lenw;
	end.y=centery+templeny;
	DrawcvLine(m_dccv,&start,&end,colour,1);

	start.x=centerx+templenx-lenw;
	start.y=centery+templeny;
	end.x=centerx+templenx;
	end.y=centery+templeny;
	DrawcvLine(m_dccv,&start,&end,colour,1);

	//h
	start.x=centerx-templenx;
	start.y=centery-templeny;
	end.x=centerx-templenx;
	end.y=centery-templeny+lenw;
	DrawcvLine(m_dccv,&start,&end,colour,1);

	start.x=centerx+templenx;
	start.y=centery-templeny;
	end.x=centerx+templenx;
	end.y=centery-templeny+lenw;
	DrawcvLine(m_dccv,&start,&end,colour,1);


	start.x=centerx-templenx;
	start.y=centery+templeny-lenw;
	end.x=centerx-templenx;
	end.y=centery+templeny;
	DrawcvLine(m_dccv,&start,&end,colour,1);

	start.x=centerx+templenx;
	start.y=centery+templeny-lenw;
	end.x=centerx+templenx;
	end.y=centery+templeny;
	DrawcvLine(m_dccv,&start,&end,colour,1);

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
		DrawcvDashcross(m_dccv,&lineparm,dashlen,dashlen);
		startx=secBak[0].x;
		starty=secBak[0].y;
		endx=secBak[1].x;
		endy=secBak[1].y;
		
		drawdashlinepri(m_dccv,startx,starty,endx,endy,dashlen,dashlen,colour);
	}

	else if(extInCtrl->DispGrp[extInCtrl->SensorStat]<3)
	{
		DrawcvDashcross(m_dccv,&lineparm,dashlen,dashlen);
		startx=PiexltoWindowsxzoom(extInCtrl->AvtPosX[extInCtrl->SensorStat ],extInCtrl->SensorStat);
		starty=PiexltoWindowsyzoom(extInCtrl->AvtPosY[extInCtrl->SensorStat ],extInCtrl->SensorStat);
		endx=lineparm.x;
		endy=lineparm.y;
		
		drawdashlinepri(m_dccv,startx,starty,endx,endy,dashlen,dashlen,colour);

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
	drawdashlinepri(m_dccv,startx,starty,endx,starty,dashlen,dashlen,colour);
	drawdashlinepri(m_dccv,startx,endy,endx,endy,dashlen,dashlen,colour);
	drawdashlinepri(m_dccv,endx,starty,endx,endy,dashlen,dashlen,colour);
	drawdashlinepri(m_dccv,startx,starty,startx,endy,dashlen,dashlen,colour);
}

bool CProcess::OnProcess(int chId, Mat &frame)
{
	//track
	int frcolor= extInCtrl->osdDrawColor;//extInCtrl->DispColor[extInCtrl->SensorStat];
	int startx=0;
	int starty=0;
	int endx=0;
	int endy=0;
	int crossshiftx=cvRound(vdisWH[0][0]/3);
	int crossshifty=cvRound(vdisWH[0][1]/3);
	int detect_num = 0,i;
	int coastRectx,coastRecty;
	cv::Rect recIn;
	static int coastCnt = 1;
	static int bDraw = 0;


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
		//OSA_mutexLock(&osd_mutex);
		
		//OSA_mutexUnlock(&osd_mutex);
	}
	process_osd1();
	
	countnofresh++;

	osdindex=0;	//picp cross
	{
		if(Osdflag[osdindex]==1)
		{
			recIn.x=crosspicpBak.x;
			recIn.y=crosspicpBak.y;
			recIn.width = crosspicpWHBak.x;
			recIn.height = crosspicpWHBak.y;
			DrawCross(recIn,frcolor,false);
			Osdflag[osdindex]=0;
		}
		if(((extInCtrl->PicpSensorStat==1)||(extInCtrl->PicpSensorStat==0))&&(extInCtrl->DispGrp[extInCtrl->SensorStat]<=3))
		{
			recIn.x=PiexltoWindowsx(extInCtrl->SensorStat?extInCtrl->AxisPosX[0]:extInCtrl->AxisPosX[1],extInCtrl->SensorStat);
			recIn.y=PiexltoWindowsy(extInCtrl->SensorStat?extInCtrl->AxisPosY[0]:extInCtrl->AxisPosY[1],extInCtrl->SensorStat);
			recIn.width = extInCtrl->picpCrossAxisWidth;
			recIn.height = extInCtrl->picpCrossAxisHeight;
			extInCtrl->PicpPosStat = 0;
			switch(extInCtrl->PicpPosStat)
			{
				case 0:
					startx+=crossshiftx;
					starty-=crossshifty;
					break;
				case 1:
					startx+=crossshiftx;
					starty+=crossshifty;
					break;
				case 2:
					startx-=crossshiftx;
					starty+=crossshifty;
					break;
				case 3:
					startx-=crossshiftx;
					starty-=crossshifty;
					break;

				default:
					break;
			}
			
			if(startx<0)
				startx=0;
			else if(startx>vdisWH[0][0])
				startx=0;
			if(starty<0)
				starty=0;
			else if(starty>vdisWH[0][0])
				starty=0;
			
			DrawCross(recIn,frcolor,true);
			//printf("picp***********lat the startx=%d  starty=%d\n ",startx,starty);
			Osdflag[osdindex]=1;

			crosspicpBak.x=recIn.x;
			crosspicpBak.y=recIn.y;
			crosspicpWHBak.x = recIn.width;
			crosspicpWHBak.y = recIn.height;

		}
	}
#if __TRACK__
	osdindex++;
	{
		 UTC_RECT_float rcResult = m_rcTrack;
		 UTC_RECT_float rcResult_algRect = m_rcTrack;
		 
		 trackinfo_obj->trackrect=m_rcTrack;
		 trackinfo_obj->TrkStat = extInCtrl->AvtTrkStat;
		 m_SensorStat = extInCtrl->SensorStat;
		 int aimw= extInCtrl->AimW[extInCtrl->SensorStat];//trkWinWH[extInCtrl->SensorStat][extInCtrl->AvtTrkAimSize][0];
		 int aimh=  extInCtrl->AimW[extInCtrl->SensorStat];//trkWinWH[extInCtrl->SensorStat][extInCtrl->AvtTrkAimSize][1];
		 if((extInCtrl->FovCtrl==5)&&(extInCtrl->SensorStat==0))
		{
			aimw=aimw/2;
			aimh=aimh/2;
	 	}
		 
		if(Osdflag[osdindex]==1)
 		{
			rectangle( m_dccv,
				Point( rcTrackBak.x, rcTrackBak.y ),
				Point( rcTrackBak.x+rcTrackBak.width, rcTrackBak.y+rcTrackBak.height),
				cvScalar(0,0,0, 0), 1, 8 );
			#if 1//dft alg reply x y w h
			if(algOsdRect == true)
				rectangle( m_dccv,
					Point( resultTrackBak.x, resultTrackBak.y ),
					Point( resultTrackBak.x+resultTrackBak.width, resultTrackBak.y+resultTrackBak.height),
					cvScalar(0,0,0,0), 1, 8 );
			#endif			
			Osdflag[osdindex]=0;
		}
		 if(m_bTrack)
		 {
			extInCtrl->TrkXtmp =rcResult.x+rcResult.width/2;
			extInCtrl->TrkYtmp = rcResult.y+rcResult.height/2;
			coastRectx = extInCtrl->AvtPosX[0];
			coastRecty = extInCtrl->AvtPosY[0];
			if(extInCtrl->FovCtrl==5&&extInCtrl->SensorStat==0)
			{
				if(extInCtrl->TrkXtmp>=vdisWH[0][0]/2)
					extInCtrl->TrkXtmp = (extInCtrl->TrkXtmp-vdisWH[0][0]/2)*2+vdisWH[0][0]/2 ;
				else if(extInCtrl->TrkXtmp<vdisWH[0][0]/2)
					extInCtrl->TrkXtmp = vdisWH[0][0]/2 -(vdisWH[0][0]/2 - extInCtrl->TrkXtmp)*2 ;

				if(extInCtrl->TrkYtmp>=vdisWH[0][1]/2)
					extInCtrl->TrkYtmp = (extInCtrl->TrkYtmp-vdisWH[0][1]/2)*2+vdisWH[0][1]/2;
				else if(extInCtrl->TrkYtmp<vdisWH[0][1]/2)
					extInCtrl->TrkYtmp = vdisWH[0][1]/2 -(vdisWH[0][1]/2 - extInCtrl->TrkYtmp)*2;

				startx=PiexltoWindowsxzoom_TrkRect(extInCtrl->TrkXtmp-aimw,extInCtrl->SensorStat);			
				starty=PiexltoWindowsyzoom_TrkRect(extInCtrl->TrkYtmp-aimh ,extInCtrl->SensorStat);
				endx=PiexltoWindowsxzoom_TrkRect(extInCtrl->TrkXtmp+aimw,extInCtrl->SensorStat);
				endy=PiexltoWindowsyzoom_TrkRect(extInCtrl->TrkYtmp+aimh ,extInCtrl->SensorStat);
			}
			else
			{
				 startx=PiexltoWindowsxzoom_TrkRect(rcResult.x+rcResult.width/2-aimw/2,extInCtrl->SensorStat);			
				 starty=PiexltoWindowsyzoom_TrkRect(rcResult.y+rcResult.height/2-aimh/2 ,extInCtrl->SensorStat);
				 endx=PiexltoWindowsxzoom_TrkRect(rcResult.x+rcResult.width/2+aimw/2,extInCtrl->SensorStat);
			 	 endy=PiexltoWindowsyzoom_TrkRect(rcResult.y+rcResult.height/2+aimh/2 ,extInCtrl->SensorStat);
			}
			 //OSA_printf("startxy=(%d,%d) endXY=(%d,%d)resultXY(%f,%f)\n",startx,starty,endx,endy,rcResult.x+rcResult.width/2-aimw/2,rcResult.y+rcResult.height/2-aimh/2);
			#if 1//dft alg reply x y w h			
			if(algOsdRect == true)
			{
				rcResult_algRect.x = PiexltoWindowsx(rcResult_algRect.x,extInCtrl->SensorStat);
				rcResult_algRect.y = PiexltoWindowsy(rcResult_algRect.y,extInCtrl->SensorStat);
				rcResult_algRect.width = PiexltoWindowsx(rcResult_algRect.width,extInCtrl->SensorStat);
				rcResult_algRect.height = PiexltoWindowsy(rcResult_algRect.height,extInCtrl->SensorStat);
			}
			#endif
			if( m_iTrackStat == 1)
			{
				#if 1	// trackRect
					rectangle( m_dccv,
						Point( startx, starty ),
						Point( endx, endy),
						colour, 1, 8 );
				#endif
				#if 1 //dft alg reply x y w h
				if(algOsdRect == true)
					rectangle( m_dccv,
						Point( rcResult_algRect.x, rcResult_algRect.y ),
						Point( rcResult_algRect.x+rcResult_algRect.width, rcResult_algRect.y+rcResult_algRect.height),
						cvScalar(0,255,0,255), 1, 8 );
				#endif
			}
			else
			{
				if(extInCtrl->FovCtrl==5&&extInCtrl->SensorStat==0){
					
					startx=PiexltoWindowsxzoom_TrkRect(coastRectx-aimw,extInCtrl->SensorStat);			
					starty=PiexltoWindowsyzoom_TrkRect(coastRecty-aimh ,extInCtrl->SensorStat);
					endx=PiexltoWindowsxzoom_TrkRect(coastRectx+aimw,extInCtrl->SensorStat);
					endy=PiexltoWindowsyzoom_TrkRect(coastRecty+aimh ,extInCtrl->SensorStat);

				}else{
					startx=PiexltoWindowsxzoom(extInCtrl->AvtPosX[extInCtrl->SensorStat]-aimw/2,extInCtrl->SensorStat);			
					starty=PiexltoWindowsyzoom(extInCtrl->AvtPosY[extInCtrl->SensorStat]-aimh/2 ,extInCtrl->SensorStat);
					endx=PiexltoWindowsxzoom(extInCtrl->AvtPosX[extInCtrl->SensorStat]+aimw/2,extInCtrl->SensorStat);
					endy=PiexltoWindowsyzoom(extInCtrl->AvtPosY[extInCtrl->SensorStat]+aimh/2 ,extInCtrl->SensorStat);
					}
				if(bDraw != 0){
					DrawdashRect(startx,starty,endx,endy,frcolor);	// track lost DashRect				
				}
				#if 1 //dft alg reply x y w h
				if(algOsdRect == true)
					rectangle( m_dccv,
						Point( rcResult_algRect.x, rcResult_algRect.y ),
						Point( rcResult_algRect.x+rcResult_algRect.width, rcResult_algRect.y+rcResult_algRect.height),
						cvScalar(0,255,0,255), 1, 8 );		
				#endif
			}

			Osdflag[osdindex]=1;
			rcTrackBak = rcResult;
			rcTrackBak.x=startx;
			rcTrackBak.y=starty;
			rcTrackBak.width=endx-startx;
			rcTrackBak.height=endy-starty;

			#if 1 //dft alg reply x y w h
			if(algOsdRect == true)
			{
				resultTrackBak = rcResult_algRect;
				resultTrackBak.x = rcResult_algRect.x;
				resultTrackBak.y = rcResult_algRect.y;
				resultTrackBak.width = rcResult_algRect.width;
				resultTrackBak.height = rcResult_algRect.height;
			}
			#endif
			
			extInCtrl->unitAimX=rcResult.x+rcResult.width/2;
			extInCtrl->unitAimY=rcResult.y+rcResult.height/2;
			extInCtrl->unitAimW=rcResult.width;
			extInCtrl->unitAimH=rcResult.height;
		 }
		 
		 if(m_bTrack)
		 {
		 	extInCtrl->TrkStat=m_iTrackStat;
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
				
				if((OSA_getCurTimeInMsec()-rememtime)>5000)
				{							
					extInCtrl->TrkStat=3;	
				}
				else
				{
					//printf("rcResult.xy =(%f,%f)   wh=(%f,%f)\n",rcResult.x,rcResult.y,rcResult.width,rcResult.height);
					extInCtrl->TrkStat=2;
				}
			}
		 	 if((extInCtrl->TrkStat == 1)||(extInCtrl->TrkStat == 2))
		 	 {
				extInCtrl->TrkX =rcResult.x+rcResult.width/2;
				extInCtrl->TrkY = rcResult.y+rcResult.height/2;
				if(extInCtrl->FovCtrl==5&&extInCtrl->SensorStat==0){
					if(extInCtrl->TrkX>=vdisWH[0][0]/2)
						extInCtrl->TrkX = (extInCtrl->TrkX-vdisWH[0][0]/2)*2+vdisWH[0][0]/2 ;
					else if(extInCtrl->TrkX<vdisWH[0][0]/2)
						extInCtrl->TrkX = vdisWH[0][0]/2 -(vdisWH[0][0]/2 - extInCtrl->TrkX)*2 ;

					if(extInCtrl->TrkY>=vdisWH[0][1]/2)
						extInCtrl->TrkY = (extInCtrl->TrkY-vdisWH[0][1]/2)*2+vdisWH[0][1]/2;
					else if(extInCtrl->TrkY<vdisWH[0][1]/2)
						extInCtrl->TrkY = vdisWH[0][1]/2 -(vdisWH[0][1]/2 - extInCtrl->TrkY)*2;
				}
				
				//OSA_printf("transferbefore ********* trkxy(%f,%f)\n",extInCtrl.TrkX,extInCtrl.TrkY);
				extInCtrl->trkerrx=(PiexltoWindowsxf(extInCtrl->TrkX ,extInCtrl->SensorStat));//*10;
				extInCtrl->trkerry=(PiexltoWindowsyf(extInCtrl->TrkY ,extInCtrl->SensorStat));//*10;
				//OSA_printf("transferafter ********* trkxy(%d,%d)\n",extInCtrl.trkerrx,extInCtrl.trkerry);
				
				if(0)//(extInCtrl.unitTrkStat == 2)
				{
					extInCtrl->trkerrx=(PiexltoWindowsx(m_ImageAxisx ,extInCtrl->SensorStat));//*10;
					extInCtrl->trkerry=(PiexltoWindowsy(m_ImageAxisy ,extInCtrl->SensorStat));//*10;
				}
				//OSA_printf("send ********* trkxy(%d,%d)\n",extInCtrl->trkerrx,extInCtrl->trkerry);
				extInCtrl->TrkErrFeedback = 1;
		 	 }
			 else
			 	extInCtrl->TrkErrFeedback = 0;
			 
			if(extInCtrl->TrkStat!=extInCtrl->TrkStatpri)
			{
				extInCtrl->TrkStatpri=extInCtrl->TrkStat;
				//MSGAPI_AckSnd( AckTrkType);
			}

			#if __IPC__
					if(extInCtrl->TrkStat != 3)
					{
						extInCtrl->trkerrx = extInCtrl->trkerrx - VIDEO_IMAGE_WIDTH_0/2;
						extInCtrl->trkerry = extInCtrl->trkerry - VIDEO_IMAGE_HEIGHT_0/2;
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

	//mtd
osdindex++;
	{
		if(Osdflag[osdindex]==1)
		{
			erassdrawmmtnew(Mdrawbak, false);
			Osdflag[osdindex]=0;
		}
		if(m_bMtd)
		{
			drawmmtnew(m_mtd[chId]->tg, true);		
		}
	}

osdindex++;
	// blob detect
	{
		if(Osdflag[osdindex]==1)
		{
			DrawBlob(blob_rectBak, false);
			Osdflag[osdindex]=0;
		}
		if(m_bBlobDetect&&(extInCtrl->SensorStat==0)){
			DrawBlob(m_blobRect, true);
			memcpy(&blob_rectBak, &m_blobRect, sizeof(BlobRect));
			Osdflag[osdindex]=1;
		}
	}
osdindex++; //dash little cross
	{
		if(Osdflag[osdindex]==1)
			{
				DrawdashCross(0,0,frcolor,false);
				Osdflag[osdindex]=0;
			}
	//	DrawdashCross(0,0,frcolor,true);
		
	}
osdindex++;	//cross aim
	{
	 	if(Osdflag[osdindex]==1)
		{
			recIn.x=crossBak.x;
	 		recIn.y=crossBak.y;
			recIn.width = crossWHBak.x;
			recIn.height = crossWHBak.y;
			DrawCross(recIn,frcolor,false);
			Osdflag[osdindex]=0;
 		}

		if(extInCtrl->DispGrp[extInCtrl->SensorStat]<=3)
		{
			startx=PiexltoWindowsx(extInCtrl->AxisPosX[extInCtrl->SensorStat],extInCtrl->SensorStat);
	 		starty=PiexltoWindowsy(extInCtrl->AxisPosY[extInCtrl->SensorStat],extInCtrl->SensorStat);

			recIn.x = startx;
			recIn.y = starty;
			recIn.width = extInCtrl->crossAxisWidth;
			recIn.height = extInCtrl->crossAxisHeight;
			if(extInCtrl->AvtTrkStat == eTrk_mode_acq)
			{
				DrawCross(recIn,frcolor,true);
				Osdflag[osdindex]=1;
			}
			else if(extInCtrl->AvtTrkStat == eTrk_mode_search)
			{
				frcolor = 3;
				DrawCross(recIn,frcolor,true);
				Osdflag[osdindex]=1;
			}
			crossBak.x=startx;
			crossBak.y=starty;
			crossWHBak.x = recIn.width;
			crossWHBak.y = recIn.height;
		}
	}

osdindex++;	//acqRect
	{
	 	if(Osdflag[osdindex]==1){
			recIn = acqRectBak;
			DrawAcqRect(m_dccv,recIn,frcolor,false);
			Osdflag[osdindex]=0;
 		}
		if(extInCtrl->AvtTrkStat == eTrk_mode_acq){
			recIn.x  = PiexltoWindowsx(extInCtrl->AxisPosX[extInCtrl->SensorStat],extInCtrl->SensorStat);
	 		recIn.y  = PiexltoWindowsy(extInCtrl->AxisPosY[extInCtrl->SensorStat],extInCtrl->SensorStat);
			recIn.width  = 300;//extInCtrl->AcqRectW[extInCtrl->SensorStat];
			recIn.height = 300;//extInCtrl->AcqRectH[extInCtrl->SensorStat]; 
			if(recIn.width%2 == 1)
				recIn.width++;
			if(recIn.height%2 == 1)
				recIn.height++;
			recIn.x = recIn.x  - recIn.width/2;
			recIn.y = recIn.y  + recIn.height/2;
			DrawAcqRect(m_dccv,recIn,frcolor,true);
			Osdflag[osdindex]=1;

			acqRectBak = recIn;
		}
	}

	
#if __MOVE_DETECT__
#if __DETECT_SWITCH_Z__
	osdindex++;
	{
		if(Osdflag[osdindex]==1)
		{
			detect_num = detect_bak.size();
			for(i=0;i<detect_num;i++)
			{	
				DrawRect(m_dccv, detect_bak[i].targetRect,0);
			}			
			Osdflag[osdindex]=0;
		}
		if(m_bMoveDetect)
		{
			detect_num = detect_vect.size();		
			DrawMoveDetect = 1;
			for(i =0;i<detect_num;i++)
			{
				if(detect_vect[i].targetRect.width > 20 && detect_vect[i].targetRect.height > 20 )
				{
					DrawRect(m_dccv, detect_vect[i].targetRect,2);
					random.x = detect_vect[i].targetRect.x;
					random.y = detect_vect[i].targetRect.y;
					random.h = detect_vect[i].targetRect.height;
					random.w =detect_vect[i].targetRect.width;			
				}
			}		
			detect_bak = detect_vect;
			Osdflag[osdindex]=1;
		}
		else
			DrawMoveDetect = 0 ;
	}
#else
	osdindex++;
	{
		if(Osdflag[osdindex]==1){
			for(i=0;i<6;i++){	
				DrawRect(m_dccv, backRect[i],0);
			}			
			Osdflag[osdindex]=0;
		}
		if(m_bMoveDetect){
			for(i=0;i<6;i++){	
				DrawRect(m_dccv, boundRect[i],4);
				backRect[i] = boundRect[i];
			}			
			Osdflag[osdindex]=1;
		}		
	}
#endif	
#endif
	prisensorstatus=extInCtrl->SensorStat;

	static unsigned int count = 0;
	if((count & 1) == 1)
		OSA_semSignal(&(sThis->m_display.tskdisSemmain));
	count++;
	//sThis->m_display.UpDateOsd(1);
	return true;
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

void CProcess::OnMouseLeftDwn(int x, int y){};
void CProcess::OnMouseLeftUp(int x, int y){};
void CProcess::OnMouseRightDwn(int x, int y){};
void CProcess::OnMouseRightUp(int x, int y){};

void CProcess::OnKeyDwn(unsigned char key)
{
	CMD_EXT *pIStuts = extInCtrl;
	CMD_EXT tmpCmd = {0};

	if(key == 'a' || key == 'A')
	{
		pIStuts->SensorStat = (pIStuts->SensorStat + 1)%eSen_Max;
		msgdriv_event(MSGID_EXT_INPUT_SENSOR, NULL);
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
		if(pIStuts->AvtTrkStat)
			pIStuts->AvtTrkStat = eTrk_mode_acq;
		else
			pIStuts->AvtTrkStat = eTrk_mode_target;
		msgdriv_event(MSGID_EXT_INPUT_TRACK, NULL);
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
		if(pIStuts->ImgEnhStat[pIStuts->SensorStat])
			pIStuts->ImgEnhStat[pIStuts->SensorStat] = eImgAlg_Disable;
		else
			pIStuts->ImgEnhStat[pIStuts->SensorStat] = eImgAlg_Enable;
		msgdriv_event(MSGID_EXT_INPUT_ENENHAN, NULL);
	}

	if (key == 'k' || key == 'K')
	{
		
		if(pIStuts->MtdState[pIStuts->SensorStat])
			pIStuts->MtdState[pIStuts->SensorStat] = eImgAlg_Disable;
		else
			pIStuts->MtdState[pIStuts->SensorStat] = eImgAlg_Enable;
		msgdriv_event(MSGID_EXT_MVDETECT, NULL);
		printf("pIStuts->MtdState[pIStuts->SensorStat]  = %d\n",pIStuts->MtdState[pIStuts->SensorStat] );
	}

	if (key == 'o' || key == 'O')
	{
		if(pIStuts->ImgBlobDetect[pIStuts->SensorStat])
			pIStuts->ImgBlobDetect[pIStuts->SensorStat] = eImgAlg_Disable;
		else
			pIStuts->ImgBlobDetect[pIStuts->SensorStat] = eImgAlg_Enable;
		msgdriv_event(MSGID_EXT_INPUT_ENBDT, NULL);
	}

	if (key == 't' || key == 'T')
		{
			if(pIStuts->ImgVideoTrans[pIStuts->SensorStat])
				pIStuts->ImgVideoTrans[pIStuts->SensorStat] = eImgAlg_Disable;
			else
				pIStuts->ImgVideoTrans[pIStuts->SensorStat] = eImgAlg_Enable;
			msgdriv_event(MSGID_EXT_INPUT_RST_THETA, NULL);
		}
	if (key == 'f' || key == 'F')
		{
			if(pIStuts->ImgFrezzStat[pIStuts->SensorStat])
				pIStuts->ImgFrezzStat[pIStuts->SensorStat] = eImgAlg_Disable;
			else
				pIStuts->ImgFrezzStat[pIStuts->SensorStat] = eImgAlg_Enable;
			
			msgdriv_event(MSGID_EXT_INPUT_ENFREZZ, NULL);
		}
	if (key == 'p'|| key == 'P')
		{
			
			//pIStuts->PicpPosStat=(pIStuts->PicpPosStat+1)%4;
			msgdriv_event(MSGID_EXT_INPUT_PICPCROP, NULL);
		}
	if (key == 'g'|| key == 'G')
	{
		/***************posmov**************/
		printf("before set AxisPos\n");
		tmpCmd.axisMoveStepX = 1;
		tmpCmd.axisMoveStepY = 0;
		app_ctrl_setAxisPos(&tmpCmd);

		//msgdriv_event(MSGID_EXT_INPUT_COAST, NULL);
	}

	if (key == 'h'|| key == 'H')
	{
		printf("before set AxisPos\n");
		tmpCmd.axisMoveStepX= eTrk_ref_left;
		app_ctrl_setAxisPos(&tmpCmd);
		
		//pIStuts->AvtTrkAimSize = 3;
		//app_ctrl_setAimSize(pIStuts);
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

	if (key == 'y'|| key == 'Y')
	{		
		if(moveDetectRect == true)
		{
			moveDetectRect = false;
			rectangle( m_dccv,
				Point( preAcpSR.x, preAcpSR.y ),
				Point( preAcpSR.x+preAcpSR.width, preAcpSR.y+preAcpSR.height),
				cvScalar(0,0,0,0), 1, 8 );

			rectangle( m_dccv,
				Point( preWarnRect.x, preWarnRect.y ),
				Point( preWarnRect.x+preWarnRect.width, preWarnRect.y+preWarnRect.height),
				cvScalar(0,0,0,0), 2, 8 );
		}
		else
			moveDetectRect = true;
		OSA_printf("moveDetectRect = %d\n",moveDetectRect);
	}

	
	if (key == 'z'|| key == 'Z')
	{
		
		pIStuts->ImgZoomStat[0]=(pIStuts->ImgZoomStat[0]+1)%2;
		pIStuts->ImgZoomStat[1]=(pIStuts->ImgZoomStat[1]+1)%2;
		msgdriv_event(MSGID_EXT_INPUT_ENZOOM, NULL);
	}

	
}


void CProcess::msgdriv_event(MSG_PROC_ID msgId, void *prm)
{
	int tempvalue=0;
	CMD_EXT *pIStuts = extInCtrl;
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
		m_rcAcq.width		=	pIStuts->AimW[pIStuts->SensorStat];//trkWinWH[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][0];
		m_rcAcq.height	=	pIStuts->AimH[pIStuts->SensorStat];//trkWinWH[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][1];

		m_rcAcq.x=pIStuts->opticAxisPosX[pIStuts->SensorStat]-m_rcAcq.width/2;
		m_rcAcq.y=pIStuts->opticAxisPosY[pIStuts->SensorStat]-m_rcAcq.height/2;

		OSA_printf("recv   the rctrack x=%f y=%f w=%f h=%f  sensor=%d picpsensor=%d\n",m_rcAcq.x,m_rcAcq.y,
			m_rcAcq.width,m_rcAcq.height,pIStuts->SensorStat,pIStuts->PicpSensorStat);
		
		itmp = pIStuts->SensorStat;
		dynamic_config(VP_CFG_MainChId, itmp, NULL);

#if 1//change the sensor picp change too
		if(pIStuts->PicpSensorStat==0||pIStuts->PicpSensorStat==1)
		{
			if(pIStuts->SensorStat==0)
				pIStuts->PicpSensorStat=0;
			else
				pIStuts->PicpSensorStat=1;

			pIStuts->PicpSensorStatpri=pIStuts->PicpSensorStat;
		}
#endif

		itmp = pIStuts->PicpSensorStat;//freeze change
		dynamic_config(VP_CFG_SubChId, itmp, NULL);
////enhance 

		if(pIStuts->ImgEnhStat[pIStuts->SensorStat^1] ==0x01)
		{
			int ENHStatus=0;
			if(pIStuts->ImgEnhStat[pIStuts->SensorStat] ==0x00)
			{
				ENHStatus=pIStuts->ImgEnhStat[pIStuts->SensorStat]=pIStuts->ImgEnhStat[pIStuts->SensorStat^1];		
				OSA_printf("the enhstaus=%d  pIStuts->SensorStat=%d\n",ENHStatus,pIStuts->SensorStat);
				dynamic_config(CDisplayer::DS_CFG_EnhEnable, pIStuts->SensorStat, &ENHStatus);
			}
			ENHStatus=0;
			dynamic_config(CDisplayer::DS_CFG_EnhEnable, pIStuts->SensorStat^1, &ENHStatus);
			pIStuts->ImgEnhStat[pIStuts->SensorStat^1]=0;
				
		}
		
//sec track sync
		if(pIStuts->SensorStat==0)
		{
			pIStuts->AvtPosX[pIStuts->SensorStat] =PiexltoWindowsx( pIStuts->AvtPosX[pIStuts->SensorStat^1],pIStuts->SensorStat^1);
			pIStuts->AvtPosY[pIStuts->SensorStat] =PiexltoWindowsy( pIStuts->AvtPosY[pIStuts->SensorStat^1],pIStuts->SensorStat^1);
		}
		else
		{
			pIStuts->AvtPosX[pIStuts->SensorStat] =WindowstoPiexlx( pIStuts->AvtPosX[pIStuts->SensorStat^1],pIStuts->SensorStat);
			pIStuts->AvtPosY[pIStuts->SensorStat] =WindowstoPiexly( pIStuts->AvtPosY[pIStuts->SensorStat^1],pIStuts->SensorStat);
		}

//sensor 1 rect

		DS_Rect lay_rect;
	#if 1
		lay_rect.w =vdisWH[0][0]/6;
		lay_rect.h = vdisWH[0][1]/6;
		lay_rect.x = crossBak.x - lay_rect.w/2;//vdisWH[0][0]/2-lay_rect.w/2;
		lay_rect.y = crossBak.y - lay_rect.h/2;//vdisWH[0][1]/2-lay_rect.h/2;
	#endif
		if(pIStuts->PicpSensorStat==1)
		{
			lay_rect.w = vcapWH[1][0]/3;
			lay_rect.h = vcapWH[1][1]/3;
			lay_rect.x = vcapWH[1][0]/2-lay_rect.w/2;
			lay_rect.y = vcapWH[1][1]/2-lay_rect.h/2;
		}
		m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 1, &lay_rect);

//picp position
		lay_rect=rendpos[pIStuts->PicpPosStat];
		m_ImageAxisx=pIStuts->AvtPosX[extInCtrl->SensorStat ];
		m_ImageAxisy=pIStuts->AvtPosY[extInCtrl->SensorStat ];
		//printf("m_ImageAxisx,m_ImageAxisy = (%d,%d) \n",m_ImageAxisx,m_ImageAxisy);
		//OSA_printf("%s: lay_rect: %d, %d,  %d x %d\n", __func__, lay_rect.x, lay_rect.y, lay_rect.w, lay_rect.h);
		
		
		m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 1, &lay_rect);

///sensor zoom

		if(pIStuts->ImgZoomStat[pIStuts->SensorStat])
		{
			memset(&lay_rect, 0, sizeof(DS_Rect));
			//if(pIStuts->SensorStat==0)//just tv zooom
			{
				lay_rect.w = vcapWH[pIStuts->SensorStat][0]/2;
				lay_rect.h = vcapWH[pIStuts->SensorStat][1]/2;
				lay_rect.x = vcapWH[pIStuts->SensorStat][0]/4;
				lay_rect.y = vcapWH[pIStuts->SensorStat][1]/4;
			}
			
			m_display.dynamic_config(CDisplayer::DS_CFG_CropRect, 0, &lay_rect);
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
		
			if(DrawMoveDetect)
				pIStuts->unitAimX =  random.x+random.w/2;
			else
				pIStuts->unitAimX = pIStuts->AvtPosX[extInCtrl->SensorStat] ;
			if(pIStuts->unitAimX<0)
			{
				pIStuts->unitAimX=0;
			}
			if(DrawMoveDetect)
				pIStuts->unitAimY = random.y+random.h/2;
			else
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

		int iSens = (pIStuts->SensorStat+1)%eSen_Max;
		 if (pIStuts->AvtTrkStat == eTrk_mode_sectrk)
		{
			OSA_printf(" %d:%s line:%d set track to [%s]\n", OSA_getCurTimeInMsec(), __func__,
					   __LINE__,procStr[pIStuts->AvtTrkStat]);

			//pIStuts->AvtTrkStat = eTrk_mode_sectrk;
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

			
			OSA_printf(" %d:%s line:%d set track to [%s]\n", OSA_getCurTimeInMsec(), __func__,
					   __LINE__,procStr[pIStuts->AvtTrkStat]);


			pIStuts->AvtTrkStat = eTrk_mode_target;

			//zoom for mtdTrk change xy 
			 if(pIStuts->FovCtrl==5&&pIStuts->SensorStat==0)
		 	{
		 		pIStuts->MmtPixelX=pIStuts->MmtPixelX-320;
				pIStuts->MmtPixelX=2*pIStuts->MmtPixelX;

				pIStuts->MmtPixelY=pIStuts->MmtPixelY-256;
				pIStuts->MmtPixelY=2*pIStuts->MmtPixelY;
				
				pIStuts->unitAimX=pIStuts->MmtPixelX;
				pIStuts->unitAimY=pIStuts->MmtPixelY;

		 	}else{
			
				pIStuts->unitAimX=pIStuts->MmtPixelX;
				pIStuts->unitAimY=pIStuts->MmtPixelY;
		 	}

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
			//pIStuts->unitTrkProc = eTrk_proc_target;

			//return ;
		}

		OSA_printf(" %d:%s line:%d set track to [%s]\n", OSA_getCurTimeInMsec(), __func__,
					   __LINE__,procStr[pIStuts->AvtTrkStat]);
		
		if((pIStuts->FovCtrl==5)&&(pIStuts->SensorStat==0))
		{
			if(TrkAim43 == true)
			{
				rc.x=pIStuts->unitAimX-trkWinWH43[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][0]/4;
				rc.y=pIStuts->unitAimY-trkWinWH43[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][1]/4;
				rc.width= trkWinWH43[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][0]/2;
				rc.height= trkWinWH43[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][1]/2;
			}
			else
			{

				if(pIStuts->unitAimX>=vdisWH[0][0]/2)
					pIStuts->unitAimX = (pIStuts->unitAimX-vdisWH[0][0]/2)/2+vdisWH[0][0]/2 ;
				else if(pIStuts->unitAimX<vdisWH[0][0]/2)
					pIStuts->unitAimX = vdisWH[0][0]/2 -(vdisWH[0][0]/2 - pIStuts->unitAimX)/2 ;

				if(pIStuts->unitAimY>=vdisWH[0][1]/2)
					pIStuts->unitAimY = (pIStuts->unitAimY-vdisWH[0][1]/2)/2+vdisWH[0][1]/2;
				else if(pIStuts->unitAimY<vdisWH[0][1]/2)
					pIStuts->unitAimY = vdisWH[0][1]/2 -(vdisWH[0][1]/2 - pIStuts->unitAimY)/2;

				
				rc.x=pIStuts->unitAimX-trkWinWHZoom[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][0]/2;
				rc.y=pIStuts->unitAimY-trkWinWHZoom[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][1]/2;

				//zoom need
				rc.width= trkWinWHZoom[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][0];
				rc.height= trkWinWHZoom[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][1];
			}
		}
		else
		{
			if(TrkAim43 == true)
			{
				rc.x=pIStuts->unitAimX-trkWinWH43[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][0]/2;
				rc.y=pIStuts->unitAimY-trkWinWH43[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][1]/2;
				rc.width= trkWinWH43[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][0];
				rc.height= trkWinWH43[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][1];
			}
			else
			{
				printf("%s,line:%d   aimx,aimy=(%d,%d)\n",__func__,__LINE__,pIStuts->AvtPosX[0],pIStuts->AvtPosY[0]);
				if(pIStuts->AvtTrkStat == eTrk_mode_sectrk){
					pIStuts->unitAimX = pIStuts->AvtPosX[0];
					pIStuts->unitAimY = pIStuts->AvtPosY[0];
				}
				rc.width= pIStuts->AimW[pIStuts->SensorStat];//trkWinWH[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][0];
				rc.height=pIStuts->AimW[pIStuts->SensorStat];//trkWinWH[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][1];
				rc.x=pIStuts->unitAimX-rc.width/2;
				rc.y=pIStuts->unitAimY-rc.height/2;
			}
		}
			
		OSA_printf("%s,line:%d   rc. xy(%f,%f),wh(%f,%f)\n",__func__,__LINE__,rc.x,rc.y,rc.width,rc.height);
		dynamic_config(VP_CFG_TrkEnable, 1,&rc);
		if(pIStuts->AvtTrkStat == eTrk_mode_sectrk)
		{
			m_intervalFrame=2;
			m_rcAcq=rc;
			pIStuts->AvtTrkStat = eTrk_mode_target;
			OSA_printf("%s  line:%d		set sec track\n ",__func__,__LINE__);	
		}
	//	printf("the rc.x=%d rc.y=%d ,unitAimX=%d  unitAimY=%d \n",rc.x,rc.y,pIStuts->unitAimX,pIStuts->unitAimY);
	//	printf("w=%d h=%d\n",pIStuts->unitAimW,pIStuts->unitAimH);
		
 	}

	if(msgId == MSGID_EXT_INPUT_ENMTD)
	{
		if(prm != NULL)
		{
			pInCmd = (CMD_EXT *)prm;
			pIStuts->MmtStat[0] = pInCmd->MmtStat[0];
			pIStuts->MmtStat[1] = pInCmd->MmtStat[1];
		}

		int MTDStatus = (pIStuts->MmtStat[pIStuts->SensorStat]&0x01) ;

//		OSA_printf(" %d:%s set mtd enMask %x\n", OSA_getCurTimeInMsec(),__func__,m_mtd_ctrl.un_mtd.enMask);

		if(MTDStatus)
			dynamic_config(VP_CFG_MtdEnable, 1);
		else
			dynamic_config(VP_CFG_MtdEnable, 0);
		//FOR DUMP FRAME
		if(MTDStatus)
			dynamic_config(CDisplayer::DS_CFG_MMTEnable, pIStuts->SensorStat, &MTDStatus);
		else
			dynamic_config(CDisplayer::DS_CFG_MMTEnable, pIStuts->SensorStat, &MTDStatus);
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

	if(msgId == MSGID_EXT_INPUT_ENBDT)
	{
		if(prm != NULL)
		{
			pInCmd = (CMD_EXT *)prm;
			pIStuts->ImgBlobDetect[0] = pInCmd->ImgBlobDetect[0];
			pIStuts->ImgBlobDetect[1] = pInCmd->ImgBlobDetect[1];
		}

		int BlobStatus = (pIStuts->ImgBlobDetect[pIStuts->SensorStat]&0x01) ;

		//		OSA_printf(" %d:%s set mtd enMask %x\n", OSA_getCurTimeInMsec(),__func__,m_mtd_ctrl.un_mtd.enMask);

		if(BlobStatus)
			dynamic_config(VP_CFG_BlobEnable, 1);
		else
			dynamic_config(VP_CFG_BlobEnable, 0);
	}

	if(msgId == MSGID_EXT_INPUT_RST_THETA)
	{
		if(prm != NULL)
		{
			pInCmd = (CMD_EXT *)prm;
		}

		int videoTrans = (pIStuts->ImgVideoTrans[pIStuts->SensorStat]&0x01) ;

//		OSA_printf(" %d:%s set mtd enMask %x\n", OSA_getCurTimeInMsec(),__func__,m_mtd_ctrl.un_mtd.enMask);
		GLfloat result[16];

		if(videoTrans)
		{
			my_rotate(result, 45.0);
			dynamic_config(CDisplayer::DS_CFG_VideoTransMat, pIStuts->SensorStat, result);
		}
		else
		{
			my_rotate(result, 0.0);
			dynamic_config(CDisplayer::DS_CFG_VideoTransMat, pIStuts->SensorStat, result);
		}
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

		if(pIStuts->AvtTrkAimSize<0||pIStuts->AvtTrkAimSize>4)
		{
			pIStuts->AvtTrkAimSize=2;
		}
		if(pIStuts->AvtTrkStat)
		{
			UTC_RECT_float rc;
			if(msgId == MSGID_EXT_INPUT_AIMSIZE)
			{
				pIStuts->unitAimW 	= pIStuts->AimW[pIStuts->SensorStat];//trkWinWH[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][0];
				pIStuts->unitAimH		= pIStuts->AimH[pIStuts->SensorStat];//trkWinWH[pIStuts->SensorStat][pIStuts->AvtTrkAimSize][1];
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
				/*
				if(pIStuts->unitAimW > trkWinWH[pIStuts->SensorStat][4][0])
					pIStuts->unitAimW = trkWinWH[pIStuts->SensorStat][4][0];
				if(pIStuts->unitAimH > trkWinWH[pIStuts->SensorStat][4][1])
					pIStuts->unitAimH = trkWinWH[pIStuts->SensorStat][4][1];
				*/
				printf("111W,H : (%d,%d)\n",pIStuts->unitAimW,pIStuts->unitAimH);
				rc.width=pIStuts->unitAimW;
				rc.height=pIStuts->unitAimH;
				printf("222rc.width,rc.height : (%f,%f)\n",rc.width,rc.height);
				
				rc.x = pIStuts->unitAimX-pIStuts->unitAimW/2 + pIStuts->aimRectMoveStepX;
				rc.y = pIStuts->unitAimY-pIStuts->unitAimH/2  + pIStuts->aimRectMoveStepY;
				printf("333rc.x,rc.y : (%d,%d)\n",rc.x,rc.y);

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
			
			lay_rect.w = vcapWH[1][0]/3;
			lay_rect.h = vcapWH[1][1]/3;
			lay_rect.x = vcapWH[1][0]/2-lay_rect.w/2;
			lay_rect.y = vcapWH[1][1]/2-lay_rect.h/2;	
			
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
	if(msgId ==MSGID_EXT_INPUT_ENFREZZ)
	{
		int freeze=pIStuts->ImgFrezzStat[pIStuts->SensorStat];
		DS_Rect rendposr;
		//wj20180319
		#if 0
		rendposr.x=vdisWH[0][0]*2/3;
		rendposr.y=vdisWH[0][1]*2/3;
		rendposr.w=vdisWH[0][0]/3;
		rendposr.h=vdisWH[0][1]/3;
		#else
		rendposr.x=0;
		rendposr.y=vdisWH[0][1]*2/3;
		rendposr.w=vdisWH[0][0]/3;
		rendposr.h=vdisWH[0][1]/3;
		
		#endif
		if(freeze)		//QQQQQQ
			;//m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 1, &rendposr);
		else
		{
			rendposr=rendpos[pIStuts->PicpPosStat];
			//m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 1, &rendposr);
		}
		//tui chu freeze
		m_display.dynamic_config(CDisplayer::DS_CFG_FreezeEnable, 1, &freeze);
		m_display.dynamic_config(CDisplayer::DS_CFG_FreezeEnable, 0, &freeze);

	}
	if(msgId ==MSGID_EXT_INPUT_PICPCROP)
	{		
		m_display.dynamic_config(CDisplayer::DS_CFG_RenderPosRect, 1, &rendpos[pIStuts->PicpPosStat]);
	}
	if(msgId ==MSGID_EXT_INPUT_COAST)
	{
		m_castTm=OSA_getCurTimeInMsec();
		m_bCast=true;
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

	if(msgId == MSGID_EXT_MVDETECT)
	{	
		int Mtdstatus = (pIStuts->MtdState[pIStuts->validChId]&0x01) ;
		if(Mtdstatus)
		{
			dynamic_config(VP_CFG_MvDetect, 1,NULL);
			tmpCmd.MtdState[pIStuts->SensorStat] = 1;
			app_ctrl_setMtdStat(&tmpCmd);
		}
		else
		{
			dynamic_config(VP_CFG_MvDetect, 0,NULL);
			tmpCmd.MtdState[pIStuts->SensorStat] = 0;
			app_ctrl_setMtdStat(&tmpCmd);
		}
	}
	
}


/////////////////////////////////////////////////////
//int majormmtid=0;

 int  CProcess::MSGAPI_initial()
{
   MSGDRIV_Handle handle=&g_MsgDrvObj;
    assert(handle != NULL);
    memset(handle->msgTab, 0, sizeof(MSGTAB_Class) * MAX_MSG_NUM);
//MSGID_EXT_INPUT_MTD_SELECT
    MSGDRIV_attachMsgFun(handle,    MSGID_SYS_INIT,           				MSGAPI_init_device,       		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_SENSOR,           	   	MSGAPI_inputsensor,       		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_PICPCROP,      		MSGAPI_croppicp,       		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_TRACK,          		MSGAPI_inputtrack,     		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ENMTD,                   MSGAPI_inpumtd,       		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_MTD_SELECT,     	MSGAPI_inpumtdSelect,    		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_AIMPOS,          	 	MSGAPI_setAimRefine,    		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_AIMSIZE,          	       MSGAPI_setAimSize,    		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ENENHAN,           	MSGAPI_inpuenhance,       	            0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ENBDT,           		MSGAPI_inputbdt,         		    0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ENZOOM,           	MSGAPI_inputzoom,                     0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_ENFREZZ,           	MSGAPI_inputfrezz,                      0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_MTD_SELECT,      	MSGAPI_inputmmtselect,              0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_AXISPOS,     	  	MSGAPI_inputpositon,                   0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_COAST,             	MSGAPI_inputcoast,                      0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_FOVSELECT,             MSGAPI_inputfovselect,                 0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_FOVSTAT,                	MSGAPI_inputfovchange,               0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_SEARCHMOD,            MSGAPI_inputsearchmod,              0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_VIDEOEN,            	MSGAPI_inputvideotect,                 0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_MMTSHOW,             	MSGAPI_mmtshow,                 	     0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_FOVCMD,             	MSGAPI_FOVcmd,                 	     0);
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_INPUT_CFGSAVE,             	MSGAPI_SaveCfgcmd,                 	     0);	
    MSGDRIV_attachMsgFun(handle,    MSGID_EXT_MVDETECT,             		MSGAPI_setMtdState,                 	     0);	

  
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
	CMD_EXT *pIStuts = sThis->extInCtrl;
	if(pIStuts->MtdState[pIStuts->validChId] == 0)
	{
		rectangle( sThis->m_display.m_imgOsd[1],
			Point( sThis->preWarnRect.x, sThis->preWarnRect.y ),
			Point( sThis->preWarnRect.x+sThis->preWarnRect.width, sThis->preWarnRect.y+sThis->preWarnRect.height),
			cvScalar(0,0,0,0), 2, 8 );
	}

	sThis->msgdriv_event(MSGID_EXT_MVDETECT,NULL);
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
	if(pIStuts->SensorStat==0)
	{
		if((pIStuts->AxisPosX[0]>=50)&&(pIStuts->AxisPosX[0]<=vcapWH[pIStuts->SensorStat][0]-50))
		{
			if(pIStuts->axisMoveStepX != 0)
			{
				pIStuts->AxisPosX[0] += pIStuts->axisMoveStepX;
				pIStuts->axisMoveStepX = 0;
			}	
			pIStuts->unitAimX = pIStuts->AxisPosX[0];
			sThis->m_ImageAxisx=pIStuts->AvtPosX[pIStuts->SensorStat ];			
		}
		if((pIStuts->AxisPosY[0]>=50)&&(pIStuts->AxisPosY[0]<=vcapWH[pIStuts->SensorStat][1]-50))
		{
			if(pIStuts->axisMoveStepY != 0)
			{
				pIStuts->AxisPosY[0] += pIStuts->axisMoveStepY;
				pIStuts->axisMoveStepY = 0;
			}
			pIStuts->unitAimY = pIStuts->AxisPosY[0];
			sThis->m_ImageAxisy=pIStuts->AvtPosY[pIStuts->SensorStat ];
		}
	}
	OSA_printf("%s   THE=unitAimX=%d unitAxisY=%d\n",__func__,pIStuts->opticAxisPosX[pIStuts->SensorStat ],pIStuts->opticAxisPosY[pIStuts->SensorStat ]);
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
		if(pIStuts->SensorStat == 0)
		{
		#if __TRACK__
			if(pIStuts->FovStat == 1)
				sThis->Track_fovreacq( 2400,pIStuts->SensorStat,0);
			else if(pIStuts->FovStat == 3)
				sThis->Track_fovreacq( 330,pIStuts->SensorStat,0);
			else if(pIStuts->FovStat == 4)	
				sThis->Track_fovreacq( 110,pIStuts->SensorStat,0);					
			else if(pIStuts->FovStat == 5){
				sThis->Track_fovreacq( 55,pIStuts->SensorStat,0);
			}
		#endif
		}
		else if(pIStuts->SensorStat == 1){
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
	pIStuts->AcqRectH[0] = gConfig_Osd_param.ch0_acqRect_height;
	pIStuts->AcqRectH[1] = gConfig_Osd_param.ch1_acqRect_height;
	pIStuts->AcqRectH[2] = gConfig_Osd_param.ch2_acqRect_height;
	pIStuts->AcqRectH[3] = gConfig_Osd_param.ch3_acqRect_height;
	pIStuts->AcqRectH[4] = gConfig_Osd_param.ch4_acqRect_height;
	return ;
}

void CProcess::initAimRect()
{
	CMD_EXT *pIStuts = extInCtrl;
	
	
	return ;
}

void CProcess::updateConfigOsdParm()
{
	CMD_EXT *pIStuts = extInCtrl;

	pIStuts->SensorStatBegin 		= gConfig_Osd_param.MAIN_Sensor;
	pIStuts->osdTextShow 			= gConfig_Osd_param.OSD_text_show;
	pIStuts->osdDrawShow 		= gConfig_Osd_param.OSD_draw_show;	
	pIStuts->osdTextColor 			= gConfig_Osd_param.OSD_text_color;
	pIStuts->osdTextAlpha			= gConfig_Osd_param.OSD_text_alpha;
	pIStuts->osdTextFont			= gConfig_Osd_param.OSD_text_font;
	pIStuts->osdTextSize			= gConfig_Osd_param.OSD_text_size;
	pIStuts->osdDrawColor 		= gConfig_Osd_param.OSD_draw_color;
	pIStuts->crossAxisWidth 		= gConfig_Osd_param.CROSS_AXIS_WIDTH;
	pIStuts->crossAxisHeight		= gConfig_Osd_param.CROSS_AXIS_HEIGHT;
	pIStuts->picpCrossAxisWidth	= gConfig_Osd_param.Picp_CROSS_AXIS_WIDTH;
	pIStuts->picpCrossAxisHeight	= gConfig_Osd_param.Picp_CROSS_AXIS_HEIGHT;
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

	pIStuts->AimW[0] 			= gConfig_Osd_param.ch0_aim_width;
	pIStuts->AimW[1] 			= gConfig_Osd_param.ch1_aim_width;
	pIStuts->AimW[2] 			= gConfig_Osd_param.ch2_aim_width;
	pIStuts->AimW[3] 			= gConfig_Osd_param.ch3_aim_width;
	pIStuts->AimW[4] 			= gConfig_Osd_param.ch4_aim_width;
	pIStuts->AimW[5] 			= gConfig_Osd_param.ch5_aim_width;
	pIStuts->AimH[0] 				= gConfig_Osd_param.ch0_aim_height;
	pIStuts->AimH[1] 				= gConfig_Osd_param.ch1_aim_height;
	pIStuts->AimH[2] 				= gConfig_Osd_param.ch2_aim_height;
	pIStuts->AimH[3] 				= gConfig_Osd_param.ch3_aim_height;
	pIStuts->AimH[4] 				= gConfig_Osd_param.ch4_aim_height;
	pIStuts->AimH[5] 				= gConfig_Osd_param.ch5_aim_height;


	m_acqRectW = pIStuts->AimW[pIStuts->SensorStat];
	m_acqRectH = pIStuts->AimH[pIStuts->SensorStat];
	
	m_display.disptimeEnable = gConfig_Osd_param.Timedisp_9;
	m_display.m_bOsd = pIStuts->osdDrawShow;
	
	

	return;
}
