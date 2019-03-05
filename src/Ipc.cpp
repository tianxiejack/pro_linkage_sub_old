#include "Ipcctl.h"
#include <string.h>
#include "osa_thr.h"
#include "osa_buf.h"
#include "app_status.h"
#include "app_ctrl.h"
#include "configable.h"
#include "osd_text.hpp"
#include "msgDriv.h"
#include "Ipcctl.h"
#include "locale.h"
#include "wchar.h"
#include "trigonometric.hpp"

using namespace cr_trigonometricInterpolation;

#define DATAIN_TSK_PRI              (2)
#define DATAIN_TSK_STACK_SIZE       (0)
#define SDK_MEM_MALLOC(size)                                            OSA_memAlloc(size)

extern pthread_cond_t event_cond;
extern pthread_mutex_t event_mutex;

extern  bool startEnable;
extern OSDSTATUS gConfig_Osd_param ;
extern UTCTRKSTATUS gConfig_Alg_param;
int IrisAndFocusAndExit = 0;
CMD_triangle cmd_triangle;
OSD_param m_osd;
CMD_Mtd_Frame Mtd_Frame;
int ipc_loop = 1;
BallCOMConfig CurrentBallConfig ;
int g_ballAddress =0;
int g_ballRate =0;
extern void inputtmp(unsigned char cmdid);

extern osdbuffer_t disOsdBuf[32];
extern osdbuffer_t disOsdBufbak[32];
extern wchar_t disOsd[32][33];
extern int glosttime;
OSA_BufCreate msgSendBufCreate;
OSA_BufHndl msgSendBuf;

OSA_ThrHndl thrHandleDataIn_recv;
OSA_ThrHndl thrHandleDataIn_send;

extern SingletonSysParam* g_sysParam;
extern LinkagePos_t linkagePos ; 
class CProcess ;
extern CProcess *proc;
extern GB_WorkMode g_AppWorkMode;
extern MenuDisplay g_displayMode;

void initmessage()
{
    int status;
    int i=0;
    msgSendBufCreate.numBuf = OSA_BUF_NUM_MAX;
    for (i = 0; i < msgSendBufCreate.numBuf; i++)
    {
        msgSendBufCreate.bufVirtAddr[i] = SDK_MEM_MALLOC(64);
        OSA_assert(msgSendBufCreate.bufVirtAddr[i] != NULL);
        memset(msgSendBufCreate.bufVirtAddr[i], 0, 64);
    }
    OSA_bufCreate(&msgSendBuf, &msgSendBufCreate);
 

}

void MSGAPI_msgsend(int cmdID)
{
   int bufId = 0;
   unsigned char *quebuf;

   OSA_bufGetEmpty(&(msgSendBuf), &bufId, OSA_TIMEOUT_NONE);
   quebuf=(unsigned char *)msgSendBuf.bufInfo[bufId].virtAddr;

   //msgSendBuf.bufInfo[bufId].size = 7;
   quebuf[0]=cmdID & 0xFF;	

   OSA_bufPutFull(&(msgSendBuf), bufId);
}

int  send_msgpth(SENDST * RS422)
{
	memset(RS422,0,sizeof(char)*(PARAMLEN+1));
	int bufId=0;
	int sendLen=0;
	OSA_bufGetFull(&msgSendBuf, &bufId, OSA_TIMEOUT_FOREVER);
	memcpy(RS422, msgSendBuf.bufInfo[bufId].virtAddr,sizeof(SENDST));
	OSA_bufPutEmpty(&msgSendBuf, bufId);
	send_msg(RS422);
	return 0;
}

void get_acqRect_from_aim(OSDSTATUS *gConfig_Osd_param)
{
	if(gConfig_Osd_param->ch0_aim_width < 10)
		gConfig_Osd_param->ch0_aim_width = 10;
	if(gConfig_Osd_param->ch1_aim_width < 10)
		gConfig_Osd_param->ch1_aim_width = 10;
	if(gConfig_Osd_param->ch2_aim_width < 10)
		gConfig_Osd_param->ch2_aim_width = 10;
	if(gConfig_Osd_param->ch3_aim_width < 10)
		gConfig_Osd_param->ch3_aim_width = 10;
	if(gConfig_Osd_param->ch4_aim_width < 10)
		gConfig_Osd_param->ch4_aim_width = 10;
	if(gConfig_Osd_param->ch5_aim_width < 10)
		gConfig_Osd_param->ch5_aim_width = 10;
	if(gConfig_Osd_param->ch0_aim_height < 10)
		gConfig_Osd_param->ch0_aim_height = 10;
	if(gConfig_Osd_param->ch1_aim_height < 10)
		gConfig_Osd_param->ch1_aim_height = 10;
	if(gConfig_Osd_param->ch2_aim_height < 10)
		gConfig_Osd_param->ch2_aim_height = 10;
	if(gConfig_Osd_param->ch3_aim_height < 10)
		gConfig_Osd_param->ch3_aim_height = 10;
	if(gConfig_Osd_param->ch4_aim_height < 10)
		gConfig_Osd_param->ch4_aim_height = 10;
	if(gConfig_Osd_param->ch5_aim_height < 10)
		gConfig_Osd_param->ch5_aim_height = 10;
	gConfig_Osd_param->ch0_acqRect_width = gConfig_Osd_param->ch0_aim_width;
	gConfig_Osd_param->ch1_acqRect_width = gConfig_Osd_param->ch1_aim_width;
	gConfig_Osd_param->ch2_acqRect_width = gConfig_Osd_param->ch2_aim_width;
	gConfig_Osd_param->ch3_acqRect_width = gConfig_Osd_param->ch3_aim_width;
	gConfig_Osd_param->ch4_acqRect_width = gConfig_Osd_param->ch4_aim_width;
	gConfig_Osd_param->ch5_acqRect_width = gConfig_Osd_param->ch5_aim_width;
	gConfig_Osd_param->ch0_acqRect_height = gConfig_Osd_param->ch0_aim_height;
	gConfig_Osd_param->ch1_acqRect_height = gConfig_Osd_param->ch1_aim_height;
	gConfig_Osd_param->ch2_acqRect_height = gConfig_Osd_param->ch2_aim_height;
	gConfig_Osd_param->ch3_acqRect_height = gConfig_Osd_param->ch3_aim_height;
	gConfig_Osd_param->ch4_acqRect_height = gConfig_Osd_param->ch4_aim_height;
	gConfig_Osd_param->ch5_acqRect_height = gConfig_Osd_param->ch5_aim_height;
}
void* recv_msg(SENDST *RS422)
{
	unsigned char cmdID = 0;
	unsigned char imgID1 = 0;
	unsigned char imgID2 = 0;
	unsigned char imgID3 = 0;
	unsigned char imgID4 = 0;
	unsigned char imgID5 = 0;	
	unsigned char imgID6 = 0;

	CMD_SENSOR Rsensor;
	CMD_PinP Rpinp;
	CMD_TRK Rtrk;
	CMD_SECTRK Rsectrk;
	CMD_ENH Renh;
	CMD_MTD Rmtd;
	CMD_MMTSELECT Rmmtselect;
	CMD_TRKDOOR Rtrkdoor;
	CMD_SYNC422 Rsync422;
	CMD_POSMOVE Rposmove;
	CMD_POSMOVE Raxismove;
	CMD_ZOOM Rzoom;
	CMD_BoresightPos Rboresightmove;
	CMD_AcqBoxPos Racqpos;
	CMD_ALGOSDRECT Ralgosdrect;
	CMD_IPCRESOLUTION Rresolution;
	LinkagePos posOfLinkage;
	ctrlParams Rjosctrl;

	static int mouse_state = 0;

	//OSD_param* pOsd = NULL;
	//pOsd = &m_osd;

	if(RS422==NULL)
	{
		return NULL ;
	}

	cmdID	=  RS422->cmd_ID;
	imgID1	=  RS422->param[0];
	imgID2	=  RS422->param[1];
	imgID3	=  RS422->param[2];
	imgID4	=  RS422->param[3];
	imgID5 	=  RS422->param[4];

	CMD_EXT inCtrl, *pMsg = NULL;
	pMsg = &inCtrl;
	memset(pMsg,0,sizeof(CMD_EXT));
	if(startEnable)
		app_ctrl_getSysData(pMsg);

	osdbuffer_t* ppppp  = NULL;	
	//printf("cmdID : %d (%02x %02x %02x %02x %02x)\n",cmdID,imgID1,imgID2,imgID3,imgID4,imgID5);
	switch(cmdID)
	{	

	case mtdFrame:
		memcpy(&Mtd_Frame, RS422->param, sizeof(Mtd_Frame));
	#if 0
		printf("RmtdFrame->detectNum = %d\n", Mtd_Frame.detectNum);
		printf("RmtdFrame->detectSpeed = %d\n", Mtd_Frame.detectSpeed);
		printf("RmtdFrame->sensitivityThreshold = %d\n", Mtd_Frame.sensitivityThreshold);
		printf("RmtdFrame->tmpMaxPixel = %d\n", Mtd_Frame.tmpMaxPixel);
		printf("RmtdFrame->tmpMinPixel = %d\n", Mtd_Frame.tmpMinPixel);
		printf("RmtdFrame->tmpUpdateSpeed = %d\n", Mtd_Frame.tmpUpdateSpeed);
		printf("RmtdFrame->detectArea_X = %d\n", Mtd_Frame.detectArea_X);
		printf("RmtdFrame->detectArea_Y = %d\n", Mtd_Frame.detectArea_Y);
		printf("RmtdFrame->detectArea_high = %d\n", Mtd_Frame.detectArea_high);
		printf("RmtdFrame->detectArea_wide = %d\n", Mtd_Frame.detectArea_wide);
		printf("RmtdFrame->priority = %d\n", Mtd_Frame.priority);
		printf("RmtdFrame->alarm_delay = %d\n", Mtd_Frame.alarm_delay);
	#endif
		break;

		case BoresightPos:
			memcpy(&Rboresightmove, RS422->param, sizeof(Rboresightmove));
			pMsg->opticAxisPosX[pMsg->SensorStat] = Rboresightmove.BoresightPos_x;
			pMsg->opticAxisPosY[pMsg->SensorStat] = Rboresightmove.BoresightPos_y;

			pMsg->AxisPosX[pMsg->SensorStat] = pMsg->opticAxisPosX[pMsg->SensorStat];
			pMsg->AxisPosY[pMsg->SensorStat] = pMsg->opticAxisPosY[pMsg->SensorStat];

			pMsg->AvtPosX[pMsg->SensorStat] = pMsg->AxisPosX[pMsg->SensorStat];
			pMsg->AvtPosY[pMsg->SensorStat] = pMsg->AxisPosY[pMsg->SensorStat];
			
			app_ctrl_setBoresightPos(pMsg);
			break;

		case osdbuffer:
			memcpy(&disOsdBuf[imgID1],RS422->param,sizeof(osdbuffer_t));
			setlocale(LC_ALL, "zh_CN.UTF-8");
			memcpy(&disOsdBufbak[imgID1],&disOsdBuf[imgID1],sizeof(osdbuffer_t));
			swprintf(disOsd[imgID1], 33, L"%s", disOsdBuf[imgID1].buf);
			break;
		case ballbaud:
			memcpy((void *)&(CurrentBallConfig.ballAdrress), &(RS422->param[0]), sizeof(CurrentBallConfig.ballAdrress));
			memcpy((void *)&(CurrentBallConfig.ballRate), &(RS422->param[4]), sizeof(CurrentBallConfig.ballRate));
			//printf("\r\nOOOOOOOOOOOOOOOO  Address = %d, \t BaudRate = %d \r\n", CurrentBallConfig.ballAdrress,
			//	CurrentBallConfig.ballRate);			
			break;

			
		case read_shm_osdtext:
			{
				osdtext_t *osdtexttmp = ipc_getosdtextstatus_p();

				for(int i = 0; i < 32; i++)
				{
					disOsdBuf[i].osdID = osdtexttmp->osdID[i];
					disOsdBuf[i].color = osdtexttmp->color[i];
					disOsdBuf[i].alpha = osdtexttmp->alpha[i];
					disOsdBuf[i].ctrl = osdtexttmp->ctrl[i];
					disOsdBuf[i].posx = osdtexttmp->posx[i];
					disOsdBuf[i].posy = osdtexttmp->posy[i];
					memcpy((void *)disOsdBuf[i].buf, (void *)osdtexttmp->buf[i], sizeof(disOsdBuf[i].buf));
					setlocale(LC_ALL, "zh_CN.UTF-8");
					memcpy(&disOsdBufbak[i],&disOsdBuf[i],sizeof(osdbuffer_t));
					swprintf(disOsd[i], 33, L"%s", disOsdBuf[i].buf);
				}
			}
			break;

		case Iris:
			IrisAndFocusAndExit = Enable_Iris;
			memcpy(&cmd_triangle, RS422->param, sizeof(cmd_triangle));
			break;

		case focus:
			IrisAndFocusAndExit = Enable_Focus;
			memcpy(&cmd_triangle, RS422->param, sizeof(cmd_triangle));
			break;

		case exit_IrisAndFocus:
			IrisAndFocusAndExit = Disable;
			memcpy(&cmd_triangle, RS422->param, sizeof(cmd_triangle));
			break;

		case read_shm_config:
			//if(!startEnable)
			{		
				OSDSTATUS *osdtmp = ipc_getosdstatus_p();
				UTCTRKSTATUS *utctmp = ipc_getutstatus_p();
				memcpy(&gConfig_Alg_param,utctmp,sizeof(UTCTRKSTATUS));
				memcpy(&gConfig_Osd_param,osdtmp,sizeof(OSDSTATUS));	
				get_acqRect_from_aim(&gConfig_Osd_param);
				MSGDRIV_send(MSGID_EXT_UPDATE_OSD, 0);
				startEnable = 1;
			}
			break;
			
		case read_shm_osd:
			{
				//printf("read_shm_osd  \n\n");
				OSDSTATUS *osdtmp = ipc_getosdstatus_p();
				memcpy(&gConfig_Osd_param,osdtmp,sizeof(OSDSTATUS));
				get_acqRect_from_aim(&gConfig_Osd_param);	
				
				MSGDRIV_send(MSGID_EXT_UPDATE_OSD, 0);
			}
			break;

		case read_shm_utctrk:
			{
				UTCTRKSTATUS *utctmp = ipc_getutstatus_p();
				memcpy(&gConfig_Alg_param,utctmp,sizeof(UTCTRKSTATUS));	
				MSGDRIV_send(MSGID_EXT_UPDATE_ALG, 0);	
			}
			break;

		case read_shm_camera:
			break;
		case ipclosttime:
			int losttime;
			memcpy(&losttime,RS422->param,4);
			glosttime = losttime;
			break;
	
		case mmt:
			memcpy(&Rmtd,RS422->param,sizeof(Rmtd));
			imgID1 = Rmtd.ImgMtdStat;	
			//printf("recv mmt : imgID1 : %d\n",imgID1);

			if(imgID1 == 0x01)
			{
				pMsg->MmtStat[pMsg->SensorStat] = eImgAlg_Enable;	
			}
			else if(imgID1 == 0x00)
			{
				pMsg->MmtStat[pMsg->SensorStat] = eImgAlg_Disable;
			}
			app_ctrl_setMMT(pMsg);
			MSGAPI_msgsend(mmt);			
			break;

		case mmtselect:
			memcpy(&Rmmtselect,RS422->param,sizeof(Rmmtselect));
			imgID1 = Rmmtselect.ImgMmtSelect;	
			//printf("recv mmtselect : imgID1 : %d\n",imgID1);
			imgID1--;
			if(imgID1 >5)
				imgID1 =5;
			
			app_ctrl_setMmtSelect(pMsg,imgID1);	
			pMsg->MmtStat[pMsg->SensorStat] = eImgAlg_Disable;
			app_ctrl_setMMT(pMsg);
			MSGAPI_msgsend(mmt);
			MSGAPI_msgsend(trk);
			break;
			
		case enh:
			memcpy(&Renh,RS422->param,sizeof(Renh));
			imgID1 = Renh.ImgEnhStat;	
			//printf("recv enh : imgID1 : %d\n",imgID1);
			if(imgID1 == 1){
				pMsg->ImgEnhStat[pMsg->validChId] = ipc_eImgAlg_Enable;
			}
			else if(imgID1 == 0){
				pMsg->ImgEnhStat[pMsg->validChId] = ipc_eImgAlg_Disable;
			}	
			app_ctrl_setEnhance(pMsg);
			MSGAPI_msgsend(enh);
			break;
#if __MOVE_DETECT__
		case mtd:
			memcpy(&Rmtd,RS422->param,sizeof(Rmtd));
			imgID1 = Rmtd.ImgMtdStat;	
			imgID2 = Rmtd.mtdMode;
			//printf("recv mtd : imgID1 : %d  , mode : %d \n",imgID1,imgID2);

			if(imgID1 == 1){
				pMsg->MtdState[pMsg->SensorStat] = ipc_eImgAlg_Enable;
			}
			else if(imgID1 == 0){
				pMsg->MtdState[pMsg->SensorStat] = ipc_eImgAlg_Disable;
			}
			app_ctrl_setMtdStat(pMsg);
			MSGAPI_msgsend(mtd);
			break;

		case mtdSelect:		
			memcpy(&Rmmtselect,RS422->param,sizeof(Rmmtselect));
			pMsg->MtdSelect[pMsg->SensorStat] = Rmmtselect.ImgMmtSelect;
			app_ctrl_setMtdSelect(pMsg);
			if(ipc_eMMT_Select == Rmmtselect.ImgMmtSelect)
			{
				MSGAPI_msgsend(mtd);
			}
			break;
			
#endif
		case sectrk:
			memcpy(&Rsectrk,RS422->param,sizeof(Rsectrk));
			imgID1 = Rsectrk.SecAcqStat;
			if(pMsg->AvtTrkStat != eTrk_mode_acq)
			{
				if(1 == imgID1){
					pMsg->AxisPosX[pMsg->SensorStat] = Rsectrk.ImgPixelX;
					pMsg->AxisPosY[pMsg->SensorStat] = Rsectrk.ImgPixelY;

					int width = 0,height = 0;
					if((pMsg->SensorStat == video_pal)||(pMsg->SensorStat == video_gaoqing0)||(pMsg->SensorStat == video_gaoqing)||(pMsg->SensorStat == video_gaoqing2)||(pMsg->SensorStat == video_gaoqing3)){
						width  = vdisWH[pMsg->SensorStat][0];
						height = vdisWH[pMsg->SensorStat][1];
					}
					if(pMsg->AxisPosX[pMsg->SensorStat] + pMsg->crossAxisWidth[pMsg->SensorStat]/2 > width)
						pMsg->AxisPosX[pMsg->SensorStat] = width - pMsg->crossAxisWidth[pMsg->SensorStat]/2;
					if(pMsg->AxisPosY[pMsg->SensorStat] + pMsg->crossAxisHeight[pMsg->SensorStat]/2 > height)
						pMsg->AxisPosY[pMsg->SensorStat] = height - pMsg->crossAxisHeight[pMsg->SensorStat]/2;

					if(pMsg->AxisPosX[pMsg->SensorStat] <  pMsg->crossAxisWidth[pMsg->SensorStat]/2)
						pMsg->AxisPosX[pMsg->SensorStat] =  pMsg->crossAxisWidth[pMsg->SensorStat]/2;
					if(pMsg->AxisPosY[pMsg->SensorStat]  <  pMsg->crossAxisHeight[pMsg->SensorStat]/2)
						pMsg->AxisPosY[pMsg->SensorStat] =  pMsg->crossAxisHeight[pMsg->SensorStat]/2;
	
					pMsg->AvtTrkStat =eTrk_mode_search;
					app_ctrl_setTrkStat(pMsg);
					app_ctrl_setAxisPos(pMsg);
					MSGAPI_msgsend(sectrk);	
				}
				else if(0 == imgID1){
					pMsg->AvtTrkStat = eTrk_mode_sectrk;
					pMsg->AvtPosX[pMsg->SensorStat] = Rsectrk.ImgPixelX;
					pMsg->AvtPosY[pMsg->SensorStat] = Rsectrk.ImgPixelY;				
					app_ctrl_setTrkStat(pMsg);
					pMsg->AxisPosX[pMsg->SensorStat] = pMsg->opticAxisPosX[pMsg->SensorStat];
					pMsg->AxisPosY[pMsg->SensorStat] = pMsg->opticAxisPosY[pMsg->SensorStat];
					app_ctrl_setAxisPos(pMsg);
					MSGAPI_msgsend(sectrk);						
				}			
			}
			break;

		case sensor:
			memcpy(&Rsensor,RS422->param,sizeof(Rsensor));
			//printf("recv Rsensor: %d\n",Rsensor.SensorStat);
			pMsg->SensorStat = Rsensor.SensorStat;
			app_ctrl_setSensor(pMsg);
			MSGAPI_msgsend(sensor);
			break;
		
		case pinp:	
			memcpy(&Rpinp,RS422->param,sizeof(Rpinp));
			//printf("recv pinp : Rpinp.ImgPicp : %d\n",Rpinp.ImgPicp);
			/*if(Rpinp.ImgPicp == 1)
				pMsg->PicpSensorStat = 0x1;
			else 
				pMsg->PicpSensorStat = 0xff;*/

			pMsg->ImgPicp[pMsg->SensorStat] = Rpinp.ImgPicp;	
			pMsg->PicpSensorStat = Rpinp.PicpSensorStat;
			app_ctrl_setPicp(pMsg);
			break;
					
		case trkdoor:	
			AcqBoxWH aimsize;
			memcpy(&aimsize,RS422->param,sizeof(aimsize));
			pMsg->AimW[pMsg->SensorStat] = aimsize.AimW;
			pMsg->AimH[pMsg->SensorStat]  = aimsize.AimH;
			app_ctrl_setAimSize(pMsg);	

			pMsg->AcqRectW[pMsg->SensorStat] = aimsize.AimW;
			pMsg->AcqRectH[pMsg->SensorStat]  = aimsize.AimH;
			app_ctrl_setAcqRect(pMsg);
			
			MSGAPI_msgsend(trkdoor);
			break;	
			
		case posmove:	
			memcpy(&Rposmove,RS422->param,sizeof(Rposmove));
			//printf("recv posmove : Rposmove.AvtMoveX : %d    Rposmove.AvtMoveY :%d\n",Rposmove.AvtMoveX,Rposmove.AvtMoveY);
			pMsg->aimRectMoveStepX = Rposmove.AvtMoveX;
			pMsg->aimRectMoveStepY = Rposmove.AvtMoveY;
			app_ctrl_setAimPos(pMsg);
			break;	

		case elecZoom:
			memcpy(&Rzoom,RS422->param,sizeof(Rzoom));
			//printf("recv zoom : Rzoom.ImgZoomStat : %d\n",Rzoom.ImgZoomStat);
			pMsg->ImgZoomStat[pMsg->SensorStat] = Rzoom.ImgZoomStat;
			app_ctrl_setZoom(pMsg);
			break;

		case autocheck:
			break;

		case axismove:
			memcpy(&Raxismove,RS422->param,sizeof(Raxismove));
			imgID1 = Raxismove.AvtMoveX;
			imgID2 = Raxismove.AvtMoveY;		
			//printf("recv axismove : Raxismove.AvtMoveX : %d   Raxismove.AvtMoveY : %d \n",Raxismove.AvtMoveX,Raxismove.AvtMoveY);
			if(imgID1 == eTrk_ref_left)
				pMsg->axisMoveStepX = -1;
			else if(imgID1 == eTrk_ref_right)
				pMsg->axisMoveStepX = 1;
			else
				pMsg->axisMoveStepX = 0;
			
			if(imgID2 == eTrk_ref_up)
				pMsg->axisMoveStepY = -1;
			else if(imgID2 == eTrk_ref_down)
				pMsg->axisMoveStepY = 1;
			else
				pMsg->axisMoveStepY= 0;
			
			app_ctrl_setAxisPos(pMsg);
			break;

		case acqBox:
			/*
			AcqBoxWH acqSize;	
			memcpy(&acqSize,RS422->param,sizeof(acqSize));
			pMsg->AcqRectW[pMsg->SensorStat] = acqSize.AimW;
			pMsg->AcqRectH[pMsg->SensorStat]  = acqSize.AimH;
			app_ctrl_setAcqRect(pMsg);
			MSGAPI_msgsend(acqBox);
			*/
			break;
			
		case algosdrect:
			memcpy(&Ralgosdrect,RS422->param,sizeof(Ralgosdrect));
			imgID1 = Ralgosdrect.Imgalgosdrect;
			//printf("algosdrect:%d\n",imgID1);
			if(0 == imgID1)
				pMsg->Imgalgosdrect = 0;
			else if(1 == imgID1)
				pMsg->Imgalgosdrect = 1;
			app_ctrl_setalgosdrect(pMsg);
			break;
		case exit_img:
			MSGAPI_msgsend(exit_img);
			ipc_loop = 0;			
			break;

		case ipcwordFont:
			pMsg->osdTextFont = imgID1 ; 
			app_ctrl_setWordFont(pMsg);		
			break;
		case ipcwordSize:
			pMsg->osdTextSize = imgID1 ; 
			app_ctrl_setWordSize(pMsg);		
			break;
		case ipcresolution:
			memcpy(&Rresolution,RS422->param,sizeof(Rresolution));
			for(int i = 0; i <= ipc_eSen_CH3; i++)
			{
					if((0 == Rresolution.resolution[i])||(1 == Rresolution.resolution[i]))
					{
						vcapWH[i][0] = 1920;
						vcapWH[i][1] = 1080;
						vdisWH[i][0] = 1920;
						vdisWH[i][1] = 1080;
						g_sysParam->getSysParam().gun_camera.raw = vdisWH[i][1];
						g_sysParam->getSysParam().gun_camera.col  = vdisWH[i][0];
					}
					else if((2 == Rresolution.resolution[i])||(3 == Rresolution.resolution[i]))
					{
						vcapWH[i][0] = 1280;
						vcapWH[i][1] = 720;
						vdisWH[i][0] = 1280;
						vdisWH[i][1] = 720;
					}
			}
			if(0 == Rresolution.resolution[ipc_eSen_CH4])
			{
				vcapWH[ipc_eSen_CH4][0] = 720;
				vcapWH[ipc_eSen_CH4][1] = 576;
				vdisWH[ipc_eSen_CH4][0] = 720;
				vdisWH[ipc_eSen_CH4][1] = 576;
			}
			switch(Rresolution.outputresol)
			{
				case 5:
					outputWHF[0] = 1920;
					outputWHF[1] = 1080;
					outputWHF[2] = 60;
					oresoltype = r1920x1080_f60;
					break;
				case 6:
					outputWHF[0] = 1280;
					outputWHF[1] = 1024;
					outputWHF[2] = 60;
					oresoltype = r1280x1024_f60;
					break;
				case 7:
					outputWHF[0] = 1024;
					outputWHF[1] = 768;
					outputWHF[2] = 60;
					oresoltype = r1024x768_f60;
					break;
				default:
					break;
			}
			break;
			
		case querypos:
			{
				memcpy(&posOfLinkage,RS422->param,sizeof(posOfLinkage));
				
				if(proc->get_trig_PTZflag())
				{
					proc->set_trig_PTZflag(0);
					proc->update_app_trig(posOfLinkage.panPos, posOfLinkage.tilPos);
				}
				
				if(proc->getPTZflag()){
					proc->setPTZflag(false);
					pthread_mutex_lock(&event_mutex);
					
					proc->RefreshBallPTZ(posOfLinkage.panPos,posOfLinkage.tilPos,posOfLinkage.zoom);
						
					pthread_cond_signal(&event_cond);
					pthread_mutex_unlock(&event_mutex);				
				}
				if(proc->getGridMapCalibrate()){
					proc->setGridMapCalibrate(false);
				#if 0
					int tmp_row =( proc->m_curNodeIndex ) / (GRID_COLS+1);
					int tmp_col = ( proc->m_curNodeIndex ) % (GRID_COLS+1);
				#else
					int tmp_row =( proc->m_curNodeIndex ) / (GRID_COLS_15+2);
					int tmp_col = ( proc->m_curNodeIndex ) % (GRID_COLS_15+2);
				#endif
					int temp_int = posOfLinkage.tilPos;
					
					proc->m_gridNodes[tmp_row][tmp_col].pano =posOfLinkage.panPos;
					if(temp_int>32768){
						temp_int = 32768 - temp_int;
						proc->m_gridNodes[tmp_row][tmp_col].tilt =temp_int;
					}
					else{
						proc->m_gridNodes[tmp_row][tmp_col].tilt =temp_int;
					}				
					
					proc->m_gridNodes[tmp_row][tmp_col].zoom = posOfLinkage.zoom;
					//printf("\r\n[%s]:Receive PTZ: m_gridNodes[%d][%d].PTZ = <%d, %d, %d>\r\n",__func__,tmp_row,tmp_col,
					//	proc->m_gridNodes[tmp_row][tmp_col].pano,
					//	proc->m_gridNodes[tmp_row][tmp_col].tilt,
					//	proc->m_gridNodes[tmp_row][tmp_col].zoom);
				}
				if(proc->getQueryZoomFlag())
				{
					proc->setQueryZoomFlag(true);
					proc->RefreshBallPTZ(posOfLinkage.panPos,posOfLinkage.tilPos,posOfLinkage.zoom);
				}
				app_ctrl_setLinkagePos(posOfLinkage.panPos, posOfLinkage.tilPos, posOfLinkage.zoom);
				
			}
			break;
		case refreshptz:
			memcpy(&posOfLinkage,RS422->param,sizeof(posOfLinkage));
			proc->RefreshBallPTZ(posOfLinkage.panPos,posOfLinkage.tilPos,posOfLinkage.zoom);
		//printf("\r\n P = %d \r\n T = %d \r\n Z = %d \r\n",posOfLinkage.panPos,posOfLinkage.tilPos, posOfLinkage.zoom);

			break;			
		case switchtarget:
			pMsg->MtdSelect[pMsg->SensorStat] = ipc_eMTD_Next;
			app_ctrl_setMtdSelect(pMsg);
			break;
		case josctrl:
			memcpy(&Rjosctrl,RS422->param,sizeof(Rjosctrl));
			#if 0
			printf("type=%d\ncurx,y(%d,%d)\njos_button:%d\njos_Dir:%d\nmouse_button:%d\nmouse_state:%d\nenter:%d\nmenu:%d\nworkMode:%d\nctrlMode:%d\n\n",
				Rjosctrl.type,Rjosctrl.cursor_x,Rjosctrl.cursor_y,Rjosctrl.jos_button,Rjosctrl.jos_Dir,Rjosctrl.mouse_button,
				Rjosctrl.mouse_state,Rjosctrl.enter,Rjosctrl.menu,Rjosctrl.workMode,Rjosctrl.ctrlMode);
			#endif
			switch(Rjosctrl.type)
			{
				case cursor_move:
				{
					int cond1 = (-1==proc->extMenuCtrl.MenuStat)||(submenu_handleMatchPoints==proc->extMenuCtrl.MenuStat)||(1==proc->extInCtrl->MtdSetRigion)||(MENU_TEST_RESULT_VIEW==g_displayMode);

					if(
						((HANDLE_LINK_MODE == g_AppWorkMode)&&(cond1)) ||
						((ONLY_BALL_MODE == g_AppWorkMode)&&(cond1))
					  )
					{
						proc->set_mouse_show(1);
						proc->dtimer.startTimer(proc->mouse_show_id,3000);
					}
					
					int x = Rjosctrl.cursor_x;
					int y = Rjosctrl.cursor_y;
					proc->draw_mouse_move(x, y);
					
					if(GLUT_DOWN == mouse_state)
						proc->mousemotion_event(x, y);
				}
					break;
				case jos_button:
				{
					int buttonnum = Rjosctrl.jos_button;
					//printf("\r\n[%s]: buttonnum = %d\r\n",__FUNCTION__,buttonnum);
					if((buttonnum >= 0) && (buttonnum <= 9))
						proc->OnKeyDwn(buttonnum + '0');
				}
					break;
				case jos_Dir:
					if(cursor_up == Rjosctrl.jos_Dir)
						proc->OnSpecialKeyDwn(SPECIAL_KEY_UP, 0, 0);
					else if(cursor_down == Rjosctrl.jos_Dir)
						proc->OnSpecialKeyDwn(SPECIAL_KEY_DOWN,0, 0);
					break;
				case mouse_button:
				{
					int param_flag = 0;
					int button = Rjosctrl.mouse_button;
					int state = Rjosctrl.mouse_state;
					int x = Rjosctrl.cursor_x;
					int y = Rjosctrl.cursor_y;
					if(3 == button)
						button = GLUT_LEFT_BUTTON;//0
					else if(4 == button)
						button = GLUT_RIGHT_BUTTON;//2
					else 
						param_flag = 1;
					
					if(1 == state)
						mouse_state = GLUT_DOWN;//0
					else if(0 == state)
						mouse_state = GLUT_UP;//1
					else 
						param_flag = 1;

					if(0 == param_flag)
					{
						proc->mouse_event(button, mouse_state, x, y);
					}
				}
					break;
				case enter:
					proc->OnKeyDwn(13);
					#if 0
						if(g_displayMode == MENU_GRID_MAP_VIEW){
							int menu_stat = Rjosctrl.menu;
							proc->OnJosCtrl(2, menu_stat);
							app_ctrl_setMenu_jos(menu_stat);
							app_ctrl_setMenuStat(mainmenu2);	
						}
					#endif
					break;
				case jos_menu:
				{
					int menu_stat = Rjosctrl.menu;
					proc->OnJosCtrl(2, menu_stat);
				}
					break;
				case workMode:
				{
					int workmode = Rjosctrl.workMode;
					proc->OnJosCtrl(1, workmode);
				}
					break;
				case ctrlMode:
					break;						
				default:
					break;
			}
			break;
		default:
			break;
	}
}

int send_msg(SENDST *RS422)
{
	if(msgextInCtrl == NULL)
		return -1;
		
	unsigned char cmdID = 0;
	unsigned char imgID1 = 0;
	unsigned char imgID2 = 0;
	unsigned char imgID3 = 0;
	unsigned char imgID4 = 0;
	unsigned char imgID5 = 0;	
	CMD_EXT pIStuts;

	memcpy(&pIStuts,msgextInCtrl,sizeof(CMD_EXT));

	if(RS422==NULL)
	{
		return  -1 ;
	}
	cmdID = RS422->cmd_ID;
	switch (cmdID)
	{
		case trk:
			RS422->param[0] = pIStuts.AvtTrkStat;
			//printf("ack trk  :  %d\n",pIStuts.TrkStat);
			break;
			
		case mmt:
			RS422->param[0] = pIStuts.MmtStat[pIStuts.SensorStat];
			//printf("ack mmt  :  %d\n",RS422->param[0]);
			break;
			
		case mmtselect:
			break;
			
		case enh:
			RS422->param[0] = pIStuts.ImgEnhStat[pIStuts.SensorStat];
			//printf("ack enh  :  %d\n",RS422->param[0]);			
			break;
			
		case mtd:
			RS422->param[0] = app_ctrl_getMtdStat();
			//printf("ack mtd  :  %d\n",RS422->param[0]);			
			break;
			
		case sectrk:
			RS422->param[0] = pIStuts.AvtTrkStat;
			//printf("ack sectrk  :  %d\n",RS422->param[0]);						
			break;
			
		case trkdoor:
			break;	
		
		case posmove:
			switch(pIStuts.aimRectMoveStepX)
			{
				case 0:
					RS422->param[0] = 0;
					break;
				case 1:
					RS422->param[0] = 2;
					break;
				case -1:
					RS422->param[0] = 1;
					break;
				default:
					break;		
			}
			switch(pIStuts.aimRectMoveStepY)
			{
				case 0:
					RS422->param[1] = 0;
					break;
				case 1:
					RS422->param[1] = 2;
					break;
				case -1:
					RS422->param[1] = 1;
					break;
				default:
					break;		
			}
			//printf("send ++++++++++ AvtMoveXY = (%02x,%02x)  ++++++++++\n",RS422->param[0],RS422->param[1]);				
			break;
		case sensor:
			RS422->param[0] = pIStuts.SensorStat;	
		case exit_img:					
			break;
		default:
			break;
	}
	return 0;
}

static void * ipc_dataRecv(Void * prm)
{
	SENDST test;
	while(ipc_loop)
	{
		ipc_recvmsg(&test,IPC_TOIMG_MSG);
		recv_msg(&test);			
	}
}


static void * ipc_dataSend(Void * prm)
{
	SENDST test;
	while(ipc_loop)
	{	
		send_msgpth(&test);
		ipc_sendmsg(&test,IPC_FRIMG_MSG);
	}
}

void Ipc_pthread_start(void)
{
	int shm_perm[IPC_MAX];
	shm_perm[IPC_SHA] = shm_rdwr;
	shm_perm[IPC_OSD_SHA] = shm_rdwr;
	shm_perm[IPC_UTCTRK_SHA] = shm_rdonly;	
	shm_perm[IPC_LKOSD_SHA] = shm_rdonly;
	shm_perm[IPC_OSDTEXT_SHA] = shm_rdwr;
	Ipc_init();
	Ipc_create(shm_perm);

	initmessage();
	OSA_thrCreate(&thrHandleDataIn_recv,
                      ipc_dataRecv,
                      DATAIN_TSK_PRI,
                      DATAIN_TSK_STACK_SIZE,
                      NULL);
       OSA_thrCreate(&thrHandleDataIn_send,
                      ipc_dataSend,
                      DATAIN_TSK_PRI,
                      DATAIN_TSK_STACK_SIZE,
                      NULL);
}

void Ipc_pthread_stop(void)
{
	OSA_thrDelete(&thrHandleDataIn_recv);
	OSA_thrDelete(&thrHandleDataIn_send);
}
