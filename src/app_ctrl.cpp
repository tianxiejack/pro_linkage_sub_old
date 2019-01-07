
#include "app_ctrl.h"
#include "osa.h"
#include "msgDriv.h"
#include "configable.h"
#include <iostream>
//#include <glew.h>
#include <glut.h>
#include "process51.hpp"

class CVideoProcess ;

using namespace std;
extern UI_CONNECT_ACTION g_connectAction;
extern bool showDetectCorners;
extern MenuDisplay g_displayMode;
bool show_circle_pointer = false;;
extern GB_WorkMode g_workMode;
extern MenuDisplay g_displayMode;
extern CProcess* plat;

CMD_EXT *msgextInCtrl;
menu_param_t *msgextMenuCtrl;
#define Coll_Save 0 		//   1:quit coll is to save  cross  or  0:using save funtion to cross axis
#define FrColl_Change 1 	//0:frcoll v1.00 1:frcoll v1.01     // ver1.01 is using 

static int pristatus=0 ;
LinkagePos_t linkagePos ; 
bool setComBaud_select = false ;
bool changeComBaud = false ;

void getMmtTg(unsigned char index,int *x,int *y);
#if __MOVE_DETECT__
void getMtdxy(int *x,int *y,int *w,int *h);
#endif

void app_ctrl_setLinkagePos(int panPos,int tilPos,int zoom)
{
	linkagePos.panPos = panPos ; 
	linkagePos.tilPos = tilPos ;
	linkagePos.zoom = zoom ;
	MSGDRIV_send(MSGID_EXT_SETCURPOS, 0);
}

void  app_ctrl_getSysData(CMD_EXT * exthandle)
{
    OSA_assert(exthandle!=NULL);
    if(msgextInCtrl==NULL)
		return ;
    memcpy(exthandle,msgextInCtrl,sizeof(CMD_EXT));
    return ;
}


void app_ctrl_setTrkStat(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;

	if (pInCmd->AvtTrkStat != pIStuts->AvtTrkStat)
	{
		pIStuts->AvtTrkStat = pInCmd->AvtTrkStat;
		if(pIStuts->AvtTrkStat == eTrk_mode_search)
		{
			return ;
		}	
		else if(pIStuts->AvtTrkStat==eTrk_mode_sectrk || pIStuts->AvtTrkStat == eTrk_mode_acqmove)
		{
			pIStuts->AvtPosX[pIStuts->SensorStat] = pInCmd->AvtPosX[pIStuts->SensorStat];
			pIStuts->AvtPosY[pIStuts->SensorStat] = pInCmd->AvtPosY[pIStuts->SensorStat];
			pIStuts->AimW[pIStuts->SensorStat] = pInCmd->AimW[pIStuts->SensorStat];
			pIStuts->AimH[pIStuts->SensorStat] = pInCmd->AimH[pIStuts->SensorStat];
		}
		
		if(pInCmd->AvtTrkStat == eTrk_mode_acq)
			pIStuts->TrkStat = 0;
		else if(pInCmd->AvtTrkStat == eTrk_mode_target)
			pIStuts->TrkStat = 1;
		MSGDRIV_send(MSGID_EXT_INPUT_TRACK, 0);
	}
	return ;
}



void app_ctrl_setSysmode(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	if(pInCmd->SysMode != pIStuts->SysMode)
		pIStuts->SysMode = pInCmd->SysMode;
	return ;
}

unsigned char app_ctrl_getSysmode()
{
	if(msgextInCtrl==NULL)
		return 255;
	return  msgextInCtrl->SysMode;
}


void app_ctrl_setAimPos(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;

	if (pIStuts->aimRectMoveStepX != pInCmd->aimRectMoveStepX ||pIStuts->aimRectMoveStepY != pInCmd->aimRectMoveStepY)
	{
		pIStuts->aimRectMoveStepX = pInCmd->aimRectMoveStepX;
		pIStuts->aimRectMoveStepY = pInCmd->aimRectMoveStepY;
		MSGDRIV_send(MSGID_EXT_INPUT_AIMPOS, 0);
	}
	else if(pIStuts->AvtPosX[pIStuts->SensorStat]!= pInCmd->AvtPosX[pIStuts->SensorStat] ||
		pIStuts->AvtPosY[pIStuts->SensorStat]!= pInCmd->AvtPosY[pIStuts->SensorStat] )
	{
		pIStuts->AvtPosX[pIStuts->SensorStat] = pInCmd->AvtPosX[pIStuts->SensorStat];
		pIStuts->AvtPosY[pIStuts->SensorStat] = pInCmd->AvtPosY[pIStuts->SensorStat];	
	}
	return ;
}


void app_ctrl_setMmtSelect(CMD_EXT * pIStuts,unsigned char index)
{	
	int curx,cury;
	getMmtTg(index, &curx, &cury);
	
	pIStuts->AvtTrkStat = eTrk_mode_sectrk;
	pIStuts->AvtPosX[pIStuts->SensorStat] = curx;
	pIStuts->AvtPosY[pIStuts->SensorStat] = cury;
	app_ctrl_setTrkStat(pIStuts);

	pIStuts->AxisPosX[pIStuts->SensorStat] = pIStuts->opticAxisPosX[pIStuts->SensorStat];
	pIStuts->AxisPosY[pIStuts->SensorStat] = pIStuts->opticAxisPosY[pIStuts->SensorStat];
	app_ctrl_setAxisPos(pIStuts);
	return ;
}


void app_ctrl_setEnhance(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;

	if(pInCmd->ImgEnhStat[pInCmd->SensorStat] != pIStuts->ImgEnhStat[pInCmd->SensorStat])
	{
		pIStuts->ImgEnhStat[pInCmd->SensorStat] = pInCmd->ImgEnhStat[pInCmd->SensorStat];
		if(pIStuts->ImgEnhStat[pInCmd->SensorStat]==0)
		{
			pIStuts->ImgEnhStat[pInCmd->SensorStat^1]=0;
		}
		MSGDRIV_send(MSGID_EXT_INPUT_ENENHAN, 0);
	}
	return ;
}


void app_ctrl_setAxisPos(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	unsigned char mask = 0;
	if(pInCmd->axisMoveStepX != 0  || pInCmd->axisMoveStepY != 0)
	{	
		pIStuts->axisMoveStepX = pInCmd->axisMoveStepX;
		pIStuts->axisMoveStepY = pInCmd->axisMoveStepY;
		mask++;
	}

	if(pIStuts->AxisPosX[pIStuts->SensorStat] != pInCmd->AxisPosX[pIStuts->SensorStat] || pIStuts->AxisPosY[pIStuts->SensorStat]!= pInCmd->AxisPosY[pIStuts->SensorStat])
	{
		pIStuts->AxisPosX[pIStuts->SensorStat] = pInCmd->AxisPosX[pIStuts->SensorStat];
		pIStuts->AxisPosY[pIStuts->SensorStat] = pInCmd->AxisPosY[pIStuts->SensorStat];
		mask++;
	}
	if(mask)
		MSGDRIV_send(MSGID_EXT_INPUT_AXISPOS, 0);	
	
	return ;
}



void app_ctrl_setBoresightPos(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;

	if(pIStuts->opticAxisPosX[pIStuts->SensorStat] != pInCmd->opticAxisPosX[pIStuts->SensorStat] 
		|| pIStuts->opticAxisPosY[pIStuts->SensorStat]!= pInCmd->opticAxisPosY[pIStuts->SensorStat])
	{
		pIStuts->opticAxisPosX[pIStuts->SensorStat] = pInCmd->opticAxisPosX[pIStuts->SensorStat];
		pIStuts->opticAxisPosY[pIStuts->SensorStat] = pInCmd->opticAxisPosY[pIStuts->SensorStat];
	}

	if(pIStuts->AxisPosX[pIStuts->SensorStat] != pInCmd->AxisPosX[pIStuts->SensorStat] 
		|| pIStuts->AxisPosY[pIStuts->SensorStat]!= pInCmd->AxisPosY[pIStuts->SensorStat])
	{
		pIStuts->AxisPosX[pIStuts->SensorStat] = pInCmd->AxisPosX[pIStuts->SensorStat];
		pIStuts->AxisPosY[pIStuts->SensorStat] = pInCmd->AxisPosY[pIStuts->SensorStat];
	}

	if(pIStuts->AvtPosX[pIStuts->SensorStat] != pInCmd->AvtPosX[pIStuts->SensorStat] 
		|| pIStuts->AvtPosY[pIStuts->SensorStat]!= pInCmd->AvtPosY[pIStuts->SensorStat])
	{
		pIStuts->AvtPosX[pIStuts->SensorStat] = pInCmd->AvtPosX[pIStuts->SensorStat];
		pIStuts->AvtPosY[pIStuts->SensorStat] = pInCmd->AvtPosY[pIStuts->SensorStat];
	}


	//printf("pIStuts->opticAxisPosX[%d] = %d \n",pIStuts->SensorStat,pIStuts->opticAxisPosX[pIStuts->SensorStat]);
	//printf("pIStuts->opticAxisPosY[%d] = %d \n",pIStuts->SensorStat,pIStuts->opticAxisPosY[pIStuts->SensorStat]);

	//printf("pIStuts->AxisPosX[%d] = %d \n",pIStuts->SensorStat,pIStuts->AxisPosX[pIStuts->SensorStat]);
	//printf("pIStuts->AvtPosX[%d] = %d \n",pIStuts->SensorStat,pIStuts->AvtPosX[pIStuts->SensorStat]);

	//printf("address  pIStuts->AvtPosX = %x \n",pIStuts->AvtPosX);

	return ;
}


#if __MOVE_DETECT__
void app_ctrl_setMtdStat(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	if(pIStuts->MtdState[pIStuts->SensorStat] != pInCmd->MtdState[pIStuts->SensorStat])
	{
		pIStuts->MtdState[pIStuts->SensorStat] = pInCmd->MtdState[pIStuts->SensorStat];
		MSGDRIV_send(MSGID_EXT_MVDETECT, 0);
	}
	return ;
}

void app_ctrl_setMtdSelect(CMD_EXT * pInCmd)
{
	CMD_EXT pMsg;
	memset(&pMsg,0,sizeof(CMD_EXT));
	
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	
	if(2 == pInCmd->MtdSelect[pInCmd->SensorStat] || 1 == pInCmd->MtdSelect[pInCmd->SensorStat])
	{
		if(eImgAlg_Enable == pIStuts->MtdState[pIStuts->SensorStat])
		{
			pIStuts->MtdSelect[pIStuts->SensorStat] = pInCmd->MtdSelect[pIStuts->SensorStat];
			MSGDRIV_send(MSGID_EXT_MVDETECTSELECT, 0);
		}
	}
	else if(3 == pInCmd->MtdSelect[pInCmd->SensorStat])
	{
		int curx,cury,curw,curh;
		getMtdxy(&curx, &cury, &curw, &curh);
				
		pMsg.AvtTrkStat =eTrk_mode_sectrk;
		pMsg.AvtPosX[pIStuts->SensorStat]  = curx;
		pMsg.AvtPosY[pIStuts->SensorStat]  = cury;
		pMsg.AimW[pIStuts->SensorStat]  = curw;
		pMsg.AimH[pIStuts->SensorStat]  = curh;
		app_ctrl_setTrkStat(&pMsg);//track

		pMsg.MtdState[pMsg.SensorStat] = eImgAlg_Disable;
		app_ctrl_setMtdStat(&pMsg);//close
	}
	return ;
}

void app_ctrl_setMtdRigionStat(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;

	CMD_EXT *pIStuts = msgextInCtrl;
	if (pIStuts->MtdSetRigion != pInCmd->MtdSetRigion)
	{
		pIStuts->MtdSetRigion = pInCmd->MtdSetRigion;
		MSGDRIV_send(MSGID_EXT_MVDETECT_SETRIGIONSTAT, 0);
	}
	return;
}

void app_ctrl_setMtdRigion(CMD_EXT * pInCmd)
{
	//printf("app_ctrl_setMtdRigion start, button=%d,state=%d,x,y(%d,%d)\n", pInCmd->Mtdmouseclick.button, pInCmd->Mtdmouseclick.state, pInCmd->Mtdmouseclick.x, pInCmd->Mtdmouseclick.y);
	if(msgextInCtrl==NULL)
		return ;

	CMD_EXT *pIStuts = msgextInCtrl;
	memcpy(&pIStuts->Mtdmouseclick, &pInCmd->Mtdmouseclick, sizeof(pIStuts->Mtdmouseclick));
	if (pIStuts->MtdSetRigion)
	{
		MSGDRIV_send(MSGID_EXT_MVDETECT_SETRIGION, 0);
	}
	return;
}

#endif

unsigned char app_ctrl_getMtdStat()
{
	if(msgextInCtrl==NULL)
		return 0xff;
	CMD_EXT *pIStuts = msgextInCtrl;

	return pIStuts->MtdState[pIStuts->validChId];
}


void app_ctrl_setMMT(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;

	if(pInCmd->MMTTempStat != pIStuts->MMTTempStat)
		pIStuts->MMTTempStat = pInCmd->MMTTempStat;

	if(pInCmd->AvtTrkStat != eTrk_mode_target)
	{
		if (pIStuts->MmtStat[pIStuts->SensorStat] != pInCmd->MmtStat[pIStuts->SensorStat])
		{     
			pIStuts->MmtStat[pIStuts->SensorStat] = pInCmd->MmtStat[pIStuts->SensorStat];
			{
			    MSGDRIV_send(MSGID_EXT_INPUT_ENMTD, 0);
			}
		}
	}
	return ;
}

void app_ctrl_Sensorchange(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	pIStuts->AxisPosX[pIStuts->SensorStat]=pIStuts->opticAxisPosX[pIStuts->SensorStat];
	pIStuts->AxisPosY[pIStuts->SensorStat]=pIStuts->opticAxisPosY[pIStuts->SensorStat];
}

void app_ctrl_setReset(CMD_EXT * pInCmd)
{

	if(msgextInCtrl==NULL)
		return ;
    CMD_EXT *pIStuts = msgextInCtrl;
	if(pIStuts->AvtTrkStat != eTrk_mode_acq){
		pIStuts->AvtTrkStat = eTrk_mode_acq;
		 MSGDRIV_send(MSGID_EXT_INPUT_TRACK, 0);
	}
	if(pIStuts->MmtStat[pIStuts->SensorStat] != eImgAlg_Disable){
		pIStuts->MmtStat[pIStuts->SensorStat] = eImgAlg_Disable;
		MSGDRIV_send(MSGID_EXT_INPUT_ENMTD, 0);
	}
	if(pIStuts->ImgEnhStat[pIStuts->SensorStat] == 0x01){
		pIStuts->ImgEnhStat[pIStuts->SensorStat] = 0x00;
		MSGDRIV_send(MSGID_EXT_INPUT_ENENHAN, 0);
	}
}

void app_ctrl_setSensor(CMD_EXT * pInCmd)
{
	
		return ;

	if(msgextInCtrl==NULL)
		return ;

	CMD_EXT *pIStuts = msgextInCtrl;
	if (pIStuts->SensorStat != pInCmd->SensorStat)
	{
		pIStuts->changeSensorFlag = 1;
		pIStuts->SensorStatpri = pIStuts->SensorStat;
		pIStuts->SensorStat = pInCmd->SensorStat;
		//app_ctrl_Sensorchange(pInCmd);
		MSGDRIV_send(MSGID_EXT_INPUT_SENSOR, 0);
	}
	return ;
}


void app_ctrl_setZoom(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	   
	if(pIStuts->ImgZoomStat[0] != pInCmd->ImgZoomStat[0])
	{
		pIStuts->ImgZoomStat[0] = pInCmd->ImgZoomStat[0];
		MSGDRIV_send(MSGID_EXT_INPUT_ENZOOM, 0);
	}
	return ;
}


void app_ctrl_setAimSize(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	bool enable = 0;
	if (pIStuts->AimH[pIStuts->validChId] != pInCmd->AimH[pInCmd->validChId])
	{
		pIStuts->AimH[pIStuts->validChId] = pInCmd->AimH[pInCmd->validChId];
		enable=1;
	}
	if (pIStuts->AimW[pIStuts->validChId] != pInCmd->AimW[pInCmd->validChId])
	{
		pIStuts->AimW[pIStuts->validChId] = pInCmd->AimW[pInCmd->validChId];
		enable=1;
	}
	if(enable)
		MSGDRIV_send(MSGID_EXT_INPUT_AIMSIZE, 0);
	return ;
}


void app_ctrl_setSerTrk(CMD_EXT * pInCmd )
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;

	if((pInCmd->AvtPosX[pIStuts->SensorStat] != pIStuts->AvtPosX[pIStuts->SensorStat])
		|| (pInCmd->AvtPosY[pIStuts->SensorStat] != pIStuts->AvtPosY[pIStuts->SensorStat]))
	{
		pIStuts->AvtPosX[pIStuts->SensorStat] = pInCmd->AvtPosX[pIStuts->SensorStat];
		pIStuts->AvtPosY[pIStuts->SensorStat] = pInCmd->AvtPosY[pIStuts->SensorStat];
	}	
	return ;	
}


void app_ctrl_setDispGrade(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;

    if (pIStuts->DispGrp[0] != pInCmd->DispGrp[0] 
                        || pIStuts->DispGrp[1] != pInCmd->DispGrp[1])
    {
        pIStuts->DispGrp[0] = pInCmd->DispGrp[0];
        pIStuts->DispGrp[1] = pInCmd->DispGrp[1];
    }
	return ;
}


void app_ctrl_setDispColor(CMD_EXT * pInCmd )
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	  
	if(pInCmd->DispColor[0] !=0x07)
	{
		if (pIStuts->DispColor[0] != pInCmd->DispColor[0] 
	                    || pIStuts->DispColor[1] != pInCmd->DispColor[1])
		{
		    pIStuts->DispColor[0] = pInCmd->DispColor[0];
		    pIStuts->DispColor[1] = pInCmd->DispColor[1]; 
		}
	}
   return ;
}


void app_ctrl_setPicp(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	bool enable = false;
	if(pIStuts->PicpSensorStat != pInCmd->PicpSensorStat)
	{
		pIStuts->PicpSensorStat = pInCmd->PicpSensorStat;
		enable = true;
	}
	if(pIStuts->ImgPicp[pIStuts->SensorStat] != pInCmd->ImgPicp[pIStuts->SensorStat])
	{
		pIStuts->ImgPicp[pIStuts->SensorStat] = pInCmd->ImgPicp[pIStuts->SensorStat];
		if(pIStuts->ImgPicp[pIStuts->SensorStat]==0)
			pIStuts->PicpSensorStat = 255;
		
		enable = true;
	}
	if(enable)
		MSGDRIV_send(MSGID_EXT_INPUT_PICPCROP, 0);
	return ;
}


void app_ctrl_setAcqRect(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	if(pIStuts->AcqRectW[pIStuts->validChId] != pInCmd->AcqRectW[pInCmd->validChId])
	{
		pIStuts->AcqRectW[pIStuts->validChId] = pInCmd->AcqRectW[pInCmd->validChId];
	}
	if(pIStuts->AcqRectH[pIStuts->validChId] != pInCmd->AcqRectH[pInCmd->validChId])
	{
		pIStuts->AcqRectH[pIStuts->validChId] = pInCmd->AcqRectH[pInCmd->validChId];
	}
}

void app_ctrl_setalgosdrect(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	if(pIStuts->Imgalgosdrect != pInCmd->Imgalgosdrect)
	{
		pIStuts->Imgalgosdrect = pInCmd->Imgalgosdrect;
		MSGDRIV_send(MSGID_EXT_INPUT_ALGOSDRECT, 0);
	}

}

void app_ctrl_setWordFont(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	if(pIStuts->osdTextFont != pInCmd->osdTextFont)
	{
		pIStuts->osdTextFont = pInCmd->osdTextFont;
	}
}

void app_ctrl_setWordSize(CMD_EXT * pInCmd)
{
	if(msgextInCtrl==NULL)
		return ;
	CMD_EXT *pIStuts = msgextInCtrl;
	if(pIStuts->osdTextSize != pInCmd->osdTextSize)
	{
		pIStuts->osdTextSize = pInCmd->osdTextSize;
	}
}

void app_ctrl_setMenuStat(int index)
{
	if(msgextInCtrl==NULL)
		return;
	CMD_EXT *pIStuts = msgextInCtrl;

	pIStuts->MenuStat = index;
	
	memset(pIStuts->Passwd, 0, sizeof(pIStuts->Passwd));

	MSGDRIV_send(MSGID_EXT_MENUSWITCH, 0);
}

void app_ctrl_setMenu()
{
	if(msgextInCtrl==NULL)
		return;
	CMD_EXT *pIStuts = msgextInCtrl;
	
	if(-1 == pIStuts->MenuStat)
		app_ctrl_setMenuStat(mainmenu0);
	else if(mainmenu0 == pIStuts->MenuStat)
		app_ctrl_setMenuStat(-1);
	else if(mainmenu1 == pIStuts->MenuStat)
		app_ctrl_setMenuStat(-1);
	else if(mainmenu2 == pIStuts->MenuStat)
		app_ctrl_setMenuStat(-1);
}

void app_ctrl_setnumber(char key)
{
	if(msgextInCtrl==NULL)
		return;
	if(msgextMenuCtrl==NULL)
		return;
	CMD_EXT *pIStuts = msgextInCtrl;
	menu_param_t *pMenuStatus = msgextMenuCtrl;
	
	if((mainmenu0 == pIStuts->MenuStat) || (mainmenu1 == pIStuts->MenuStat))
	{
		int offset = strlen(pIStuts->Passwd) * sizeof(char);
		if(offset < sizeof(pIStuts->Passwd) - 1)
			sprintf(pIStuts->Passwd + offset,"%c", key);
		else
			printf("password reached max length:128");
		
		printf("%s,%d,passwd=%s\n",__FILE__,__LINE__,pIStuts->Passwd);
	}
	else if(submenu_setimg == pIStuts->MenuStat)
	{
		if(key == '0')
		{
			if(plat->save_flag)
			{
				plat->dtimer.stopTimer(plat->resol_apply_id);
				pMenuStatus->resol_type_tmp = pMenuStatus->resol_type;
				plat->setresol(pMenuStatus->resol_type);
				plat->save_flag = 0;
				memset(plat->m_display.disMenu[submenu_setimg][4], 0, sizeof(plat->m_display.disMenu[submenu_setimg][4]));
				memset(plat->m_display.disMenu[submenu_setimg][5], 0, sizeof(plat->m_display.disMenu[submenu_setimg][5]));
				MSGDRIV_send(MSGID_EXT_SETRESOL, 0);
			}
		}
		else if(key == '1')
		{
			if(plat->save_flag)
			{
				plat->dtimer.stopTimer(plat->resol_apply_id);
				pMenuStatus->resol_type = pMenuStatus->resol_type_tmp;
				plat->save_flag = 0;
				memset(plat->m_display.disMenu[submenu_setimg][4], 0, sizeof(plat->m_display.disMenu[submenu_setimg][4]));
				memset(plat->m_display.disMenu[submenu_setimg][5], 0, sizeof(plat->m_display.disMenu[submenu_setimg][5]));
				MSGDRIV_send(MSGID_EXT_SAVERESOL, 0);
			}

		}
	}
}

void app_ctrl_enter()
{
	char *init_passwd = "000000";
	if(msgextInCtrl==NULL)
		return;

	if(msgextMenuCtrl==NULL)
		return;
	CMD_EXT *pIStuts = msgextInCtrl;
	menu_param_t *pMenuStatus = msgextMenuCtrl;

	if((mainmenu0 == pIStuts->MenuStat) || (mainmenu1 == pIStuts->MenuStat))
	{
		if(strcmp(init_passwd, pIStuts->Passwd))
		{
			app_ctrl_setMenuStat(mainmenu1);
		}
		else
		{
			app_ctrl_setMenuStat(mainmenu2);
		}
			
	}
	else if(mainmenu2 == pIStuts->MenuStat)
	{
		if((pIStuts->menuarray[mainmenu2].pointer >= 0) && (pIStuts->menuarray[mainmenu2].pointer <= 4))
			app_ctrl_setMenuStat(pIStuts->menuarray[mainmenu2].pointer + 3);
	}
	else if(submenu_carli == pIStuts->MenuStat)
	{
		if(2 == pIStuts->menuarray[submenu_carli].pointer) {
			app_ctrl_setMenuStat(mainmenu2);
			g_displayMode = MENU_MAIN_VIEW;
			showDetectCorners = false;
		}
		else if(1 == pIStuts->menuarray[submenu_carli].pointer) {
			g_displayMode = MENU_CALIBRA_CAP;
			g_connectAction.CurCalibraCam = CAM_1;
			//showDetectCorners = true;
			//cout <<"@@@@@@@@@@@@@@@@@@@@@@@@@@ == 1" << endl;
		}
		else if(0 == pIStuts->menuarray[submenu_carli].pointer) {
			g_displayMode = MENU_CALIBRA_CAP;
			g_connectAction.CurCalibraCam = CAM_0;
			//showDetectCorners = true;
			//cout <<"@@@@@@@@@@@@@@@@@@@@@@@@@@ == 0" << endl;
		}
		else
		{
		 }
		
	}
	else if(submenu_gunball == pIStuts->MenuStat)
	{
		g_displayMode = MENU_SBS;
		CVideoProcess::m_camCalibra->start_cloneVideoSrc = true;
		if(0 == pIStuts->menuarray[submenu_gunball].pointer) 
		{
			if(CVideoProcess::m_camCalibra->start_cloneVideoSrc = true){
				OSA_waitMsecs(1500);	
				CVideoProcess::m_camCalibra->bool_Calibrate = true;
				g_displayMode = MENU_MATCH_POINT_VIEW;
			}
		}
		else if (1 == pIStuts->menuarray[submenu_gunball].pointer)
		{
			CVideoProcess::m_camCalibra->start_cloneVideoSrc = false;
			g_displayMode = MENU_SBS;
			app_ctrl_setMenuStat(submenu_handleMatchPoints);
			//CVideoProcess::m_camCalibra->Set_Handler_Calibra = true ;
		}
		else if(2 == pIStuts->menuarray[submenu_gunball].pointer)
		{
			CVideoProcess::m_camCalibra->start_cloneVideoSrc = false;
			CVideoProcess::m_camCalibra->Set_Handler_Calibra = false ;
			app_ctrl_setMenuStat(mainmenu2);			
			g_displayMode = MENU_MAIN_VIEW;
		}
	}
	else if(submenu_handleMatchPoints == pIStuts->MenuStat)
	{
		if(0 == pIStuts->menuarray[submenu_handleMatchPoints].pointer){
			CVideoProcess::m_camCalibra->Set_Handler_Calibra = true ;
			plat->open_handleCalibra = true;
		}
		else if( 1 == pIStuts->menuarray[submenu_handleMatchPoints].pointer){
			plat->open_handleCalibra = false;
			CVideoProcess::m_camCalibra->start_cloneVideoSrc = true;
			OSA_waitMsecs(1500);	
			CVideoProcess::m_camCalibra->bool_Calibrate = true;
			g_displayMode = MENU_MATCH_POINT_VIEW;
		}
		else if(2 == pIStuts->menuarray[submenu_handleMatchPoints].pointer)
		{
			CVideoProcess::m_camCalibra->start_cloneVideoSrc = false;
			CVideoProcess::m_camCalibra->Set_Handler_Calibra = false ;
			app_ctrl_setMenuStat(submenu_gunball);
			g_displayMode = MENU_MAIN_VIEW;
			
		}

	}
	else if(submenu_mtd == pIStuts->MenuStat)
	{
		if(0 == pIStuts->menuarray[submenu_mtd].pointer)
		{
			CMD_EXT tmpCmd = {0};
			tmpCmd.MtdSetRigion = 1;
			app_ctrl_setMtdRigionStat(&tmpCmd);
			g_displayMode = MENU_GUN;
			app_ctrl_setMenuStat(submenu_setmtdrigion);
		}
		
		else if(6 == pIStuts->menuarray[submenu_mtd].pointer)
			app_ctrl_setMenuStat(mainmenu2);
	}
	else if(submenu_setimg == pIStuts->MenuStat)
	{
		if(1 == pIStuts->menuarray[submenu_setimg].pointer)
		{
			pMenuStatus->resol_deng = !pMenuStatus->resol_deng;
			if(pMenuStatus->resol_deng)
				plat->dtimer.startTimer(plat->resol_light_id,500);
			else
			{
				plat->dtimer.stopTimer(plat->resol_light_id);
				MSGDRIV_send(MSGID_EXT_SETRESOL, 0);
			}
		}
		else if(2 == pIStuts->menuarray[submenu_setimg].pointer)
		{	
			plat->setresol(pMenuStatus->resol_type_tmp);
			plat->save_flag = 1;
			plat->cnt_down = 10;
			plat->dtimer.startTimer(plat->resol_apply_id, 1000);
		}
		else if(3 == pIStuts->menuarray[submenu_setimg].pointer)
			app_ctrl_setMenuStat(mainmenu2);
	}
	else if(submenu_setball == pIStuts->MenuStat)
	{
		if(0 == pIStuts->menuarray[submenu_setball].pointer){
			app_ctrl_setMenuStat(submenu_setcom);
			show_circle_pointer = true;
		}
		else if(1 == pIStuts->menuarray[submenu_setball].pointer)
			app_ctrl_setMenuStat(submenu_setnet);
		else if(2 == pIStuts->menuarray[submenu_setball].pointer)
			app_ctrl_setMenuStat(mainmenu2);
	}
	else if(submenu_setcom == pIStuts->MenuStat)
	{
		if(0 == pIStuts->menuarray[submenu_setcom].pointer){
				pMenuStatus->baud_light= !pMenuStatus->baud_light;

				if(pMenuStatus->baud_light){
					plat->dtimer.startTimer(plat->baud_light_id,500);
				}
				else
				{
					plat->dtimer.stopTimer(plat->baud_light_id);
					MSGDRIV_send(MSGID_EXT_SETBAUD, 0);
				}

				
				if(setComBaud_select == true) {
					setComBaud_select = false;
					changeComBaud = true;
					MSGDRIV_send(MSGID_EXT_SETBAUD, 0);
				}
		}
		else if(4 == pIStuts->menuarray[submenu_setcom].pointer){
			show_circle_pointer = false;
			app_ctrl_setMenuStat(submenu_setball);
			
		}
		
	}
	else if(submenu_setnet == pIStuts->MenuStat)
	{
		if(4 == pIStuts->menuarray[submenu_setnet].pointer)
			app_ctrl_setMenuStat(submenu_setball);
	}
	else if(submenu_setmtdrigion == pIStuts->MenuStat)
	{
		app_ctrl_savemtdrigion();
	}


	printf("\r\n[%s]: pIStuts->MenuStat = %d ",__FUNCTION__, pIStuts->MenuStat);
}

void app_ctrl_upMenu()
{
	if(msgextInCtrl==NULL)
		return;
	if(msgextMenuCtrl==NULL)
		return;
		
	CMD_EXT *pIStuts = msgextInCtrl;
	menu_param_t *pMenuStatus = msgextMenuCtrl;
	int menustate = pIStuts->MenuStat; 
	if((menustate >= mainmenu2) && (menustate <= submenu_handleMatchPoints))
	{
		if((submenu_setimg == menustate) && (pMenuStatus->resol_deng == 1))
		{
			pMenuStatus->resol_type_tmp = (pMenuStatus->resol_type_tmp + 1) % maxresolid;
			MSGDRIV_send(MSGID_EXT_SETRESOL, 0);
		}
		else if( (submenu_setcom == menustate) && (pMenuStatus->baud_light == 1) ){

			setComBaud_select = true;
			pMenuStatus->baud_type = (pMenuStatus->baud_type + 1) % MAX_BAUDID;
			MSGDRIV_send(MSGID_EXT_SETBAUD, 0);
		}
		else if(pIStuts->menuarray[menustate].pointer > 0)
		{
			pIStuts->menuarray[menustate].pointer--;
			MSGDRIV_send(MSGID_EXT_UPMENU, 0);
		}
	}
}

void app_ctrl_downMenu()
{
	if(msgextInCtrl==NULL)
		return;
	if(msgextMenuCtrl==NULL)
		return;
	CMD_EXT *pIStuts = msgextInCtrl;
	menu_param_t *pMenuStatus = msgextMenuCtrl;

	int menustate = pIStuts->MenuStat; 
	if((menustate >= mainmenu2) && (menustate <= submenu_handleMatchPoints))
	{
		if((submenu_setimg == menustate) && (pMenuStatus->resol_deng == 1))
		{
			pMenuStatus->resol_type_tmp = (pMenuStatus->resol_type_tmp + maxresolid - 1) % maxresolid;
			MSGDRIV_send(MSGID_EXT_SETRESOL, 0);
		}
		else if((submenu_setcom == menustate) && (pMenuStatus->baud_light == 1))
		{
			setComBaud_select = true;
			pMenuStatus->baud_type = (pMenuStatus->baud_type + MAX_BAUDID - 1) % MAX_BAUDID;
			MSGDRIV_send(MSGID_EXT_SETBAUD, 0);
		}
		else if(pIStuts->menuarray[menustate].pointer < pIStuts->menuarray[menustate].submenu_cnt - 1)
		{
			pIStuts->menuarray[menustate].pointer++;
			MSGDRIV_send(MSGID_EXT_DOWNMENU, 0);
		}
	}
}

void app_ctrl_savemtdrigion()
{
	MSGDRIV_send(MSGID_EXT_SMR, 0);
}
