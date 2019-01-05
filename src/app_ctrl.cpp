
#include "app_ctrl.h"
#include "osa.h"
#include "msgDriv.h"
#include "configable.h"
#include <iostream>

class CVideoProcess ;

using namespace std;
extern UI_CONNECT_ACTION g_connectAction;
extern bool showDetectCorners;
extern MenuDisplay g_displayMode;
bool show_circle_pointer = false;;
extern GB_WorkMode g_workMode;
extern MenuDisplay g_displayMode;
extern CProcess* plat;
extern SelectMode mouse_workmode;

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

void app_ctrl_setMtdRigion(menu_param_t * pInCmd)
{
	//printf("app_ctrl_setMtdRigion start, button=%d,state=%d,x,y(%d,%d)\n", pInCmd->Mtdmouseclick.button, pInCmd->Mtdmouseclick.state, pInCmd->Mtdmouseclick.x, pInCmd->Mtdmouseclick.y);
	if(msgextInCtrl==NULL)
		return ;
	if(msgextMenuCtrl==NULL)
		return;
	
	CMD_EXT *pIStuts = msgextInCtrl;
	menu_param_t *pMenuStatus = msgextMenuCtrl;
		
	memcpy(&pMenuStatus->Mtdmouseclick, &pInCmd->Mtdmouseclick, sizeof(pMenuStatus->Mtdmouseclick));
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

	if(msgextMenuCtrl==NULL)
		return;
	CMD_EXT *pIStuts = msgextInCtrl;
	menu_param_t *pMenuStatus = msgextMenuCtrl;

	pMenuStatus->MenuStat = index;
	
	memset(pMenuStatus->Passwd, 0, sizeof(pMenuStatus->Passwd));

	MSGDRIV_send(MSGID_EXT_MENUSWITCH, 0);
}

void app_ctrl_setMenu()
{
	if(msgextInCtrl==NULL)
		return;
	if(msgextMenuCtrl==NULL)
		return;
	CMD_EXT *pIStuts = msgextInCtrl;
	menu_param_t *pMenuStatus = msgextMenuCtrl;
	if(-1 == pMenuStatus->MenuStat)
		app_ctrl_setMenuStat(mainmenu0);
	else if(mainmenu0 == pMenuStatus->MenuStat)
		app_ctrl_setMenuStat(-1);
	else if(mainmenu1 == pMenuStatus->MenuStat)
		app_ctrl_setMenuStat(-1);
	else if(mainmenu2 == pMenuStatus->MenuStat)
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
	
	if((mainmenu0 == pMenuStatus->MenuStat) || (mainmenu1 == pMenuStatus->MenuStat))
	{
		int offset = strlen(pMenuStatus->Passwd) * sizeof(char);
		if(offset < sizeof(pMenuStatus->Passwd) - 1)
			sprintf(pMenuStatus->Passwd + offset,"%c", key);
		else
			printf("password reached max length:128");
		
		printf("%s,%d,passwd=%s\n",__FILE__,__LINE__,pMenuStatus->Passwd);
	}
	else if(submenu_setimg == pMenuStatus->MenuStat)
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
	else if((submenu_setmtdrigion == pMenuStatus->MenuStat) && (key == '2'))
	{
		CMD_EXT tmpCmd = {0};
		tmpCmd.MtdSetRigion = 0;
		app_ctrl_setMtdRigionStat(&tmpCmd);
		app_ctrl_setMenuStat(-1);
		g_displayMode = MENU_MAIN_VIEW;
		memset(plat->m_display.disMenu[submenu_setmtdrigion][4], 0, sizeof(plat->m_display.disMenu[submenu_setmtdrigion][4]));
	}
	else if((submenu_mtd == pMenuStatus->MenuStat) && (pMenuStatus->mtdnum_deng == 1))
	{
		int offset = strlen(pMenuStatus->mtdnum_arr) * sizeof(char);
		if(offset < sizeof(pMenuStatus->mtdnum_arr) - 1)
			sprintf(pMenuStatus->mtdnum_arr + offset,"%c", key);
		else
			printf("mtdnum reached max length:128");

		int num = atoi(pMenuStatus->mtdnum_arr);
		pMenuStatus->osd_mudnum = atoi(pMenuStatus->mtdnum_arr);
		printf("%s,%d,osd_mudnum=%d\n",__FILE__,__LINE__,pMenuStatus->osd_mudnum);
		MSGDRIV_send(MSGID_EXT_SETMTDNUM, 0);
	}
	else if((submenu_mtd == pMenuStatus->MenuStat) && (pMenuStatus->trktime_deng == 1))
	{
		int offset = strlen(pMenuStatus->trktime_arr) * sizeof(char);
		if(offset < sizeof(pMenuStatus->trktime_arr) - 1)
			sprintf(pMenuStatus->trktime_arr + offset,"%c", key);
		else
			printf("trktime reached max length:128");

		int num = atoi(pMenuStatus->trktime_arr);
		pMenuStatus->osd_trktime = atoi(pMenuStatus->trktime_arr);
		printf("%s,%d,osd_trktime=%d\n",__FILE__,__LINE__,pMenuStatus->osd_trktime);
		MSGDRIV_send(MSGID_EXT_SETMTDTRKTIME, 0);
	}
	else if((submenu_mtd == pMenuStatus->MenuStat) && (pMenuStatus->maxsize_deng == 1))
	{
		int offset = strlen(pMenuStatus->maxsize_arr) * sizeof(char);
		if(offset < sizeof(pMenuStatus->maxsize_arr) - 1)
			sprintf(pMenuStatus->maxsize_arr + offset,"%c", key);
		else
			printf("maxsize reached max length:128");

		int num = atoi(pMenuStatus->maxsize_arr);
		pMenuStatus->osd_maxsize = atoi(pMenuStatus->maxsize_arr);
		printf("%s,%d,osd_maxsize=%d\n",__FILE__,__LINE__,pMenuStatus->osd_maxsize);
		MSGDRIV_send(MSGID_EXT_SETMTDMAXSIZE, 0);
	}
	else if((submenu_mtd == pMenuStatus->MenuStat) && (pMenuStatus->minsize_deng == 1))
	{
		int offset = strlen(pMenuStatus->minsize_arr) * sizeof(char);
		if(offset < sizeof(pMenuStatus->minsize_arr) - 1)
			sprintf(pMenuStatus->minsize_arr + offset,"%c", key);
		else
			printf("minsize reached max length:128");

		int num = atoi(pMenuStatus->minsize_arr);
		pMenuStatus->osd_minsize = atoi(pMenuStatus->minsize_arr);
		printf("%s,%d,osd_minsize=%d\n",__FILE__,__LINE__,pMenuStatus->osd_minsize);
		MSGDRIV_send(MSGID_EXT_SETMTDMINSIZE, 0);
	}
	else if((submenu_mtd == pMenuStatus->MenuStat) && (pMenuStatus->sensi_deng == 1))
	{
		int offset = strlen(pMenuStatus->sensi_arr) * sizeof(char);
		if(offset < sizeof(pMenuStatus->sensi_arr) - 1)
			sprintf(pMenuStatus->sensi_arr + offset,"%c", key);
		else
			printf("sensi reached max length:128");

		int num = atoi(pMenuStatus->sensi_arr);
		pMenuStatus->osd_sensi = atoi(pMenuStatus->sensi_arr);
		printf("%s,%d,osd_sensi=%d\n",__FILE__,__LINE__,pMenuStatus->osd_sensi);
		MSGDRIV_send(MSGID_EXT_SETMTDSENSI, 0);
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

	if((mainmenu0 == pMenuStatus->MenuStat) || (mainmenu1 == pMenuStatus->MenuStat))
	{
		if(strcmp(init_passwd, pMenuStatus->Passwd))
		{
			app_ctrl_setMenuStat(mainmenu1);
		}
		else
		{
			app_ctrl_setMenuStat(mainmenu2);
		}
			
	}
	else if(mainmenu2 == pMenuStatus->MenuStat)
	{
		if((pMenuStatus->menuarray[mainmenu2].pointer >= 0) && (pMenuStatus->menuarray[mainmenu2].pointer <= 4))
			app_ctrl_setMenuStat(pMenuStatus->menuarray[mainmenu2].pointer + 3);
	}
	else if(submenu_carli == pMenuStatus->MenuStat)
	{
		if(2 == pMenuStatus->menuarray[submenu_carli].pointer) {
			app_ctrl_setMenuStat(mainmenu2);
			g_displayMode = MENU_MAIN_VIEW;
			showDetectCorners = false;
		}
		else if(1 == pMenuStatus->menuarray[submenu_carli].pointer) {
			g_displayMode = MENU_CALIBRA_CAP;
			g_connectAction.CurCalibraCam = CAM_1;
			showDetectCorners = true;
			//cout <<"@@@@@@@@@@@@@@@@@@@@@@@@@@ == 1" << endl;
		}
		else if(0 == pMenuStatus->menuarray[submenu_carli].pointer) {
			g_displayMode = MENU_CALIBRA_CAP;
			g_connectAction.CurCalibraCam = CAM_0;
			showDetectCorners = true;
			//cout <<"@@@@@@@@@@@@@@@@@@@@@@@@@@ == 0" << endl;
		}
		else
		{
		 }
		
	}
	else if(submenu_gunball == pMenuStatus->MenuStat)
	{
		g_displayMode = MENU_SBS;
		CVideoProcess::m_camCalibra->start_cloneVideoSrc = true;
		if(0 == pMenuStatus->menuarray[submenu_gunball].pointer) 
		{
			if(CVideoProcess::m_camCalibra->start_cloneVideoSrc = true){
				OSA_waitMsecs(1500);	
				CVideoProcess::m_camCalibra->bool_Calibrate = true;
			}
		}
		else if (1 == pMenuStatus->menuarray[submenu_gunball].pointer)
		{
			app_ctrl_setMenuStat(submenu_handleMatchPoints);
			//CVideoProcess::m_camCalibra->Set_Handler_Calibra = true ;
		}
		else if(2 == pMenuStatus->menuarray[submenu_gunball].pointer)
		{
			app_ctrl_setMenuStat(mainmenu2);
			CVideoProcess::m_camCalibra->start_cloneVideoSrc = false;
			CVideoProcess::m_camCalibra->Set_Handler_Calibra = false ;
			g_displayMode = MENU_MAIN_VIEW;
		}
	}
	else if(submenu_handleMatchPoints == pMenuStatus->MenuStat)
	{
		if(2 == pMenuStatus->menuarray[submenu_handleMatchPoints].pointer)
		{
			app_ctrl_setMenuStat(submenu_gunball);			
		}

	}
	else if(submenu_mtd == pMenuStatus->MenuStat)
	{
		if(0 == pMenuStatus->menuarray[submenu_mtd].pointer)
		{
			CMD_EXT tmpCmd = {0};
			tmpCmd.MtdSetRigion = 1;
			mouse_workmode = SetMteRigion_Mode;
			app_ctrl_setMtdRigionStat(&tmpCmd);
			g_displayMode = MENU_GUN;
			app_ctrl_setMenuStat(submenu_setmtdrigion);
		}
		else if(1 == pMenuStatus->menuarray[submenu_mtd].pointer)
		{
			pMenuStatus->mtdnum_deng = !pMenuStatus->mtdnum_deng;
			if(pMenuStatus->mtdnum_deng)
				plat->dtimer.startTimer(plat->mtdnum_light_id,500);
			else
			{
				plat->dtimer.stopTimer(plat->mtdnum_light_id);
				MSGDRIV_send(MSGID_EXT_SETMTDNUM, 0);
				if((pMenuStatus->osd_mudnum >= MIN_MTDTARGET_NUM) && (pMenuStatus->osd_mudnum <= MAX_MTDTARGET_NUM))
					plat->detectNum = pMenuStatus->osd_mudnum;
				memset(pMenuStatus->mtdnum_arr, 0, sizeof(pMenuStatus->mtdnum_arr));
			}
		}
		else if(2 == pMenuStatus->menuarray[submenu_mtd].pointer)
		{
			pMenuStatus->trktime_deng = !pMenuStatus->trktime_deng;
			if(pMenuStatus->trktime_deng)
				plat->dtimer.startTimer(plat->trktime_light_id,500);
			else
			{
				plat->dtimer.stopTimer(plat->trktime_light_id);
				MSGDRIV_send(MSGID_EXT_SETMTDTRKTIME, 0);
				if((pMenuStatus->osd_trktime >= MIN_MTDTRKTIME) && (pMenuStatus->osd_trktime <= MAX_MTDTRKTIME))
					plat->m_display.processdurationMenu_osd(pMenuStatus->osd_trktime);
				memset(pMenuStatus->trktime_arr, 0, sizeof(pMenuStatus->trktime_arr));
			}
		}
		else if(3 == pMenuStatus->menuarray[submenu_mtd].pointer)
		{
			pMenuStatus->maxsize_deng = !pMenuStatus->maxsize_deng;
			if(pMenuStatus->maxsize_deng)
				plat->dtimer.startTimer(plat->maxsize_light_id,500);
			else
			{
				plat->dtimer.stopTimer(plat->maxsize_light_id);
				MSGDRIV_send(MSGID_EXT_SETMTDMAXSIZE, 0);
				if((pMenuStatus->osd_maxsize >= plat->minsize) && (pMenuStatus->osd_maxsize <= MAX_MTDMAXSIZE))
					plat->maxsize = pMenuStatus->osd_maxsize;
				memset(pMenuStatus->maxsize_arr, 0, sizeof(pMenuStatus->maxsize_arr));
			}
		}
		else if(4 == pMenuStatus->menuarray[submenu_mtd].pointer)
		{
			pMenuStatus->minsize_deng = !pMenuStatus->minsize_deng;
			if(pMenuStatus->minsize_deng)
				plat->dtimer.startTimer(plat->minsize_light_id,500);
			else
			{
				plat->dtimer.stopTimer(plat->minsize_light_id);
				MSGDRIV_send(MSGID_EXT_SETMTDMINSIZE, 0);
				if((pMenuStatus->osd_minsize >= MIN_MTDMINSIZE) && (pMenuStatus->osd_minsize <= MAX_MTDMAXSIZE))
					plat->minsize = pMenuStatus->osd_minsize;
				memset(pMenuStatus->minsize_arr, 0, sizeof(pMenuStatus->minsize_arr));
			}
		}
		else if(5 == pMenuStatus->menuarray[submenu_mtd].pointer)
		{
			pMenuStatus->sensi_deng = !pMenuStatus->sensi_deng;
			if(pMenuStatus->sensi_deng)
				plat->dtimer.startTimer(plat->sensi_light_id,500);
			else
			{
				plat->dtimer.stopTimer(plat->sensi_light_id);
				MSGDRIV_send(MSGID_EXT_SETMTDSENSI, 0);
				if((pMenuStatus->osd_sensi >= MIN_MTDSENSI) && (pMenuStatus->osd_sensi <= MAX_MTDSENSI))
					plat->sensi = pMenuStatus->osd_sensi;
				memset(pMenuStatus->sensi_arr, 0, sizeof(pMenuStatus->sensi_arr));
			}
		}
		
		else if(6 == pMenuStatus->menuarray[submenu_mtd].pointer)
			app_ctrl_setMenuStat(mainmenu2);
	}
	else if(submenu_setimg == pMenuStatus->MenuStat)
	{
		if(1 == pMenuStatus->menuarray[submenu_setimg].pointer)
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
		else if(2 == pMenuStatus->menuarray[submenu_setimg].pointer)
		{	
			plat->setresol(pMenuStatus->resol_type_tmp);
			plat->save_flag = 1;
			plat->cnt_down = 10;
			plat->dtimer.startTimer(plat->resol_apply_id, 1000);
		}
		else if(3 == pMenuStatus->menuarray[submenu_setimg].pointer)
			app_ctrl_setMenuStat(mainmenu2);
	}
	else if(submenu_setball == pMenuStatus->MenuStat)
	{
		if(0 == pMenuStatus->menuarray[submenu_setball].pointer){
			app_ctrl_setMenuStat(submenu_setcom);
			show_circle_pointer = true;
		}
		else if(1 == pMenuStatus->menuarray[submenu_setball].pointer)
			app_ctrl_setMenuStat(submenu_setnet);
		else if(2 == pMenuStatus->menuarray[submenu_setball].pointer)
			app_ctrl_setMenuStat(mainmenu2);
	}
	else if(submenu_setcom == pMenuStatus->MenuStat)
	{
		if(0 == pMenuStatus->menuarray[submenu_setcom].pointer){
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
		else if(4 == pMenuStatus->menuarray[submenu_setcom].pointer){
			show_circle_pointer = false;
			app_ctrl_setMenuStat(submenu_setball);
			
		}
		
	}
	else if(submenu_setnet == pMenuStatus->MenuStat)
	{
		if(4 == pMenuStatus->menuarray[submenu_setnet].pointer)
			app_ctrl_setMenuStat(submenu_setball);
	}
	else if(submenu_setmtdrigion == pMenuStatus->MenuStat)
	{
		app_ctrl_savemtdrigion();
	}


	printf("\r\n[%s]: pIStuts->MenuStat = %d ",__FUNCTION__, pMenuStatus->MenuStat);
}

void app_ctrl_upMenu()
{
	if(msgextInCtrl==NULL)
		return;
	if(msgextMenuCtrl==NULL)
		return;
		
	CMD_EXT *pIStuts = msgextInCtrl;
	menu_param_t *pMenuStatus = msgextMenuCtrl;
	int menustate = pMenuStatus->MenuStat; 
	if((menustate >= mainmenu2) && (menustate <= submenu_handleMatchPoints))
	{
		if((submenu_mtd == menustate) && (pMenuStatus->mtdnum_deng == 1))
		{
			pMenuStatus->osd_mudnum = (pMenuStatus->osd_mudnum + 1) % MAX_MTDTARGET_NUM;
			MSGDRIV_send(MSGID_EXT_SETMTDNUM, 0);
		}
		else if((submenu_mtd == menustate) && (pMenuStatus->trktime_deng == 1))
		{
			pMenuStatus->osd_trktime = (pMenuStatus->osd_trktime + 1) % MAX_MTDTRKTIME;
			MSGDRIV_send(MSGID_EXT_SETMTDTRKTIME, 0);
		}
		else if((submenu_mtd == menustate) && (pMenuStatus->maxsize_deng == 1))
		{
			pMenuStatus->osd_maxsize = (pMenuStatus->osd_maxsize + 1) % MAX_MTDMAXSIZE;
			MSGDRIV_send(MSGID_EXT_SETMTDMAXSIZE, 0);
		}
		else if((submenu_mtd == menustate) && (pMenuStatus->minsize_deng == 1))
		{
			pMenuStatus->osd_minsize = (pMenuStatus->osd_minsize + 1) % MAX_MTDMAXSIZE;
			MSGDRIV_send(MSGID_EXT_SETMTDMINSIZE, 0);
		}
		else if((submenu_mtd == menustate) && (pMenuStatus->sensi_deng == 1))
		{
			pMenuStatus->osd_sensi = (pMenuStatus->osd_sensi + 1) % MAX_MTDSENSI;
			MSGDRIV_send(MSGID_EXT_SETMTDSENSI, 0);
		}

		else if((submenu_setimg == menustate) && (pMenuStatus->resol_deng == 1))
		{
			pMenuStatus->resol_type_tmp = (pMenuStatus->resol_type_tmp + 1) % maxresolid;
			MSGDRIV_send(MSGID_EXT_SETRESOL, 0);
		}
		else if( (submenu_setcom == menustate) && (pMenuStatus->baud_light == 1) ){

			setComBaud_select = true;
			pMenuStatus->baud_type = (pMenuStatus->baud_type + 1) % MAX_BAUDID;
			MSGDRIV_send(MSGID_EXT_SETBAUD, 0);
		}
		else if(pMenuStatus->menuarray[menustate].pointer > 0)
		{
			pMenuStatus->menuarray[menustate].pointer--;
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

	int menustate = pMenuStatus->MenuStat; 
	if((menustate >= mainmenu2) && (menustate <= submenu_handleMatchPoints))
	{
		if((submenu_mtd == menustate) && (pMenuStatus->mtdnum_deng == 1))
		{
			pMenuStatus->osd_mudnum = (pMenuStatus->osd_mudnum + MAX_MTDTARGET_NUM - 1) % MAX_MTDTARGET_NUM;
			MSGDRIV_send(MSGID_EXT_SETMTDNUM, 0);
		}
		else if((submenu_mtd == menustate) && (pMenuStatus->trktime_deng == 1))
		{
			pMenuStatus->osd_trktime = (pMenuStatus->osd_trktime + MAX_MTDTRKTIME - 1) % MAX_MTDTRKTIME;
			MSGDRIV_send(MSGID_EXT_SETMTDTRKTIME, 0);
		}
		else if((submenu_mtd == menustate) && (pMenuStatus->maxsize_deng == 1))
		{
			pMenuStatus->osd_maxsize = (pMenuStatus->osd_maxsize + MAX_MTDMAXSIZE - 1) % MAX_MTDMAXSIZE;
			MSGDRIV_send(MSGID_EXT_SETMTDMAXSIZE, 0);
		}
		else if((submenu_mtd == menustate) && (pMenuStatus->minsize_deng == 1))
		{
			pMenuStatus->osd_minsize = (pMenuStatus->osd_minsize + MAX_MTDMAXSIZE - 1) % MAX_MTDMAXSIZE;
			MSGDRIV_send(MSGID_EXT_SETMTDMINSIZE, 0);
		}
		else if((submenu_mtd == menustate) && (pMenuStatus->sensi_deng == 1))
		{
			pMenuStatus->osd_sensi = (pMenuStatus->osd_sensi + MAX_MTDSENSI - 1) % MAX_MTDSENSI;
			MSGDRIV_send(MSGID_EXT_SETMTDSENSI, 0);
		}

		else if((submenu_setimg == menustate) && (pMenuStatus->resol_deng == 1))
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
		else if(pMenuStatus->menuarray[menustate].pointer < pMenuStatus->menuarray[menustate].submenu_cnt - 1)
		{
			pMenuStatus->menuarray[menustate].pointer++;
			MSGDRIV_send(MSGID_EXT_DOWNMENU, 0);
		}
	}
}

void app_ctrl_savemtdrigion()
{
	MSGDRIV_send(MSGID_EXT_SMR, 0);
}
