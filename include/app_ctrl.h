
#ifndef APP_CTRL_H_
#define APP_CTRL_H_

#include "app_status.h"
#include "process51.hpp"
extern CMD_EXT *msgextInCtrl;

void app_ctrl_setReset(CMD_EXT * pInCmd);
void app_ctrl_setSensor(CMD_EXT * pInCmd);
void app_ctrl_setTrkStat(CMD_EXT * pInCmd);
void app_ctrl_setMMT(CMD_EXT * pInCmd);
void app_ctrl_setAimPos(CMD_EXT * pInCmd);
void app_ctrl_setZoom(CMD_EXT * pInCmd);
void app_ctrl_setAimSize(CMD_EXT * pInCmd);
void app_ctrl_setSerTrk(CMD_EXT * pInCmd );
void app_ctrl_setSysmode(CMD_EXT * pInCmd);
unsigned char app_ctrl_getSysmode();
void app_ctrl_setDispGrade(CMD_EXT * pInCmd);
void app_ctrl_setDispColor(CMD_EXT * pInCmd );
void app_ctrl_setAxisPos(CMD_EXT * pInCmd);
void app_ctrl_setEnhance(CMD_EXT * pInCmd);
void app_ctrl_setPicp(CMD_EXT * pInCmd);
void app_ctrl_setMtdStat(CMD_EXT * pInCmd);
unsigned char app_ctrl_getMtdStat();
void  app_ctrl_getSysData(CMD_EXT * exthandle);
void app_ctrl_setMmtSelect(CMD_EXT * pIStuts,unsigned char index);
void app_ctrl_setMtdStat(CMD_EXT * pInCmd);
void app_ctrl_setMtdSelect(CMD_EXT * pInCmd);
void app_ctrl_setMtdRigionStat(CMD_EXT * pInCmd);
void app_ctrl_setMtdRigion(menu_param_t * pInCmd);
void app_ctrl_setAcqRect(CMD_EXT * pInCmd);
void app_ctrl_setBoresightPos(CMD_EXT * pInCmd);
void app_ctrl_setalgosdrect(CMD_EXT * pInCmd);
void app_ctrl_setWordFont(CMD_EXT * pInCmd);
void app_ctrl_setWordSize(CMD_EXT * pInCmd);
void app_ctrl_setLinkagePos(int panPos,int tilPos,int zoom);
void app_ctrl_setMenuStat(int index);
void app_ctrl_setMenu();
void app_ctrl_setnumber(char key);
void app_ctrl_enter();
void app_ctrl_upMenu();
void app_ctrl_downMenu();
void app_ctrl_savemtdrigion();

#endif /* APP_CTRL_H_ */
