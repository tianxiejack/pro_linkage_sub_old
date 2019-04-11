
#ifndef PROCESS021_HPP_
#define PROCESS021_HPP_

#include "VideoProcess.hpp"
#include "osd_cv.h"

#include "osa_sem.h"
#include "DxTimer.hpp"
#include <glut.h>


using namespace cv;
#define SPECIAL_KEY_UP			101
#define SPECIAL_KEY_DOWN 		103
#define SPECIAL_KEY_LEFT 		100
#define SPECIAL_KEY_RIGHT 		102

#define SPECIAL_KEY_PAGEUP 	104
#define SPECIAL_KEY_PAGEDOWN 	105
const int VERSION_SOFT = 0x8A;  //  130

typedef struct{
		int button;
		int state;
		int x;
		int y;
}mouse_t;

typedef struct{
	volatile unsigned int MenuStat;
	volatile int Trig_Inter_Mode;;
	AppMenu menuarray[menumaxid];
	char Passwd[128];
	char disPasswd[128];
	mouse_t Mtdmouseclick;
	PointNode Mtdmousemotion;
	
	int resol_deng, mtdnum_deng,trktime_deng,maxsize_deng,minsize_deng,sensi_deng;//1:dianmie fanying
	int resol_type_tmp, resol_type;
	int osd_mudnum, osd_trktime, osd_maxsize, osd_minsize, osd_sensi;
	int baud_light;
	int baud_type;// 2400,4800,9600, 115200
	char mtdnum_arr[128];
	char trktime_arr[128];
	char maxsize_arr[128];
	char minsize_arr[128];
	char sensi_arr[128];
}menu_param_t;


class CProcess : public CVideoProcess
{
	UTC_RECT_float rcTrackBak[2],resultTrackBak;
	TARGET tgBak[MAX_TARGET_NUMBER];
	TARGETDRAW Mdrawbak[MAX_TARGET_NUMBER];
	Osd_cvPoint crossBak;
	Osd_cvPoint crossWHBak;
	cv::Rect acqRectBak;
	Osd_cvPoint freezecrossBak;
	Osd_cvPoint crosspicpBak;
	Osd_cvPoint crosspicpWHBak;
	Osd_cvPoint rectfovBak[2];
	Osd_cvPoint secBak[2];
	DS_Rect rendpos[4];
	int renderflag[10];
	int Osdflag[20];
	int osd_flag[20];
	int osdindex;
	int osd_index;
	int Mmtsendtime;
	int prisensorstatus;
	int Fovpri[2];
	DS_Rectmmt Mmtpos[5];
	char trkFPSDisplay[128];
	char warnTargetIndex[8];
	Osd_cvPoint debugBak;
	char timedisplay[128];
	bool forwardflag,backflag;

	int key_point1_cnt ;
	int key_point2_cnt ;
	int AllPoints_Num  ;
	int string_cnt1;
	int string_cnt2;
	char show_key[64][6];
	char show_key2[64][6];
	char CStrings[20];
	char Bak_CString[20];
	int m_bak_count;
	Point key1_pos;
	Point key2_pos;
	Point key1_backup;
	Point key2_backup;
	Point textPos1_record[64];
	Point textPos2_record[64];
	Point textPos1_backup[64];
	Point textPos2_backup[64];
	Point circle_point;
private:
	bool m_bRefreshPTZValue;
	int m_capX, m_capY;
	Rect m_rectSelectPic;
	bool m_bMarkCircle;
	void Cmp_SysParam();
	SysParam m_sysparm;
	FileStorage readfs;
	FileStorage writefs;
	void getParams();
	void setParams();
	//OSA_SemHndl m_linkage_getPos;
	int panPos ;
	int tiltPos ;
	int zoomPos ;
	int m_cofx , m_cofy;
	int backMenuposX, backMenuposY;	
	Point m_curClickPoint;
	Point m_bakClickPoint;
	cv::Point2d m_ballDestPoint;
	cv::Point2d m_bakballDestPoint;
	int m_iDelta_X;
	int m_iZoom;
/*******************************************************/	
	bool m_bGridMapCalibrate;
	bool m_queryZoom;
	char tmp_str[5][40];
	char str_mode[30];
	int m_rCH;
	cv::Point back_center;
	int m_lastRow, m_lastCol;
	int m_successCalibraNum;
	int m_intervalRow[GRID_ROWS_11];
	
	int m_winWidth, m_winHeight;
	int m_AppVersion;
	char m_appVersion[30];

/*****************************************************/	
public:
	enum
	{
		MIN_VALID_RECT_WIDTH_IN_PIXEL = 10,
		JOSF1_OPEN_AUTOLINKMODE = 1,
		JOSF2_ENTER_MENU=2,
		CONST_VARIABLE_COUNT		
	};
	enum DrawBehavior
	{
		DRAW_NOTICE_TEXTS,
		ERASE_TEXTS,
		DRAW_COUNT
	};
	void setGridMapCalibrate(bool flag);
	const bool getGridMapCalibrate();
	void setQueryZoomFlag(bool flag);	
	const bool getQueryZoomFlag();	
	int getCurrentZoomValue();
	void addMarkNum();	
	void renderText(enum DrawBehavior drawbehavior);
	void renderCircles(int radius, cv::Point position);
	void DrawCenterCross(cv::Point pos, int width,int height);

	void setPTZflag(bool flag);
	bool getPTZflag();	
	void loadIPCParam();
	bool readParams(const char* file);
	bool writeParams(const char* file);
	bool loadConfigParams(const char* filename);
	bool saveConfigParams(const char* filename);

	void MvBallCamBySelectRectangle(int x, int y,bool needChangeZoom);
	void MvBallCamUseLinearDeviationSelectRect(int x, int y,bool needChangeZoom);
	void MvBallCamByClickGunImg(int x, int y,bool needChangeZoom);	
	void CvtImgCoords2CamCoords(Point &imgCoords, Point &camCoords);
	void CvtImgPoint2Camera(cv::Point2d &imgCoords, cv::Point2d &camCoords);
	void TransformPixByOriginPoints(int &X, int &Y );	
	void SetDestPosScope(int &inputX, int &inputY, int &Origin_PanPos, int &Origin_TilPos,int &DesPanPos, int &DesTilPos);
	void MvBallCamByClickBallIMg(int x, int y);
	void MoveBall();
	void Test_Match_result(int x, int y);
	void QueryCurBallCamPosition();
	void setBallPos(int in_panPos, int in_tilPos, int in_zoom);
	void RefreshBallPTZ(int in_panPos, int in_tilPos, int in_zoom);
	void refreshClickPoint(int x, int y);
	void Set_K_ByDeltaX( int delta_x);
	void Set_K_ByZoom(int Current_Zoom);
       void Set_K_ByNewDeltaX(int delta_x);
	void CaptureMouseClickPoint(int x, int y);
	void setBallImagePoint(int &x, int &y);
	void Init_CameraMatrix();
	void manualHandleKeyPoints(int &x,int &y);	
	Point getCurrentMouseClickPoint();
	Point getBallImagePoint();	
	Point replaceClickPoints(int pointX, int pointY);
	Mat undisImageGun;		
	int checkZoomPosTable(int delta);
	int checkZoomPosNewTable(int delta);	
public:
	CProcess();
	CProcess(int window_width, int window_height);
	~CProcess();
	void setDisplayResolution(CDisplayer &displayObject ,int w, int h);
	void OnCreate();
	void OnDestroy();
	void OnInit();
	void OnConfig();
	void OnRun();
	void OnStop();
	void Ontimer();
	bool OnPreProcess(int chId, Mat &frame);
	bool OnProcess(int chId, Mat &frame);
	void OnMouseLeftDwn(int x, int y);
	void OnKeyDwn(unsigned char key);
	void OnSpecialKeyDwn(int key,int x, int y);
	void OnJosCtrl(int key, int value);
	void setWorkMode(GB_WorkMode workmode);
	void SetDefaultWorkMode( GB_WorkMode workmode );
	void SetMtdConfig( MTD_Config mtdconfig );

	void DrawMtdYellowGrid();
	void DrawMtdRedGrid();
	void DrawMtd_Rigion_Target();
	void DrawGridMap(int flag);
	void DrawGridMap_16X12(int flag);

	void DrawGridMapNodeCircles(bool drawFlag);
	void DrawGridMapNodeCircles(bool drawFlag, int drawNodesCount);
	void DrawSelectedCircle(bool drawFlag, int drawNodesCount);

	void DrawGridMapNodeCircles_16X12(bool drawFlag);
	void Drawfeaturepoints();
	void Drawsubdiv();
	void Draw_subdiv_point();
	void Draw_point_triangle();
	void DrawDragRect();
	void DrawJoys();
	void DrawCircle(Mat frame, cv::Point center, int radius, int colour, int thickness);
	void DrawMouse();
	
	CMD_EXT* extInCtrl;
	menu_param_t extMenuCtrl;
	static CProcess *sThis;
	void process_osd_test(void *pPrm);
	void TimerCreate();
	static void Tcallback(void *p);
	int setresol(int resoltype);
	int udoutputresol(int resoltype);
	int writeshell(int resoltype);

protected:
	void msgdriv_event(MSG_PROC_ID msgId, void *prm);
	void osd_mtd_show(TARGET tg[], bool bShow = true);
	void GB_DrawCross(Mat &textureImg,cv::Point center, bool needShow);
	void DrawCross(cv::Rect rec,int fcolour ,int sensor,bool bShow /*= true*/);
	void drawmmt(TARGET tg[], bool bShow = true);
	void drawmmtnew(TARGET tg[], bool bShow = true);
	void erassdrawmmt(TARGET tg[], bool bShow = true);
	void erassdrawmmtnew(TARGETDRAW tg[], bool bShow = true);
	void DrawdashCross(int x,int y,int fcolour , bool bShow = true);
	void DrawdashRect(int startx,int starty,int endx,int endy,int colour);
	void DrawMeanuCross(int x,int y,int fcolour , bool bShow,int centerx,int centery);
	float  PiexltoWindowsx(int x,int channel);
	float  PiexltoWindowsy(int y,int channel);
	int  PiexltoWindowsxzoom(int x,int channel);
	int  PiexltoWindowsyzoom(int y,int channel);
	int  PiexltoWindowsxzoom_TrkRect(int x,int channel);
	int  PiexltoWindowsyzoom_TrkRect(int y,int channel);
	int  WindowstoPiexlx(int x,int channel);
	int  WindowstoPiexly(int y,int channel);
	float PiexltoWindowsxf(float x,int channel);
	float PiexltoWindowsyf(float y,int channel);
	int updateredgrid();
	int updateredgridfrrectL();
	int updateredgridfrrectR();
	int getmtdedge();
	int usopencvapi2();

	 static int  MSGAPI_initial(void);
	 static void MSGAPI_init_device(long lParam );
	 static void MSGAPI_inputsensor(long lParam );
	 static void MSGAPI_picp(long lParam );
	 static void MSGAPI_inputtrack(long lParam );
	 static void MSGAPI_inpumtd(long lParam );
	 static void MSGAPI_inpuenhance(long lParam );
	 static void MSGAPI_inputbdt(long lParam );
	 static void MSGAPI_inputzoom(long lParam );
	 static void  MSGAPI_setAimRefine(long lParam          /*=NULL*/);
	 static void MSGAPI_setAimSize(long lParam );
	 static void MSGAPI_inputfrezz(long lParam );
	 static void MSGAPI_inputmmtselect(long lParam );
	 static void MSGAPI_croppicp(long lParam );
	 static void MSGAPI_inpumtdSelect(long lParam );
	 static void MSGAPI_inputpositon(long lParam );
	 static void MSGAPI_inputcoast(long lParam );
	 static void MSGAPI_inputfovselect(long lParam );
	 static void MSGAPI_inputfovchange(long lParam );
	 static void MSGAPI_inputsearchmod(long lParam );
	 static void MSGAPI_inputvideotect(long lParam );
	 static void MSGAPI_mmtshow(long lParam );
	 static void MSGAPI_FOVcmd(long lParam );
	 static void MSGAPI_SaveCfgcmd(long lParam );
	 static void MSGAPI_setMtdState(long lParam);

	 static void MSGAPI_update_osd(long IParam);
	 static void MSGAPI_update_alg(long IParam);
	 static void MSGAPI_update_camera(long IParam);
	 static void MSGAPI_input_algosdrect(long lParam);
	 static void MSGAPI_setMtdSelect(long lParam );
	 static void MSGAPI_update_ballPos(long lParam );
	 static void MSGAPI_update_menuindex(long lParam );
	 static void MSGAPI_up_menu(long lParam);
	 static void MSGAPI_down_menu(long lParam);
	 static void MSGAPI_setMtdSetRigionStat(long lParam);
	 static void MSGAPI_setMtdSetRigion(long lParam);
	 static void MSGAPI_save_mtdrigion(long lParam);
	 static void MSGAPI_set_resol(long lParam);
	 static void MSGAPI_set_baud(long lParam);
	 static void MSGAPI_save_resol(long lParam);
	 static void MSGAPI_set_mtdnum(long lParam);
	 static void MSGAPI_set_mtdtrktime(long lParam);
	 static void MSGAPI_set_mtdmaxsize(long lParam);
	 static void MSGAPI_set_mtdminsize(long lParam);
	 static void MSGAPI_set_mtdsensi(long lParam);

private:
	ACK_EXT extOutAck;
	bool    m_bCast;
	UInt32 	m_castTm;

	void process_status(void);

	void osd_init(void);
	void DrawLine(Mat frame, int startx, int starty, int endx, int endy, int width, UInt32 colorRGBA);
	void DrawHLine(Mat frame, int startx, int starty, int width, int len, UInt32 colorRGBA);
	void DrawVLine(Mat frame, int startx, int starty, int width, int len, UInt32 colorRGBA);
	void DrawChar(Mat frame, int startx, int starty, char *pChar, UInt32 frcolor, UInt32 bgcolor);
	void DrawString(Mat frame, int startx, int starty, char *pString, UInt32 frcolor, UInt32 bgcolor);
	void DrawStringcv(Mat& frame, int startx, int starty, char *pChar, UInt32 frcolor, UInt32 bgcolor);
	void osd_draw_cross(Mat frame, void *prm);
	void osd_draw_rect(Mat frame, void *prm);
	void osd_draw_rect_gap(Mat frame, void *prm);
	void osd_draw_text(Mat frame, void *prm);
	void osd_draw_cross_black_white(Mat frame, void *prm);

	int process_draw_line(Mat frame, int startx, int starty, int endx, int linewidth,char R, char G, char B, char A);
	int process_draw_instance(Mat frame);
	int draw_circle_display(Mat frame);

	void DrawRect(Mat frame,cv::Rect rec,int frcolor);
	void DrawAcqRect(cv::Mat frame,cv::Rect rec,int frcolor,bool bshow);

	void initAcqRect();
	void initAimRect();
	void set_trktype(CMD_EXT *p, unsigned int stat);

	#if __MOVE_DETECT__
	char getMvListValidNum();
	void switchMvTargetForwad();
	void addMvListValidNum(char num);
	char getMvListFirstUnusedNum();
	void removeMvListValidNum(char num);
	char getMvListNextValidNum(char index);
	void getTargetNearToCenter(std::vector<TRK_RECT_INFO> *pVec);
	void mvIndexHandle(std::vector<TRK_INFO_APP> *mvList,std::vector<TRK_RECT_INFO> &detect,int detectNum);
	#endif
	
public:
	void update_param_alg();
	void update_param_osd();
	RectfNode mRectbak[MAX_CHAN][100];
	int m_rectnbak[MAX_CHAN];
	char timearr[128];
	char timearrbak[128];
	int timexbak, timeybak;

	DxTimer dtimer;
    int resol_light_id, resol_apply_id, mtdnum_light_id, trktime_light_id, maxsize_light_id, minsize_light_id, sensi_light_id, baud_light_id;
	int save_flag;
	int cnt_down;

	int mouse_show_id;
	bool validMtdRecord[10];
};



#endif /* PROCESS021_HPP_ */
