
#ifndef PROCESS021_HPP_
#define PROCESS021_HPP_

#include "VideoProcess.hpp"
#include "osd_cv.h"

#include "osa_sem.h"


using namespace cv;
#define SPECIAL_KEY_UP			101
#define SPECIAL_KEY_DOWN 		103
#define SPECIAL_KEY_LEFT 		100
#define SPECIAL_KEY_RIGHT 		102

#define SPECIAL_KEY_PAGEUP 		104
#define SPECIAL_KEY_PAGEDOWN 	105

typedef struct{
	int resol_deng;//1:deng liang   0:deng mie
	int resol_type;

	int baud_light;
	int baud_type;// 2400,4800,9600, 115200
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
	int Osdflag[20];
	int osdindex;
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
public:
	bool readParams(const char* file);
	bool writeParams(const char* file);
	void reMapCoords(int x, int y , bool mode);
	void moveToDest( );
	void QueryCurBallCamPosition();
	void setBallPos(int in_panPos, int in_tilPos, int in_zoom);
	void Set_K_ByDeltaX( int delta_x);


	void clickOnBallImage(int x, int y);

	void Init_CameraMatrix();
	Mat undisImageGun;
	
	void manualHandleKeyPoints(int &x,int &y);
	
	int checkZoomPosTable(int delta);

public:
	CProcess();
	~CProcess();

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
	void OnMouseLeftUp(int x, int y);
	void OnMouseRightDwn(int x, int y);
	void OnMouseRightUp(int x, int y);
	void OnKeyDwn(unsigned char key);
	void OnSpecialKeyDwn(int key,int x, int y);
	void DrawMtdYellowGrid(int flag);
	void DrawMtdRedGrid(int flag);
	
	CMD_EXT* extInCtrl;
	menu_param_t extMenuCtrl;
	static CProcess *sThis;
	void process_osd_test(void *pPrm);


protected:
	void msgdriv_event(MSG_PROC_ID msgId, void *prm);
	void osd_mtd_show(TARGET tg[], bool bShow = true);
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
	int updatemtdrigion();
	int isborder(int rigionindex, int x, int y);
	int createrigion(int rigionindex, int x, int y);
	int mergenode(int rigionindex, int x, int y);
	int mergerigion(int rigionindex_dst, int rigionindex_src);
	int getmtdedge();
	int cp2pointarray();
	int usopencvapi();
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
	void mvIndexHandle(std::vector<TRK_RECT_INFO> &mvList,std::vector<TRK_RECT_INFO> &detect,int detectNum);
	#endif
	
public:
	void update_param_alg();
	void update_param_osd();
	RectfNode mRectbak[MAX_CHAN][100];
	int m_tempXbak, m_tempYbak, m_rectnbak[MAX_CHAN];
	char timearr[128];
	char timearrbak[128];
	int timexbak, timeybak;

	PointNode polyRectbak[MAX_CHAN][100];
	int polytempXbak, polytempYbak, polyrectnbak[MAX_CHAN];

};



#endif /* PROCESS021_HPP_ */
