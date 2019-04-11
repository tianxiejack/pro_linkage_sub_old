
#ifndef VIDEOPROCESS_HPP_
#define VIDEOPROCESS_HPP_
#include <opencv2/core.hpp>
#include "MultiChVideo.hpp"
#include "Displayer.hpp"
#include "UtcTrack.h"
#include "multitarget.h"
#include "app_status.h"

#include "MMTD.h"
#include "mvdectInterface.hpp"
#include "configable.h"

#include "CcCamCalibra.h"
#include "IntrinsicMatrix.h"

#include "trigonometric.hpp"
#include "autoManualFindRelation.hpp"
#include "GridMap.h"
#include "Ipc.hpp"


using namespace cr_trigonometricInterpolation;
using namespace cr_automanualfindrelation;

#define MAX_SUCCESS_IMAGES 160
#define GRID_CNT_X 19
#define GRID_CNT_Y 10

const int IMG_WIDTH = 1920;
const int IMG_HEIGHT = 1080;
const int GRID_ROWS = 11;//17;
const int GRID_COLS = 15;//23;
const int GRID_ROWS_11 = 11;
const int GRID_COLS_15 = 15;
const int GRID_WIDTH = 120;//80;
const int GRID_HEIGHT = 90;//60;
const int GRID_WIDTH_120 = 120;
const int GRID_HEIGHT_90 = 90;

const int MAX_ANGLE = 36000;
const int PER_ANGLE = 100;
const int ZERO_ANGLE = 0;

typedef struct
{
	int state;//1:is clicked,  0:not be clicked
	int rigionindex;//rigion num
}grid_node;

typedef struct
{
	int x;
	int y;
	int w;
	int h;
}mouserect;

typedef struct
{
	float x;
	float y;
	float w;
	float h;
}mouserectf;

typedef struct _main_thr_obj_cxt{
	bool bTrack;
	bool bMtd;
	bool bMoveDetect;
	int chId;
	int iTrackStat;
	
	//Mat frame;
}MAIN_ProcThrObj_cxt;

typedef struct _main_thr_obj{
	MAIN_ProcThrObj_cxt cxt[2];
	OSA_ThrHndl		thrHandleProc;
	OSA_SemHndl	procNotifySem;
	int pp;
	bool bFirst;
	volatile bool	exitProcThread;
	bool						initFlag;
	void 						*pParent;
}MAIN_ProcThrObj;


typedef struct _Track_info{
	UTC_RECT_float trackrect;
	UTC_RECT_float reAcqRect;
	unsigned int  trackfov;
	unsigned int TrkStat;

}Track_InfoObj;

typedef struct RectfNode {  
    int x1;  
    int y1;  
    int x2;  
    int y2;  
}RectfNode;

typedef struct{  
    int x;  
    int y;   
}PointNode;

typedef struct{
	char number;
	TRK_RECT_INFO trkobj;
}TRK_INFO_APP;

class CVideoProcess
{
	MAIN_ProcThrObj	mainProcThrObj;
	Mat mainFrame[2];
public:
	enum {
		GRIDMAP_FILE_LINENUM = 613
	};
	static CcCamCalibra *m_camCalibra;	
	static DetectCorners *m_detectCorners;	
	static IntrinsicMatrix *m_intrMatObj;
	Mat gun_srcMat_remap;
	Point LeftPoint;
	Point RightPoint;
	#if 0
	GridMapNode m_gridNodes[GRID_ROWS+1][GRID_COLS+1];
	GridMapNode m_readGridNodes[GRID_ROWS+1][GRID_COLS+1];
	MarkMapNode m_calibratedNodes[GRID_ROWS+1][GRID_COLS+1];
	cv::Point m_nodePos[GRID_ROWS+1][GRID_COLS+1];
	#else
	GridMapNode m_gridNodes[GRID_ROWS_11+1][GRID_COLS_15+3];	
	GridMapNode m_readGridNodes[GRID_ROWS_11+1][GRID_COLS_15+1];
	MarkMapNode m_calibratedNodes[GRID_ROWS_11+1][GRID_COLS_15+3];
	cv::Point m_nodePos[GRID_ROWS_11+1][GRID_COLS_15+2];
	int m_intervalCOl[GRID_COLS_15+10];

	#endif
	cv::Point m_backNodePos;
	int m_curNodeIndex;
	//cv::Point temp_backPoint[GRID_ROWS+1][GRID_COLS+1] ;
	cv::FileStorage m_readfs;
	cv::FileStorage m_writefs;

	cv::FileStorage m_fsReadMtd;
	cv::FileStorage m_fsWriteMtd;

	vector<position_t> m_trigonoMetricVector;
	position_t  m_trigonoMetric_Node;
	std::vector<FEATUREPOINT_T> app_recommendPoints;
	std::vector<FEATUREPOINT_T> app_recommendPoints_bak;

public:
	bool get_find_featurepoint_stat(){return find_featurepoint_stat;};
	void set_find_featurepoint_stat(bool value){find_featurepoint_stat = value;};
	bool get_cloneSrcImage_stat(){return cloneSrcImage_stat;};
	void set_cloneSrcImage_stat(bool value){cloneSrcImage_stat = value;};
	bool get_manualInsertRecommendPoints_stat(){return manualInsertRecommendPoints_stat;};
	void set_manualInsertRecommendPoints_stat(bool value){manualInsertRecommendPoints_stat = value;};
	bool get_drawpoints_stat(){return drawpoints_stat;};
	void set_drawpoints_stat(bool value){drawpoints_stat = value;};
	bool get_drawsubdiv_stat(){return drawsubdiv_stat;};
	void set_drawsubdiv_stat(bool value){drawsubdiv_stat = value;};
	bool get_drawsubdiv_point_stat(){return drawsubdiv_stat_point;};
	void set_drawsubdiv_point_stat(bool value){drawsubdiv_stat_point = value;};
	
	void auto_insertpoint(int x, int y);
	void auto_selectpoint(int x, int y);
	void insertPos(int x, int y);
	
	static void pnotify_callback(std::vector<FEATUREPOINT_T>& recommendPoints);
	void InitGridMapNodes();
	void InitGridMap16X12();
	void useTrigonometric(int px, int py);

	GridMapNode getLinearDeviation(int px, int py);
	GridMapNode getLinearDeviation(int px, int py, int grid_width,int grid_height,bool needChangeZoom);
	GridMapNode getLinearDeviationForSelectRect(int px, int py, int grid_width,int grid_height,bool needChangeZoom);
	GridMapNode AutoLinkMoveBallCamera(int px, int py, int grid_width,int grid_height,bool needChangeZoom);

	bool readParams(const char* filename);
	bool writeParams(const char* filename);
	bool writeParamsForTriangle(const char* filename);

	int read_param_trig();
	void setMtdState(bool flag);
	const bool getMtdState();
	void set_trig_PTZflag(bool flag);
	bool get_trig_PTZflag();
private:
	Mat m_GrayMat;
	Mat m_Gun_GrayMat;
	Mat m_rgbMat;
	int m_ScreenWidth,m_ScreenHeight;
	int m_gridWidth,m_gridHeight;
	static int m_staticScreenWidth,m_staticScreenHeight;
	
	bool find_featurepoint_stat = false;
	bool cloneSrcImage_stat = false;
	bool manualInsertRecommendPoints_stat = false;
	bool drawpoints_stat = false;
	bool drawsubdiv_stat = false;
	bool drawsubdiv_stat_point = false;
	
public:
	CVideoProcess();
	CVideoProcess(int w, int h);
	~CVideoProcess();
	int creat();
	int destroy();
	int init();
	typedef enum{
		VP_CFG_MainChId = CDisplayer::DS_CFG_Max,
		VP_CFG_SubChId,
		VP_CFG_TrkEnable,
		VP_CFG_MmtEnable,
		VP_CFG_SubPicpChId,
		VP_CFG_MvDetect,
		VP_CFG_Max
	}VP_CFG;
	int dynamic_config(int type, int iPrm, void* pPrm = NULL);
	int run();
	int stop();

public:
	void draw_mouse_move(GLint xMouse, GLint yMouse);
	static void mouse_event(int button, int state, int x, int y);
	static void mousemotion_event(GLint xMouse, GLint yMouse);
	static void setStaticScreenResolution(int w, int h){
		m_staticScreenWidth = w;
		m_staticScreenHeight = h;
		return ;
	};
	virtual void setDisplayResolution(CDisplayer &displayObject, int w, int h ){};
	virtual void OnCreate(){};
	virtual void OnDestroy(){};
	virtual void OnInit(){};
	virtual void OnConfig(){};
	virtual void OnRun(){};
	virtual void OnStop(){};
	virtual void Ontimer(){};
	virtual bool OnPreProcess(int chId, Mat &frame){return true;}
	virtual bool OnProcess(int chId, Mat &frame){return true;}
	virtual void OnMouseLeftDwn(int x, int y){};
	virtual void OnKeyDwn(unsigned char key){};
	virtual void OnSpecialKeyDwn(int key,int  x,int  y){};

	virtual void CaptureMouseClickPoint(int x, int y){};
	virtual Point getCurrentMouseClickPoint(){};
	virtual void MvBallCamBySelectRectangle(int x, int y,bool needChangeZoom){ };
	virtual void MvBallCamUseLinearDeviationSelectRect(int x, int y,bool needChangeZoom){};

	virtual void MvBallCamByClickGunImg(int x, int y,bool needChangeZoom){ };
	virtual void MvBallCamByClickBallIMg(int x, int y){};
	virtual void Test_Match_result(int x, int y){};
	virtual void setQueryZoomFlag(bool flag){};
	virtual const bool getQueryZoomFlag(){};
	virtual int getCurrentZoomValue(){};
	virtual void addMarkNum(){};
	//virtual void QueryCurBallCamPosition() { };
	//virtual void setBallPos(int in_panPos, int in_tilPos, int in_zoom) {};
	void linkage_init();
	bool Set_SelectByRect;
	bool open_handleCalibra;
	int m_SensorStat;
	int m_acqRectW;
	int m_acqRectH;
	static bool m_bLDown;
	static bool m_bIsClickMode;
public :
	//CMSTracker trackInit;
	int detState ;
	int trackEnd ;
	int trackStart ;
	bool nextDetect;
	int lastFrameBox ;
	int prichnalid;
	//vector<Rect> Box;
	bool moveStat;
	bool algOsdRect;	
	bool TrkAim43;
	bool wFileFlag;
	bool tvzoomStat;
	ALGMTD_HANDLE m_mtd[MAX_CHAN];
public:
	CDisplayer m_display;
	int m_time_show,m_time_flag;
	int click_in_area;
	int m_click;
	int m_draw;
	RectfNode mRect[MAX_CHAN][100];
	int m_tempX, m_tempY, m_rectn[MAX_CHAN];
	int setrigon_flag;
	int setrigon_polygon;
	
	int m_click_v20L, m_click_v20R;
	RectfNode mRectv20L;
	RectfNode mRectv20R;

	cv::Point jos_mouse;
	int mouse_show = 0;
	void set_mouse_show(int param);
	cv::Point cur_trig_inter_P;
	int trig_inter_flag = 0;	
	Trigonometric m_trig = Trigonometric(outputWHF[0],outputWHF[1]);
	CAutoManualFindRelation m_autofr = CAutoManualFindRelation(outputWHF[0],outputWHF[1], 6, 6);
	vector<position_t> app_trig;
	void update_cur_trig_inter_P(int x, int y);
	void SaveMtdSelectArea(const char* filename, std::vector< std::vector< cv::Point > > edge_contours);
	void LoadMtdSelectArea(const char* filename, std::vector< std::vector< cv::Point > > &edge_contours);

private:
	Trigonometric *m_pTrigonometric;
public:
	Trigonometric* createTrigonometric(int imgaeWidth,int imageHeight);	
protected:
	MultiChVideo MultiCh;
	//BigChVideo		BigChannel;	
	int adaptiveThred;
	UTCTRACK_HANDLE m_track;	
	static bool m_bTrack;
	static bool m_bMtd;			// old singla for mmt : multi target detect
	static bool m_bBlobDetect;
	static bool m_bMoveDetect;
	static int m_iTrackStat;
	static int m_iTrackLostCnt;	
	Uint32 rememtime;
	bool rememflag;
	int m_curChId;
	int m_curSubChId;
	int trackchange;
	int m_searchmod;
	int Enhmod;
	float Enhparm;
	int DetectGapparm;
	int MinArea;
	int MaxArea;
	int stillPixel;
	int movePixel;
	float lapScaler;
	int lumThred;	
	int configEnhFromFile();	
	void process_event(int type, int iPrm, void *pPrm);
	int process_frame(int chId, int virchID, Mat frame);
	int process_mtd(ALGMTD_HANDLE pChPrm, Mat frame_gray, Mat frame_dis);
	#if __TRACK__
	Track_InfoObj *trackinfo_obj;
	int process_track(int trackStatus, Mat frame_gray, Mat frame_dis, UTC_RECT_float &rcResult);
	int ReAcqTarget();
	void Track_reacq(UTC_RECT_float & m_rcTrack,int acqinterval);
	void Track_fovreacq(int fov,int sensor,int sensorchange);
	#endif
	static int m_mouseEvent, m_mousex, m_mousey;
	static CVideoProcess *pThis;
	static void call_run(int value);
	static int callback_process(void *handle, int chId, int virchId, Mat frame);

	static void processtimeMenu(int value);
	static void processsmanualcarliMenu(int value);
	static void processsautocarliMenu(int value);
	
	int click_legal(int x, int y);
	int move_legal(int x, int y);
	int in_gun_area(int x, int y);
	int in_ball_area(int x, int y);
	mouserect map2preview(mouserect rectcur);
	mouserect mappip2preview(mouserect rectcur);
	mouserect mapsbs2preview(mouserect rectcur);
	mouserect maplbrg2preview(mouserect rectcur);
	mouserect mapfullscreen2gun(mouserect rectcur);
	mouserect mapfullscreen2gunv20(mouserect rectcur);
	mouserect mapgun2fullscreen(mouserect rectcur);
	int mapgun2fullscreen_point(int *x, int *y);
	int mapfullscreen2gun_pointv20(int *x, int *y);
	mouserect maprect(mouserect rectcur,mouserect rectsrc,mouserect rectdest);
	int maprect_point(int *x, int *y, mouserect rectsrc,mouserect rectdest);
	void sendIPC_Time(int value);

	int map1080p2normal_point(float *x, float *y);
	int mapnormal2curchannel_point(float *x, float *y, int w, int h);
	int map1080p2normal_rect(mouserectf *rect);
	int mapnormal2curchannel_rect(mouserectf *rect, int w, int h);
	static void mousemove_event(GLint xMouse, GLint yMouse);
	void mouse_eventv20(int button, int state, int x, int y);
	static void menu_event(int value);
	static void processrigionMenu(int value);
	static void processrigionselMenu(int value);
	static void processrigionpolygonMenu(int value);
	int InJoys(int x, int y);
	void mapout2inresol(cv::Point *tmppoint);
	void sendjoyevent(cv::Point tmppoint);
	void mapcapresol2joy(cv::Point *tmppoint);
	int get_joyradius();
	cv::Point get_joycenter();
	void addstartpoint(int x, int y, int curId);
	void addendpoint(int x, int y, int curId);
		
#if __MOVE_DETECT__
	static void processmaxnumMenu(int value);
	static void processmaxtargetsizeMenu(int value);
	static void processmintargetsizeMenu(int value);
#endif
	static void keyboard_event(unsigned char key, int x, int y);
	static void keySpecial_event( int key, int x, int y);
	static void visibility_event(int state);
	static void close_event(void);
	void getImgRioDelta(unsigned char* pdata,int width ,int height,UTC_Rect rio,double * value);
	
private:
	OSA_MutexHndl m_mutex;
//	unsigned char *m_grayMem[2];
	char m_strDisplay[128];
	void main_proc_func();
	int MAIN_threadCreate(void);
	int MAIN_threadDestroy(void);
	static void *mainProcTsk(void *context)
	{
		MAIN_ProcThrObj  * pObj= (MAIN_ProcThrObj*) context;
		if(pObj==NULL)
			{

			OSA_printf("++++++++++++++++++++++++++\n");

			}
		CVideoProcess *ctxHdl = (CVideoProcess *) pObj->pParent;
		ctxHdl->main_proc_func();
		OSA_printf("****************************************************\n");
		return NULL;
	}
	static void extractYUYV2Gray2(Mat src, Mat dst);
	static int64 tstart;

protected: //track
	UTC_RECT_float m_rcTrack, m_rcAcq;
	UTC_Rect preAcpSR;
	UTC_Rect preWarnRect[MAX_CHAN];
	UTC_Rect preWarnRectBak[MAX_CHAN];
	UTC_Rect MoveAcpSR;
	UTC_Rect TRKMoveAcpSR;
	int			    m_intervalFrame;
	int			  m_intervalFrame_change;
	int 			    m_bakChId;
	
#if __MMT__
	CMMTD	m_MMTDObj;
	TARGETBOX	m_tgtBox[MAX_TARGET_NUMBER];	
#endif

#if __MOVE_DETECT__
public:
		CMvDectInterface *m_pMovDetector;
		void	initMvDetect();
		void	DeInitMvDetect();
		static void NotifyFunc(void *context, int chId);
		std::vector<TRK_RECT_INFO> detect_vect;
		std::vector<TRK_RECT_INFO> detect_bak;
		std::vector<TRK_INFO_APP> mvListsum;
		std::vector< std::vector<TRK_RECT_INFO> > detect_vect_arr;
		std::vector< std::vector<TRK_RECT_INFO> > detect_vect_arr_bak;
		std::vector< std::vector<TRK_INFO_APP> > mvList_arr;
		int detectNum;
		signed char chooseDetect;
		int maxsize;
		int minsize;
		int sensi;
		cv::Rect cur_targetRect, cur_targetRect_bak;
		int losenumber = -1;
		int lose_timer_flag = 0;
		

		int setrigion_flagv20;
		struct{
			int button;
			int state;
			int x;
			int y;
		} mtdrigionv20;

		grid_node grid19x10[GRID_CNT_X][GRID_CNT_Y];
		grid_node grid19x10_bak[GRID_CNT_X][GRID_CNT_Y];
		int mtdcnt;
		std::vector< std::vector< cv::Point > > edge_contours;
		std::vector< std::vector< cv::Point > > edge_contours_bak;
		cv::Point jcenter_s;
		int joys_click;
#endif


};



#endif /* VIDEOPROCESS_HPP_ */
