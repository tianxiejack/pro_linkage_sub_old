
#ifndef DISPLAYER_HPP_
#define DISPLAYER_HPP_

#define DS_RENDER_MAX	       (9)
#define DS_CHAN_MAX            (5)
//4
#define DS_CUSTREAM_CNT	(4)

#define DS_DC_CNT		       (5)
#include "osa.h"
#include <osa_sem.h>
#include <cuda.h>
#include "cuda_runtime_api.h"


#include "osa.h"
#include "osa_thr.h"
#include "osa_buf.h"
#include "osa_sem.h"
#include "app_status.h"
#include "CcCamCalibra.h"
#include <math.h>
#include "configable.h"
#include "ipc_custom_head.hpp"
#include "CELLMath.hpp"
#include <vector>
#include "freetype/ftglyph.h"
#include <wchar.h>
#include <glut.h>
#include "trigonometric.hpp"

#include FT_GLYPH_H
#include FT_TRUETYPE_TABLES_H
#include FT_BITMAP_H
#include FT_WINFONTS_H

using namespace CELL;
using namespace std;
using namespace cv;
using namespace cr_trigonometricInterpolation;


struct UIObject {
public:
    UIObject(){};
    virtual ~UIObject(){};
	FLOAT2 _pos;
	FLOAT2 _size;
	unsigned int _SN;
	
};
typedef void (*LPDWON )(UIObject *obj);
struct UIText:public UIObject{
public:
	//Rgba   _color;	
	unsigned char r,g,b,a;
	wchar_t  _text[32];
	LPDWON _eventDown;
	LPDWON _clickDown;	
};
typedef std::vector<UIText> ArrayText;


typedef struct _GB_MENU{
	ArrayText _texts;
	ArrayText workMode;
	UIText text1;
	UIText text2;
	UIText text3;
	UIText text4;
	UIText text5;
	UIText text6;
	UIText text7;
	UIText text8;
	UIText text9;
	bool _bRButton ;
	CELL::int2  _mouseDown;
	bool showSubMenu ;
	UIObject* _pSelect;
	UIObject* _LDown ;
}GB_MENU;

struct Vertex3F {
	float x, y, z;
	float u,v ;
};
struct RectColor{
	float x, y, z;
	float r, g, b;
};
typedef struct _ds_size{
	int w;
	int h;
	int c;
}DS_Size;

typedef struct _ds_rect{
	int x;
	int y;
	int w;
	int h;
}DS_Rect;
typedef struct _ds_rectmmt{
	int x;
	int y;
	int w;
	int h;
	int valid;
}DS_Rectmmt;

typedef struct _ds_frect{
	float x;
	float y;
	float w;
	float h;
}DS_fRect;

typedef struct _ds_render
{
	int video_chId;
	DS_Rect displayrect;
	bool bCrop;
	DS_Rect croprect;
	DS_fRect bindrect;
	bool bBind;
	float transform[4][4];
	bool bFreeze;
	bool videodect;
}DS_Render;

typedef struct _ds_init_param{
	bool bFullScreen;
	int winPosX;
	int winPosY;
	int winWidth;
	int winHeight;
	bool bScript;
	char szScriptFile[256];
	int initloyerId;
	//void (*displayfunc)(void);
	
	int disFPS;      // Add 20181219
	float disSched;  // Add 20181219
	
	
	void (*timefunc)(int value);
	void (*manualcarli)(int value);
	void (*autocarli)(int value);
	void (*closecarli)(int value);
	void (*passivemotionfunc)(GLint xMouse, GLint yMouse);
	void (*motionfunc)(GLint xMouse, GLint yMouse);
	void (*mousefunc)(int button, int state, int x, int y);
	void (*menufunc)(int value);
	void (*setrigion)(int value);
	void (*rigionsel)(int value);
	void (*rigionpolygon)(int value);
#if __MOVE_DETECT__
	void (*maxnum)(int value);
	void (*maxsize)(int value);
	void (*minsize)(int value);
#endif

	//void (*reshapefunc)(int width, int height);
	void (*keyboardfunc)(unsigned char key, int x, int y);
	void (*keySpecialfunc)( int, int, int );
	void (*visibilityfunc)(int state);
	void (*timerfunc)(int value);
	void (*idlefunc)(void);
	void (*closefunc)(void);
	int timerfunc_value;//context
	int timerInterval;//ms
}DS_InitPrm;


	typedef enum _DISPLAYMODE {
		PREVIEW_MODE = 0,
		MAIN_VIEW,
		SIDE_BY_SIDE,
		LEFT_BALL_RIGHT_GUN,
		GUN_FULL_SCREEN,
		BALL_FULL_SCREEN,
		CALIBRATE_CAPTURE,  // get camera intrinsic matrix, and save picture of patterns
		CALIBRATE_RESULT, // display camera calibrate result and average error  
		MATCH_POINT_VIEW,
		TEST_RESULT_VIEW,
		TRIG_INTER_MODE,
		GRID_MAP_VIEW,
		TOTAL_MODE_COUNT
	}DISPLAYMODE;

	const int osdID_time = 30;
	const int osdID_name = 31;



class CDisplayer 
{
private:
	int fontPosX, fontPosY;
	int captureBMP_channel;
public:
	typedef struct _MENU_POS{
		int posX,posY;
		bool isShow;
	}MENU_POS;
	int selected_PicIndex;
	int m_currentSecondMenuIndex;	// add by swj
	int m_currentFirstMenuIndex;
	MENU_POS m_currentMenuPos[32][7];
public:
	int getSelectPicIndex(){
		return selected_PicIndex;
	};
	void setFontPosition(int x, int y){
		fontPosX = x;
		fontPosY = y;
	};
	FLOAT2 getFontPosition(){
		FLOAT2 fontpos = FLOAT2(fontPosX,fontPosY);
		return fontpos;
	};
	void setCapBMPChannel(int index){
		captureBMP_channel = index;
	};
	int getCapBMPChannel(){
		return captureBMP_channel;
	};
public:
	cv::Mat gun_UndistorMat;
	cv::Mat gun_BMP;
    DISPLAYMODE displayMode;
    char savePicName[20];
	int videonamex, videonamey, timex, timey, videonamefs;

	DetectCorners *m_detectCorners;

	typedef enum _WindowSize{
		WINDOW_WIDTH = 1920,
		WINDOW_HEIGHT = 1080,
			
	}WindowSize;
	typedef enum _VIDEOCNANNEL {
		VIDEO_0 = 0,
		VIDEO_1,
		VIDEO_2,
		VIDEO_3,
		VIDEO_4,
		VIDEO_COUNT			
	}VIDEOCNANNEL;
public:
	DISPLAYMODE g_CurDisplayMode;
	void RenderVideoOnOrthoView(int videoChannel,int x, int y, int width, int height);
	void changeDisplayMode(DISPLAYMODE mode);
	void switchDisplayMode( );
	DISPLAYMODE getDisplayMode( );
	void linkage_init();
	void linkageSwitchMode(void);
	void RenderSavedBMPImage(void);
	void RenderSavedBMPImageByIndex(int Index);
	void RenderDetectCornerView(GLint x, GLint y, GLint width, GLint height);	
	void RenderWarpImageView(GLint x, GLint y, GLint width, GLint height);	
	void RenderMatchPointsImageView(GLint x, GLint y, GLint width, GLint height);
	
	GLbyte* gltReadBMPBits(const char *szFileName, int *nWidth, int *nHeight);
	bool LoadBMPTexture(const char *szFileName, GLenum minFilter, GLenum magFilter, GLenum wrapMode);	
	
public:
	GLuint _textureId[100];
	
	char BMPName[100][20];
public:
	GLuint _texCornerId;	
	GLuint _texWarpId;	
	GLuint _texMatchId;	
	GLboolean _bCornerDetect;
public:
	unsigned char Cur_BMPIndex;

private:
	void sendIPC_Videoname(int value);
	void sendIPC_VideoName_pos();
	void sendIPC_Time_pos();
	int m_WinWidth, m_WinHeight;
	int m_viewPortX,m_viewPortY;
	int m_viewWidth,m_viewHeight;
public:
	void setGridViewPortPosition(int x, int y)
	{
		m_viewPortX = x;
		m_viewPortY = y;
	};
	void setGridViewPortWindowSize(int width, int height)
	{
		m_viewWidth = width;
		m_viewHeight = height;
	};
	cv::Point getGridViewBallImgCenter()
	{
		cv::Point temp;
		temp.x = m_viewPortX+m_viewWidth/2;
		temp.y = 1080- (m_viewPortY+m_viewHeight/2);
		return temp;
	};
public:
	CDisplayer();
	CDisplayer(int window_width, int window_height);
	~CDisplayer();
	void setDisplayResolution(int w, int h);
	int create();
	int destroy();
	int init(DS_InitPrm *pPrm);
	void run();
	void stop();

	typedef enum{
		DS_CFG_ChId = 0,
		DS_CFG_RenderPosRect,
		DS_CFG_EnhEnable,
		DS_CFG_CropEnable,
		DS_CFG_CropRect,
		DS_CFG_VideoTransMat,
		DS_CFG_ViewTransMat,
		DS_CFG_BindRect,
		DS_CFG_FreezeEnable,
		DS_CFG_VideodetEnable,
		DS_CFG_RenderPosinit,
		DS_CFG_Renderdisplay,
		DS_CFG_Rendercount,
		DS_CFG_MMTEnable,
		DS_CFG_Max
	}DS_CFG;

	int dynamic_config(DS_CFG type, int iPrm, void* pPrm);
	int get_videoSize(int chId, DS_Size &size);
	void display(Mat frame, int chId, int code = -1);/*CV_YUV2BGR_UYVY*/
	void transfer();// add 20181219
	int setFPS(float fps); // add 20181219
	GLuint async_display(int chId, int width, int height, int channels);
	int setFullScreen(bool bFull);
	void reDisplay(void);
	void UpDateOsd(int idc);

	int m_mainWinWidth_new[eSen_Max];
	int m_mainWinHeight_new[eSen_Max];
	bool m_bRun;
	bool m_bFullScreen;
	bool m_bOsd;
	bool m_crossOsd;

	bool savePic_once;
	bool showDetectCorners;
	Mat m_disOsd[DS_DC_CNT];
	Mat m_imgOsd[DS_DC_CNT];
	DS_Size m_videoSize[DS_CHAN_MAX];
	GLuint buffId_input[DS_CHAN_MAX];
	bool m_bEnh[DS_CHAN_MAX];
	bool m_Mmt[DS_CHAN_MAX];
	bool  Videoeable[DS_CHAN_MAX];
	unsigned int dismodchanag;
	unsigned int dismodchanagcount;
	 int tv_pribuffid0;
	 int tv_pribuffid1;
	 int tv_pribuffid2;
	 int tv_pribuffid3;
	 int fir_pribuffid;
	 int pal_pribuffid;
	unsigned  int freezeonece;

	#if 1
	OSA_MutexHndl disLock;
  	OSA_SemHndl tskdisSemmain;
  	OSA_ThrHndl tskdisHndlmain;
	#endif
	void gl_Loadinit();
	static  void* displayerload(void *pPrm);
	void disp_fps();
	
	int enhancemod;
	float enhanceparam;

	char capstrDisplay[128];
	char dispstrDisplay[128];
	int disptimeEnable;

	OSA_BufCreate tskSendBufCreatetv0;
       OSA_BufHndl tskSendBuftv0;
	OSA_BufCreate tskSendBufCreatetv1;
       OSA_BufHndl tskSendBuftv1;
	OSA_BufCreate tskSendBufCreatetv2;
       OSA_BufHndl tskSendBuftv2;
	OSA_BufCreate tskSendBufCreatetv3;
       OSA_BufHndl tskSendBuftv3;

	OSA_BufCreate tskSendBufCreatefir;
        OSA_BufHndl tskSendBuffir;

	OSA_BufCreate tskSendBufCreatepal;
	OSA_BufHndl tskSendBufpal;

	int m_menuindex;
	AppMenu dismenuarray[menumaxid];
	osdbuffer_t disMenuBuf[32][MAX_SUBMENU];
	osdbuffer_t disMtdBuf[1][MAX_SUBMENU];
	wchar_t disMenu[menumaxid][MAX_SUBMENU][33];
	wchar_t disMtd[1][MAX_SUBMENU][33];
	int disresol_type, disresol_type_tmp;
	int disbaud_type; // add baud settings
	int curBaudRate;
	int curBaudAddress;
	volatile int saveBaudrate;
	volatile int ballAddressID;
public:
	bool LoadComConfigs( const string& filename);
	bool saveComConfigs( const char* filename);

protected:
	DS_InitPrm m_initPrm;
	DS_Render m_renders[DS_RENDER_MAX];
	int m_renderCount;
	Mat m_img[DS_CHAN_MAX];
	Mat dism_img[DS_CHAN_MAX];
	Mat m_img_novideo;

	Mat x11m_img;
	unsigned char *x11disbuffer;
	int initRender(bool bInitBind = true);
	void uninitRender();
/******************************Add 20181219 Belows**********************************/
protected:
	Mat  m_frame[DS_CHAN_MAX][2];
	int	 m_code[DS_CHAN_MAX];
	int	pp[DS_CHAN_MAX];

	uint64  m_interval;
	double m_telapse;
	uint64  m_tmBak[DS_CHAN_MAX];
	int64   m_tmRender;
	bool m_waitSync;

	pthread_mutex_t render_lock;    /**< Used for synchronization. */
	pthread_cond_t render_cond;     /**< Used for synchronization. */
	uint64_t render_time_sec;       /**< Seconds component of the time for which a
										 frame should be displayed. */
	uint64_t render_time_nsec;      /**< Nanoseconds component of the time for which
										 a frame should be displayed. */
	struct timespec last_render_time;   /**< Rendering time for the last buffer. */
	int m_nSwapTimeOut;
	int64 tStamp[10];
/**************************************************************************************/

protected:
	static void _display(void);
	static void _timeFunc(int value);
	static void _reshape(int width, int height);
	static void processLinkageMenu(int value);
	static void processDMMenu(int value);
	static void processgunResMenu(int value);
	static void processballResMenu(int value);
	static void processposMenu(int value);
	static void processsizeMenu(int value);
	static void processnameMenu(int value);
	static void processfontsizeMenu(int value);
	static void processosdposMenu(int value);
	static void processbuadrateMenu(int value);
	static void processdatabitMenu(int value);
	static void processstopbitMenu(int value);
	static void processparityMenu(int value);
	static void processaddressMenu(int value);
	static void processprotocolMenu(int value);
	static void processSenMenu(int value);
	static void processtargetspeedMenu(int value);
	static void processtargetdircMenu(int value);
	static void processdetectcondMenu(int value);
	static void processpolarMenu(int value);
	static void processdurationMenu(int value);
	static void processmtdmodeMenu(int value);
	static void processredetectMenu(int value);
	static void processalarmputMenu(int value);

	static void _close(void);
	void gl_resize(void);

protected:
	GLint	m_glProgram;
	GLint	m_fontProgram;
	GLfloat m_glvVerts[DS_RENDER_MAX][8];
	GLfloat m_glvTexCoords[DS_RENDER_MAX][8];
	bool m_bUpdateVertex;
	GLfloat m_glmat44fTrans[DS_CHAN_MAX][16];
	GLuint textureId_input[DS_CHAN_MAX];
	//GLuint textureId_osd;//[DS_DC_CNT];

	GLuint textureId_osd[DS_DC_CNT];
	GLuint buffId_osd[DS_DC_CNT];
	unsigned char *dev_pbo_osd[DS_DC_CNT];
	GLboolean updata_osd[DS_DC_CNT];
	
	struct cudaGraphicsResource *cuda_pbo_resource[DS_CHAN_MAX];


	int gl_create();
	void gl_destroy();
	void gl_init();
	void gl_uninit();
	void gl_display();
	void gl_textureLoad();
	int gl_updateVertex();
	void gltLoadShaderSrc(const char *szShaderSrc, GLuint shader);
	bool gltLoadShaderFile(const char *szFile, GLuint shader);
	GLuint gltLoadShaderPairWithAttributes(const char *szVertexProg, const char *szFragmentProg, ...);
	int menu_init();

	Mat m_temp;

private:
	OSA_MutexHndl m_mutex;
	//unsigned char *m_dev_src_rgb[DS_CHAN_MAX];
	cudaStream_t m_cuStream[DS_CUSTREAM_CNT];

	cudaEvent_t	m_startEvent, m_stopEvent;


public:
	float frameRate ;
	int frameCount;
	void setFrameRate(float rate)	{ frameRate  = rate;	}; 
	void setFrameCount(int count)	{ frameCount = count;	};
	float getFrameRate()			{return frameRate;	};
	void GetFPS();
	FLOAT2 chinese_osd(int x,int y,wchar_t* text,char font,char fontsize,unsigned char r,unsigned char g,unsigned char b,unsigned char a,int win_width,int win_height);

	void IrisAndFocus();
	int OSDFunc();
	void MtdOSDFunc();
	void drawtriangle(Mat frame, char direction, char alpha);
	int MenuFunc(int index);
	void processdurationMenu_osd(int value);
	int trig_pip_mode = 0;
	void settrig_pip_mode(int mode)
	{
		trig_pip_mode = mode;
	};
	int gettrig_pip_mode()
	{
		return trig_pip_mode;
	};
};

#define mallocwidth 1920
#define mallocheight 1080

#define PICBUFFERCOUNT 4

#endif /* DISPLAYER_HPP_ */
