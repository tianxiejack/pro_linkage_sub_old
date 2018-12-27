
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
//#include <gl.h>
#include <glew.h>
#include <glut.h>
#include <freeglut_ext.h>
#include <cuda.h>
#include <cuda_gl_interop.h>
#include "cuda_runtime_api.h"
#include "osa.h"
#include "osa_mutex.h"
#include "osa_tsk.h"
#include "Displayer.hpp"
#include "enh.hpp"

#include "cuda_mem.cpp"
#include "app_status.h"

#include "osd_text.hpp"
#include "string.h"
#include "process51.hpp"
#include "locale.h"
#include "Ipcctl.h"
#include "arm_neon.h"

#include "CcCamCalibra.h"
#include "freetype.hpp"
using namespace CELL;

extern int captureCount;
UI_CONNECT_ACTION g_connectAction;
extern vector<Mat> imageListForCalibra;
bool SubImage = true;
bool g_bSubmitTexture = false;
extern Mat g_CornerImage;
MenuDisplay g_displayMode = MENU_SBS;
GB_WorkMode g_workMode = HANDLE_LINK_MODE;

SingletonSysParam* SingletonSysParam::m_uniqueInstance = SingletonSysParam::getInstance();
SingletonSysParam* g_sysParam = SingletonSysParam::getInstance();
#if 1
#pragma pack(1)
struct RGB { 
  GLbyte blue;
  GLbyte green;
  GLbyte red;
  GLbyte alpha;
};

struct BMPInfoHeader {
  GLuint	size;
  GLuint	width;
  GLuint	height;
  GLushort  planes;
  GLushort  bits;
  GLuint	compression;
  GLuint	imageSize;
  GLuint	xScale;
  GLuint	yScale;
  GLuint	colors;
  GLuint	importantColors;
};

struct BMPHeader {
  GLushort	type; 
  GLuint	size; 
  GLushort	unused; 
  GLushort	unused2; 
  GLuint	offset; 
}; 

struct BMPInfo {
  BMPInfoHeader		header;
  RGB				colors[1];
};
#pragma pack(8)
#endif
//=========================================================
#if 1
GB_MENU run_Mode;
unsigned int textSelect[20]={0};
unsigned int textSelect2[20]={0};
void TextDown(UIObject *obj)
{
	g_displayMode = MENU_MAIN_VIEW;
	memset(textSelect, 0, sizeof(textSelect));
	textSelect[0] = 1;
	
}
void ClickDown(UIObject *obj){}
void TextDown2(UIObject *obj)
{
	g_displayMode = MENU_SBS;
	memset(textSelect, 0, sizeof(textSelect));
	textSelect[1] = 1;
}
void ClickDown2(UIObject *obj){}
void TextDown3(UIObject *obj)
{
	g_displayMode = MENU_GUN;	
	memset(textSelect, 0, sizeof(textSelect));
	textSelect[2] = 1;
}
void ClickDown3(UIObject *obj){}
void TextDown4(UIObject *obj)
{
	g_displayMode = MENU_BALL;	
	memset(textSelect, 0, sizeof(textSelect));
	textSelect[3] = 1;
}
void ClickDown4(UIObject *obj){}

void TextDown5(UIObject *obj)
{
	g_displayMode = MENU_CALIBRA_CAP;	
	memset(textSelect, 0, sizeof(textSelect));
	textSelect[4] = 1;
}
void ClickDown5(UIObject *obj){}

void TextDown6(UIObject *obj)
{
	g_displayMode = MENU_CALIBRA_RESULT;	
	memset(textSelect, 0, sizeof(textSelect));
	textSelect[5] = 1;
}
void ClickDown6(UIObject *obj){}

void TextDown7(UIObject *obj)
{
	g_workMode = HANDLE_LINK_MODE;
	memset(textSelect2, 0, sizeof(textSelect2));
	textSelect2[0] = 1;
}
void ClickDown7(UIObject *obj){}

void TextDown8(UIObject *obj)
{
	g_workMode = ONLY_BALL_MODE;
	memset(textSelect2, 0, sizeof(textSelect2));
	textSelect2[1] = 1;
}
void ClickDown8(UIObject *obj){}
void TextDown9(UIObject *obj)
{
	g_workMode = AUTO_LINK_MODE;
	memset(textSelect2, 0, sizeof(textSelect2));
	textSelect2[2] = 1;
}
void ClickDown9(UIObject *obj){}


#endif
//=========================================================
FreeTypeFont*	_font_hd_big_st;
FreeTypeFont*	_font_hd_mid_st;
FreeTypeFont*	_font_hd_small_st;

FreeTypeFont*	_font_hd_big_ht;
FreeTypeFont*	_font_hd_mid_ht;
FreeTypeFont*	_font_hd_small_ht;

FreeTypeFont* _font_hd_IrisFocus_ht;

void OSDCreatText()
{
	_font_hd_big_st   =   new FreeTypeFont();
	_font_hd_big_st->create("simsun.ttc",40,512,512);

	_font_hd_mid_st   =   new FreeTypeFont();
	_font_hd_mid_st->create("simsun.ttc",30,512,512);	
	
	_font_hd_small_st   =   new FreeTypeFont();
	_font_hd_small_st->create("simsun.ttc",20,512,512);
	
	_font_hd_big_ht   =   new FreeTypeFont();
	_font_hd_big_ht->create("SIMLI.TTF",40,512,512);

	_font_hd_mid_ht   =   new FreeTypeFont();
	_font_hd_mid_ht->create("SIMLI.TTF",30,512,512);
	
	_font_hd_small_ht   =   new FreeTypeFont();
	_font_hd_small_ht->create("SIMLI.TTF",20,512,512);
	
	_font_hd_IrisFocus_ht = new FreeTypeFont();
	_font_hd_IrisFocus_ht->create("simsun.ttc",25,512,512);

}

FLOAT2 OSDdrawText(int x,int y,wchar_t* text,char font,char fontsize,int win_width,int win_height)
{
	FLOAT2 VerPos;
	FreeTypeFont* pTmp  = NULL;
	if(font == 0x02){
		if(fontsize == 0x03)
			pTmp = _font_hd_big_ht;
		else if(fontsize == 0x02)
			pTmp = _font_hd_mid_ht;
		else
			pTmp = _font_hd_small_ht;
	}else{
		if(fontsize == 0x03)
			pTmp = _font_hd_big_st;
		else if(fontsize == 0x02)
			pTmp = _font_hd_mid_st;
		else if(fontsize == 0x04)
			pTmp = _font_hd_IrisFocus_ht;
		else
			pTmp = _font_hd_small_st;
	}
	
	pTmp->begin(win_width,win_height);
	VerPos = pTmp->drawText(x,y,0,Rgba(255,255,255,255),text,0,0,0);
	pTmp->end();
	return VerPos;
}




//=========================================================
Vertex3F BMPVertex[] = {
		{ -1.0f, -1.0f, 1.0f , 0, 0 },
		{ -1.0f, 1.0f, 1.0f , 0, 1 },
		{ 1.0f, 1.0f, 1.0f  , 1, 1 },
		{ 1.0f, -1.0f, 1.0f  , 1, 0 },			
};
RectColor SelectRect[] ={
	{ 10, 10,  0.0f,  0.0f, 1.0f, 1.0f },
    { 110, 10, 0.0f,  0.0f, 1.0f, 1.0f },
    { 10, 110, 0.0f,  0.0f, 1.0f, 1.0f },
    { 110, 110,0.0f,  0.0f, 1.0f, 1.0f }
};

void PrintMsDISPLAY( const char* text="" )
{
	static long long last = 0;
	long long Cur = getTickCount();
	if(last == 0){
		last = Cur;
		return ;
	}
	long long ms = 0;
	ms = ( (double)(Cur-last)/getTickFrequency() ) * 1000;
	if(*text != 0){
		printf("\r\n\r\n[RunTimme]++++++++++++++++++++ [%s] = %d ms\r\n\r\n", text, ms);
	}
	last = getTickCount();

}
#define HISTEN 0
#define CLAHEH 1
#define DARKEN 0

static CDisplayer *gThis = NULL;
extern int IrisAndFocusAndExit;
extern CMD_triangle cmd_triangle;
extern OSD_param m_osd;
extern CProcess* plat;

extern CamParameters g_camParams;


double capTime = 0;

GLint Uniform_tex_in = -1;
GLint Uniform_tex_osd = -1;
GLint Uniform_tex_pbo = -1;
GLint Uniform_osd_enable = -1;
GLint Uniform_mattrans = -1;
GLint Uniform_font_color = -1;
static GLfloat m_glmat44fTransDefault[16] ={
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f
};
static GLfloat m_glmat44fTransDefault2[16] ={
	1.1f, 0.0f, 0.0f, 0.0f,
	0.0f, 1.1f, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f
};

GLfloat _fontColor[4] = {1.0,1.0,1.0,1.0};

static GLfloat m_glvVertsDefault[8] = {-1.0f, 1.0f, 1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f};
static GLfloat m_glvTexCoordsDefault[8] = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};

osdbuffer_t disOsdBuf[32]={0};
//char disOsdBuf[32][128] = {0};
osdbuffer_t disOsdBufbak[32] = {0};
wchar_t disOsd[32][33];
wchar_t capNum[32];
void CDisplayer::linkage_init()
{
	displayMode = PREVIEW_MODE;
	gun_BMP = imread("gun.bmp");
	if(gun_BMP.empty())
		cout << "Open Gun BMP Failed!!! " << endl;
	else
		cout << "Open Gun BMP Success!!! " << endl;

	if(!gun_BMP.empty()) {
		gun_BMP.copyTo(plat->m_camCalibra->gun_fromBMP);
	}
	
	gun_UndistorMat.create(1080,1920,CV_8UC3);
	if(gun_UndistorMat.empty())
		cout << "Create gun_UndistorMat Failed !!" << endl;
	else
		cout << "Create gun_UndistorMat Success !!" << endl;
/*
	videonamex = 100;
	videonamey = 130;
*/
	timex = disOsdBuf[osdID_time].posx;
	timey = disOsdBuf[osdID_time].posy;
	videonamefs = 2;
}

CDisplayer::CDisplayer()
:captureBMP_channel(0),selected_PicIndex(0),m_renderCount(0),m_bRun(false),m_bFullScreen(false),m_bOsd(false),
 m_glProgram(0), m_bUpdateVertex(false),m_tmRender(0ul),m_waitSync(false),
 m_telapse(5.0), m_nSwapTimeOut(0),m_detectCorners(NULL)
{
	int i;
	gThis = this;
	memset(&m_initPrm, 0, sizeof(m_initPrm));
	memset(cuda_pbo_resource, 0, sizeof(cuda_pbo_resource));
	//memset(m_dev_src_rgb, 0, sizeof(m_dev_src_rgb));
	memset(m_cuStream, 0, sizeof(m_cuStream));
	memset(m_bEnh, 0, sizeof(m_bEnh));
	memset(m_Mmt, 0, sizeof(m_Mmt));
	for(i=0; i<DS_DC_CNT; i++)
		buffId_osd[i] = -1;
	memset(updata_osd, 0, sizeof(updata_osd));
	dismodchanagcount=0;
	tv_pribuffid0=-1;
	tv_pribuffid1=-1;
	tv_pribuffid2=-1;
	tv_pribuffid3=-1;
	fir_pribuffid=-1;
	freezeonece=0;

	frameCount = 0;
	frameRate = 0.0;

	linkage_init();
	g_sysParam->getSysParam().gunposition.leftUp.x = 1440;
	g_sysParam->getSysParam().gunposition.leftUp.y = 810;
	g_sysParam->getSysParam().gun_camera.col = 1920;
	g_sysParam->getSysParam().gun_camera.raw = 1080;
	g_sysParam->getSysParam().gunposition.general.width = 480;
	g_sysParam->getSysParam().gunposition.general.height = 270;
	g_sysParam->setGunSize(SingletonSysParam::ONE_4);
	g_sysParam->setGunPosition(SingletonSysParam::RU);
	savePic_once = false;
	showDetectCorners  = false;
	
	g_connectAction.CurCalibraCam = CAM_0;
	
/************************************Add 20181219**************************/
	for(i=0; i<DS_DC_CNT;	i++){
		pp[i] = 0;
		m_code[i] = -1;
	}
	m_initPrm.disSched =33;// 3.5;
/*************************************************************************/
	_bCornerDetect = false;
/************************************************************************/
	run_Mode._bRButton = false;
	run_Mode.showSubMenu = false;
	run_Mode._pSelect = NULL;
	run_Mode._LDown = NULL;
/*************************************************************************/
	setFontPosition(100, 100);	
	run_Mode._texts.clear();
	run_Mode.text1._pos = FLOAT2(fontPosX,fontPosY);
	run_Mode.text1.r = 0;
	run_Mode.text1.g = 0;
	run_Mode.text1.b = 255;
	run_Mode.text1.a = 255;
	run_Mode.text1._eventDown = TextDown;
	run_Mode.text1._clickDown = ClickDown;
	wcscpy(run_Mode.text1._text, L"主界面");

	run_Mode.text2._pos = FLOAT2(fontPosX,fontPosY+60);
	run_Mode.text2.r = 0;
	run_Mode.text2.g = 0;
	run_Mode.text2.b = 255;
	run_Mode.text2.a = 255;
	run_Mode.text2._eventDown = TextDown2;
	run_Mode.text2._clickDown = ClickDown2;
	wcscpy(run_Mode.text2._text, L"标定界面");

	run_Mode.text3._pos = FLOAT2(fontPosX,fontPosY+120);
	run_Mode.text3.r = 0;
	run_Mode.text3.g = 0;
	run_Mode.text3.b = 255;
	run_Mode.text3.a = 255;
	run_Mode.text3._eventDown = TextDown3;
	run_Mode.text3._clickDown = ClickDown3;
	wcscpy(run_Mode.text3._text, L"枪机画面");

	run_Mode.text4._pos = FLOAT2(fontPosX,fontPosY+180);
	run_Mode.text4.r = 0;
	run_Mode.text4.g = 0;
	run_Mode.text4.b = 255;
	run_Mode.text4.a = 255;
	run_Mode.text4._eventDown = TextDown4;
	run_Mode.text4._clickDown = ClickDown4;
	wcscpy(run_Mode.text4._text, L"球机画面");

	run_Mode.text5._pos = FLOAT2(fontPosX,fontPosY+240);
	run_Mode.text5.r = 0;
	run_Mode.text5.g = 0;
	run_Mode.text5.b = 255;
	run_Mode.text5.a = 255;
	run_Mode.text5._eventDown = TextDown5;
	run_Mode.text5._clickDown = ClickDown5;
	wcscpy(run_Mode.text5._text, L"拍摄标定图片");

	run_Mode.text6._pos = FLOAT2(fontPosX,fontPosY+300);
	run_Mode.text6.r = 0;
	run_Mode.text6.g = 0;
	run_Mode.text6.b = 255;
	run_Mode.text6.a = 255;
	run_Mode.text6._eventDown = TextDown6;
	run_Mode.text6._clickDown = ClickDown6;
	wcscpy(run_Mode.text6._text, L"显示标定结果");

	run_Mode.text7._pos = FLOAT2(fontPosX,fontPosY+360);
	run_Mode.text7.r = 0;
	run_Mode.text7.g = 0;
	run_Mode.text7.b = 255;
	run_Mode.text7.a = 255;
	run_Mode.text7._eventDown = TextDown7;
	run_Mode.text7._clickDown = ClickDown7;
	wcscpy(run_Mode.text7._text, L"手动联动模式");

	run_Mode.text8._pos = FLOAT2(fontPosX,fontPosY+420);
	run_Mode.text8.r = 0;
	run_Mode.text8.g = 0;
	run_Mode.text8.b = 255;
	run_Mode.text8.a = 255;
	run_Mode.text8._eventDown = TextDown8;
	run_Mode.text8._clickDown = ClickDown8;
	wcscpy(run_Mode.text8._text, L"球机单控模式");

	run_Mode.text9._pos = FLOAT2(fontPosX,fontPosY+480);
	run_Mode.text9.r = 0;
	run_Mode.text9.g = 0;
	run_Mode.text9.b = 255;
	run_Mode.text9.a = 255;
	run_Mode.text9._eventDown = TextDown9;
	run_Mode.text9._clickDown = ClickDown9;
	wcscpy(run_Mode.text9._text, L"自动联动模式");

	run_Mode._texts.push_back(run_Mode.text1);
	run_Mode.text1._SN = run_Mode._texts.size() -1;	
	
	run_Mode._texts.push_back(run_Mode.text2);
	run_Mode.text2._SN = run_Mode._texts.size() -1;	
	
	run_Mode._texts.push_back(run_Mode.text3);
	run_Mode.text3._SN = run_Mode._texts.size() -1;	
	
	run_Mode._texts.push_back(run_Mode.text4);
	run_Mode.text4._SN = run_Mode._texts.size() -1;
	
	run_Mode._texts.push_back(run_Mode.text5);
	run_Mode.text5._SN = run_Mode._texts.size() -1;

	run_Mode._texts.push_back(run_Mode.text6);
	run_Mode.text6._SN = run_Mode._texts.size() -1;

	run_Mode.workMode.push_back(run_Mode.text7);
	run_Mode.text7._SN = run_Mode.workMode.size() -1;

	run_Mode.workMode.push_back(run_Mode.text8);
	run_Mode.text8._SN = run_Mode.workMode.size() -1;

	run_Mode.workMode.push_back(run_Mode.text9);
	run_Mode.text9._SN = run_Mode.workMode.size() -1;
}

CDisplayer::~CDisplayer()
{
	//destroy();
	gThis = NULL;
}

inline int extractYUYV2Gray2(Mat src, Mat dst)
{
	int ImgHeight, ImgWidth,ImgStride;

	ImgWidth = src.cols;
	ImgHeight = src.rows;
	ImgStride = ImgWidth*2;
	uint8_t  *  pDst8_t;
	uint8_t *  pSrc8_t;

	pSrc8_t = (uint8_t*)(src.data);
	pDst8_t = (uint8_t*)(dst.data);

	for(int y = 0; y < ImgHeight*ImgWidth; y++)
	{
		pDst8_t[y] = pSrc8_t[y*2];
	}
}

int CDisplayer::create()
{
	int i;
	unsigned char *d_src_rgb_novideo = NULL;
	cudaError_t et;
	memset(m_renders, 0, sizeof(m_renders));
	for(i=0; i<4; i++){
		memcpy(m_renders[i].transform, m_glmat44fTransDefault, sizeof(float)*16);
	}
	for(; i<DS_RENDER_MAX; i++){
		memcpy(m_renders[i].transform, m_glmat44fTransDefault2, sizeof(float)*16);
	}
	memset(m_videoSize, 0, sizeof(m_videoSize));

	for(i=0; i<DS_CHAN_MAX; i++){
		m_img[i].cols = 0;
		m_img[i].rows = 0;
	}
	m_img_novideo.cols=0;
	m_img_novideo.rows=0;

	for(i=0; i<DS_CUSTREAM_CNT; i++){
		et = cudaStreamCreate(&m_cuStream[i]);
		OSA_assert(et == cudaSuccess);
	}

	gl_create();

	for(i=0; i<DS_CHAN_MAX; i++){
		Videoeable[i]=1;
	}
	//tvvideo=1;
	//firvideo=1;
	unsigned int byteCount = mallocwidth * mallocheight* 3 * sizeof(unsigned char);
	
	cudaMalloc_share((void**)&d_src_rgb_novideo, byteCount, 5 + DS_CHAN_MAX);
	m_img_novideo= cv::Mat(mallocheight,mallocwidth, CV_8UC3, d_src_rgb_novideo);

	cudaMalloc((void **)&d_src_rgb_novideo,byteCount);
	m_img_novideo= cv::Mat(mallocheight,mallocwidth, CV_8UC3, d_src_rgb_novideo);

	OSA_mutexCreate(&m_mutex);

	return 0;
}

int CDisplayer::destroy()
{
	int i;
	cudaError_t et;

	stop();

	gl_destroy();

	uninitRender();

	OSA_mutexDelete(&m_mutex);

	for(i=0; i<SHAREARRAY_CNT; i++)
		cudaFree_share(NULL, i);

	for(i=0; i<RESOURCE_CNT; i++){
		cudaResource_unmapBuffer(i);
		cudaResource_UnregisterBuffer(i);
	}

	for(i=0; i<DS_CUSTREAM_CNT; i++){
		if(m_cuStream[i] != NULL){
			et = cudaStreamDestroy(m_cuStream[i]);
			OSA_assert(et == cudaSuccess);
			m_cuStream[i] = NULL;
		}
	}

/*******************************************************/
	pthread_mutex_lock(&render_lock);
	pthread_cond_broadcast(&render_cond);
	pthread_mutex_unlock(&render_lock);

	pthread_mutex_destroy(&render_lock);
	pthread_cond_destroy(&render_cond);
/*******************************************************/
	return 0;
}

int CDisplayer::initRender(bool bInitBind)
{	
	

	m_renderCount = 2;
#if 0
	m_renders[0].croprect.x=0;
	m_renders[0].croprect.y=0;
	m_renders[0].croprect.w=0;
	m_renders[0].croprect.h=0;
#endif
	

	//m_renders[0].videodect=1;
	

	for(int chId=0; chId<DS_CHAN_MAX; chId++)
	{
		m_img[chId].cols =0 ;
		m_img[chId].rows=0;

		m_renders[chId].videodect=0;

		m_renders[chId].croprect.x=0;
		m_renders[chId].croprect.y=0;
		m_renders[chId].croprect.w=0;
		m_renders[chId].croprect.h=0;

		m_renders[chId].video_chId = -1;
		m_renders[chId].displayrect.x = 0;
		m_renders[chId].displayrect.y = 0;
		m_renders[chId].displayrect.w = VIDEO_DIS_WIDTH/2;
		m_renders[chId].displayrect.h =  VIDEO_DIS_HEIGHT/2;

		m_renders[chId].bFreeze=0;
	}


	m_renders[0].video_chId = video_gaoqing;
	m_renders[0].displayrect.x = 0;
	m_renders[0].displayrect.y = 540;
	m_renders[0].displayrect.w = 960;
	m_renders[0].displayrect.h = 540;
	m_renders[0].videodect=1;
	
	m_renders[1].video_chId = video_gaoqing0;
	m_renders[1].displayrect.x = 960;
	m_renders[1].displayrect.y = 540;
	m_renders[1].displayrect.w = 960;
	m_renders[1].displayrect.h = 540;	
	m_renders[1].videodect=1;
	
	
	m_img_novideo.cols=0;
	m_img_novideo.rows=0;
	
	return 0;
}

void CDisplayer::uninitRender()
{
	m_renderCount = 0;
}

void CDisplayer::_display(void)
{
	static unsigned int count = 0;

	gThis->gl_textureLoad();
	gThis->gl_display();
	
	count ++;
}

void CDisplayer::_timeFunc(int value)
{
	if(!gThis->m_bRun){
		return ;
	}
	gThis->_display();
}

void CDisplayer::_reshape(int width, int height)
{
	assert(gThis != NULL);
	glViewport(0, 0, width, height);
	gThis->m_mainWinWidth_new[video_pal] = width;
	gThis->m_mainWinHeight_new[video_pal] = height;
	gThis->m_mainWinWidth_new[video_gaoqing0] = width;
	gThis->m_mainWinHeight_new[video_gaoqing0] = height;
	gThis->m_mainWinWidth_new[video_gaoqing] = width;
	gThis->m_mainWinHeight_new[video_gaoqing] = height;
	gThis->m_mainWinWidth_new[video_gaoqing2] = width;
	gThis->m_mainWinHeight_new[video_gaoqing2] = height;
	gThis->m_mainWinWidth_new[video_gaoqing3] = width;
	gThis->m_mainWinHeight_new[video_gaoqing3] = height;
	gThis->initRender(false);
	gThis->gl_updateVertex();
	gThis->gl_resize();
}

void CDisplayer::processLinkageMenu(int value)
{
	//printf("%s start, value=%d\n", __FUNCTION__, value);
	if(value == 1){
		g_sysParam->getSysParam().cameracalibrate.Enable_AutoDetectMoveTargets = true;
	}
	else if(value == 0){
		g_sysParam->getSysParam().cameracalibrate.Enable_AutoDetectMoveTargets = false;
	}else{
	}
	SENDST	tmp;
	tmp.cmd_ID = mtdmode;
	tmp.param[0] = value ;
	ipc_sendmsg(&tmp, IPC_FRIMG_MSG);
}

void CDisplayer::processDMMenu(int value)
{
	//printf("%s start, value=%d\n", __FUNCTION__, value);
	switch(value) {
		case 0:		
			g_displayMode = MENU_MAIN_VIEW;		
			break;
		case 1:
			g_displayMode = MENU_SBS;		
			break;
		case 2:
			g_displayMode = MENU_GUN;		
			break;
		case 3:
			g_displayMode = MENU_BALL;	
			break;
		default:
			break;

	}
	
}

void CDisplayer::processgunResMenu(int value)
{
	//printf("%s start, value=%d\n", __FUNCTION__, value);
}

void CDisplayer::processballResMenu(int value)
{
	//printf("%s start, value=%d\n", __FUNCTION__, value);
}

void CDisplayer::processposMenu(int value)
{
	//printf("%s start, value=%d\n", __FUNCTION__, value);
	if(g_sysParam->getSysParam().gun_camera.col != 0) {
		switch(value) {
			case 0: // LU
		#if 0
				g_sysParam->getSysParam().gunposition.leftUp.x = 0;
				g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw*3/4;
		#else
				if(g_sysParam->getGunSize(SingletonSysParam::ONE_4)== 1){					
					g_sysParam->getSysParam().gunposition.leftUp.x = 0;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw*3/4;
				}
				else if (g_sysParam->getGunSize(SingletonSysParam::ONE_3)== 1){					
					g_sysParam->getSysParam().gunposition.leftUp.x = 0;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw*2/3;

				}
				else
				{
					g_sysParam->getSysParam().gunposition.leftUp.x = 0;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw/2;
				}
		#endif
				g_sysParam->setGunPosition(SingletonSysParam::LU);
				break;
			case 1: // RU
				if(g_sysParam->getGunSize(SingletonSysParam::ONE_4)== 1){					
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col*3/4;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw*3/4;
				}
				else if (g_sysParam->getGunSize(SingletonSysParam::ONE_3)== 1){					
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col*2/3;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw*2/3;

				}
				else
				{
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col /2;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw/2;
				}
				g_sysParam->setGunPosition(SingletonSysParam::RU);
				break;
			case 2:
				g_sysParam->getSysParam().gunposition.leftUp.x = 0;
				g_sysParam->getSysParam().gunposition.leftUp.y = 0;
				g_sysParam->setGunPosition(SingletonSysParam::LD);
				break;
			case 3:
				if(g_sysParam->getGunSize(SingletonSysParam::ONE_4)== 1){					
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col*3/4;
					g_sysParam->getSysParam().gunposition.leftUp.y = 0;
				}
				else if (g_sysParam->getGunSize(SingletonSysParam::ONE_3)== 1){					
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col*2/3;
					g_sysParam->getSysParam().gunposition.leftUp.y = 0;

				}
				else
				{
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col /2;
					g_sysParam->getSysParam().gunposition.leftUp.y = 0;
				}
				g_sysParam->setGunPosition(SingletonSysParam::RD);
				break;
			default:
				break;
		}
	}
	
}

void CDisplayer::processsizeMenu(int value)
{
	//printf("%s start, value=%d\n", __FUNCTION__, value);
	if(g_sysParam->getSysParam().gun_camera.raw != 0) {
		switch(value) {
			case 0: //   1/2
				if(g_sysParam->getGunPosition(SingletonSysParam::LU) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = 0;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw/2;
				}else if(g_sysParam->getGunPosition(SingletonSysParam::RU) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col /2;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw /2;
				}
				else if(g_sysParam->getGunPosition(SingletonSysParam::LD) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = 0;
					g_sysParam->getSysParam().gunposition.leftUp.y = 0;
				}
				else if(g_sysParam->getGunPosition(SingletonSysParam::RD) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col /2;
					g_sysParam->getSysParam().gunposition.leftUp.y = 0;
				}
				g_sysParam->getSysParam().gunposition.general.width = g_sysParam->getSysParam().gun_camera.col*1/2;
				g_sysParam->getSysParam().gunposition.general.height= g_sysParam->getSysParam().gun_camera.raw*1/2;
				g_sysParam->setGunSize(SingletonSysParam::ONE_2);
				break;
			case 1:  //  1/3
				if(g_sysParam->getGunPosition(SingletonSysParam::LU) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = 0;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw*2/3;
				}else if(g_sysParam->getGunPosition(SingletonSysParam::RU) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col *2/3;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw *2/3;
				}
				else if(g_sysParam->getGunPosition(SingletonSysParam::LD) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = 0;
					g_sysParam->getSysParam().gunposition.leftUp.y = 0;
				}
				else if(g_sysParam->getGunPosition(SingletonSysParam::RD) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col *2/3;
					g_sysParam->getSysParam().gunposition.leftUp.y = 0;
				}
				g_sysParam->getSysParam().gunposition.general.width = g_sysParam->getSysParam().gun_camera.col*1/3;
				g_sysParam->getSysParam().gunposition.general.height= g_sysParam->getSysParam().gun_camera.raw*1/3;
				g_sysParam->setGunSize(SingletonSysParam::ONE_3);
				break;
				
			case 2:  //   1/4
			if(g_sysParam->getGunPosition(SingletonSysParam::LU) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = 0;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw*3/4;
				}else if(g_sysParam->getGunPosition(SingletonSysParam::RU) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col *3/4;
					g_sysParam->getSysParam().gunposition.leftUp.y = g_sysParam->getSysParam().gun_camera.raw *3/4;
				}
				else if(g_sysParam->getGunPosition(SingletonSysParam::LD) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = 0;
					g_sysParam->getSysParam().gunposition.leftUp.y = 0;
				}
				else if(g_sysParam->getGunPosition(SingletonSysParam::RD) == 1){
					g_sysParam->getSysParam().gunposition.leftUp.x = g_sysParam->getSysParam().gun_camera.col *3/4;
					g_sysParam->getSysParam().gunposition.leftUp.y = 0;
				}
				g_sysParam->getSysParam().gunposition.general.width = g_sysParam->getSysParam().gun_camera.col*1/4;
				g_sysParam->getSysParam().gunposition.general.height= g_sysParam->getSysParam().gun_camera.raw*1/4;
				g_sysParam->setGunSize(SingletonSysParam::ONE_4);
				break;
			default:
				break;
		}
	}

}

void CDisplayer::processnameMenu(int value)
{
	const int _osdID = 31;
	CMD_EXT *extInCtrl = (CMD_EXT*)ipc_getimgstatus_p();
	extInCtrl->osdTextSize = gThis->videonamefs;

	if(0 == value)
	{
		disOsdBuf[osdID_name].ctrl = 0;
	}
	else if(1 == value)
	{
		disOsdBuf[osdID_name].ctrl = 1;
	}
#if 0
	disarr.color = 2;
	disarr.osdID = 31;
	disarr.posx = gThis->videonamex;
	disarr.posy = gThis->videonamey;
	disarr.alpha= 0;
	sprintf((char *)disarr.buf, "Tracker  ShangHai");
#endif

	//memcpy(&disOsdBuf[31],&disarr,sizeof(disarr));
	setlocale(LC_ALL, "zh_CN.UTF-8");
	memcpy(&disOsdBufbak[osdID_name],&disOsdBuf[osdID_name],sizeof(osdbuffer_t));
	swprintf(disOsd[osdID_name], 33, L"%s", disOsdBuf[osdID_name].buf);

	gThis->sendIPC_Videoname(value);
}

void CDisplayer::sendIPC_Videoname(int value)
{
	osdtext_t* osd_ipc = ipc_getosdtextstatus_p();
	osd_ipc->osdID[osdID_name] = osdID_name;

	if(0 == value)
	{
		osd_ipc->ctrl[osdID_name] = 0;
	}
	else if(1 == value)
	{
		osd_ipc->ctrl[osdID_name] = 1;
	}

	SENDST test = {0};
	test.cmd_ID = read_shm_osdtext;
	test.param[0] = osdID_name;
	ipc_sendmsg(&test, IPC_FRIMG_MSG);
}

void CDisplayer::processfontsizeMenu(int value)
{
	CMD_EXT *extInCtrl = (CMD_EXT*)ipc_getimgstatus_p();
	OSDSTATUS *osdtmp = ipc_getosdstatus_p();

	if(0 == value)
		gThis->videonamefs = 1;
	else if(1 == value)
		gThis->videonamefs = 2;
	else if(2 == value)
		gThis->videonamefs = 3;

	osdtmp->OSD_text_size = extInCtrl->osdTextSize = gThis->videonamefs;

	SENDST test = {0};
	test.cmd_ID = ipcwordSize;
	test.param[0] = osdtmp->OSD_text_size;
	ipc_sendmsg(&test, IPC_FRIMG_MSG);

}

void CDisplayer::processosdposMenu(int value)
{

	if(0 == value)
	{
		gThis->timex = 100;
		gThis->timey = 100;
		gThis->videonamex = 100;
		gThis->videonamey = 130;
	}
	else if(1 == value)
	{
		gThis->timex = 1550;
		gThis->timey = 100;
		gThis->videonamex = 1550;
		gThis->videonamey = 130;
		
	}

	gThis->sendIPC_Time_pos();
	struct timeval tmp;
	tmp.tv_sec = 0;
	tmp.tv_usec = 5000;
	select(0, NULL, NULL, NULL, &tmp);
	gThis->sendIPC_VideoName_pos();

}

void CDisplayer::sendIPC_VideoName_pos()
{
	osdtext_t* osd_ipc = ipc_getosdtextstatus_p();

	osd_ipc->osdID[osdID_name] = disOsdBuf[osdID_name].osdID = osdID_name;
	osd_ipc->posx[osdID_name] = disOsdBuf[osdID_name].posx= gThis->videonamex;
	osd_ipc->posy[osdID_name] = disOsdBuf[osdID_name].posy= gThis->videonamey;

	setlocale(LC_ALL, "zh_CN.UTF-8");
	memcpy(&disOsdBufbak[osdID_name],&disOsdBuf[osdID_name],sizeof(osdbuffer_t));
	swprintf(disOsd[osdID_name], 33, L"%s", disOsdBuf[osdID_name].buf);

	SENDST test = {0};
	test.cmd_ID = read_shm_osdtext;
	test.param[0] = osdID_name;
	ipc_sendmsg(&test, IPC_FRIMG_MSG);
}

void CDisplayer::sendIPC_Time_pos()
{
	osdtext_t* osd_ipc = ipc_getosdtextstatus_p();

	osd_ipc->osdID[osdID_time] = disOsdBuf[osdID_time].osdID = osdID_time;
	osd_ipc->posx[osdID_time] = disOsdBuf[osdID_time].posx= gThis->timex;
	osd_ipc->posy[osdID_time] = disOsdBuf[osdID_time].posy= gThis->timey;

	setlocale(LC_ALL, "zh_CN.UTF-8");
	memcpy(&disOsdBufbak[osdID_time],&disOsdBuf[osdID_time],sizeof(osdbuffer_t));
	swprintf(disOsd[osdID_time], 33, L"%s", disOsdBuf[osdID_time].buf);

	SENDST test = {0};
	test.cmd_ID = read_shm_osdtext;
	test.param[0] = osdID_time;
	ipc_sendmsg(&test, IPC_FRIMG_MSG);
}

void CDisplayer::processbuadrateMenu(int value)
{
	SENDST test;
	CMD_SETCONFIG cmdsetconfig;
	int send_flag = 1;

	test.cmd_ID = setconfig;
	cmdsetconfig.block = 52;
	cmdsetconfig.field = 0;
	switch(value)
	{
		case 0:
			cmdsetconfig.value= 1200;
			break;
		case 1:
			cmdsetconfig.value = 2400;
			break;
		case 2:
			cmdsetconfig.value = 4800;
			break;
		case 3:
			cmdsetconfig.value = 9600;
			break;
		case 4:
			cmdsetconfig.value = 19200;
			break;
		case 5:
			cmdsetconfig.value = 38400;
			break;
		case 6:
			cmdsetconfig.value = 57600;
			break;
		case 7:
			cmdsetconfig.value = 115200;
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

void CDisplayer::processdatabitMenu(int value)
{
	SENDST test;
	CMD_SETCONFIG cmdsetconfig;
	int send_flag = 1;

	test.cmd_ID = setconfig;
	cmdsetconfig.block = 52;
	cmdsetconfig.field = 1;
	switch(value)
	{
		case 0:
			cmdsetconfig.value = 5;
			break;
		case 1:
			cmdsetconfig.value = 6;
			break;
		case 2:
			cmdsetconfig.value = 7;
			break;
		case 3:
			cmdsetconfig.value = 8;
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

void CDisplayer::processstopbitMenu(int value)
{
	SENDST test;
	CMD_SETCONFIG cmdsetconfig;
	int send_flag = 1;

	test.cmd_ID = setconfig;
	cmdsetconfig.block = 52;
	cmdsetconfig.field = 2;
	switch(value)
	{
		case 0:
			cmdsetconfig.value = 1;
			break;
		case 1:
			cmdsetconfig.value = 1.5;
			break;
		case 2:
			cmdsetconfig.value = 2;
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

void CDisplayer::processparityMenu(int value)
{
	SENDST test;
	CMD_SETCONFIG cmdsetconfig;
	int send_flag = 1;

	test.cmd_ID = setconfig;
	cmdsetconfig.block = 52;
	cmdsetconfig.field = 3;
	switch(value)
	{
		case 0:
			cmdsetconfig.value = 0;
			break;
		case 1:
			cmdsetconfig.value = 1;
			break;
		case 2:
			cmdsetconfig.value = 2;
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

void CDisplayer::processaddressMenu(int value)
{
	SENDST test;
	CMD_SETCONFIG cmdsetconfig;
	int send_flag = 1;

	test.cmd_ID = setconfig;
	cmdsetconfig.block = 52;
	cmdsetconfig.field = 4;
	switch(value)
	{
		case 0:
			cmdsetconfig.value = 0;
			break;
		case 1:
			cmdsetconfig.value = 1;
			break;
		case 2:
			cmdsetconfig.value = 2;
			break;
		case 3:
			cmdsetconfig.value = 3;
			break;
		case 4:
			cmdsetconfig.value = 4;
			break;
		case 5:
			cmdsetconfig.value = 5;
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

void CDisplayer::processprotocolMenu(int value)
{
	SENDST test;
	CMD_SETCONFIG cmdsetconfig;
	int send_flag = 1;

	test.cmd_ID = setconfig;
	cmdsetconfig.block = 52;
	cmdsetconfig.field = 5;
	switch(value)
	{
		case 0:
			cmdsetconfig.value = 0;
			break;
		case 1:
			cmdsetconfig.value = 1;
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

void CDisplayer::processSenMenu(int value)
{
	printf("%s start, value=%d\n", __FUNCTION__, value);
}

void CDisplayer::processtargetspeedMenu(int value)
{
	printf("%s start, value=%d\n", __FUNCTION__, value);
}

void CDisplayer::processtargetdircMenu(int value)
{
	printf("%s start, value=%d\n", __FUNCTION__, value);
}

void CDisplayer::processdetectcondMenu(int value)
{
	printf("%s start, value=%d\n", __FUNCTION__, value);
}

void CDisplayer::processpolarMenu(int value)
{
	SENDST test = {0};
	
	test.cmd_ID = mtdpolar;
	if(0 == value)
		test.param[0] = 0;
	else if(1 == value)
		test.param[0] = 1;
	
	ipc_sendmsg(&test, IPC_FRIMG_MSG);
}

void CDisplayer::processdurationMenu(int value)
{
	SENDST test = {0};
	CMD_MTDTRKTIME mtdtime;
	
	test.cmd_ID = mtdtrktime;
	if(0 == value)
		mtdtime.seconds = 1;
	else if(1 == value)
		mtdtime.seconds = 3;
	else if(2 == value)
		mtdtime.seconds = 5;
	else if(3 == value)
		mtdtime.seconds = 7;
	else if(4 == value)
		mtdtime.seconds = 9;
	
	memcpy(test.param, &mtdtime, sizeof(mtdtime));
	ipc_sendmsg(&test, IPC_FRIMG_MSG);
		
}

void CDisplayer::processmtdmodeMenu(int value)
{
	SENDST test = {0};
	
	test.cmd_ID = mtdmode;
	if(0 == value)
		test.param[0] = 0;
	else if(1 == value)
		test.param[0] = 1;
	
	ipc_sendmsg(&test, IPC_FRIMG_MSG);
}

void CDisplayer::processredetectMenu(int value)
{
	SENDST test = {0};
	
	test.cmd_ID = mtdredetect;
	if(0 == value)
		test.param[0] = 0;
	else if(1 == value)
		test.param[0] = 1;
	
	ipc_sendmsg(&test, IPC_FRIMG_MSG);
}

void CDisplayer::processalarmputMenu(int value)
{
	SENDST test = {0};
	
	test.cmd_ID = mtdoutput;
	if(0 == value)
		test.param[0] = 0;
	else if(1 == value)
		test.param[0] = 1;
	
	ipc_sendmsg(&test, IPC_FRIMG_MSG);
}

void CDisplayer::gl_resize()
{
}

void CDisplayer::_close(void)
{
	if(gThis->m_initPrm.closefunc != NULL)
		gThis->m_initPrm.closefunc();
}

int CDisplayer::init(DS_InitPrm *pPrm)
{
#if 0   // Removed By swj 20181219
		//for tv buffer
	   tskSendBufCreatetv0.numBuf = PICBUFFERCOUNT;
       for (int i = 0; i < tskSendBufCreatetv0.numBuf; i++)
       {
           cudaMalloc((void **)&tskSendBufCreatetv0.bufVirtAddr[i],mallocwidth*mallocheight*3);
           OSA_assert(tskSendBufCreatetv0.bufVirtAddr[i] != NULL);
       }
       OSA_bufCreate(&tskSendBuftv0, &tskSendBufCreatetv0);
       tskSendBufCreatetv1.numBuf = PICBUFFERCOUNT;
       for (int i = 0; i < tskSendBufCreatetv1.numBuf; i++)
       {
           cudaMalloc((void **)&tskSendBufCreatetv1.bufVirtAddr[i],mallocwidth*mallocheight*3);
           OSA_assert(tskSendBufCreatetv1.bufVirtAddr[i] != NULL);
       }
       OSA_bufCreate(&tskSendBuftv1, &tskSendBufCreatetv1);

	   tskSendBufCreatetv2.numBuf = PICBUFFERCOUNT;
       for (int i = 0; i < tskSendBufCreatetv2.numBuf; i++)
       {
           cudaMalloc((void **)&tskSendBufCreatetv2.bufVirtAddr[i],mallocwidth*mallocheight*3);
           OSA_assert(tskSendBufCreatetv2.bufVirtAddr[i] != NULL);
       }
       OSA_bufCreate(&tskSendBuftv2, &tskSendBufCreatetv2);

	   tskSendBufCreatetv3.numBuf = PICBUFFERCOUNT;
       for (int i = 0; i < tskSendBufCreatetv3.numBuf; i++)
       {
           cudaMalloc((void **)&tskSendBufCreatetv3.bufVirtAddr[i],mallocwidth*mallocheight*3);
           OSA_assert(tskSendBufCreatetv3.bufVirtAddr[i] != NULL);
       }
       OSA_bufCreate(&tskSendBuftv3, &tskSendBufCreatetv3);
	   

	   tskSendBufCreatefir.numBuf = PICBUFFERCOUNT;
       for (int i = 0; i < tskSendBufCreatefir.numBuf; i++)
       {
           cudaMalloc((void **)&tskSendBufCreatefir.bufVirtAddr[i],mallocwidth*mallocheight*3);
           OSA_assert(tskSendBufCreatefir.bufVirtAddr[i] != NULL);
       }
       OSA_bufCreate(&tskSendBuffir, &tskSendBufCreatefir);

	   tskSendBufCreatepal.numBuf = PICBUFFERCOUNT;
       for (int i = 0; i < tskSendBufCreatepal.numBuf; i++)
       {
           cudaMalloc((void **)&tskSendBufCreatepal.bufVirtAddr[i],mallocwidth*mallocheight*3);
           OSA_assert(tskSendBufCreatepal.bufVirtAddr[i] != NULL);
          
       }
       OSA_bufCreate(&tskSendBufpal, &tskSendBufCreatepal);
#endif

	if(pPrm != NULL)
		memcpy(&m_initPrm, pPrm, sizeof(DS_InitPrm));
	
	if(m_initPrm.timerInterval<=0)
		m_initPrm.timerInterval = 40;
/*******************************************************Add 20181219*****************/
	if(m_initPrm.disFPS<=0)
		m_initPrm.disFPS = 25;
	pthread_mutex_init(&render_lock, NULL);
	pthread_cond_init(&render_cond, NULL);
	setFPS(m_initPrm.disFPS);

/*************************************************************************************/
    //glutInitWindowPosition(m_initPrm.winPosX, m_initPrm.winPosY);
    glutInitWindowSize(VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
    glutCreateWindow("DSS");
	//glutSetCursor(GLUT_CURSOR_NONE);
	glutDisplayFunc(&_display);
	if(m_initPrm.idlefunc != NULL)
		glutIdleFunc(m_initPrm.idlefunc);
	glutReshapeFunc(_reshape);

	//
	if(m_initPrm.keyboardfunc != NULL)
		glutKeyboardFunc(m_initPrm.keyboardfunc);
	if(m_initPrm.keySpecialfunc != NULL)
		glutSpecialFunc(m_initPrm.keySpecialfunc);

	//mouse event:
	if(m_initPrm.mousefunc != NULL)
		glutMouseFunc(m_initPrm.mousefunc);//GLUT_LEFT_BUTTON GLUT_MIDDLE_BUTTON GLUT_RIGHT_BUTTON; GLUT_DOWN GLUT_UP
	if(m_initPrm.passivemotionfunc != NULL)
		glutPassiveMotionFunc(m_initPrm.passivemotionfunc);
	
	if(m_initPrm.menufunc != NULL)
	{
		int sub_menu = glutCreateMenu(processLinkageMenu);
		glutAddMenuEntry("Prohibit",0);
		glutAddMenuEntry("Allow",1);



		int sub_menu2 = glutCreateMenu(processDMMenu);
		glutAddMenuEntry("PIP",0);
		glutAddMenuEntry("SBS",1);
		glutAddMenuEntry("GUN FULLSCREEN",2);
		glutAddMenuEntry("BALL FULLSCREEN",3);



		int gunres_submenu = glutCreateMenu(processgunResMenu);
		glutAddMenuEntry("1080P@25HZ",0);
		glutAddMenuEntry("1080P@30HZ",1);
		glutAddMenuEntry("720P@50HZ",2);
		glutAddMenuEntry("720P@60HZ",3);
		int ballres_submenu = glutCreateMenu(processballResMenu);
		glutAddMenuEntry("1080P@25HZ",0);
		glutAddMenuEntry("1080P@30HZ",1);
		glutAddMenuEntry("720P@50HZ",2);
		glutAddMenuEntry("720P@60HZ",3);
		int sen_submenu = glutCreateMenu(NULL);
		glutAddSubMenu("Gun Camera",gunres_submenu);
		glutAddSubMenu("Ball Camera",ballres_submenu);
		
		int pos_submenu = glutCreateMenu(processposMenu);
		glutAddMenuEntry("LU",0);
		glutAddMenuEntry("RU",1);
		glutAddMenuEntry("LD",2);
		glutAddMenuEntry("RD",3);
		int size_submenu = glutCreateMenu(processsizeMenu);
		glutAddMenuEntry("1/2",0);
		glutAddMenuEntry("1/3",1);
		glutAddMenuEntry("1/4",2);
		int dis_submenu = glutCreateMenu(NULL);
		glutAddSubMenu("PIP Position",pos_submenu);
		glutAddSubMenu("PIP Size",size_submenu);

		int name_submenu = glutCreateMenu(processnameMenu);
		glutAddMenuEntry("Enable",0);
		glutAddMenuEntry("Disable",1);
		int time_submenu = glutCreateMenu(m_initPrm.timefunc);
		glutAddMenuEntry("Enable",0);
		glutAddMenuEntry("Disable",1);
		int fontsize_submenu = glutCreateMenu(processfontsizeMenu);
		glutAddMenuEntry("Small",0);
		glutAddMenuEntry("Middle",1);
		glutAddMenuEntry("Big",2);
		int osdpos_submenu = glutCreateMenu(processosdposMenu);
		glutAddMenuEntry("LU",0);
		glutAddMenuEntry("RU",1);
		int osd_submenu = glutCreateMenu(NULL);
		glutAddSubMenu("Video Name",name_submenu);
		glutAddSubMenu("Time",time_submenu);
		glutAddSubMenu("Font Size",fontsize_submenu);
		glutAddSubMenu("OSD Position",osdpos_submenu);

		int manualcarli_submenu = glutCreateMenu(m_initPrm.manualcarli);
		glutAddMenuEntry("On",0);
		glutAddMenuEntry("Screentshot",1);
		glutAddMenuEntry("Carlibration",2);
		glutAddMenuEntry("Save Parameter",3);
		glutAddMenuEntry("Close Calibrate",4);
		int autocarli_submenu = glutCreateMenu(m_initPrm.autocarli);
		glutAddMenuEntry("On",0);
		glutAddMenuEntry("Screentshot",1);
		glutAddMenuEntry("Carlibration",2);
		glutAddMenuEntry("Save Parameter",3);
		glutAddMenuEntry("Close Calibrate",4);
		
		int car_submenu = glutCreateMenu(NULL);
		glutAddSubMenu("Manual Carlibration",manualcarli_submenu);
		glutAddSubMenu("Auto Carlibration",autocarli_submenu);
		

		int rig_submenu = glutCreateMenu(m_initPrm.setrigion);
		glutAddMenuEntry("Rigion1",0);
		int rigsel_submenu = glutCreateMenu(m_initPrm.rigionsel);
		glutAddMenuEntry("Rigion1",0);
		int rigpolygon_submenu = glutCreateMenu(m_initPrm.rigionpolygon);
		glutAddMenuEntry("Rigion1",0);
		int tminsi_submenu = glutCreateMenu(m_initPrm.minsize);
		glutAddMenuEntry("100",0);
		glutAddMenuEntry("1000",1);
		int tmaxsi_submenu = glutCreateMenu(m_initPrm.maxsize);
		glutAddMenuEntry("40000",0);
		glutAddMenuEntry("50000",1);
		int tsp_submenu = glutCreateMenu(processtargetspeedMenu);
		glutAddMenuEntry("Speed1",0);
		glutAddMenuEntry("Speed2",1);
		int td_submenu = glutCreateMenu(processtargetdircMenu);
		glutAddMenuEntry("Direction1",0);
		glutAddMenuEntry("Direction2",1);
		glutAddMenuEntry("Direction3",2);
		glutAddMenuEntry("Direction4",3);
		int maxnum_submenu = glutCreateMenu(m_initPrm.maxnum);
		glutAddMenuEntry("5",0);
		glutAddMenuEntry("10",1);
		int dc_submenu = glutCreateMenu(processdetectcondMenu);
		glutAddMenuEntry("Condition1",0);
		glutAddMenuEntry("Condition2",1);
		int output_submenu = glutCreateMenu(processalarmputMenu);
		glutAddMenuEntry("Disable",0);
		glutAddMenuEntry("Enable",1);
		int polar_submenu = glutCreateMenu(processpolarMenu);
		glutAddMenuEntry("+",0);
		glutAddMenuEntry("-",1);
		int dur_submenu = glutCreateMenu(processdurationMenu);
		glutAddMenuEntry("1s",0);
		glutAddMenuEntry("3s",1);
		glutAddMenuEntry("5s",2);
		glutAddMenuEntry("7s",3);
		glutAddMenuEntry("9s",4);
		int mtd_submenu = glutCreateMenu(NULL);
		glutAddSubMenu("Rigion Set",rig_submenu);
		glutAddSubMenu("Rigion Select",rigsel_submenu);
		glutAddSubMenu("Rigion Set Polygon",rigpolygon_submenu);
		glutAddSubMenu("Detect Condition",dc_submenu);
#if __MOVE_DETECT__
		glutAddSubMenu("Max Num",maxnum_submenu);
		glutAddSubMenu("Minimum Target Size",tminsi_submenu);
		glutAddSubMenu("Maximum Target Size",tmaxsi_submenu);
#endif
		glutAddSubMenu("Target Speed",tsp_submenu);
		glutAddSubMenu("Target Direction",td_submenu);
		glutAddSubMenu("Alarm Output",output_submenu);
		glutAddSubMenu("Output Polar",polar_submenu);
		glutAddSubMenu("Duration",dur_submenu);
		int buad_submenu = glutCreateMenu(processbuadrateMenu);
		glutAddMenuEntry("1200",0);
		glutAddMenuEntry("2400",1);
		glutAddMenuEntry("4800",2);
		glutAddMenuEntry("9600",3);
		glutAddMenuEntry("19200",4);
		glutAddMenuEntry("38400",5);
		glutAddMenuEntry("57600",6);
		glutAddMenuEntry("115200",7);
		int data_submenu = glutCreateMenu(processdatabitMenu);
		glutAddMenuEntry("5",0);
		glutAddMenuEntry("6",1);
		glutAddMenuEntry("7",2);
		glutAddMenuEntry("8",3);
		int stop_submenu = glutCreateMenu(processstopbitMenu);
		glutAddMenuEntry("1",0);
		glutAddMenuEntry("1.5",1);
		glutAddMenuEntry("2",2);
		int par_submenu = glutCreateMenu(processparityMenu);
		glutAddMenuEntry("None",0);
		glutAddMenuEntry("Even",1);
		glutAddMenuEntry("Odd",2);
		int ip_submenu = glutCreateMenu(processaddressMenu);
		glutAddMenuEntry("0",0);
		glutAddMenuEntry("1",1);
		glutAddMenuEntry("2",2);
		glutAddMenuEntry("3",3);
		glutAddMenuEntry("4",4);
		glutAddMenuEntry("5",5);
		int pro_submenu = glutCreateMenu(processprotocolMenu);
		glutAddMenuEntry("Pelco-D",0);
		glutAddMenuEntry("Pelco-P",1);
		int com_submenu = glutCreateMenu(NULL);
		glutAddSubMenu("Buad Rate",buad_submenu);
		glutAddSubMenu("Data Bit",data_submenu);
		glutAddSubMenu("Stop Bit",stop_submenu);
		glutAddSubMenu("Parity",par_submenu);
		glutAddSubMenu("Address",ip_submenu);
		glutAddSubMenu("Protocol",pro_submenu);
		
		int sub_menu3 = glutCreateMenu(NULL);
		glutAddSubMenu("Sensor Source",sen_submenu);
		glutAddSubMenu("Display",dis_submenu);
		glutAddSubMenu("OSD",osd_submenu);
		glutAddSubMenu("Carlibration",car_submenu);
		glutAddSubMenu("MTD",mtd_submenu);
		glutAddSubMenu("Ball COM",com_submenu);



		glutCreateMenu(NULL);
		//glutCreateMenu(m_initPrm.menufunc);
		//glutAddMenuEntry("Sensor Switch",0);
		glutAddSubMenu("Auto Linkage Enable",sub_menu);
		glutAddSubMenu("Display Mode",sub_menu2);
		glutAddSubMenu("Setup",sub_menu3);
		glutAttachMenu(GLUT_RIGHT_BUTTON);
	}

	if(m_initPrm.visibilityfunc != NULL)
		glutVisibilityFunc(m_initPrm.visibilityfunc);
	if(m_initPrm.bFullScreen){
		glutFullScreen();
		m_bFullScreen = true;
	}

	glutCloseFunc(_close);
	
	initRender();

	gl_updateVertex();

	gl_init();

	gl_Loadinit();

	menu_init();

	return 0;
}

int CDisplayer::get_videoSize(int chId, DS_Size &size)
{
	if(chId < 0 || chId >= DS_CHAN_MAX)
		return -1;
	size = m_videoSize[chId];

	return 0;
}
int CDisplayer::dynamic_config(DS_CFG type, int iPrm, void* pPrm)
{
	int iRet = 0;
	int chId;
	bool bEnable;
	DS_Rect *rc;
	DS_fRect *frc;

	
	if(type == DS_CFG_ChId){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		chId = *(int*)pPrm;

		m_renders[iPrm].video_chId = chId;

		OSA_printf("%%%%%%%%%%%%render=%d  chid=%d\n",iPrm,chId);
	}
	
	if(type == DS_CFG_RenderPosRect){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		rc = (DS_Rect*)pPrm;
		OSA_mutexLock(&m_mutex);
		m_renders[iPrm].displayrect= *rc;
		OSA_mutexUnlock(&m_mutex);
		printf("the DS_CFG_RenderPosRect x=%d y=%d w=%d h=%d  pIStuts->PicpPosStat=%d\n",
			m_renders[iPrm].displayrect.x,m_renders[iPrm].displayrect.y,
			m_renders[iPrm].displayrect.w,m_renders[iPrm].displayrect.h,iPrm);
	}

	if(type == DS_CFG_EnhEnable){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		bEnable = *(bool*)pPrm;
		m_bEnh[iPrm] = bEnable;
		OSA_printf("the video enhanceiPrm=%d =%d\n",iPrm,m_bEnh[iPrm]);
	}

	if(type == DS_CFG_MMTEnable){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		bEnable = *(bool*)pPrm;
		m_Mmt[iPrm] = bEnable;
		OSA_printf("the video mmtEnableiPrm=%d =%d\n",iPrm,m_Mmt[iPrm]);
	}

	if(type == DS_CFG_CropEnable){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		bEnable = *(bool*)pPrm;
		m_renders[iPrm].bCrop = bEnable;
	}

	if(type == DS_CFG_CropRect){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		rc = (DS_Rect*)pPrm;
		m_renders[iPrm].croprect = *rc;

		gl_updateVertex();
	}

	if(type == DS_CFG_VideoTransMat){ //src transform
		if(iPrm >= DS_CHAN_MAX || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		memcpy(m_glmat44fTrans[iPrm], pPrm, sizeof(float)*16);
	}

	if(type == DS_CFG_ViewTransMat){ //dst transform
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		memcpy(m_renders[iPrm].transform , pPrm, sizeof(float)*16);
		gl_updateVertex();
	}

	if(type == DS_CFG_BindRect){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		frc = (DS_fRect*)pPrm;
		m_renders[iPrm].bindrect = *frc;
		//initRender(false);
		gl_updateVertex();
	}

	if(type == DS_CFG_FreezeEnable){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		bEnable = *(bool*)pPrm;
		m_renders[iPrm].bFreeze = bEnable;
	}

	if(type == DS_CFG_VideodetEnable){
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		bEnable = *(bool*)pPrm;
		m_renders[iPrm].videodect = bEnable;
		//printf("DS_CFG_VideodetEnableiPrm=%d status=%d\n",iPrm,bEnable);
	}

	if(type ==DS_CFG_Renderdisplay)
		{
		if(iPrm >= m_renderCount || iPrm < 0)
			return -1;
		if(pPrm == NULL)
			return -2;
		bEnable = *(bool*)pPrm;
		Videoeable[iPrm]=bEnable;
		}

	if(type ==DS_CFG_Rendercount)
		{

		if(iPrm >= DS_RENDER_MAX || iPrm < 0)
			return -1;

		m_renderCount=iPrm;
		}

	return iRet;
}

extern "C" int yuyv2bgr_(
	unsigned char *dst, const unsigned char *src,
	int width, int height, cudaStream_t stream);

extern "C" int uyvy2bgr_(
	unsigned char *dst, const unsigned char *src,
	int width, int height, cudaStream_t stream);


void extractYUYV2Gray(Mat src, Mat dst)
{
	int ImgHeight, ImgWidth,ImgStride, stride16x8;

	ImgWidth = src.cols;
	ImgHeight = src.rows;
	ImgStride = ImgWidth*2;
	stride16x8 = ImgStride/16;

	OSA_assert((ImgStride&15)==0);
//#pragma omp parallel for
	for(int y = 0; y < ImgHeight; y++)
	{
		uint8x8_t  * __restrict__ pDst8x8_t;
		uint8_t * __restrict__ pSrc8_t;
		pSrc8_t = (uint8_t*)(src.data+ ImgStride*y);
		pDst8x8_t = (uint8x8_t*)(dst.data+ ImgWidth*y);
		for(int x=0; x<stride16x8; x++)
		{
			uint8x8x2_t d;
			d = vld2_u8((uint8_t*)(pSrc8_t+16*x));
			pDst8x8_t[x] = d.val[0];
		}
	}
}


void extractUYVY2Gray1(Mat src, Mat dst)
{
	int ImgHeight, ImgWidth,ImgStride;

	ImgWidth = src.cols;
	ImgHeight = src.rows;
	ImgStride = ImgWidth*2;
	uint8_t  *  pDst8_t;
	uint8_t *  pSrc8_t;

	pSrc8_t = (uint8_t*)(src.data);
	pDst8_t = (uint8_t*)(dst.data);
//#pragma UNROLL 4
//#pragma omp parallel for
	for(int y = 0; y < ImgHeight*ImgWidth; y++)
	{
		pDst8_t[y] = pSrc8_t[y*2+1];
	}
}
#if 0
void CDisplayer::display(Mat frame, int chId, int code)
{
	static int saveCount = 0;
	int i;
	int bufId=0;
	unsigned char *d_src = NULL;
	unsigned char *d_src_rgb = NULL;

	unsigned char tvbuffer0=0;
	unsigned char tvbuffer1=0;
	unsigned char tvbuffer2=0;
	unsigned char tvbuffer3=0;
	unsigned char firbuffer=0;
	unsigned char palbuffer=0;

	int nChannel = frame.channels();
	unsigned int byteCount = frame.rows * frame.cols * nChannel * sizeof(unsigned char);
	assert(chId>=0 && chId<DS_CHAN_MAX);

	if(disptimeEnable == 1){	
		//test zhou qi  time
		int64 captime = 0;

		captime = getTickCount();
		
		double curtime = (captime/getTickFrequency())*1000;///( (getTickCount() - trktime)/getTickFrequency())*1000;
		
		static double pretime = 0.0;
		capTime = curtime - pretime;
		pretime = curtime;
	}

/*
	cv::Mat frame_gray;
	frame_gray = Mat(frame.rows, frame.cols, CV_8UC1);

	extractYUYV2Gray2(frame, frame_gray);
	cv::imshow("111",frame_gray);
	cv::waitKey(1);
*/

if(chId == 0 && savePic_once == true){
		savePic_once = false;
		memset(savePicName, 0, 20);
		sprintf(savePicName,"%02d.bmp",saveCount);
		saveCount ++;
		Mat Dst(1080,1920,CV_8UC3);
		cvtColor(frame,Dst,CV_YUV2BGR_YUYV);		
		imwrite(savePicName,Dst);
		saveImageList.push_back(Dst);
}


	if(nChannel == 1 || code == -1){
		cudaMalloc_share((void**)&d_src_rgb, byteCount, chId + DS_CHAN_MAX);
		firbuffer= OSA_bufGetEmpty(&(tskSendBuffir), &bufId, OSA_TIMEOUT_NONE);
		if(firbuffer==0)
			d_src_rgb=(unsigned char *)tskSendBuffir.bufInfo[bufId].virtAddr;

		OSA_mutexLock(&m_mutex);

		cudaMemcpyAsync(d_src_rgb, frame.data, byteCount, cudaMemcpyHostToDevice,  m_cuStream[0]);
		m_img[chId] = cv::Mat(frame.rows, frame.cols, CV_8UC1, d_src_rgb);

		OSA_mutexUnlock(&m_mutex);

		if(firbuffer==0)
			OSA_bufPutFull(&(tskSendBuffir), bufId);

		cudaFree_share(d_src_rgb, chId + DS_CHAN_MAX);

	}else{
		unsigned int byteCount_rgb = frame.rows * frame.cols * 3* sizeof(unsigned char);
		cudaMalloc_share((void**)&d_src, byteCount, chId);
		cudaMalloc_share((void**)&d_src_rgb, byteCount_rgb, chId + DS_CHAN_MAX);
		if(chId == video_pal)
		{
			palbuffer= OSA_bufGetEmpty(&(tskSendBufpal), &bufId, OSA_TIMEOUT_NONE);
			if(palbuffer==0)
				d_src_rgb=(unsigned char *)tskSendBufpal.bufInfo[bufId].virtAddr;
		}
		else if(chId==video_gaoqing0)
		{
			tvbuffer0= OSA_bufGetEmpty(&(tskSendBuftv0), &bufId, OSA_TIMEOUT_NONE);
			if(tvbuffer0==0)
				d_src_rgb=(unsigned char *)tskSendBuftv0.bufInfo[bufId].virtAddr;
		}
		else if(chId==video_gaoqing)
		{
			tvbuffer1= OSA_bufGetEmpty(&(tskSendBuftv1), &bufId, OSA_TIMEOUT_NONE);
			if(tvbuffer1==0)
				d_src_rgb=(unsigned char *)tskSendBuftv1.bufInfo[bufId].virtAddr;
		}
		else if(chId==video_gaoqing2)
		{
			tvbuffer2= OSA_bufGetEmpty(&(tskSendBuftv2), &bufId, OSA_TIMEOUT_NONE);
			if(tvbuffer2==0)
				d_src_rgb=(unsigned char *)tskSendBuftv2.bufInfo[bufId].virtAddr;
		}
		else if(chId==video_gaoqing3)
		{
			tvbuffer3= OSA_bufGetEmpty(&(tskSendBuftv3), &bufId, OSA_TIMEOUT_NONE);
			if(tvbuffer3==0)
				d_src_rgb=(unsigned char *)tskSendBuftv3.bufInfo[bufId].virtAddr;
		}

		OSA_mutexLock(&m_mutex);


		cudaMemcpyAsync(d_src + (byteCount>>2)*0, frame.data + (byteCount>>2)*0, (byteCount>>2), cudaMemcpyHostToDevice, m_cuStream[0]);
		cudaMemcpyAsync(d_src + (byteCount>>2)*1, frame.data + (byteCount>>2)*1, (byteCount>>2), cudaMemcpyHostToDevice, m_cuStream[1]);
		cudaMemcpyAsync(d_src + (byteCount>>2)*2, frame.data + (byteCount>>2)*2, (byteCount>>2), cudaMemcpyHostToDevice, m_cuStream[2]);
		cudaMemcpyAsync(d_src + (byteCount>>2)*3, frame.data + (byteCount>>2)*3, (byteCount>>2), cudaMemcpyHostToDevice, m_cuStream[3]);
		
		if(code == CV_YUV2BGR_YUYV){
			yuyv2bgr_(d_src_rgb + (byteCount_rgb>>2)*0, d_src + (byteCount>>2)*0, frame.cols, (frame.rows>>2), m_cuStream[0]);
			yuyv2bgr_(d_src_rgb + (byteCount_rgb>>2)*1, d_src + (byteCount>>2)*1, frame.cols, (frame.rows>>2), m_cuStream[1]);
			yuyv2bgr_(d_src_rgb + (byteCount_rgb>>2)*2, d_src + (byteCount>>2)*2, frame.cols, (frame.rows>>2), m_cuStream[2]);
			yuyv2bgr_(d_src_rgb + (byteCount_rgb>>2)*3, d_src + (byteCount>>2)*3, frame.cols, (frame.rows>>2), m_cuStream[3]);
		}
		else if(code == CV_YUV2BGR_UYVY){
			uyvy2bgr_(d_src_rgb + (byteCount_rgb>>2)*0, d_src + (byteCount>>2)*0, frame.cols, (frame.rows>>2), m_cuStream[0]);
			uyvy2bgr_(d_src_rgb + (byteCount_rgb>>2)*1, d_src + (byteCount>>2)*1, frame.cols, (frame.rows>>2), m_cuStream[1]);
			uyvy2bgr_(d_src_rgb + (byteCount_rgb>>2)*2, d_src + (byteCount>>2)*2, frame.cols, (frame.rows>>2), m_cuStream[2]);
			uyvy2bgr_(d_src_rgb + (byteCount_rgb>>2)*3, d_src + (byteCount>>2)*3, frame.cols, (frame.rows>>2), m_cuStream[3]);
		}
		m_img[chId] = cv::Mat(frame.rows, frame.cols, CV_8UC3, d_src_rgb);
		OSA_mutexUnlock(&m_mutex);
		cudaFree_share(d_src, chId);
		if((chId==video_gaoqing0)&&(tvbuffer0==0))
		{
			OSA_bufPutFull(&(tskSendBuftv0), bufId);			
		}
		else if((chId==video_gaoqing)&&(tvbuffer1==0))
		{
			OSA_bufPutFull(&(tskSendBuftv1), bufId);			
		}
		else if((chId==video_gaoqing2)&&(tvbuffer2==0))
		{
			OSA_bufPutFull(&(tskSendBuftv2), bufId);			
		}
		else if((chId==video_gaoqing3)&&(tvbuffer3==0))
		{
			OSA_bufPutFull(&(tskSendBuftv3), bufId);			
		}
		else if((chId==video_pal)&&(palbuffer==0))
		{
			OSA_bufPutFull(&(tskSendBufpal), bufId);
		}
		cudaFree_share(d_src_rgb, chId + DS_CHAN_MAX);
	}
}
#else
void CDisplayer::display(Mat frame, int chId, int code)
{
	assert(chId>=0 && chId<DS_CHAN_MAX);
	OSA_mutexLock(&m_mutex);
	m_frame[chId][pp[chId]] = frame;
	m_code[chId] = code;
	OSA_mutexUnlock(&m_mutex);
}
#endif
void CDisplayer::transfer()
{
	int winId, chId, nChannel,code;
	unsigned char *d_src = NULL;
	unsigned char *d_src_rgb = NULL;
	unsigned int byteCount;// = frame.rows * frame.cols * nChannel * sizeof(unsigned char);
	unsigned int byteBlock ;//= byteCount/DS_CUSTREAM_CNT;
	int mask = 0;

	for(winId=0; winId<m_renderCount; winId++)
	{
		chId = m_renders[winId].video_chId;

		if(chId < 0 || chId >= DS_CHAN_MAX)
		{
			continue;
		}

		OSA_mutexLock(&m_mutex);
		dism_img[chId]=	m_frame[chId][pp[chId]];
		code = m_code[chId];
		OSA_mutexUnlock(&m_mutex);

		if(dism_img[chId].cols <=0 || dism_img[chId].rows <=0 || dism_img[chId].channels() == 0)
		{
			continue;
		}
		nChannel = dism_img[chId].channels();
		byteCount = dism_img[chId].rows * dism_img[chId].cols * nChannel * sizeof(unsigned char);
		byteBlock = byteCount/DS_CUSTREAM_CNT;

		if(!((mask >> chId)&1))
		{
			if(nChannel == 1|| code == -1)
			{
				cudaMalloc_share((void**)&d_src_rgb, byteCount, chId + DS_CHAN_MAX);
				cudaMemcpyAsync(d_src_rgb, dism_img[chId].data, byteCount, cudaMemcpyHostToDevice,  m_cuStream[0]);
				m_img[chId] = cv::Mat(dism_img[chId].rows, dism_img[chId].cols, CV_MAKETYPE(CV_8U,nChannel), d_src_rgb);
				cudaFree_share(d_src_rgb, chId + DS_CHAN_MAX);
			}
			else
			{
				unsigned int byteCount_rgb = dism_img[chId].rows * dism_img[chId].cols * 3* sizeof(unsigned char);
				unsigned int byteBlock_rgb = byteCount_rgb/DS_CUSTREAM_CNT;
				unsigned char *d_src_gray = NULL;
				cudaMalloc_share((void**)&d_src, byteCount, chId);
				cudaMalloc_share((void**)&d_src_rgb, byteCount_rgb, chId + DS_CHAN_MAX);
				for(int i = 0; i<DS_CUSTREAM_CNT; i++)
					cudaMemcpyAsync(d_src + byteBlock*i, dism_img[chId].data + byteBlock*i, byteBlock, cudaMemcpyHostToDevice, m_cuStream[i]);

				if(code == CV_YUV2BGR_YUYV)
				{
					for(int i = 0; i<DS_CUSTREAM_CNT; i++)
					{
						yuyv2bgr_(d_src_rgb + byteBlock_rgb*i, d_src + byteBlock*i, dism_img[chId].cols, (dism_img[chId].rows/DS_CUSTREAM_CNT), m_cuStream[i]);
					}
				}

				if(code == CV_YUV2BGR_UYVY)
				{
					for(int i = 0; i<DS_CUSTREAM_CNT; i++)
					{
						uyvy2bgr_(d_src_rgb + byteBlock_rgb*i, d_src + byteBlock*i, dism_img[chId].cols, (dism_img[chId].rows/DS_CUSTREAM_CNT), m_cuStream[i]);
					}
				}

				m_img[chId] = cv::Mat(dism_img[chId].rows, dism_img[chId].cols, CV_8UC3, d_src_rgb);

				cudaFree_share(d_src, chId);
				cudaFree_share(d_src_rgb, chId + DS_CHAN_MAX);
			}
			mask |= (1<<chId);
		}
	}
}

GLuint CDisplayer::async_display(int chId, int width, int height, int channels)
{
	assert(chId>=0 && chId<DS_CHAN_MAX);

	if(m_videoSize[chId].w  == width  && m_videoSize[chId].h == height && m_videoSize[chId].c == channels )
		return buffId_input[chId];

	//OSA_printf("%s: w = %d h = %d (%dx%d) cur %d\n", __FUNCTION__, width, height, m_videoSize[chId].w, m_videoSize[chId].h, buffId_input[chId]);

	if(m_videoSize[chId].w != 0){
		glDeleteBuffers(1, &buffId_input[chId]);
		glGenBuffers(1, &buffId_input[chId]);
	}
	
	
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, buffId_input[chId]);
	glBufferData(GL_PIXEL_UNPACK_BUFFER, width*height*channels, NULL, GL_DYNAMIC_COPY);//GL_STATIC_DRAW);//GL_DYNAMIC_DRAW);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

	m_videoSize[chId].w = width;
	m_videoSize[chId].h = height;
	m_videoSize[chId].c = channels;

	gl_updateVertex();

	//OSA_printf("%s: w = %d h = %d (%dx%d) out %d\n", __FUNCTION__, width, height, m_videoSize[chId].w, m_videoSize[chId].h, buffId_input[chId]);
	return buffId_input[chId];
}

void CDisplayer::run()
{
	m_bRun = true;
	glutTimerFunc(0, _timeFunc, m_initPrm.timerfunc_value);
}

void CDisplayer::stop()
{
	m_bRun = false;
}

int CDisplayer::setFullScreen(bool bFull)
{
	if(bFull)
		glutFullScreen();
	else
		;
	m_bFullScreen = bFull;
	return 0;
}
void CDisplayer::reDisplay(void)
{
	glutPostRedisplay();
}

void CDisplayer::UpDateOsd(int idc)
{
	if(idc<0 || idc>=DS_DC_CNT )
		return;
	updata_osd[idc] = true;
}

/***********************************************************************/



static char glName[] = {"DS_RENDER"}; 

int CDisplayer::gl_create()
{
	char *argv[1] = {glName};
	int argc = 1;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glClearColor(0.0f, 0.0f, 0.01f, 0.0f );
	cudaEventCreate(&m_startEvent);
	cudaEventCreate(&m_stopEvent);
	return 0;
}
void CDisplayer::gl_destroy()
{
	gl_uninit();
	glutLeaveMainLoop();
	cudaEventDestroy(m_startEvent);
	cudaEventDestroy(m_stopEvent);
}

#define TEXTURE_ROTATE (0)
#define ATTRIB_VERTEX 3
#define ATTRIB_TEXTURE 4

void CDisplayer::gl_init()
{
	int i;

	GLenum err = glewInit();
	if (GLEW_OK != err) {
		fprintf(stderr, "GLEW Error: %s\n", glewGetErrorString(err));
		return;
	}

	m_glProgram = gltLoadShaderPairWithAttributes("Shader.vsh", "Shader.fsh", 2, 
		ATTRIB_VERTEX, "vVertex", ATTRIB_TEXTURE, "vTexCoords");

	m_fontProgram = gltLoadShaderPairWithAttributes("fontShader.vp", "fontShader.fp", 0);
	
	glGenBuffers(DS_CHAN_MAX, buffId_input);
	glGenTextures(DS_CHAN_MAX, textureId_input);
	
	for(i=0; i<DS_CHAN_MAX; i++)
	{
		glBindTexture(GL_TEXTURE_2D, textureId_input[i]);
		assert(glIsTexture(textureId_input[i]));
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//GL_NEAREST);//GL_NEAREST_MIPMAP_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//GL_NEAREST);//GL_NEAREST_MIPMAP_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);//GL_CLAMP);//GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);//GL_CLAMP);//GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0,3, vcapWH[i][0], vcapWH[i][1], 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, NULL);
	}
	glGenBuffers(DS_DC_CNT, buffId_osd);
	glGenTextures(DS_DC_CNT, textureId_osd); 

	for(i=0; i<DS_DC_CNT; i++)
	{
		m_disOsd[i]    = cv::Mat(vcapWH[i][1], vcapWH[i][0], CV_8UC4, cv::Scalar(0,0,0,0));
		m_imgOsd[i] = cv::Mat(vcapWH[i][1], vcapWH[i][0], CV_8UC4, cv::Scalar(0,0,0,0));
		
		glBindTexture(GL_TEXTURE_2D, textureId_osd[i]);
		assert(glIsTexture(textureId_osd[i]));
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_imgOsd[i].cols, m_imgOsd[i].rows, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, NULL);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, buffId_osd[i]);
		glBufferData(GL_PIXEL_UNPACK_BUFFER, m_imgOsd[i].rows*m_imgOsd[i].cols*m_imgOsd[i].channels(), m_imgOsd[i].data, GL_DYNAMIC_COPY);//GL_STATIC_DRAW);//GL_DYNAMIC_DRAW);
		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}
// Texture For SHow Save BMP Images
	cv::Mat blackIMG = cv::Mat::zeros(1080,1920,CV_8UC3);
	for(int i =0; i<50;i++) {
		glGenTextures(1, &_textureId[i]);
		glBindTexture(GL_TEXTURE_2D, _textureId[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1920, 1080, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, blackIMG.data);
	}	
// Texture For Show Corner Detect Images
		glGenTextures(1, &_texCornerId);
		glBindTexture(GL_TEXTURE_2D, _texCornerId);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1920, 1080, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, blackIMG.data);
	
	
	x11disbuffer=(unsigned char *)malloc(mallocwidth*mallocheight*4);

	for(i=0; i<DS_CHAN_MAX; i++){
		memcpy(m_glmat44fTrans[i], m_glmat44fTransDefault, sizeof(m_glmat44fTransDefault));
	}

	glClear(GL_COLOR_BUFFER_BIT);
	OSDCreatText();
}

void CDisplayer::gl_uninit()
{
	int i;
	if(m_glProgram != 0)
		glDeleteProgram(m_glProgram);
	m_glProgram = 0;

	glDeleteTextures(DS_CHAN_MAX, textureId_input);
	glDeleteBuffers(DS_CHAN_MAX, buffId_input);
	//glDeleteTextures(1, &textureId_osd); 
	//glDeleteTextures(1, &textureId_pbo);
	//glDeleteBuffers(1, pixBuffObjs);
	glDeleteTextures(DS_DC_CNT, textureId_osd);
	glDeleteBuffers(DS_DC_CNT, buffId_osd);
}


void* CDisplayer::displayerload(void *pPrm)
{
	int i=0;	
	while(1)
	{
		
		OSA_semWait(&(gThis->tskdisSemmain),OSA_TIMEOUT_FOREVER);
		for(i=0; i<DS_DC_CNT; i++)
		{
			if(i == gThis->m_renders[0].video_chId)
			{
				if(!gThis->updata_osd[i])
				{
					unsigned int  size=gThis->m_imgOsd[i].cols*gThis->m_imgOsd[i].rows*gThis->m_imgOsd[i].channels();
					memcpy(gThis->m_disOsd[i].data,gThis->m_imgOsd[i].data,size);
					gThis->updata_osd[i]=true;
				}
			}
		}
	}
}


void CDisplayer::gl_Loadinit()
{
	int status=0;	
       status = OSA_semCreate(&tskdisSemmain, 1, 0);
       OSA_assert(status == OSA_SOK);

       status = OSA_thrCreate(
                 &tskdisHndlmain,
                 displayerload,
                 OSA_THR_PRI_DEFAULT,
                 0,
             NULL
             );
  	OSA_assert(status == OSA_SOK);
}


int CDisplayer::gl_updateVertex(void)
{
	int iRet = 0;
	int winId, chId, i;
	DS_Rect rc;
	//GLfloat ftmp;

	for(winId=0; winId<m_renderCount; winId++)
	{
		m_glvVerts[winId][0] = -1.0f; m_glvVerts[winId][1] = 1.0f;
		m_glvVerts[winId][2] = 1.0f; m_glvVerts[winId][3] = 1.0f;
		m_glvVerts[winId][4] = -1.0f; m_glvVerts[winId][5] = -1.0f;
		m_glvVerts[winId][6] = 1.0f; m_glvVerts[winId][7] = -1.0f;

		for(i=0; i<4; i++){
			float x = m_glvVerts[winId][i*2+0];
			float y = m_glvVerts[winId][i*2+1];
			m_glvVerts[winId][i*2+0] = m_renders[winId].transform[0][0] * x + m_renders[winId].transform[0][1] * y + m_renders[winId].transform[0][3];
			m_glvVerts[winId][i*2+1] = m_renders[winId].transform[1][0] * x + m_renders[winId].transform[1][1] * y + m_renders[winId].transform[1][3];
		}

		if(m_renders[winId].bBind){
			GLfloat subw, subh;
			subw = m_glvVerts[winId][2] - m_glvVerts[winId][0];
			subh = m_glvVerts[winId][5] - m_glvVerts[winId][1];
			m_glvVerts[winId][0] += m_renders[winId].bindrect.x*subw;
			m_glvVerts[winId][1] += m_renders[winId].bindrect.y*subh;
			m_glvVerts[winId][2] = m_glvVerts[winId][0] + m_renders[winId].bindrect.w*subw;
			m_glvVerts[winId][3] = m_glvVerts[winId][1];
			m_glvVerts[winId][4] = m_glvVerts[winId][0];
			m_glvVerts[winId][5] = m_glvVerts[winId][1] + m_renders[winId].bindrect.h*subh;
			m_glvVerts[winId][6] = m_glvVerts[winId][0] + m_renders[winId].bindrect.w*subw;
			m_glvVerts[winId][7] = m_glvVerts[winId][1] + m_renders[winId].bindrect.h*subh;
		}

		m_glvTexCoords[winId][0] = 0.0; m_glvTexCoords[winId][1] = 0.0;
		m_glvTexCoords[winId][2] = 1.0; m_glvTexCoords[winId][3] = 0.0;
		m_glvTexCoords[winId][4] = 0.0; m_glvTexCoords[winId][5] = 1.0;
		m_glvTexCoords[winId][6] = 1.0; m_glvTexCoords[winId][7] = 1.0;
	}

	//rc.x=0,rc,y=0,m_videoSize[0].w=720,m_videoSize[0].h=576,m_videoSize[1].w=1920,m_videoSize[1].h=1080
	for(winId=0; winId<m_renderCount; winId++)
	{		
		chId = m_renders[winId].video_chId;
		if(chId < 0 || chId >= DS_CHAN_MAX)
			continue;
		rc = m_renders[winId].croprect;
		//winId=0/1,rc.x=840,rc.y=444,rc.w=240,rc.h=192
		if(m_videoSize[chId].w<=0 || m_videoSize[chId].h<=0){
			iRet ++;
			continue;
		}
		if(rc.w == 0 && rc.h == 0){
			continue;
		}
		//OSA_printf("%s: crop : %d,%d  %dx%d\n", __func__, rc.x, rc.y, rc.w, rc.h);
		//OSA_printf("%s: crop : curvideo %d  %d x %d\n", __func__, chId, m_videoSize[chId].w, m_videoSize[chId].h);
		m_glvTexCoords[winId][0] = (GLfloat)rc.x/m_videoSize[chId].w; 
		m_glvTexCoords[winId][1] = (GLfloat)rc.y/m_videoSize[chId].h;

		m_glvTexCoords[winId][2] = (GLfloat)(rc.x+rc.w)/m_videoSize[chId].w;
		m_glvTexCoords[winId][3] = (GLfloat)rc.y/m_videoSize[chId].h;

		m_glvTexCoords[winId][4] = (GLfloat)rc.x/m_videoSize[chId].w;
		m_glvTexCoords[winId][5] = (GLfloat)(rc.y+rc.h)/m_videoSize[chId].h;

		m_glvTexCoords[winId][6] = (GLfloat)(rc.x+rc.w)/m_videoSize[chId].w;
		m_glvTexCoords[winId][7] = (GLfloat)(rc.y+rc.h)/m_videoSize[chId].h;
		//pal
		// m_glvTexCoords[1][0]=1.166667    840/720
		// m_glvTexCoords[1][1]=0.770833    440/576
		//m_glvTexCoords[1][2]=1.500000    
		//m_glvTexCoords[1][3]=0.770833
		//m_glvTexCoords[1][4]=1.166667
		//m_glvTexCoords[1][5]=1.104167
		//m_glvTexCoords[1][6]=1.500000
		//m_glvTexCoords[1][7]=1.104167
		
		//gaoqing
		//m_glvTexCoords[1][0]=0.437500    840/1920
		//m_glvTexCoords[1][1]=0.411111    440/1080
		//m_glvTexCoords[1][2]=0.562500
		//m_glvTexCoords[1][3]=0.411111
		//m_glvTexCoords[1][4]=0.437500
		// m_glvTexCoords[1][5]=0.588889
		//m_glvTexCoords[1][6]=0.562500
		//m_glvTexCoords[1][7]=0.588889
	}

	return iRet;
}

int CDisplayer::menu_init()
{
	m_menuindex = -1;
	memcpy(&dismenuarray, plat->extInCtrl->menuarray, sizeof(dismenuarray));
	unsigned char menubuf[menumaxid][7][128] = {
            {"请输入密码呼出菜单", "按回车确认", "按F2退出"},
            {"请输入密码呼出菜单", "********", "密码输入错误，按回车后再次输入", "按回车确认", "按F2退出"},
            {"内参标定","枪球画面标定","移动检测设置","画面设置","球机设定","固件升级","密码更改"},
            {"枪机内参标定","球机内参标定","返回"},
            {"枪球自动标定","枪球手动标定","返回"},
            {"检测区域选择","目标个数","跟踪持续时间","最大目标面积","最小目标面积","灵敏度","返回"},
            {"扫描方式均为逐行扫描","格式","应用","返回"},
            {"使用串口设置","使用网络设置","返回"},
            {"波特率","球机地址","球机协议","工作模式","返回"},
            {"网络协议","IP地址","登录用户名","登录密码","返回"}};


	
	for(int i = 0; i < menumaxid; i++)
	{
		for(int j = 0; j < MAX_SUBMENU; j++)
		{
			if(j >= dismenuarray[i].submenu_cnt)
				break;
			disMenuBuf[i][j].alpha = 2;
			disMenuBuf[i][j].ctrl = 0;
			disMenuBuf[i][j].posx = 1500;
			disMenuBuf[i][j].color = 2;
			disMenuBuf[i][j].posy = (j + 1) * 60;
			setlocale(LC_ALL, "zh_CN.UTF-8");
			swprintf(disMenu[i][j], 33, L"%s", menubuf[i][j]);
		}
	}

	disMenuBuf[mainmenu0][1].posy = 4 * 60;
	disMenuBuf[mainmenu0][2].posy = 5 * 60;
	disMenuBuf[submenu_carli][2].posy = 4 * 60;
	disMenuBuf[submenu_gunball][2].posy = 4 * 60;
	disMenuBuf[submenu_setball][2].posy = 4 * 60;
	disMenuBuf[submenu_setcom][4].posy = 6 * 60;
	disMenuBuf[submenu_setnet][4].posy = 6 * 60;

	disMenuBuf[mainmenu1][2].color= 3;
	disMenuBuf[mainmenu2][0].color= 3;
	disMenuBuf[submenu_carli][0].color= 3;
	disMenuBuf[submenu_gunball][0].color= 3;
	disMenuBuf[submenu_mtd][0].color= 3;
	disMenuBuf[submenu_setball][0].color= 3;
	disMenuBuf[submenu_setcom][0].color= 3;
	disMenuBuf[submenu_setnet][0].color= 3;		
}

static int64 tstart = 0;
static int64 tstartBK = 0;
static float offtime = 0;
double enhancetime=0;
static unsigned  int frametextcout=0;
static unsigned  int frametextcout1=0;

static unsigned int tvframecount=0;
static unsigned int tvdupcount=800;
static unsigned int firframecount=0;
static unsigned int firdupcount=800;

static unsigned int tvclearbuffer=1;
static unsigned int firclearbuffer=1;
#if 0
void CDisplayer::gl_textureLoad(void)
{
		
	int winId, chId;
	unsigned int mask = 0;
	float elapsedTime;

	unsigned char disbuffer=0;
	int bufid=0;
	unsigned char *disbuf=NULL;
	unsigned int byteCount=0;

	tstart = getTickCount();
	if(tstartBK != 0){
		offtime = (tstart - tstartBK)/getTickFrequency();
		//OSA_printf("chId = %d, gl_display offtime: time = %f sec \n", chId,  offtime);
	}
	tstartBK = tstart;
	cudaEventRecord(m_startEvent, 0);

	
	for(winId=0; winId<m_renderCount; winId++)
	{
		chId = m_renders[winId].video_chId;
		//if(winId>2)
		//	chId=1;
		if(chId < 0 || chId >= DS_CHAN_MAX)
		{
			continue;
		}
		dism_img[chId]=m_img[chId];

		if(dism_img[chId].cols <=0 || dism_img[chId].rows <=0 || dism_img[chId].channels() == 0)
		{
			//printf("[chId =%d  winId=%d] w=%d h=%d c=%d\n",chId,winId,m_img[chId].cols,m_img[chId].rows,m_img[chId].channels());
			continue;
		}
		
		if(!((mask >> chId)&1))
		{
			if(!m_renders[winId].bFreeze)
			{
				GLuint pbo = async_display(chId, dism_img[chId].cols, dism_img[chId].rows, dism_img[chId].channels());
				byteCount = dism_img[chId].cols*dism_img[chId].rows*dism_img[chId].channels()*sizeof(unsigned char);
				unsigned char *dev_pbo = NULL;
				size_t tmpSize;
				freezeonece=1;
				OSA_assert(pbo == buffId_input[chId]);
	
				cudaResource_RegisterBuffer(chId, pbo, byteCount);
				cudaResource_mapBuffer(chId, (void **)&dev_pbo, &tmpSize);

				frametextcout++;
				tvframecount++;
				firframecount++;

				int drupfram=PICBUFFERCOUNT;
				if(chId==video_gaoqing0)
				{
					int framecount=OSA_bufGetBufcount(&(tskSendBuftv0),0);
					if(framecount>=drupfram)
					{
						if(OSA_bufGetFull(&tskSendBuftv0, &bufid, OSA_TIMEOUT_NONE)==0)
							OSA_bufPutEmpty(&tskSendBuftv0, bufid);
						if(OSA_bufGetFull(&tskSendBuftv0, &bufid, OSA_TIMEOUT_NONE)==0)
							OSA_bufPutEmpty(&tskSendBuftv0, bufid);
					}
					disbuffer=OSA_bufGetFull(&tskSendBuftv0, &bufid, OSA_TIMEOUT_NONE);
				}
				else if(chId==video_gaoqing)
				{
					int framecount=OSA_bufGetBufcount(&(tskSendBuftv1),0);
					if(framecount>=drupfram)
					{
						if(OSA_bufGetFull(&tskSendBuftv1, &bufid, OSA_TIMEOUT_NONE)==0)
							OSA_bufPutEmpty(&tskSendBuftv1, bufid);
						if(OSA_bufGetFull(&tskSendBuftv1, &bufid, OSA_TIMEOUT_NONE)==0)
							OSA_bufPutEmpty(&tskSendBuftv1, bufid);
					}
					disbuffer=OSA_bufGetFull(&tskSendBuftv1, &bufid, OSA_TIMEOUT_NONE);
				}
				else if(chId==video_gaoqing2)
				{
					int framecount=OSA_bufGetBufcount(&(tskSendBuftv2),0);
					if(framecount>=drupfram)
					{
						if(OSA_bufGetFull(&tskSendBuftv2, &bufid, OSA_TIMEOUT_NONE)==0)
							OSA_bufPutEmpty(&tskSendBuftv2, bufid);
						if(OSA_bufGetFull(&tskSendBuftv2, &bufid, OSA_TIMEOUT_NONE)==0)
							OSA_bufPutEmpty(&tskSendBuftv2, bufid);
					}
					disbuffer=OSA_bufGetFull(&tskSendBuftv2, &bufid, OSA_TIMEOUT_NONE);
				}
				else if(chId==video_gaoqing3)
				{
					int framecount=OSA_bufGetBufcount(&(tskSendBuftv3),0);
					if(framecount>=drupfram)
					{
						if(OSA_bufGetFull(&tskSendBuftv3, &bufid, OSA_TIMEOUT_NONE)==0)
							OSA_bufPutEmpty(&tskSendBuftv3, bufid);
						if(OSA_bufGetFull(&tskSendBuftv3, &bufid, OSA_TIMEOUT_NONE)==0)
							OSA_bufPutEmpty(&tskSendBuftv3, bufid);
					}
					disbuffer=OSA_bufGetFull(&tskSendBuftv3, &bufid, OSA_TIMEOUT_NONE);
				}
				else if(chId==video_pal)
				{
					int framecount=OSA_bufGetBufcount(&(tskSendBufpal),0);
					if(framecount>=drupfram)
					{
						if(OSA_bufGetFull(&tskSendBufpal, &bufid, OSA_TIMEOUT_NONE)==0)
							OSA_bufPutEmpty(&tskSendBufpal, bufid);
						if(OSA_bufGetFull(&tskSendBufpal, &bufid, OSA_TIMEOUT_NONE)==0)
							OSA_bufPutEmpty(&tskSendBufpal, bufid);
					}
					disbuffer=OSA_bufGetFull(&tskSendBufpal, &bufid, OSA_TIMEOUT_NONE);
				}
				if((disbuffer==0)&&(chId==video_pal))
				{
					dism_img[chId].data=(unsigned char *)tskSendBufpal.bufInfo[bufid].virtAddr;
					pal_pribuffid=bufid;
				}
				else if((disbuffer==0)&&(chId==video_gaoqing0))
				{				
					dism_img[chId].data=(unsigned char *)tskSendBuftv0.bufInfo[bufid].virtAddr;
					tv_pribuffid0=bufid;
				}
				else if((disbuffer==0)&&(chId==video_gaoqing))
				{					
					dism_img[chId].data=(unsigned char *)tskSendBuftv1.bufInfo[bufid].virtAddr;
					tv_pribuffid1=bufid;
				}
				else if((disbuffer==0)&&(chId==video_gaoqing2))
				{
					dism_img[chId].data=(unsigned char *)tskSendBuftv2.bufInfo[bufid].virtAddr;
					tv_pribuffid2=bufid;
				}
				else if((disbuffer==0)&&(chId==video_gaoqing3))
				{			
					dism_img[chId].data=(unsigned char *)tskSendBuftv3.bufInfo[bufid].virtAddr;
					tv_pribuffid3=bufid;
				}

			
			if(disptimeEnable == 1){//0
				//test zhou qi  time
				int64 disptime = 0;

				disptime = getTickCount();
				
				double curtime = (disptime/getTickFrequency())*1000;///( (getTickCount() - trktime)/getTickFrequency())*1000;
				
				static double pretime = 0.0;
				double time = curtime - pretime;
				pretime = curtime;

				putText(m_imgOsd[1],dispstrDisplay,
						Point( m_imgOsd[1].cols-350, 80),
						FONT_HERSHEY_TRIPLEX,0.8,
						cvScalar(0,0,0,0), 1
						);
				sprintf(dispstrDisplay, "disp time = %0.3fFPS", 1000.0/time);

				putText(m_imgOsd[1],dispstrDisplay,
						Point(m_imgOsd[1].cols-350, 80),
						FONT_HERSHEY_TRIPLEX,0.8,
						cvScalar(255,255,0,255), 1
						);

				putText(m_imgOsd[1],capstrDisplay,
						Point( m_imgOsd[1].cols-350, 55),
						FONT_HERSHEY_TRIPLEX,0.8,
						cvScalar(0,0,0,0), 1
						);
				sprintf(capstrDisplay, "cap time = %0.3fFPS", 1000.0/capTime);

				putText(m_imgOsd[1],capstrDisplay,
						Point(m_imgOsd[1].cols-350, 55),
						FONT_HERSHEY_TRIPLEX,0.8,
						cvScalar(255,255,0,255), 1
						);
			}


				if(m_renders[chId].videodect)
				{
					//cudaMemcpy(dev_pbo, m_img[chId].data, byteCount, cudaMemcpyDeviceToDevice);
					cudaMemcpy(x11disbuffer, dism_img[chId].data,byteCount, cudaMemcpyDeviceToHost);
				}
				else
				{
					//cudaMemcpy(dev_pbo, m_img_novideo.data, byteCount, cudaMemcpyDeviceToDevice);
					cudaMemcpy(x11disbuffer, m_img_novideo.data,byteCount, cudaMemcpyDeviceToHost);
				}

	
				if( (chId == 0 )&& (plat->m_camCalibra->Set_Handler_Calibra == true || g_sysParam->isEnable_Undistortion())) {
					memcpy(gun_UndistorMat.data, x11disbuffer, 1080*1920*3);
					remap(gun_UndistorMat, gun_UndistorMat, g_camParams.map1, g_camParams.map2, INTER_LINEAR);
					memcpy( x11disbuffer,gun_UndistorMat.data, 1080*1920*3);
				}

				if((disbuffer==0)&&(chId==video_gaoqing0))
					OSA_bufPutEmpty(&tskSendBuftv0, bufid);
				if((disbuffer==0)&&(chId==video_gaoqing))
					OSA_bufPutEmpty(&tskSendBuftv1, bufid);
				if((disbuffer==0)&&(chId==video_gaoqing2))
					OSA_bufPutEmpty(&tskSendBuftv2, bufid);
				if((disbuffer==0)&&(chId==video_gaoqing3))
					OSA_bufPutEmpty(&tskSendBuftv3, bufid);		
				if((disbuffer==0)&&(chId==video_pal))
					OSA_bufPutEmpty(&tskSendBufpal, bufid);		
			
			}
			//glBindBuffer(GL_PIXEL_UNPACK_BUFFER, buffId_input[chId]);
			glBindTexture(GL_TEXTURE_2D, textureId_input[chId]);
			if(dism_img[chId].channels() == 1){
				if(!m_renders[winId].bFreeze)
				glTexImage2D(GL_TEXTURE_2D, 0, m_videoSize[chId].c, m_videoSize[chId].w, m_videoSize[chId].h, 0, GL_RED, GL_UNSIGNED_BYTE, x11disbuffer);
			}else{
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_videoSize[chId].w, m_videoSize[chId].h, GL_BGR_EXT, GL_UNSIGNED_BYTE, x11disbuffer);
			}

			mask |= (1<<chId);

		}
	}

	if(m_bOsd)
	{
		for(int i=0; i<DS_DC_CNT;  i++)
		{
			if(updata_osd[i])
			{
			
				glBindTexture(GL_TEXTURE_2D, textureId_osd[i]);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_imgOsd[i].cols, m_imgOsd[i].rows, GL_BGRA_EXT, GL_UNSIGNED_BYTE, m_disOsd[i].data);
				updata_osd[i] = false;
			}			
		}
	}	
	cudaEventRecord(m_stopEvent, 0);
	cudaEventSynchronize(m_stopEvent);
	cudaEventElapsedTime(	&elapsedTime, m_startEvent, m_stopEvent);
	//if(elapsedTime > 5.0f)
	//	OSA_printf("%s: -------elapsed %.3f ms.\n", __func__, elapsedTime);
	float telapse = ( (getTickCount() - tstart)/getTickFrequency());
}

#else
static int64  tend = 0ul;
void CDisplayer::gl_textureLoad(void)
{
	int winId, chId;
	unsigned int mask = 0;
	unsigned int byteCount=0;

	tStamp[0] = getTickCount();
	tstart = tStamp[0];
	if(tend == 0ul)
		tend = tstart;
	if(1)
	{
		double wms = m_interval*0.000001 - m_telapse;
		double wmsl = (m_tmRender == 0ul) ? 0.f : ((tStamp[0] - m_tmRender)*0.000001f);
		wms -= wmsl;
		if(m_waitSync && wms>2.0){
			struct timeval timeout;
			timeout.tv_sec = 0;
			timeout.tv_usec = wms*1000.0;
			select( 0, NULL, NULL, NULL, &timeout );
		}else{
			wms = 0.0;
		}
	}
	tStamp[1] = getTickCount();

	transfer();

	tStamp[2] = getTickCount();

	for(winId=0; winId<m_renderCount; winId++)
	{
		chId = m_renders[winId].video_chId;

		if(chId < 0 || chId >= DS_CHAN_MAX)
		{
			continue;
		}
		dism_img[chId]=m_img[chId];

		if(dism_img[chId].cols <=0 || dism_img[chId].rows <=0 || dism_img[chId].channels() == 0)
		{
			continue;
		}

		if(!((mask >> chId)&1))
		{
			if(!m_renders[winId].bFreeze)
			{
				GLuint pbo = async_display(chId, dism_img[chId].cols, dism_img[chId].rows, dism_img[chId].channels());
				byteCount = dism_img[chId].cols*dism_img[chId].rows*dism_img[chId].channels()*sizeof(unsigned char);
				unsigned char *dev_pbo = NULL;
				size_t tmpSize;
				freezeonece=1;
				OSA_assert(pbo == buffId_input[chId]);

				cudaResource_RegisterBuffer(chId, pbo, byteCount);
				cudaResource_mapBuffer(chId, (void **)&dev_pbo, &tmpSize);
				if(tmpSize != byteCount)
				{
					OSA_printf("%s: WARNING!!! %d - %d \n", __func__, (unsigned int)tmpSize, byteCount);
				}

				if( (chId == 0 )&& (plat->m_camCalibra->Set_Handler_Calibra == true || g_sysParam->isEnable_Undistortion())) {
					if(m_renders[chId].videodect)
						cudaMemcpy(x11disbuffer, dism_img[chId].data,byteCount, cudaMemcpyDeviceToHost);
					else
						cudaMemcpy(x11disbuffer, m_img_novideo.data,byteCount, cudaMemcpyDeviceToHost);

					cv::Mat undistorMat = cv::Mat(dism_img[chId].rows, dism_img[chId].cols, CV_8UC3, x11disbuffer);
					remap(undistorMat, undistorMat, g_camParams.map1, g_camParams.map2, INTER_LINEAR);

				}else{
					if(m_renders[chId].videodect)
						cudaMemcpy(dev_pbo, dism_img[chId].data, byteCount, cudaMemcpyDeviceToDevice);
					else
						cudaMemcpy(dev_pbo, m_img_novideo.data, byteCount, cudaMemcpyDeviceToDevice);
				}
				cudaResource_unmapBuffer(chId);
				cudaResource_UnregisterBuffer(chId);
			}

			if( (chId == 0 )&& (plat->m_camCalibra->Set_Handler_Calibra == true || g_sysParam->isEnable_Undistortion())) {
				glBindTexture(GL_TEXTURE_2D, textureId_input[chId]);
				if(dism_img[chId].channels() == 1){
					glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_videoSize[chId].w, m_videoSize[chId].h, GL_RED, GL_UNSIGNED_BYTE, x11disbuffer);
				}else{
					glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_videoSize[chId].w, m_videoSize[chId].h, GL_BGR_EXT, GL_UNSIGNED_BYTE, x11disbuffer);
				}
			}else{
				glBindBuffer(GL_PIXEL_UNPACK_BUFFER, buffId_input[chId]);
				glBindTexture(GL_TEXTURE_2D, textureId_input[chId]);
				if(dism_img[chId].channels() == 1){
					glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_videoSize[chId].w, m_videoSize[chId].h, GL_RED, GL_UNSIGNED_BYTE, NULL);
				}else{
					glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_videoSize[chId].w, m_videoSize[chId].h, GL_BGR_EXT, GL_UNSIGNED_BYTE, NULL);
				}
				glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
			}

			mask |= (1<<chId);
		}
	}

	if(m_bOsd)
	{
		for(int i=0; i<DS_DC_CNT;  i++)	{
			if(updata_osd[i]) {
				glBindTexture(GL_TEXTURE_2D, textureId_osd[i]);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, m_disOsd[i].cols, m_disOsd[i].rows, GL_BGRA_EXT, GL_UNSIGNED_BYTE, m_disOsd[i].data);
				updata_osd[i] = false;
			}
		}
	}
//=============================================================================================
	int nsize = imageListForCalibra.size();
	for(int i=0; i<100; i++)	{
		if(m_detectCorners->_bCutIMG[i])  {
			assert(nsize>i);				
			cv::Mat IMG = imageListForCalibra[i];				
			glBindTexture(GL_TEXTURE_2D, _textureId[i]);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, IMG.cols, IMG.rows, GL_BGR_EXT, GL_UNSIGNED_BYTE, IMG.data);
			m_detectCorners->_bCutIMG[i] = false;
		}
	}
//=============================================================================================	
	
		if(g_bSubmitTexture && g_CornerImage.empty() == false){
			g_bSubmitTexture = false;
			cv::Mat IMG = g_CornerImage;			
			glBindTexture(GL_TEXTURE_2D, _texCornerId);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, IMG.cols, IMG.rows, GL_BGR_EXT, GL_UNSIGNED_BYTE, IMG.data);
		}
	

}

#endif

int CDisplayer::setFPS(float fps)
{
    if (fps == 0)
    {
        OSA_printf("[Render]Fps 0 is not allowed. Not changing fps");
        return -1;
    }
    pthread_mutex_lock(&render_lock);
    m_initPrm.disFPS = fps;
    m_interval = (1000000000ul)/(uint64)m_initPrm.disFPS;
    render_time_sec = m_interval / 1000000000ul;
    render_time_nsec = (m_interval % 1000000000ul);
    memset(&last_render_time, 0, sizeof(last_render_time));
    pthread_mutex_unlock(&render_lock);
    return 0;
}

void CDisplayer::disp_fps(){
    static GLint frames = 0;
    static GLint t0 = 0;
    static char  fps_str[20] = {'\0'};
    GLint t = glutGet(GLUT_ELAPSED_TIME);
 //   sprintf(fps_str, "%6.1f FPS\n", 0);
    if (t - t0 >= 200) {
        GLfloat seconds = (t - t0) / 1000.0;
        GLfloat fps = frames / seconds;
        sprintf(fps_str, "%6.1f FPS\n", fps);
       // printf("%6.1f FPS\n", fps);
        t0 = t;
        frames = 0;
    }
    glColor3f(0.0, 0.0, 1.0);
    glRasterPos2f(0, 0);
    glutBitmapString(GLUT_BITMAP_HELVETICA_18, (unsigned char *)fps_str);
    frames++;
}

DISPLAYMODE CDisplayer::getDisplayMode()
{
	return displayMode;
}


void CDisplayer::switchDisplayMode()
{
	displayMode = DISPLAYMODE( (int)(displayMode+1)%TOTAL_MODE_COUNT);
}


void CDisplayer::changeDisplayMode(DISPLAYMODE mode)
{
	displayMode =DISPLAYMODE( mode%TOTAL_MODE_COUNT);
}


void CDisplayer::RenderVideoOnOrthoView( int videoChannel, int x, int y, int width, int height )
{
	glViewport( x, y, width, height );
	glPushMatrix();
	glLoadIdentity();
	glUniformMatrix4fv(Uniform_mattrans, 1, GL_FALSE, m_glmat44fTrans[0]);
	glUniform1i(Uniform_tex_in, 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureId_input[videoChannel]);
	glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, m_glvVerts[0]);
	glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, m_glvTexCoords[0]);
	glEnableVertexAttribArray(ATTRIB_VERTEX);
	glEnableVertexAttribArray(ATTRIB_TEXTURE);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glPopMatrix();	
}

void CDisplayer::RenderDetectCornerView(GLint x, GLint y, GLint width, GLint height)
{
		glViewport( x, y, width,height);
		glPushMatrix();
		glLoadIdentity();
		glUniformMatrix4fv(Uniform_mattrans, 1, GL_FALSE, m_glmat44fTrans[0]);	
		glUniform1i(Uniform_tex_in, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, _texCornerId);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glVertexPointer(3,GL_FLOAT, sizeof(Vertex3F), &BMPVertex[0].x);
		glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex3F), &BMPVertex[0].u);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glPopMatrix();	
}

void CDisplayer::RenderSavedBMPImageByIndex(int Index)
{
	glViewport( 480, 540,960,540 );
	glPushMatrix();
	glLoadIdentity();
	glUniformMatrix4fv(Uniform_mattrans, 1, GL_FALSE, m_glmat44fTrans[0]);	
	glUniform1i(Uniform_tex_in, 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, _textureId[Index]);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glVertexPointer(3,GL_FLOAT, sizeof(Vertex3F), &BMPVertex[0].x);
	glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex3F), &BMPVertex[0].u);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glPopMatrix();	
}
void CDisplayer::RenderSavedBMPImage(void)
{
	if(imageListForCalibra.size() != 0) {		
		for(int Index =0; Index< imageListForCalibra.size() && Index < 50; Index++)	
		{			
			int raw = 5;
			int col = 10;
			glViewport( 192.0*(Index%col), 540.0-108.0*((Index/col)+1), 192.0,108.0 );
			glPushMatrix();
			glLoadIdentity();
			glUniformMatrix4fv(Uniform_mattrans, 1, GL_FALSE, m_glmat44fTrans[0]);	
			glUniform1i(Uniform_tex_in, 0);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, _textureId[Index]);
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			glVertexPointer(3,GL_FLOAT, sizeof(Vertex3F), &BMPVertex[0].x);
			glTexCoordPointer(2, GL_FLOAT, sizeof(Vertex3F), &BMPVertex[0].u);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
			glPopMatrix();	
		}	
	}
	

}

void CDisplayer::linkageSwitchMode(void)
{
	int winId, chId;
	unsigned int mask = 0;
	switch( g_displayMode ){
		case MENU_MAIN_VIEW:
			displayMode = MAIN_VIEW;
			break;
		case MENU_SBS:
			displayMode = PREVIEW_MODE;
			break;
		case MENU_GUN:
			displayMode = GUN_FULL_SCREEN;
			break;
		case MENU_BALL:
			displayMode = BALL_FULL_SCREEN;
			break;
		case MENU_CALIBRA_CAP:
			displayMode = CALIBRATE_CAPTURE;
			break;
		case MENU_CALIBRA_RESULT:
			displayMode = CALIBRATE_RESULT;
			break;
		default:
			break;
	}
	
	switch(displayMode) 
	{
		case PREVIEW_MODE:
			RenderVideoOnOrthoView(VIDEO_1, 0,vdisWH[0][1]/2,vdisWH[0][0]/2,vdisWH[0][1]/2);
			RenderVideoOnOrthoView(VIDEO_0, vdisWH[0][0]/2,vdisWH[0][1]/2,vdisWH[0][0]/2,vdisWH[0][1]/2);			
			setFontPosition(100, 640);
			if( g_CurDisplayMode != PREVIEW_MODE)
				g_CurDisplayMode = PREVIEW_MODE;		
			break;

		case MAIN_VIEW:	
		
			RenderVideoOnOrthoView(VIDEO_1,vdisWH[0][0]/4,vdisWH[0][1]/2,vdisWH[0][0]/2,vdisWH[0][1]/2);
			RenderVideoOnOrthoView(VIDEO_0, 0,0,vdisWH[0][0],vdisWH[0][1]/2);
			
	
			if( g_CurDisplayMode != MAIN_VIEW)
				g_CurDisplayMode = MAIN_VIEW;			
			break;
		case GUN_FULL_SCREEN:				
			RenderVideoOnOrthoView(VIDEO_0, 0,0,vdisWH[0][0],vdisWH[0][1]);
			if( g_CurDisplayMode != GUN_FULL_SCREEN)
				g_CurDisplayMode = GUN_FULL_SCREEN;
			break;
		case BALL_FULL_SCREEN:		
			RenderVideoOnOrthoView(VIDEO_1, 0,0,vdisWH[0][0],vdisWH[0][1]);	
			if( g_CurDisplayMode != BALL_FULL_SCREEN)
				g_CurDisplayMode = BALL_FULL_SCREEN;			
			break;	
		case CALIBRATE_CAPTURE:
		{
			RenderDetectCornerView(0,810,480,270);
			RenderVideoOnOrthoView(g_connectAction.CurCalibraCam/*captureBMP_channel*/, 480,270,1440,810);

			if( g_CurDisplayMode != CALIBRATE_CAPTURE)
				g_CurDisplayMode = CALIBRATE_CAPTURE;	
		}
			break;
		case CALIBRATE_RESULT:
			//RenderVideoOnOrthoView(VIDEO_0, 480,540,960,540);
			RenderSavedBMPImage();
			if( imageListForCalibra.size() >= 1 ){
				RenderSavedBMPImageByIndex( imageListForCalibra.size() - 1 );
			}
			if( g_CurDisplayMode != CALIBRATE_RESULT)
				g_CurDisplayMode = CALIBRATE_RESULT;
			break;
		default:
			break;	
	}
#if 0
	for(winId=0; winId<m_renderCount; winId++)
	{			
		chId = m_renders[winId].video_chId;
		if(chId < 0 || chId >= DS_CHAN_MAX) {
			continue;
		}
		if(m_img[chId].cols <=0 || m_img[chId].rows <=0 || m_img[chId].channels() == 0) {
			continue;
		}	
		glUniformMatrix4fv(Uniform_mattrans, 1, GL_FALSE, m_glmat44fTrans[0]);
		glUniform1i(Uniform_tex_in, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureId_input[chId]);
		glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, m_glvVerts[0]);
		glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, m_glvTexCoords[0]);
		glEnableVertexAttribArray(ATTRIB_VERTEX);
		glEnableVertexAttribArray(ATTRIB_TEXTURE);
		glViewport(m_renders[winId].displayrect.x,m_renders[winId].displayrect.y,m_renders[winId].displayrect.w,m_renders[winId].displayrect.h);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	}	
#endif
	
}

void CDisplayer::gl_display(void)
{	
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(m_glProgram);
	
	Uniform_tex_in = glGetUniformLocation(m_glProgram, "tex_in");
	//Uniform_tex_osd = glGetUniformLocation(m_glProgram, "tex_osd");
	//Uniform_tex_pbo = glGetUniformLocation(m_glProgram, "tex_pbo");
	//Uniform_osd_enable = glGetUniformLocation(m_glProgram, "bOsd");
	Uniform_mattrans = glGetUniformLocation(m_glProgram, "mTrans");
	Uniform_font_color = glGetUniformLocation(m_fontProgram,"fontColor");

	linkageSwitchMode();	
	if(m_bOsd)
	{
		glUniformMatrix4fv(Uniform_mattrans, 1, GL_FALSE, m_glmat44fTransDefault);
		glUniform1i(Uniform_tex_in, 0);
		glActiveTexture(GL_TEXTURE0);
		glEnable(GL_BLEND);		
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		for(int i=0; i<DS_DC_CNT; i++)
		{
			if(m_renders[0].video_chId != i){
				continue;
			}
			glBindTexture(GL_TEXTURE_2D, textureId_osd[i]);
			glVertexAttribPointer(ATTRIB_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, m_glvVertsDefault);
			glVertexAttribPointer(ATTRIB_TEXTURE, 2, GL_FLOAT, GL_FALSE, 0, m_glvTexCoordsDefault);
			glEnableVertexAttribArray(ATTRIB_VERTEX);
			glEnableVertexAttribArray(ATTRIB_TEXTURE);
			glViewport(0, 0, m_mainWinWidth_new[i], m_mainWinHeight_new[i]);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		}
		glDisable(GL_BLEND);		
		IrisAndFocus();
		OSDFunc();
		MenuFunc(m_menuindex);
	}		

/***************************************************** Menu *******************************************************************/

ArrayText::iterator itr = run_Mode._texts.begin();
	   for(int i=0; i<run_Mode._texts.size()-1,itr != run_Mode._texts.end(); itr++, i++ )
	   {
	   		(*itr)._pos.x = fontPosX;
			(*itr)._pos.y = fontPosY + i*60;
		   if( textSelect[i] == 1) {
	   			(*itr)._size = chinese_osd((*itr)._pos.x,(*itr)._pos.y,(*itr)._text,
			   	 1,4,255,0,0,255,VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
		   	}else{
	   			(*itr)._size = chinese_osd((*itr)._pos.x,(*itr)._pos.y,(*itr)._text,
			   	 1,4,(*itr).r,(*itr).g,(*itr).b,(*itr).a,VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
		   	}
	   }	
#if 0
ArrayText::iterator itr2 = run_Mode.workMode.begin();
	   for(int i=0; i<run_Mode.workMode.size()-1,itr2 != run_Mode.workMode.end(); itr2++, i++ )
	   {
	   		(*itr2)._pos.x = fontPosX;
			(*itr2)._pos.y = fontPosY + i*60;
		   if( textSelect2[i] == 1) {
	   			(*itr2)._size = chinese_osd((*itr2)._pos.x,(*itr2)._pos.y,(*itr2)._text,
			   	 1,4,255,0,0,255,VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
		   	}else{
	   			(*itr2)._size = chinese_osd((*itr2)._pos.x,(*itr2)._pos.y,(*itr2)._text,
			   	 1,4,(*itr2).r,(*itr2).g,(*itr2).b,(*itr2).a,VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
		   	}
	   }	
#endif
/******************************************************************************************************************************/
	if(displayMode == MAIN_VIEW){
		chinese_osd(50,150,L"工作模式：",1,4,100,180,200,255,VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
		switch(g_workMode){
			case HANDLE_LINK_MODE:
				chinese_osd(80,200,L"手动联动模式",1,4,100,180,200,255,VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
				break;
			case AUTO_LINK_MODE:
				chinese_osd(80,200,L"自动联动模式",1,4,100,180,200,255,VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
				break;
			case ONLY_BALL_MODE:
				chinese_osd(80,200,L"单控球机模式",1,4,100,180,200,255,VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
				break;
			default:
				break;	
		}
	}

/******************************************************************************************************************************/
	   
	glUseProgram(0);	
	
/***************************************************************************************************************************/	
	tStamp[5] = getTickCount();
	m_waitSync = true;
	int64 tcur = tStamp[5];
	m_telapse = (tStamp[5] - tStamp[1])*0.000001f + m_initPrm.disSched;
#if 1
	if (last_render_time.tv_sec != 0)
	{
		pthread_mutex_lock(&render_lock);
		last_render_time.tv_sec += render_time_sec;
		last_render_time.tv_nsec += render_time_nsec;
		last_render_time.tv_sec += last_render_time.tv_nsec / 1000000000UL;
		last_render_time.tv_nsec %= 1000000000UL;
		pthread_cond_timedwait(&render_cond, &render_lock,	&last_render_time);
		pthread_mutex_unlock(&render_lock);
	}
#endif
	int64 tSwap = getTickCount();
/******************************************************************************/

	glutSwapBuffers();

tStamp[6] = getTickCount();

if(0)
{
	static int64 tmpStamp = 0ul;
	if(tmpStamp != 0){
		int32 interval = (tStamp[6] - tmpStamp)*0.000001;
		static int32 tMax = 0;
		static int32 tMin = 10000;
		static unsigned int tcnt = 0;
		tMax = max(tMax, interval);
		tMin = min(tMin, interval);
		tcnt ++;
		if(tcnt == 100){
			OSA_printf("%s %d: %d (%d, %d)", __FILE__, __LINE__, interval, tMin, tMax);
			tcnt = 0;tMax = 0; tMin = 10000;
		}
	}
	tmpStamp = tStamp[6];
}

#if 1
	if(tStamp[6]-tSwap>5000000UL)
		m_nSwapTimeOut++;
	else
		m_nSwapTimeOut = 0;
	if (last_render_time.tv_sec == 0 || m_nSwapTimeOut>3)
	{
		struct timeval now;
		gettimeofday(&now, NULL);
		last_render_time.tv_sec = now.tv_sec;
		last_render_time.tv_nsec = now.tv_usec * 1000L;
		printf("\r\nReset render timer. fps(%d) swp(%ld)", m_initPrm.disFPS, tStamp[6]-tSwap);
		fflush(stdout);
	}
#endif
	tend = tStamp[6];
	float renderIntv = (tend - m_tmRender)/getTickFrequency();
#if 1
	static unsigned long rCount = 0;
	if(rCount%(m_initPrm.disFPS*100) == 0){
		printf("\r\n[%d] %.4f (ws%.4f,cu%.4f,tv%.4f,to%.4f,rd%.4f,wp%.4f) %.4f(%.4f)",
			OSA_getCurTimeInMsec(),renderIntv,
			(tStamp[1]-tStamp[0])/getTickFrequency(),
			(tStamp[2]-tStamp[1])/getTickFrequency(),
			(tStamp[3]-tStamp[2])/getTickFrequency(),
			(tStamp[4]-tStamp[3])/getTickFrequency(),
			(tStamp[5]-tStamp[4])/getTickFrequency(),
			(tStamp[6]-tStamp[5])/getTickFrequency(),
			m_telapse, m_initPrm.disSched
			);
		fflush(stdout);
	}
	rCount ++;
#endif
	m_tmRender = tend;

	glutPostRedisplay();
	
	GetFPS();
//	cout << "==========<GetFPS()>=====================  FPS = "<< frameCount << "ms"<<endl;
}

void CDisplayer::IrisAndFocus()
{
	switch(IrisAndFocusAndExit)
	{
	case Enable_Iris:
		chinese_osd(905,1000,L"光圈调节",1,4,255,0,0,255,VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
		drawtriangle(plat->m_display.m_imgOsd[plat->extInCtrl->SensorStat], cmd_triangle.dir, cmd_triangle.alpha);
		break;

	case Enable_Focus:
		chinese_osd(905,1000,L"聚焦调节",1,4,255,0,0,255,VIDEO_DIS_WIDTH,VIDEO_DIS_HEIGHT);
		drawtriangle(plat->m_display.m_imgOsd[plat->extInCtrl->SensorStat], cmd_triangle.dir, cmd_triangle.alpha);
		break;
	case Disable:
		drawtriangle(plat->m_display.m_imgOsd[plat->extInCtrl->SensorStat], cmd_triangle.dir, cmd_triangle.alpha);
		break;
	}
}

int CDisplayer::OSDFunc()
{
	//OSD_param* posd = NULL;
	//posd = &m_osd;

	unsigned char r, g, b, a, color, colorbak, Enable;
	short x, y;
	char font,fontsize;

	for(int i = 0;i<32;i++){
		Enable = disOsdBufbak[i].ctrl;
		if(!Enable){
			 x = disOsdBufbak[i].posx;
			 y = disOsdBufbak[i].posy;
		 	 a = disOsdBufbak[i].alpha;
			 color = disOsdBufbak[i].color;
			 font = plat->extInCtrl->osdTextFont;
			 fontsize = plat->extInCtrl->osdTextSize;

			switch(color)
			{
				case 1:
					r = 0;
					g = 0;
					b = 0;
					break;
				case 2:
					r = 255;
					g = 255;
					b = 255;
					break;
				case 3:
					r = 255;
					g = 0;
					b = 0;
					break;
				case 4:
					r = 255;
					g = 255;
					b = 0;
					break;
				case 5:
					r = 0;
					g = 0;
					b = 255;
					break;
				case 6:
					r = 0;
					g = 255;
					b = 0;
					break;
				case 7:
					color = colorbak;
					break;
				}

			if(a > 0x0a)
				a = 0x0a;
			if(a == 0x0a)
				a = 0;
			else
				a = 255 - a*16;
			
			chinese_osd(x, y, disOsd[i],font ,fontsize, r, g, b, a, VIDEO_DIS_WIDTH, VIDEO_DIS_HEIGHT);
		}
	}

	return 0;
}

int CDisplayer::MenuFunc(int index)
{
	unsigned char r, g, b, a, color, colorbak, Enable;
	short x, y;
	char font,fontsize;
	
	if(-1 == index)
		return -1;

	for(int i = 0; i < MAX_SUBMENU; i++)
	{
		if(i == dismenuarray[index].submenu_cnt)
			break;
		Enable = disMenuBuf[index][i].ctrl;
		if(!Enable)
		{
			 x = disMenuBuf[index][i].posx;
			 y = disMenuBuf[index][i].posy;
		 	 a = disMenuBuf[index][i].alpha;
			 color = disMenuBuf[index][i].color;
			 font = plat->extInCtrl->osdTextFont;
			 fontsize = plat->extInCtrl->osdTextSize;
			 //font = ;
			 //fontsize = ;

			switch(color)
			{
				case 1:
					r = 0;
					g = 0;
					b = 0;
					break;
				case 2:
					r = 255;
					g = 255;
					b = 255;
					break;
				case 3:
					r = 255;
					g = 0;
					b = 0;
					break;
				case 4:
					r = 255;
					g = 255;
					b = 0;
					break;
				case 5:
					r = 0;
					g = 0;
					b = 255;
					break;
				case 6:
					r = 0;
					g = 255;
					b = 0;
					break;
				case 7:
					color = colorbak;
					break;
			}

			if(a > 0x0a)
				a = 0x0a;
			if(a == 0x0a)
				a = 0;
			else
				a = 255 - a*16;
			chinese_osd(x, y, disMenu[index][i],font ,fontsize, r, g, b, a, VIDEO_DIS_WIDTH, VIDEO_DIS_HEIGHT);
		}
	}
	
	return 0;
}

void CDisplayer::drawtriangle(Mat frame, char direction, char alpha)
{
	Point root_points[1][3];
	int npt[] = {3};

	switch(direction)
	{
	case up:
	root_points[0][0] = Point(1015,1006);
	root_points[0][1] = Point(1005,1026);
	root_points[0][2] = Point(1025,1026);
	break;

	case down:
		root_points[0][0] = Point(1015,1026);
		root_points[0][1] = Point(1005,1006);
		root_points[0][2] = Point(1025,1006);
		break;
		default:
		break;
	}
	const Point* ppt[1] = {root_points[0]};
	polylines(frame, ppt, npt, 1, 1, Scalar(255),1,8,0);
	fillPoly(frame, ppt, npt, 1, Scalar(0,0,255,alpha));
}

//////////////////////////////////////////////////////////////////////////
// Load the shader from the source text
void CDisplayer::gltLoadShaderSrc(const char *szShaderSrc, GLuint shader)
{
	GLchar *fsStringPtr[1];

	fsStringPtr[0] = (GLchar *)szShaderSrc;
	glShaderSource(shader, 1, (const GLchar **)fsStringPtr, NULL);
}

#define MAX_SHADER_LENGTH   8192
static GLubyte shaderText[MAX_SHADER_LENGTH];
////////////////////////////////////////////////////////////////
// Load the shader from the specified file. Returns false if the
// shader could not be loaded
bool CDisplayer::gltLoadShaderFile(const char *szFile, GLuint shader)
{
	GLint shaderLength = 0;
	FILE *fp;

	// Open the shader file
	fp = fopen(szFile, "r");
	if(fp != NULL)
	{
		// See how long the file is
		while (fgetc(fp) != EOF)
			shaderLength++;

		// Allocate a block of memory to send in the shader
		assert(shaderLength < MAX_SHADER_LENGTH);   // make me bigger!
		if(shaderLength > MAX_SHADER_LENGTH)
		{
			fclose(fp);
			return false;
		}

		// Go back to beginning of file
		rewind(fp);

		// Read the whole file in
		if (shaderText != NULL){
			size_t ret = fread(shaderText, 1, (size_t)shaderLength, fp);
			OSA_assert(ret == shaderLength);
		}

		// Make sure it is null terminated and close the file
		shaderText[shaderLength] = '\0';
		fclose(fp);
	}
	else
		return false;    

	// Load the string
	gltLoadShaderSrc((const char *)shaderText, shader);

	return true;
}   


/////////////////////////////////////////////////////////////////
// Load a pair of shaders, compile, and link together. Specify the complete
// source text for each shader. After the shader names, specify the number
// of attributes, followed by the index and attribute name of each attribute
GLuint CDisplayer::gltLoadShaderPairWithAttributes(const char *szVertexProg, const char *szFragmentProg, ...)
{
	// Temporary Shader objects
	GLuint hVertexShader;
	GLuint hFragmentShader; 
	GLuint hReturn = 0;   
	GLint testVal;

	// Create shader objects
	hVertexShader = glCreateShader(GL_VERTEX_SHADER);
	hFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	// Load them. If fail clean up and return null
	// Vertex Program
	if(gltLoadShaderFile(szVertexProg, hVertexShader) == false)
	{
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		fprintf(stderr, "The shader at %s could ot be found.\n", szVertexProg);
		return (GLuint)NULL;
	}

	// Fragment Program
	if(gltLoadShaderFile(szFragmentProg, hFragmentShader) == false)
	{
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		fprintf(stderr,"The shader at %s  could not be found.\n", szFragmentProg);
		return (GLuint)NULL;
	}

	// Compile them both
	glCompileShader(hVertexShader);
	glCompileShader(hFragmentShader);

	// Check for errors in vertex shader
	glGetShaderiv(hVertexShader, GL_COMPILE_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
		char infoLog[1024];
		glGetShaderInfoLog(hVertexShader, 1024, NULL, infoLog);
		fprintf(stderr, "The shader at %s failed to compile with the following error:\n%s\n", szVertexProg, infoLog);
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
	}

	// Check for errors in fragment shader
	glGetShaderiv(hFragmentShader, GL_COMPILE_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
		char infoLog[1024];
		glGetShaderInfoLog(hFragmentShader, 1024, NULL, infoLog);
		fprintf(stderr, "The shader at %s failed to compile with the following error:\n%s\n", szFragmentProg, infoLog);
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
	}

	// Create the final program object, and attach the shaders
	hReturn = glCreateProgram();
	glAttachShader(hReturn, hVertexShader);
	glAttachShader(hReturn, hFragmentShader);


	// Now, we need to bind the attribute names to their specific locations
	// List of attributes
	va_list attributeList;
	va_start(attributeList, szFragmentProg);

	// Iterate over this argument list
	char *szNextArg;
	int iArgCount = va_arg(attributeList, int);	// Number of attributes
	for(int i = 0; i < iArgCount; i++)
	{
		int index = va_arg(attributeList, int);
		szNextArg = va_arg(attributeList, char*);
		glBindAttribLocation(hReturn, index, szNextArg);
	}
	va_end(attributeList);

	// Attempt to link    
	glLinkProgram(hReturn);

	// These are no longer needed
	glDeleteShader(hVertexShader);
	glDeleteShader(hFragmentShader);  

	// Make sure link worked too
	glGetProgramiv(hReturn, GL_LINK_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
		char infoLog[1024];
		glGetProgramInfoLog(hReturn, 1024, NULL, infoLog);
		fprintf(stderr,"The programs %s and %s failed to link with the following errors:\n%s\n",
			szVertexProg, szFragmentProg, infoLog);
		glDeleteProgram(hReturn);
		return (GLuint)NULL;
	}

	// All done, return our ready to use shader program
	return hReturn;  
}   


void CDisplayer::GetFPS()
{
	#define FR_SAMPLES 10
	static struct timeval last={0,0};
	struct timeval now;
	float delta;
	if ((++frameCount) >= FR_SAMPLES) {
		gettimeofday(&now, NULL);
		delta= (now.tv_sec - last.tv_sec +(now.tv_usec - last.tv_usec)/1000000.0);
		last = now;
		setFrameRate(FR_SAMPLES / delta);
		setFrameCount(0);
	}
}

FLOAT2 CDisplayer::chinese_osd(int x,int y,wchar_t* text,char font,char fontsize,unsigned char r,unsigned char g,unsigned char b,unsigned char a,int win_width,int win_height)
{
	FLOAT2 VerPos;
	glUseProgram(m_fontProgram);
	_fontColor[0] = (float)r/float(255);
	_fontColor[1] = (float)g/float(255);
	_fontColor[2] = (float)b/float(255);
	_fontColor[3] = (float)a/float(255);
	glUniform4fv(Uniform_font_color,1,_fontColor);
	VerPos = OSDdrawText(x,y,text,font,fontsize,win_width,win_height);
	glUseProgram(0);
	return VerPos;
}
GLbyte* CDisplayer::gltReadBMPBits(const char *szFileName, int *nWidth, int *nHeight)
{
	FILE*	pFile;
	BMPInfo *pBitmapInfo = NULL;
	unsigned long lInfoSize = 0;
	unsigned long lBitSize = 0;
	GLbyte *pBits = NULL;					// Bitmaps bits
	BMPHeader	bitmapHeader;

    // Attempt to open the file
    pFile = fopen(szFileName, "rb");
    if(pFile == NULL)
        return NULL;

	// File is Open. Read in bitmap header information
    fread(&bitmapHeader, sizeof(BMPHeader), 1, pFile);

	// Read in bitmap information structure
	lInfoSize = bitmapHeader.offset - sizeof(BMPHeader);
	pBitmapInfo = (BMPInfo *) malloc(sizeof(GLbyte)*lInfoSize);
	if(fread(pBitmapInfo, lInfoSize, 1, pFile) != 1)
	{
	free(pBitmapInfo);
	fclose(pFile);
	return false;
	}

	// Save the size and dimensions of the bitmap
	*nWidth = pBitmapInfo->header.width;
	*nHeight = pBitmapInfo->header.height;
	lBitSize = pBitmapInfo->header.imageSize;

	// If the size isn't specified, calculate it anyway	
	if(pBitmapInfo->header.bits != 24)
	{
	free(pBitmapInfo);
	return false;
	}

	if(lBitSize == 0)
		lBitSize = (*nWidth *
           pBitmapInfo->header.bits + 7) / 8 *
  		  abs(*nHeight);

	// Allocate space for the actual bitmap
	free(pBitmapInfo);
	pBits = (GLbyte*)malloc(sizeof(GLbyte)*lBitSize);

	// Read in the bitmap bits, check for corruption
	if(fread(pBits, lBitSize, 1, pFile) != 1)
		{
		free(pBits);
		pBits = NULL;
		}

	// Close the bitmap file now that we have all the data we need
	fclose(pFile);

	return pBits;
	}
bool CDisplayer::LoadBMPTexture(const char *szFileName, GLenum minFilter, GLenum magFilter, GLenum wrapMode)	
{
	GLbyte *pBits;
	GLint iWidth, iHeight;
	cv::Mat bmpImage;

	pBits = gltReadBMPBits(szFileName, &iWidth, &iHeight);
	if(pBits == NULL)
		return false;
	bmpImage = cv::Mat(iHeight, iWidth, CV_8UC3, pBits);
	cv::flip(bmpImage, bmpImage, 0);

	// Set Wrap modes
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapMode);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapMode);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, iWidth, iHeight, 0, GL_BGR, GL_UNSIGNED_BYTE, pBits);

    // Do I need to generate mipmaps?
	if(minFilter == GL_LINEAR_MIPMAP_LINEAR || minFilter == GL_LINEAR_MIPMAP_NEAREST || minFilter == GL_NEAREST_MIPMAP_LINEAR || minFilter == GL_NEAREST_MIPMAP_NEAREST)
		glGenerateMipmap(GL_TEXTURE_2D);    

	return true;
}



