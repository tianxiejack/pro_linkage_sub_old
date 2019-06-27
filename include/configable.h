
#ifndef CONFIG_ABLE_H__
#define CONFIG_ABLE_H__

#include "string.h"

#define AIM_WIDTH				64
#define AIM_HEIGHT				64

#define AVT_TRK_AIM_SIZE			2
#define MAX_MTDRIGION_NUM 		1
#define MIN_MTDTARGET_NUM		1
#define MAX_MTDTARGET_NUM		10
#define MIN_MTDTRKTIME		1
#define MAX_MTDTRKTIME		20
#define MIN_MTDMINSIZE		225
#define MAX_MTDMAXSIZE		70000
#define MIN_MTDSENSI		10
#define MAX_MTDSENSI		100

#define MIN_VALID_RECT_WIDTH_IN_PIXEL 10

#define TRIG_RADIUS	4


typedef enum{
	mainmenu0=0,
	mainmenu1,
	mainmenu2,
	submenu_DefaultWorkMode,// submenu_carli
	//submenu_gridMapCalibrate, //"submenu_gunball" is replaced by "submenu_gridMapCalibrate"
	submenu_mtd,
	submenu_setimg,
	submenu_setball,
	submenu_setcom,
	submenu_setnet,
	submenu_handleMatchPoints,// for handle calibrate match points 
	menumaxid
}AppMenuId;

typedef struct
{
	int id;
	int pointer;
	int submenu_cnt;
	int start;
	int end;
}AppMenu;

typedef struct _Resolution{
	int raw, col ;
}Resolution;

typedef struct _viewPortConfig{
	int x,y; // viewport left down position coordinates;
	int width,height; // viewport window width and height;

}viewPortConfig;
typedef struct _gunPositionPIP{
	viewPortConfig leftUp,leftDown;
	viewPortConfig rightUp,rightDown;
	viewPortConfig general;
}gunPositionPIP;
typedef struct _posValue{
	unsigned char LeftUp, LeftDown, RightUp, RightDown;
}posValue;
typedef struct _sizeValue{
	unsigned char oneOf2,oneOf3,oneOf4;
}sizeValue;
typedef struct _cameraCalibrate{
	bool Enable_handleCalibrate; // 'Y'
	bool Enable_cloneSrcImage;  // 'V'
	bool Enable_calculateMatrix; // 'M'
	bool Enable_saveParameter;  // 'U'
	bool Enable_Undistortion;
	bool Enable_AutoDetectMoveTargets;

}cameraCalibrate;
typedef struct _SysParam{
	Resolution gun_camera;
	Resolution ball_camera;
	gunPositionPIP gunposition;
	posValue posvalue;
	sizeValue sizevalue;
	cameraCalibrate cameracalibrate;
}SysParam;

#define MAX_SUBMENU 8

class SingletonSysParam {
	public:
		typedef enum{
			LU=0,
			RU,
			LD,
			RD,
			POS_COUNT
		}POS;
		typedef enum{
			ONE_2 =0,
			ONE_3,
			ONE_4,
			SIZE_COUNT
		}SIZE;
	private:
		SysParam sysParameters;
		
	public:
		/*volatile*/ static SingletonSysParam* getInstance(){
			if(m_uniqueInstance == NULL) {
				m_uniqueInstance = new SingletonSysParam();
			};
			return m_uniqueInstance;
		};
		SysParam& getSysParam(){
			return sysParameters;
		};
		void setGunPosition(POS value){
			switch(value){
				case LU:
					sysParameters.posvalue.LeftUp = 1;
					sysParameters.posvalue.RightUp =0;
					sysParameters.posvalue.LeftDown =0;
					sysParameters.posvalue.RightDown =0;
					break;
				case RU:
					sysParameters.posvalue.LeftUp = 0;
					sysParameters.posvalue.RightUp =1;
					sysParameters.posvalue.LeftDown =0;
					sysParameters.posvalue.RightDown =0;
					break;
				case LD:
					sysParameters.posvalue.LeftUp = 0;
					sysParameters.posvalue.RightUp =0;
					sysParameters.posvalue.LeftDown =1;
					sysParameters.posvalue.RightDown =0;
					break;
				case RD:
					sysParameters.posvalue.LeftUp = 0;
					sysParameters.posvalue.RightUp =0;
					sysParameters.posvalue.LeftDown =0;
					sysParameters.posvalue.RightDown =1;
					break;
				default:
					break;					
			}
		};
		unsigned char getGunPosition(POS value){
				switch(value){
					case LU:
						return sysParameters.posvalue.LeftUp;
						break;
					case RU:
						return sysParameters.posvalue.RightUp;
						break;
					case LD:
						return sysParameters.posvalue.LeftDown;
						break;
					case RD:
						return sysParameters.posvalue.RightDown;
						break;
					default:
						break;
				}
		};
		void setGunSize(SIZE size){
			switch(size){
				case ONE_2:
					sysParameters.sizevalue.oneOf2 =1;
					sysParameters.sizevalue.oneOf3 =0;
					sysParameters.sizevalue.oneOf4 =0;
					break;
				case ONE_3:
					sysParameters.sizevalue.oneOf2 =0;
					sysParameters.sizevalue.oneOf3 =1;
					sysParameters.sizevalue.oneOf4 =0;
					break;
				case ONE_4:
					sysParameters.sizevalue.oneOf2 =0;
					sysParameters.sizevalue.oneOf3 =0;
					sysParameters.sizevalue.oneOf4 =1;
					break;
				default:
					break;
			}
		};
		unsigned char getGunSize(SIZE size){
			switch(size){
				case ONE_2:
					return sysParameters.sizevalue.oneOf2;
					break;
				case ONE_3:
					return sysParameters.sizevalue.oneOf3;
					break;
				case ONE_4:
					return sysParameters.sizevalue.oneOf4;
					break;
				default:
					break;					
			}
		};
		bool isEnable_HandleCalibrate(){
			return sysParameters.cameracalibrate.Enable_handleCalibrate;
		};
		bool isEnable_cloneSrcImage(){
			return sysParameters.cameracalibrate.Enable_cloneSrcImage;
		};
		bool isEnable_calculateMatrix(){
			return sysParameters.cameracalibrate.Enable_calculateMatrix;
		};
		bool isEnable_saveParameter(){
			return sysParameters.cameracalibrate.Enable_saveParameter;
		};
		bool isEnable_Undistortion(){
			return sysParameters.cameracalibrate.Enable_Undistortion;
		};
		bool isEnable_AutoDetectMoveTargets(){
			return sysParameters.cameracalibrate.Enable_AutoDetectMoveTargets;
		};
	private:
		/*volatile*/ static SingletonSysParam* m_uniqueInstance;
		SingletonSysParam(){
			memset(&sysParameters,0,sizeof(SysParam)); 	// initialize system parameters;
			sysParameters.cameracalibrate.Enable_calculateMatrix = false;
			sysParameters.cameracalibrate.Enable_cloneSrcImage = false;
			sysParameters.cameracalibrate.Enable_handleCalibrate = false;
			sysParameters.cameracalibrate.Enable_saveParameter = false;
			sysParameters.cameracalibrate.Enable_Undistortion = false;
			sysParameters.cameracalibrate.Enable_AutoDetectMoveTargets = false;
		};
		virtual ~SingletonSysParam(){};
};
//=================================================================
#define CAM_0  0
#define CAM_1  1

typedef enum _MenuDisplay{
	MENU_MAIN_VIEW=0,
	MENU_SBS,
	MENU_GUN,
	MENU_BALL,
	MENU_CALIBRA_CAP,
	MENU_CALIBRA_RESULT,
	MENU_MATCH_POINT_VIEW,
	MENU_TEST_RESULT_VIEW,
	MENU_TRIG_INTER_MODE,
	MENU_GRID_MAP_VIEW,
	MENU_DISPLAY_COUNT		
}MenuDisplay;
typedef enum _GB_WorkMode{ // This App Has three work mode : Handle, Auto, and Only Contrl Ball camera
	HANDLE_LINK_MODE =0,
	AUTO_LINK_MODE,
	ONLY_BALL_MODE,
	MODE_COUNT		
}GB_WorkMode;

typedef enum _GRIDINTER_Mode{
	GRIDINTER_CTRLBALL_MODE = 0,
	GRIDINTER_MOUSE_MODE,
	GRIDINTER_COUNT,
}GRIDINTER_Mode;

typedef struct _UI_CONNECT_ACTION{	
	int CurCalibraCam;   // gun camera == 0;  ball camera == 1;
}UI_CONNECT_ACTION;

typedef enum _GB_CLICK_MODE{
	CLICK_MODE=0,
	RECT_SELECT_MODE,
	MOUSE_MODE_COUNT
}GB_CLICK_MODE;


typedef enum _SelectMode{
	Click_Mode =0,
	DrawRectangle_Mode,
	SetMteRigion_Mode,
	Mode_Count
}SelectMode;

typedef struct _BallCOMConfig{
	volatile int ballAdrress;
	volatile int ballRate;

}BallCOMConfig;

//================================================================
enum devvideo{
	video_gaoqing0=0,
	video_gaoqing,
	video_gaoqing2,
	video_gaoqing3,
	video_pal,
	MAX_CHAN,
};

enum resol_t{
	r1920x1080_f60,
	r1024x768_f60,
	r1280x1024_f60,
	maxresolid,
};

enum baud_t{
	baud_2400,
	baud_4800,
	baud_9600,
	baud_15200,
	MAX_BAUDID
};


#define MAIN_CHID					video_gaoqing
#define PAL_VIRCHID					0

#define VIDEO_DIS_WIDTH		1920
#define VIDEO_DIS_HEIGHT		1080

#define BALL_CHID			video_gaoqing
#define GUN_CHID			video_gaoqing0

#define min_width_ratio 0.2
#define max_width_ratio 0.8
#define min_height_ratio 0.2
#define max_height_ratio 0.8

extern int vcapWH[5][2];
extern int vdisWH[5][2];
extern int outputWHF[3];
extern int oresoltype;

#endif
