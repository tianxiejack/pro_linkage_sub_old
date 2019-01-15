/*
 * CcCamCalibra.cpp
 *
 *  Created on: Oct 16, 2018
 *      Author: d
 */

#include "CcCamCalibra.h"
#include <sys/time.h>
#include "configable.h"
#include "Ipcctl.h"
#include <vector>
#include <iostream>
using namespace std;

CamParameters g_camParams;
OSA_SemHndl m_linkage_getPos;
OSA_SemHndl g_detectCorners;
extern SingletonSysParam* g_sysParam;
extern bool showDetectCorners ;
extern volatile bool rendeFlag;
extern bool saveOnePicture;
extern int saveCount;

Mat g_CornerImage;
Mat g_WarpImage;
Mat g_MatchImage;
extern bool g_bSubmitWarpTexture;
extern bool g_bSubmitTexture;
extern bool g_bSubmitMatchTexture;

bool captureOnePicture = false;
int captureCount = 0;
extern vector<Mat> imageListForCalibra;
bool _bCutIMG[100]= {false};
bool start_calibrate = false;

CcCamCalibra::CcCamCalibra():scale(0.5),bCal(false),ret1(false),ret2(false),
	panPos(1024), tiltPos(13657), zoomPos(16),writeParam_flag(false),
	Set_Handler_Calibra(false),bool_Calibrate(false),start_cloneVideoSrc(false)
{
	gun_BMP = imread("gun.bmp");
	
	if(gun_BMP.empty())
		cout << "Open BMP gun Picture Failed!" <<endl;
	else
		cout << "Open BMP gun Picture Success!" <<endl;
	
	ball_BMP = imread("ball.bmp");

	if(ball_BMP.empty())
		cout << "Open BALL gun Picture Failed!" <<endl;
	else
		cout << "Open BALL gun Picture Success!" <<endl;
	
	OSA_semCreate(&m_linkage_getPos, 1, 0);
	getCurrentPosFlag = 0 ;
	ImageLists.clear();
	corner_points_of_all_imgs.clear();
	corner_points_buf.clear();
	
	int pattern_cols = 13;
  	int pattern_raws = 8;
   	pattern_size = Size(pattern_cols,pattern_raws);     
}

CcCamCalibra::~CcCamCalibra() {
	StopService();
}
bool CcCamCalibra::Load_CameraParams(const string& gun_file,const string& ball_file)
{	
	ret1 = load_OriginCameraParams(ball_file, flags, cameraMatrix_ball, distCoeffs_ball, imageSize);
	ret2 = load_OriginCameraParams(gun_file, flags, cameraMatrix_gun, distCoeffs_gun, imageSize);
	if(ret1 && ret2) {
		return true;
	}
	else{
		return false;
	}
}


void CcCamCalibra::PrintMs( const char* text )
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

void CcCamCalibra::Init_CameraParams()
{
	if(ret1 && ret2) {
		newCameraMatrix = cameraMatrix_ball;	
		remapImage();
	}
}

void CcCamCalibra::cvtGunYuyv2Bgr()
{
	if(!gun_yuyv.empty())
		cvtColor(gun_yuyv,gun_frame,CV_YUV2BGR_YUYV);
}

void CcCamCalibra::cvtBallYuyv2Bgr()
{
	if(!ball_yuyv.empty()){
		cvtColor(ball_yuyv,ball_frame,CV_YUV2BGR_YUYV);
	}
}


void CcCamCalibra::getBallSrcImage(Mat &src)
{
	ball_frame = src;	
}

void CcCamCalibra::getGunSrcImage(Mat &src)
{
	gun_frame = src;	
}

void CcCamCalibra::cloneBallSrcImgae(Mat &src)
{
	src.copyTo(ball_yuyv);
}
void CcCamCalibra::cloneGunSrcImgae(Mat &src)
{
	src.copyTo(gun_yuyv);
}


int CcCamCalibra::RunService()
{
	m_prm.pThis = this;
	return RunThread(RunProxy, &m_prm);
}

int CcCamCalibra::StopService()
{
	 SetThreadExit();
	 WaitThreadExit();
	 return 0;
}

void* CcCamCalibra::RunProxy(void* pArg)
{
	 struct timeval tv;
	 struct RunPrm *pPrm = (struct RunPrm*)pArg;
	 while(pPrm->pThis->m_bRun)
	 {		
	 
	 	tv.tv_sec = 0;
    		tv.tv_usec = (30%1000)*1000;
   		select(0, NULL, NULL, NULL, &tv);
		
		pPrm->pThis->Run();
	 }
	 return NULL;
}
bool CcCamCalibra::load_OriginCameraParams( const string& filename, int& flags, Mat& cameraMatrix2, Mat& distCoeffs2, Size& imageSize)
{
    FileStorage fs2( filename, FileStorage::READ );
    bool ret = fs2.isOpened();
    if(!ret){
        cout << filename << " can't opened !\n" << endl;
        return ret;
    }
    cv::String date;
    fs2["calibration_time"] >> date;
    int frameCount = (int)fs2["nframes"];
    int imgwidth = (int)fs2["image_width"];
    int imgheight = (int)fs2["image_height"];
    imageSize.width = imgwidth;
    imageSize.height = imgheight;
    double totalAvgErr;
    fs2["camera_matrix"] >> cameraMatrix2;
    fs2["distortion_coefficients"] >> distCoeffs2;
    fs2["avg_reprojection_error"] >> totalAvgErr;
    cout << "calibration date: " << date << endl
         << "image width:      " << imgwidth << endl
         << "image height:     " << imgheight << endl
         << "camera matrix:    \n" << cameraMatrix2 << endl
         << "distortion coeffs:\n" << distCoeffs2 << endl
         << "avg error:        " << totalAvgErr << endl;   
    fs2.release();
    return ret;
}

 bool CcCamCalibra::saveLinkageParams( const char* filename,
                       Size imageSize, const Mat& cameraMatrix_gun, const Mat& distCoeffs_gun,
                               const Mat& cameraMatrix_ball, const Mat& distCoeffs_ball, const Mat& homography,
                               int panPos, int tiltPos, int zoomPos)
{
    FileStorage fs( filename, FileStorage::WRITE );
    bool ret = fs.isOpened();
    if(!ret){
        cout << filename << " can't opened !\n" << endl;
        return ret;
    }
    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );
    fs << "calibration_time" << buf;
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "camera_matrix_gun" << cameraMatrix_gun;
    fs << "distortion_coefficients_gun" << distCoeffs_gun;
    fs << "camera_matrix_ball" << cameraMatrix_ball;
    fs << "distortion_coefficients_ball" << distCoeffs_ball;
    fs << "homography" << homography;
    fs << "ballPanPos" << panPos;
    fs << "ballTiltPos" << tiltPos;
    fs << "ballZoomPos" << zoomPos;
    fs.release();
    return ret;
}

void CcCamCalibra::setBallPos(int in_panPos, int in_tilPos, int in_zoom)
{
	panPos = in_panPos ;
	tiltPos = in_tilPos ;
	zoomPos = in_zoom ;
	OSA_semSignal(&m_linkage_getPos);
}

int CcCamCalibra::Run()
{
#if 0
	//static bool calibrate_once = true;
	if(/*calibrate_once && */start_calibrate ){
		 start_calibrate = false;
		//calibrate_once = true;
		 FindCorners();
		 //FindPatternCorners();
		 getObjectCoordinates();
		 calibrate();		
	}
#endif
	char flag = 0;
	Mat frame = ball_frame;
	cvtGunYuyv2Bgr();
	cvtBallYuyv2Bgr();
#if 1	
	if(!gun_frame.empty()){
		remap(gun_frame, undisImage, map1, map2, INTER_LINEAR);
	}
#else
	if(!gun_BMP.empty()){
		remap(gun_BMP, undisImage, map1, map2, INTER_LINEAR);
	}
#endif
	Mat gunDraw, ballDraw;
	if( (!ball_frame.empty()) && (!gun_frame.empty()) )
	{
		if( (Set_Handler_Calibra /*|| g_sysParam->isEnable_Undistortion() */) && (bool_Calibrate /*|| g_sysParam->isEnable_calculateMatrix()*/) ) {		
			printf("%s : start manual calibrate \n",__func__);
			
			if(key_points1.size() > 4 && key_points2.size() > 4){
				Mat R,t;
				vector<Point2f> keypt1, keypt2;
				keypt1.clear();
				keypt2.clear();
				for(int i=0; i<key_points1.size(); i++){
					keypt1.push_back(cv::Point2f(key_points1[i].x*2, key_points1[i].y*2));
					keypt2.push_back(cv::Point2f(key_points2[i].x*2, key_points2[i].y*2));
				}
				handle_pose_2d2d( keypt2, keypt1,newCameraMatrix, R, t, homography);
				
				cout << "Key_points1.size() = " << key_points1.size() << endl;
				cout << "Key_points2.size() = " << key_points2.size() << endl;
				bool_Calibrate = false;
			}
			
			if(!homography.empty()) {
				Mat warp;
				if( !undisImage.empty()) {
					warpPerspective(undisImage, warp, homography, undisImage.size());
					resize(warp, warp, Size(warp.cols*scale, warp.rows*scale));					
					drawChessboardCorners(warp, boardSize, key_points1, false);		
					//imshow("camera gun warp", warp);
					warp.copyTo(g_WarpImage);
					g_bSubmitWarpTexture = true;
				}
			}
		}
		
		else 
		{	
			if( bool_Calibrate/* || g_sysParam->isEnable_calculateMatrix()*/) {
				
				vector<KeyPoint> keypoints_1, keypoints_2;
				vector<DMatch> matches;
				find_feature_matches ( undisImage, frame,  keypoints_1, keypoints_2, matches , 60.0, true);
				if(matches.size() > 4){
					Mat R,t;
					pose_2d2d ( keypoints_1, keypoints_2, matches, newCameraMatrix, R, t, homography);					
					pts.clear();
					for(int i=0; i<matches.size(); ++i) {
						cv::Point2f pt = keypoints_2[matches[i].trainIdx].pt;
						pt.x *= scale;
						pt.y *= scale;
						pts.push_back(pt);
					}					
					bool_Calibrate = false;
					//g_sysParam->getSysParam().cameracalibrate.Enable_calculateMatrix = false;
					cout << "match points " << matches.size() << endl;
//-----------------------------------------------------------------------------------------------------
					SENDST trkmsg={0};
					trkmsg.cmd_ID = querypos;
					ipc_sendmsg(&trkmsg, IPC_FRIMG_MSG);
					printf("\r\n [%s]===== Send Message to Ball Camera : MsgID =  querypos \r\n",__FUNCTION__);

					flag = OSA_semWait(&m_linkage_getPos, /*OSA_TIMEOUT_FOREVER*/500);
					if( -1 == flag ) {
						getCurrentPosFlag = false;
						printf("%s:LINE :%d    could not get the ball current Pos \n",__func__,__LINE__ );
					}
					else{
						printf("\r\n [%s]:=====Recive Meaasge From Ball Camera !!!",__FUNCTION__);
						getCurrentPosFlag = true;
						cout << "\n\n******************Get It << Query Ball Camera Position Sucess ! *****************" << endl;
						cout << " panPos = "<< panPos << endl;
						cout << " tiltPos = "<< tiltPos << endl;
						cout << " zoomPos = "<< zoomPos << endl;
						cout << "*****************************************************************" << endl;
					}
//----------------------------------------------------------------------------------------------------
					
				}
			}
			//printf("[%s] :  bool_Calibrate = %d\r\n", __FUNCTION__,bool_Calibrate);
			
			if(!homography.empty()) {
				Mat warp;
				if( !undisImage.empty()){
					warpPerspective(undisImage, warp, homography, undisImage.size());
					resize(warp, warp, Size(warp.cols*scale, warp.rows*scale));	
					if(Set_Handler_Calibra == true) {
						drawChessboardCorners(warp, boardSize, key_points1, false);			
					}
					else{
						drawChessboardCorners(warp, boardSize, pts, false);	
					}
					//imshow("camera gun warp", warp);
					warp.copyTo(g_WarpImage);
					g_bSubmitWarpTexture = true;
				}
			}	
		}
	}
	if( writeParam_flag || g_sysParam->isEnable_saveParameter()) 
	{
		writeParam_flag = false;
		g_sysParam->getSysParam().cameracalibrate.Enable_saveParameter = false;
		if( !getCurrentPosFlag )
		{
			cout << "could not get the cuurent Flag \n" << endl;
			return -1;
		}
		else
			getCurrentPosFlag = false;
				
		if(!saveLinkageParams("GenerateCameraParam.yml", imageSize, cameraMatrix_gun, distCoeffs_gun,
                    cameraMatrix_ball, distCoeffs_ball, homography,
                     panPos, tiltPos,zoomPos)) {
			cout << "Write Camera Parameters Failed !!!" << endl;
		}else{
			g_camParams.cameraMatrix_ball = cameraMatrix_ball;
			g_camParams.cameraMatrix_gun = cameraMatrix_gun;
			g_camParams.distCoeffs_ball = distCoeffs_ball;
			g_camParams.distCoeffs_gun = distCoeffs_gun;
			g_camParams.homography = homography;
			g_camParams.imageSize = imageSize;
			g_camParams.panPos = panPos;
			g_camParams.tiltPos = tiltPos;
			g_camParams.zoomPos = zoomPos;		
			cout << "Write Camera Parameters Success !!!" << endl;
			cout << "****************************** Current Newest Camera Matrix data ***************************" << endl;
			cout << "g_camParams.homography = " << g_camParams.homography << endl;
			cout << "g_camParams.panPos = " << g_camParams.panPos << endl;
			cout << "g_camParams.tiltPos = " << g_camParams.tiltPos << endl;
			cout << "g_camParams.zoomPos = " << g_camParams.zoomPos << endl;
			cout << "*********************************************************************************************************" << endl;
		}
	}
	waitKey(1);
	
	return 0;
}

void CcCamCalibra::getCamParam(const char* FileName,int type)
{
   //TODO
}

bool CcCamCalibra::loadLinkageParams( const char* filename,
                       Size& imageSize, Mat& cameraMatrix_gun, Mat& distCoeffs_gun,
                               Mat& cameraMatrix_ball, Mat& distCoeffs_ball, Mat& homography,
                               int& panPos, int& tiltPos, int& zoomPos)
{
    FileStorage fs2( filename, FileStorage::READ );

    bool ret = fs2.isOpened();
    if(!ret){
        cout << filename << " can't opened !\n" << endl;
    }
    cv::String date;
    fs2["calibration_time"] >> date;
    int imgwidth = (int)fs2["image_width"];
    int imgheight = (int)fs2["image_height"];
    fs2["camera_matrix_gun"] >> cameraMatrix_gun;
    fs2["distortion_coefficients_gun"] >> distCoeffs_gun;
    fs2["camera_matrix_ball"] >> cameraMatrix_ball;
    fs2["distortion_coefficients_ball"] >> distCoeffs_ball;
    fs2["homography"] >> homography;
    fs2["ballPanPos"] >> panPos;
    fs2["ballTiltPos"] >> tiltPos;
    fs2["ballZoomPos"] >> zoomPos;
    cout << "calibration date: " << date << endl
         << "image width:      " << imgwidth << endl
         << "image height:     " << imgheight << endl
         << "camera matrix gun:\n" << cameraMatrix_gun << endl
         << "distortion coeffs gun:\n" << distCoeffs_gun << endl
         << "camera matrix ball:\n" << cameraMatrix_ball << endl
         << "distortion coeffs ball:\n" << distCoeffs_ball << endl
         << "homography matrix:\n" << homography << endl
         << "ballPanPos:\n" << panPos << endl
         << "ballTiltPos:\n" << tiltPos << endl
         << "ballZoomPos:\n" << zoomPos << endl;
    imageSize.width = imgwidth;
    imageSize.height = imgheight;
    fs2.release();
    return ret;
}
void CcCamCalibra::setCamMatrix()
{
 // TODO
}

void CcCamCalibra::remapImage()
{
	initUndistortRectifyMap(cameraMatrix_gun, distCoeffs_gun, Mat(),
			newCameraMatrix,	imageSize, CV_16SC2, map1, map2);
}
int CcCamCalibra::find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches ,
                            double distThreshold , bool bDraw )
{

    if(img_1.empty() || img_1.empty())
        return 0;

    //-- åˆå§‹åŒ–
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    //Ptr<FeatureDetector> detector = ORB::create();
    //Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- ç¬¬ä¸€æ­¥:æ£€æµ‹ Oriented FAST è§’ç‚¹ä½ç½®
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- ç¬¬äºŒæ­¥:æ ¹æ®è§’ç‚¹ä½ç½®è®¡ç®— BRIEF æè¿°å­
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    if(descriptors_1.empty() || descriptors_2.empty())
        return 0;

    //-- ç¬¬ä¸‰æ­¥:å¯¹ä¸¤å¹…å›¾åƒä¸­çš„BRIEFæè¿°å­è¿›è¡ŒåŒ¹é…ï¼Œä½¿ç”¨ Hamming è·ç¦»
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- ç¬¬å››æ­¥:åŒ¹é…ç‚¹å¯¹ç­›é€‰
    double min_dist=10000, max_dist=0;

    //æ‰¾å‡ºæ‰€æœ‰åŒ¹é…ä¹‹é—´çš„æœ€å°è·ç¦»å’Œæœ€å¤§è·ç¦», å³æ˜¯æœ€ç›¸ä¼¼çš„å’Œæœ€ä¸ç›¸ä¼¼çš„ä¸¤ç»„ç‚¹ä¹‹é—´çš„è·ç¦»
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    //printf ( "-- Max dist : %f \n", max_dist );
    //printf ( "-- Min dist : %f \n", min_dist );

    //å½“æè¿°å­ä¹‹é—´çš„è·ç¦»å¤§äºŽä¸¤å€çš„æœ€å°è·ç¦»æ—¶,å³è®¤ä¸ºåŒ¹é…æœ‰è¯¯.ä½†æœ‰æ—¶å€™æœ€å°è·ç¦»ä¼šéžå¸¸å°,è®¾ç½®ä¸€ä¸ªç»éªŒå€¼30ä½œä¸ºä¸‹é™.
    double curDistThreshold = min( (max ( 1.5*min_dist, 30.0 )), distThreshold);
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= curDistThreshold )
        {
            matches.push_back ( match[i] );
        }
    }
    //-- ç¬¬äº”æ­¥:ç»˜åˆ¶åŒ¹é…ç»“æžœ
    if(bDraw){
        //Mat img_match;
        Mat img_match;
        drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
        resize(img_match, img_match, Size(img_match.cols/2, img_match.rows/2));		
       //imshow ( "matchWnd", img_match );
        img_match.copyTo(g_MatchImage);
	g_bSubmitMatchTexture = true;
    }
    return matches.size();
}

Point2d CcCamCalibra::pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d(( p.x-K.at<double> (0,2))/K.at<double> (0,0),( p.y-K.at<double> (1,2))/K.at<double> (1,1));
}
void CcCamCalibra::cr_decomposeEssentialMat( InputArray _E, OutputArray _R1, OutputArray _R2, OutputArray _t )
{
    Mat E = _E.getMat().reshape(1, 3);
    CV_Assert(E.cols == 3 && E.rows == 3);

    Mat D, U, Vt;
    SVD::compute(E, D, U, Vt);
    if (determinant(U) < 0) U *= -1.;
    if (determinant(Vt) < 0) Vt *= -1.;

    Mat W = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
    W.convertTo(W, E.type());

    Mat R1, R2, t;
    R1 = U * W * Vt;
    R2 = U * W.t() * Vt;
    t = U.col(2) * 1.0;

    R1.copyTo(_R1);
    R2.copyTo(_R2);
    t.copyTo(_t);
}

int CcCamCalibra::cr_recoverPose( InputArray E, InputArray _points1, InputArray _points2, InputArray _cameraMatrix,
                     OutputArray _R, OutputArray _t, InputOutputArray _mask)
{
//    CV_INSTRUMENT_REGION()

    Mat points1, points2, cameraMatrix;
    _points1.getMat().convertTo(points1, CV_64F);
    _points2.getMat().convertTo(points2, CV_64F);
    _cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

    int npoints = points1.checkVector(2);
    CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
                              points1.type() == points2.type());

    CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.channels() == 1);

    if (points1.channels() > 1)
    {
        points1 = points1.reshape(1, npoints);
        points2 = points2.reshape(1, npoints);
    }

    double fx = cameraMatrix.at<double>(0,0);
    double fy = cameraMatrix.at<double>(1,1);
    double cx = cameraMatrix.at<double>(0,2);
    double cy = cameraMatrix.at<double>(1,2);

    points1.col(0) = (points1.col(0) - cx) / fx;
    points2.col(0) = (points2.col(0) - cx) / fx;
    points1.col(1) = (points1.col(1) - cy) / fy;
    points2.col(1) = (points2.col(1) - cy) / fy;

    points1 = points1.t();
    points2 = points2.t();

    Mat R1, R2, t;
    cr_decomposeEssentialMat(E, R1, R2, t);
    Mat P0 = Mat::eye(3, 4, R1.type());
    Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());
    P1(Range::all(), Range(0, 3)) = R1 * 1.0; P1.col(3) = t * 1.0;
    P2(Range::all(), Range(0, 3)) = R2 * 1.0; P2.col(3) = t * 1.0;
    P3(Range::all(), Range(0, 3)) = R1 * 1.0; P3.col(3) = -t * 1.0;
    P4(Range::all(), Range(0, 3)) = R2 * 1.0; P4.col(3) = -t * 1.0;

    // Do the cheirality check.
    // Notice here a threshold dist is used to filter
    // out far away points (i.e. infinite points) since
    // there depth may vary between postive and negtive.
    double dist = 50.0;
    Mat Q;
    triangulatePoints(P0, P1, points1, points2, Q);
    Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
    Q.row(0) /= Q.row(3);
    Q.row(1) /= Q.row(3);
    Q.row(2) /= Q.row(3);
    Q.row(3) /= Q.row(3);
    mask1 = (Q.row(2) < dist) & mask1;
    Q = P1 * Q;
    mask1 = (Q.row(2) > 0) & mask1;
    mask1 = (Q.row(2) < dist) & mask1;

    triangulatePoints(P0, P2, points1, points2, Q);
    Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
    Q.row(0) /= Q.row(3);
    Q.row(1) /= Q.row(3);
    Q.row(2) /= Q.row(3);
    Q.row(3) /= Q.row(3);
    mask2 = (Q.row(2) < dist) & mask2;
    Q = P2 * Q;
    mask2 = (Q.row(2) > 0) & mask2;
    mask2 = (Q.row(2) < dist) & mask2;

    triangulatePoints(P0, P3, points1, points2, Q);
    Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
    Q.row(0) /= Q.row(3);
    Q.row(1) /= Q.row(3);
    Q.row(2) /= Q.row(3);
    Q.row(3) /= Q.row(3);
    mask3 = (Q.row(2) < dist) & mask3;
    Q = P3 * Q;
    mask3 = (Q.row(2) > 0) & mask3;
    mask3 = (Q.row(2) < dist) & mask3;

    triangulatePoints(P0, P4, points1, points2, Q);
    Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
    Q.row(0) /= Q.row(3);
    Q.row(1) /= Q.row(3);
    Q.row(2) /= Q.row(3);
    Q.row(3) /= Q.row(3);
    mask4 = (Q.row(2) < dist) & mask4;
    Q = P4 * Q;
    mask4 = (Q.row(2) > 0) & mask4;
    mask4 = (Q.row(2) < dist) & mask4;

    mask1 = mask1.t();
    mask2 = mask2.t();
    mask3 = mask3.t();
    mask4 = mask4.t();

    // If _mask is given, then use it to filter outliers.
    if (!_mask.empty())
    {
        Mat mask = _mask.getMat();
        CV_Assert(mask.size() == mask1.size());
        bitwise_and(mask, mask1, mask1);
        bitwise_and(mask, mask2, mask2);
        bitwise_and(mask, mask3, mask3);
        bitwise_and(mask, mask4, mask4);
    }
    if (_mask.empty() && _mask.needed())
    {
        _mask.create(mask1.size(), CV_8U);
    }

    CV_Assert(_R.needed() && _t.needed());
    _R.create(3, 3, R1.type());
    _t.create(3, 1, t.type());

    int good1 = countNonZero(mask1);
    int good2 = countNonZero(mask2);
    int good3 = countNonZero(mask3);
    int good4 = countNonZero(mask4);

    if (good1 >= good2 && good1 >= good3 && good1 >= good4)
    {
        R1.copyTo(_R);
        t.copyTo(_t);
        if (_mask.needed()) mask1.copyTo(_mask);
        return good1;
    }
    else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
    {
        R2.copyTo(_R);
        t.copyTo(_t);
        if (_mask.needed()) mask2.copyTo(_mask);
        return good2;
    }
    else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
    {
        t = -t;
        R1.copyTo(_R);
        t.copyTo(_t);
        if (_mask.needed()) mask3.copyTo(_mask);
        return good3;
    }
    else
    {
        t = -t;
        R2.copyTo(_R);
        t.copyTo(_t);
        if (_mask.needed()) mask4.copyTo(_mask);
        return good4;
    }
}

int CcCamCalibra::CR_recoverPose( InputArray E, InputArray _points1, InputArray _points2, OutputArray _R,
                     OutputArray _t, double focal, Point2d pp, InputOutputArray _mask)
{
    Mat cameraMatrix = (Mat_<double>(3,3) << focal, 0, pp.x, 0, focal, pp.y, 0, 0, 1);
    return cr_recoverPose(E, _points1, _points2, cameraMatrix, _R, _t, _mask);
}

//========================================================================================

void CcCamCalibra::pose_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            const Mat& K,
                            Mat& R, Mat& t, Mat& H )
{
    // ç›¸æœºå†…å‚,TUM Freiburg2
    //Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    //-- æŠŠåŒ¹é…ç‚¹è½¬æ¢ä¸ºvector<Point2f>çš„å½¢å¼
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }
#if 1
    //-- è®¡ç®—åŸºç¡€çŸ©é˜µ
    Mat fundamental_matrix;
    //fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
    fundamental_matrix = findFundamentalMat ( points1, points2, FM_8POINT );
    //cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- è®¡ç®—æœ¬è´¨çŸ©é˜µ
    Point2d principal_point(K.at<double>(0,2), K.at<double>(1,2));//( 325.1, 249.7 );	//ç›¸æœºå…‰å¿ƒ, TUM datasetæ ‡å®šå€¼
    double focal_length = K.at<double>(0,0);//521;			//ç›¸æœºç„¦è·, TUM datasetæ ‡å®šå€¼
#if 0
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
#else
    Mat_<double> essential_matrix = K.t()*fundamental_matrix*K;
#endif
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    if(essential_matrix.rows == 3 && essential_matrix.cols == 3){

    	CR_recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point, noArray() );
        cout<<"R is "<<endl<<R<<endl;
        cout<<"t is "<<endl<<t<<endl;
    }
#endif
    //-- è®¡ç®—å•åº”çŸ©é˜µ
    H = findHomography ( points1, points2, RANSAC, 3 );
    //cout<<"homography_matrix is "<<endl<<H<<endl;


}

void CcCamCalibra::handle_pose_2d2d ( std::vector<Point2f> points1,
                            std::vector<Point2f> points2,
                            const Mat& K,
                            Mat& R, Mat& t, Mat& H )
{

    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, FM_8POINT );

    Point2d principal_point(K.at<double>(0,2), K.at<double>(1,2));
    double focal_length = K.at<double>(0,0);
#if 0
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
#else
    Mat_<double> essential_matrix = K.t()*fundamental_matrix*K;
#endif
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    if(essential_matrix.rows == 3 && essential_matrix.cols == 3){

    	CR_recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point, noArray() );
        cout<<"R is "<<endl<<R<<endl;
        cout<<"t is "<<endl<<t<<endl;
    }

    H = findHomography ( points1, points2, RANSAC, 3 );
}
/*******************************************************以下是内参标定函数**********************************/

void CcCamCalibra::FindPatternCorners()
{
if(imageListForCalibra.size() != 0){

	image_num = imageListForCalibra.size();
	for( int i =0; i<image_num; i++)
	{
		if(i==1){
			 image_size.width = imageListForCalibra[i].cols;
	            	 image_size.height = imageListForCalibra[i].rows;
			cout << "image_size.width = " << image_size.width << endl;
	            	cout << "image_size.height = " << image_size.height << endl;
		}
		
		 Mat imageInput ;
		 imageInput = imageListForCalibra[i].clone();
		if (findChessboardCorners(imageInput, pattern_size, corner_points_buf) == 0) {
	            	cout << "can not find chessboard corners!\n";  
	            	return ;	            
	        	}
		else 
		{
		            Mat gray;
		            cvtColor(imageInput, gray, CV_RGB2GRAY);
		            cornerSubPix(gray, corner_points_buf, Size(11, 11), Size(-1, -1),
		            		TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		            corner_points_of_all_imgs.push_back(corner_points_buf);
	        }
	}

}
}


void CcCamCalibra::FindCorners()
{
#if 1
	while(image_num < imgList.size()) {

	        filename = imgList[image_num++];
	        cout << "image_num = " << image_num << endl;
	        cout << filename.c_str() << endl;

	        cv::Mat imageInput = cv::imread(filename.c_str());

	        if (image_num == 1)  {
	            image_size.width = imageInput.cols;
	            image_size.height = imageInput.rows;
	            cout << "image_size.width = " << image_size.width << endl;
	            cout << "image_size.height = " << image_size.height << endl;
	        }

	        if (findChessboardCorners(imageInput, pattern_size, corner_points_buf) == 0) {
	            cout << "can not find chessboard corners!\n";   //找不到角点
	            return ;	            
	        }
	        else  
		{
	            cv::Mat gray;

	            cv::cvtColor(imageInput, gray, CV_RGB2GRAY);

//	            cv::find4QuadCornerSubpix(gray, corner_points_buf, cv::Size(5, 5));

	            cornerSubPix(gray, corner_points_buf, Size(11, 11), Size(-1, -1),
	            							TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


	            corner_points_of_all_imgs.push_back(corner_points_buf);
/*	            cv::drawChessboardCorners(gray, pattern_size, corner_points_buf, true);
	            PrintMs();
           		cv::imshow("camera calibration", gray);
	            PrintMs("imshow");
	            cv::waitKey(100);
*/	            
	        }
	    }
	#else
			cv::Mat imageInput = ImageLists[0];
	image_size.width = imageInput.cols;
	image_size.height = imageInput.rows;

	 if (findChessboardCorners(imageInput, pattern_size, corner_points_buf) == 0) {
	            cout << "can not find chessboard corners!\n";   //找不到角点
	            return ;	            
	        }
	        else  
		{
			printf("))))))))))))))))))))))))))))))))))))))))))))))))))))))))   find corners!!!\r\n");
	            cv::Mat gray;

	            cv::cvtColor(imageInput, gray, CV_RGB2GRAY);

//	            cv::find4QuadCornerSubpix(gray, corner_points_buf, cv::Size(5, 5));

	            cornerSubPix(gray, corner_points_buf, Size(11, 11), Size(-1, -1),
	            							TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


	            corner_points_of_all_imgs.push_back(corner_points_buf);
/*	            cv::drawChessboardCorners(gray, pattern_size, corner_points_buf, true);
	            PrintMs();
           		cv::imshow("camera calibration", gray);
	            PrintMs("imshow");
	            cv::waitKey(100);
*/	            
	        }

	#endif

	        cout << endl << "=============== <<  Complete Find All Chess Board Corners >> =============" << endl;
}
void CcCamCalibra::getObjectCoordinates()
{
	int i, j, k;
	square_size = Size(40,40);
	#if 1
		for (k = 0;k < image_num;k++) { 		 //遍历每一张图片
			vector<cv::Point3f> tempCornerPoints;//每一幅图片对应的角点数组
												//遍历所有的角点
			for (i = 0;i < pattern_size.height;i++)	{
				for (j = 0;j < pattern_size.width;j++) {
					cv::Point3f singleRealPoint;	//一个角点的坐标
					singleRealPoint.x = i * square_size.width;
					singleRealPoint.y = j * square_size.height;
					singleRealPoint.z = 0;			//假设z=0
					tempCornerPoints.push_back(singleRealPoint);
				}
			}
			objectPoints.push_back(tempCornerPoints);
		}
	#else
			vector<cv::Point3f> tempCornerPoints;
												
			for (i = 0;i < pattern_size.height;i++)	{
				for (j = 0;j < pattern_size.width;j++) {
					cv::Point3f singleRealPoint;	
					singleRealPoint.x = i *square_size.width;
					singleRealPoint.y = j *square_size.height;
					singleRealPoint.z = 0;			
					tempCornerPoints.push_back(singleRealPoint);
				}
			}
			objectPoints.push_back(tempCornerPoints);

	#endif
		
	
}

void CcCamCalibra::calibrate()
{
//if(objectPoints.empty() == false && corner_points_of_all_imgs.empty() == false){
	 cv::calibrateCamera(objectPoints, corner_points_of_all_imgs, image_size, cameraMatrix, distCoefficients, rvecsMat, tvecsMat, 0);
	    fout << "相机相关参数：" << endl;
	    fout << "1.内外参数矩阵:" << endl;
	    fout << "大小：" << cameraMatrix.size() << endl;
	    fout << cameraMatrix << endl;
	    fout << "2.畸变系数：" << endl;
	    fout << "大小：" << distCoefficients.size() << endl;
	    fout << distCoefficients << endl;
	        fout << endl << "图像相关参数：" << endl;
	        cv::Mat rotation_Matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));//旋转矩阵
	        for (int i = 0;i < image_num;i++)
	        {
	            fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
	            fout << rvecsMat[i] << endl;
	            fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
	            cv::Rodrigues(rvecsMat[i], rotation_Matrix);//将旋转向量转换为相对应的旋转矩阵
	            fout << rotation_Matrix << endl;
	            fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
	            fout << tvecsMat[i] << endl;
	        }
	        //对标定结果进行评价
	        cout << "================<< Start Calculate Average Error ! >> ==============================" << endl;
	        //计算每幅图像中的角点数量，假设全部角点都检测到了
	        int corner_points_counts;
	        corner_points_counts = pattern_size.width * pattern_size.height;
	        fout << "每幅图像的标定误差：" << endl;
	        double err = 0;//单张图像的误差
	        double total_err = 0;//所有图像的平均误差
	      for (int i = 0;i < image_num;i++)
	        {
	            vector<cv::Point2f> image_points_calculated;//存放新计算出的投影点的坐标
	            vector<cv::Point3f> tempPointSet = objectPoints[i];
	            cv::projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoefficients, image_points_calculated);
	            //计算新的投影点与旧的投影点之间的误差
	            vector<cv::Point2f> image_points_old = corner_points_of_all_imgs[i];
	            //将两组数据换成Mat格式
	            cv::Mat image_points_calculated_mat = cv::Mat(1, image_points_calculated.size(), CV_32FC2);
	            cv::Mat image_points_old_mat = cv::Mat(1, image_points_old.size(), CV_32FC2);
	            for (int j = 0;j < tempPointSet.size();j++)
	            {
	                image_points_calculated_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points_calculated[j].x, image_points_calculated[j].y);
	                image_points_old_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points_old[j].x, image_points_old[j].y);
	            }
	            err = cv::norm(image_points_calculated_mat, image_points_old_mat, cv::NORM_L2);
//	            err /= corner_points_counts;
	            err = std::sqrt(err*err/corner_points_counts);
	            total_err += err;
	            fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	        }
	        fout << "总体平均误差：" << total_err / image_num << "像素" << endl;
	        fout.close();
cout<<"============================== << Calculate Average Eoor Complite !!! >> ============================" << endl;
#if 1
		Mat mapx = cv::Mat(image_size, CV_32FC1);
	        Mat mapy = cv::Mat(image_size, CV_32FC1);
	        Mat R = Mat::eye(3, 3, CV_32F);
	        string imageFileName;
	        std::stringstream StrStm;
	      for (int i = 0;i < image_num;i++)
	        {
	            cout << "Frame #" << i + 1 << endl;
	            initUndistortRectifyMap(cameraMatrix, distCoefficients, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
	            Mat src_image = imread(imgList[i].c_str(), 1);
	            Mat new_image = src_image.clone();
	            remap(src_image, new_image, mapx, mapy, cv::INTER_LINEAR);
	            imshow("Undistortion Image", new_image);
	            waitKey(0);
//	            imshow("原始图像", src_image);
//	            imshow("矫正后图像", new_image);
//	            StrStm.clear();
//	            imageFileName.clear();
//	            StrStm << i + 1;
//	            StrStm >> imageFileName;
//	            imageFileName += "_d.jpg";
//	            cv::imwrite(imageFileName, new_image);
	        }
#endif
	        cout << "======================= Calibrate Complete ! >> ===================================" << endl;
	//}
}
void CcCamCalibra::undistortion(Mat distortionImage,Mat &unDistortionImage)
{
#if 0
    //cout<<"undistortion ..."<<endl;
    Size image_size = distortionImage.size();
    Mat mapx = Mat(image_size,CV_32FC1);
    Mat mapy = Mat(image_size,CV_32FC1);
    Mat R = Mat::eye(3,3,CV_32F);
    Matx33d intrinsic_matrix_new = cameraMatrix;//intrinsic_matrix;
    //调节视场大小,乘的系数越小视场越大
    intrinsic_matrix_new(0,0) *= 0.4;
    intrinsic_matrix_new(1,1) *= 0.4;
    //调节校正图中心，建议置于校正图中心
    intrinsic_matrix_new(0,2) = 0.5 * distortionImage.cols;
    intrinsic_matrix_new(1,2) = 0.5 * distortionImage.rows;
    initUndistortRectifyMap(cameraMatrix,distCoefficients,R,intrinsic_matrix_new,image_size,CV_32FC1,mapx,mapy);
    unDistortionImage = distortionImage.clone();
    remap(distortionImage,unDistortionImage,mapx, mapy, INTER_CUBIC);
#endif
}

void CcCamCalibra::showUndistortImages()
{
#if 0
	Mat unDistortionImage;
	Mat OriginImage;
	for(int i =0; i<image_num ; i++){
		OriginImage = imread(imgList[i].c_str(), 1);
		undistortion(OriginImage,unDistortionImage);
		imshow("Undistortion Image", unDistortionImage);
		waitKey(100);
	}
#endif
}


//============================================================================================

//DetectCorners

DetectCorners::DetectCorners()
{
	OSA_semCreate(&g_detectCorners, 1, 0);
	memset(_bCutIMG, 0x00, sizeof(_bCutIMG));
}
DetectCorners::~DetectCorners()
{
	StopService();
}
int DetectCorners::RunService()
{
	m_prm.pThis = this;
	return RunThread(RunProxy, &m_prm);
}

int DetectCorners::StopService()
{
	 SetThreadExit();
	 WaitThreadExit();
	 return 0;
}

void DetectCorners::PrintMs( const char* text )
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

void* DetectCorners::RunProxy(void* pArg)
{
	 struct timeval tv;
	 struct RunPrm *pPrm = (struct RunPrm*)pArg;
	 while(pPrm->pThis->m_bRun)
	 {		
	 
	 	tv.tv_sec = 0;
    		tv.tv_usec = (30%1000)*1000;
   		select(0, NULL, NULL, NULL, &tv);
		
		pPrm->pThis->Run();
	 }
	 return NULL;
}
void DetectCorners::cloneCornerSrcImgae(Mat &src)
{
	src.copyTo(corner_yuyv);
}
void DetectCorners::cvtCornerYuyv2Bgr()
{
	if(!corner_yuyv.empty()){
		cvtColor(corner_yuyv,corner_frame,CV_YUV2BGR_YUYV);
	}
}
void DetectCorners::Init()
{
	int pattern_cols = 13;
  	int pattern_raws = 8;
   	pattern_size = Size(pattern_cols,pattern_raws);      
   	corners.clear();
}
bool DetectCorners::chessBoardCornersDetect(Mat image,Mat &cornerImage,int &successImages)
{
   width = image.cols;
   height = image.rows;	
   Mat imageGray;
   cvtColor(image, imageGray , CV_RGB2GRAY);
	
    bool patternfound = findChessboardCorners(
                image, pattern_size, corners,CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK );
    if (!patternfound)    {
        cout<<"can not find chessboard corners!\n";    
        return false;
    }
    else
    {      
        cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        drawChessboardCorners(cornerImage, pattern_size, corners, patternfound);
		cornerImage.copyTo(g_CornerImage); // clone the detected image to global Mat "g_CornerImage" for submit textures
		g_bSubmitTexture = true;
		//captureOnePicture = true;
				
	}	
    return true;
}
int DetectCorners::Run()
{	
	int flag = OSA_semWait(&g_detectCorners, OSA_TIMEOUT_FOREVER/*200*/);
	if( -1 == flag ) {
		printf("%s:LINE :%d    could not get the ball current Pos \n",__func__,__LINE__ );
	}
	else{
		
		if( showDetectCorners == true ){
			cv::Mat cornerImage;
			cvtCornerYuyv2Bgr();
			cornerImage = corner_frame.clone();
			
		bool ret=	chessBoardCornersDetect( corner_frame, cornerImage, successImageNum );
		if(ret)
		{
			if(saveOnePicture == true )
			{
				saveOnePicture = false;
				int nsize = imageListForCalibra.size();
				if(nsize<50)
				{
					memset(savePicname, 0, sizeof(savePicname));
					sprintf(savePicname, "%02d.bmp",saveCount);					
					saveCount ++;
					//imwrite(savePicname,corner_frame);
		
	//				m_cutIMG[nsize] = cv::Mat(corner_frame.rows,corner_frame.cols,CV_8UC3);
	//				cvtColor(corner_frame,m_cutIMG[nsize],CV_YUV2BGR_YUYV);
					corner_frame.copyTo(m_cutIMG[nsize]);
					imageListForCalibra.push_back(m_cutIMG[nsize]);
					captureCount += 1;
					SetCutDisplay(nsize, true);
				}
			}			
		     }		
		}
	}
}


