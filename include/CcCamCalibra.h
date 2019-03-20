/*
 * CcCamCalibra.h
 *
 *  Created on: Oct 16, 2018
 *      Author: d
 */

#ifndef CCCAMCALIBRA_H_
#define CCCAMCALIBRA_H_
#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>
#include <vector>
#include "WorkThread.h"
#include <iostream>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <string>


#include "osa_sem.h"

using namespace std;
using namespace cv;

typedef struct __cameraParams{
		Size imageSize;
		Mat cameraMatrix_gun;
		Mat distCoeffs_gun;
		Mat cameraMatrix_ball;
		Mat distCoeffs_ball;
		Mat homography;
		int panPos;
		int tiltPos;
		int zoomPos;
		int panPosBase, tiltPosBase, zoomPosBase;
		Mat map1, map2;
		
}CamParameters;

class CcCamCalibra:public WorkThread {
public:
	void PrintMs( const char* text ="");
	void undistortion(Mat distortionImage,Mat &unDistortionImage);
	void showUndistortImages();
	vector<Mat> ImageLists;
private:
	vector<string> imgList;
	ifstream inImgPath;
	ofstream fout;
	
	int image_num ;
	Mat imageInput;
	Mat gray;
	vector<Point3f> tempCornerPoints;
	cv::Point3f singleRealPoint;	
	string filename;
	Size image_size;
	Size square_size;
	Size pattern_size;
	vector<cv::Point2f> corner_points_buf;
	vector<cv::Point2f>::iterator corner_points_buf_ptr;
	vector<vector< cv::Point2f> > corner_points_of_all_imgs;
	Mat cameraMatrix;
	Mat distCoefficients;
	vector<cv::Mat> tvecsMat;
	vector<cv::Mat> rvecsMat;
	vector<vector< cv::Point3f> > objectPoints;
										
public:	
	void FindCorners();	
	void FindPatternCorners();
	void getObjectCoordinates();
	void calibrate();
public:
	CcCamCalibra();
	virtual ~CcCamCalibra();
	struct RunPrm{
		CcCamCalibra *pThis;
	};
	int RunService();
	int StopService();

	struct RunPrm m_prm;
	static void* RunProxy(void* pArg);
	int Run();
	void getCamParam(const char* FileName,int type);
	void setCamMatrix();
	void getBallSrcImage(Mat &src);
	void getGunSrcImage(Mat &src);
	void cloneBallSrcImgae(Mat &src);
	void cloneGunSrcImgae(Mat &src);
	
	void remapImage();
	int find_feature_matches ( const Mat& img_1, const Mat& img_2,
	                            std::vector<KeyPoint>& keypoints_1,
	                            std::vector<KeyPoint>& keypoints_2,
	                            std::vector< DMatch >& matches ,
	                           double distThreshold = 30.0, bool bDraw = false);
	Point2d pixel2cam ( const Point2d& p, const Mat& K );
	void pose_2d2d ( std::vector<KeyPoint> keypoints_1,
	                            std::vector<KeyPoint> keypoints_2,
	                            std::vector< DMatch > matches,
	                            const Mat& K,
	                            Mat& R, Mat& t, Mat& H );
	bool saveLinkageParams( const char* filename,
			                       Size imageSize, const Mat& cameraMatrix_gun, const Mat& distCoeffs_gun,
			                               const Mat& cameraMatrix_ball, const Mat& distCoeffs_ball, const Mat& homography,
			                               int panPos, int tiltPos, int zoomPos);
	bool loadLinkageParams( const char* filename,
	                       Size& imageSize, Mat& cameraMatrix_gun, Mat& distCoeffs_gun,
	                               Mat& cameraMatrix_ball, Mat& distCoeffs_ball, Mat& homography,
	                               int& panPos, int& tiltPos, int& zoomPos);
	bool load_OriginCameraParams( const string& filename, int& flags, 
									Mat& cameraMatrix2, Mat& distCoeffs2, Size& imageSize);

	bool Load_CameraParams(const string& gun_file,const string& ball_file);
	void Init_CameraParams();
	void handle_pose_2d2d ( std::vector<Point2f> points1,
                            std::vector<Point2f> points2,                        
                            const Mat& K,
                            Mat& R, Mat& t, Mat& H );
	int CR_recoverPose( InputArray E, InputArray _points1, InputArray _points2, OutputArray _R,
                     OutputArray _t, double focal, Point2d pp, InputOutputArray _mask);
	int cr_recoverPose( InputArray E, InputArray _points1, InputArray _points2, InputArray _cameraMatrix,
                     OutputArray _R, OutputArray _t, InputOutputArray _mask);
	void cr_decomposeEssentialMat( InputArray _E, OutputArray _R1, OutputArray _R2, OutputArray _t );
	void cvtBallYuyv2Bgr();
	void cvtGunYuyv2Bgr();	
	void setBallPos(int in_panPos, int in_tilPos, int in_zoom);
	bool read_Pictures();

private:
	Mat cameraMatrix_gun ;
	Mat distCoeffs_gun ;
	Mat cameraMatrix_ball ;
	Mat distCoeffs_ball ;	
	Size imageSize ;
	Mat newCameraMatrix;
	int panPosBase, tiltPosBase, zoomPosBase;
	Mat map1, map2;
	Mat undisImage;
	Mat homography;
	Mat gun_frame;
	Mat ball_frame;
	Mat ball_yuyv;
	Mat gun_yuyv;
	
	Mat proc_frame;
	Mat gun_BMP;
	Mat ball_BMP;
	vector<Point2f> pts;
	Size boardSize;
	float scale ;
	bool bCal ;
	bool ret1,ret2;
	int panPos;
	int tiltPos;
	int zoomPos;
	int flags;
public:

	bool bool_Calibrate;
	bool writeParam_flag;
	bool start_cloneVideoSrc;
	bool Set_Handler_Calibra;
	bool getCurrentPosFlag;
	Mat gun_fromBMP;
	vector<Point2f> key_points1;
	vector<Point2f> key_points2;
};

class DetectCorners:public WorkThread
{
	private:
		char savePicname[20];
		Mat corner_yuyv;
		Mat corner_frame;
		int successImageNum ;  
		int width;  // detect Image wodth
   		int height;  // detect Height wodth
   		Size pattern_size;
   		vector<Point2f> corners; 
		
	public:
		Mat m_cutIMG[100];
		bool _bCutIMG[100];
	public:
		DetectCorners();
		virtual ~DetectCorners();
		void SetCutDisplay(int idx, bool enable = true){ _bCutIMG[idx] = enable;};
	struct RunPrm{
		DetectCorners *pThis;
	};
	int RunService();
	int StopService();

	struct RunPrm m_prm;
	static void* RunProxy(void* pArg);
	void PrintMs( const char* text= "" );
	int Run();
	void Init();
	void cvtCornerYuyv2Bgr();
	void cloneCornerSrcImgae(Mat &src);
	bool chessBoardCornersDetect(Mat image,Mat &cornerImage,int &successImages);
	
};
#endif /* CCCAMCALIBRA_H_ */
