#ifndef __INTRINSIC_MATRIX__HEAD__
#define __INTRINSIC_MATRIX__HEAD__

#include "WorkThread.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;


class IntrinsicMatrix :public WorkThread
{
public:
	IntrinsicMatrix();
	virtual ~IntrinsicMatrix();
public:
	struct RunPrm{
		IntrinsicMatrix *pThis;
	};
	int RunService();
	int StopService();

	struct RunPrm m_prm;
	static void* RunProxy(void* pArg);
	int Run();
private:
	/* Input points: from World Coordinates  */
	std::vector< std::vector<cv::Point3f> > objectPoints ;

	/* points on Image: */
	std::vector< std::vector<cv::Point2f> > imagePoints;

	/* Output matrix :*/
	cv::Mat cameraMatrix;

	/* distortion coefficient */
	cv::Mat distCoeffs;
	
	/* calibration type flag*/
	int flag;
	std::vector< cv::Mat > rvecs, tvecs;

	cv::Size boardSize;  // chessboard size
	cv::Size imageSize; // image's width and height
	cv::Size grid;
	double m_dErr, m_dTotal_Err;
	bool m_bcalibrateSwitch ;
	int m_iOneImgCornerCount;
public:
	void setCalibrateSwitch(bool flag) {
		m_bcalibrateSwitch = flag;
	};
	bool getCalibrateSwitch(){
		return m_bcalibrateSwitch;
	};
public:
	void addPoints(const std::vector<cv::Point2f>&imageCorners, const std::vector<cv::Point3f>& objectCorners);
	int addChessboardPoints( const std::vector< std::string > &filelist, cv::Size &boardSize);
	int addChessboardPoints( const std::vector< cv::Mat > &imageList, cv::Size &boardSize);
	double calibrate(cv::Size &imageSize);
	int calculateAverageErr(const cv::Mat &cameraMatrix, const cv::Mat &distCofficent,const std::vector< cv::Mat > &R, const std::vector< cv::Mat > &T);

};



#endif

