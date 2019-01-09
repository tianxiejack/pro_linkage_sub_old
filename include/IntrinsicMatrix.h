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
	std::vector< std::vector<cv::Point3f> > objectPoints ;
	std::vector< std::vector<cv::Point2f> > imagePoints;

	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	int flag;



};



#endif

