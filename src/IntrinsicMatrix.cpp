
#include "IntrinsicMatrix.h"
#include <unistd.h>

using namespace cv;

std::vector< cv::Mat > ImageList;
IntrinsicMatrix::IntrinsicMatrix():m_bcalibrateSwitch(false),m_dErr(0.0),m_dTotal_Err(0.0),m_iOneImgCornerCount(0)
{
	imageSize = cv::Size(1920,1080);
	
	boardSize = cv::Size(13,8);
	m_iOneImgCornerCount = boardSize.width * boardSize.height;

	grid = cv::Size(40,40);
	
	ImageList.clear();
	objectPoints.clear();
	imagePoints.clear();
	rvecs.clear();
	tvecs.clear();
}
IntrinsicMatrix::~IntrinsicMatrix()
{
	StopService();
}
int IntrinsicMatrix::RunService()
{
	m_prm.pThis = this;
	return RunThread(RunProxy, &m_prm);
}

int IntrinsicMatrix::StopService()
{
	 SetThreadExit();
	 WaitThreadExit();
	 return 0;
}

void* IntrinsicMatrix::RunProxy(void* pArg)
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
int IntrinsicMatrix::Run()
{
	static int cnt =0;	
	
	//sleep(2);
	
	//printf("\r\n[%s]: IntrinsicMatrix: Run()== %d ", __FUNCTION__, cnt++);
#if 0	
	if( true == getCalibrateSwitch()) {
		for(int i=0; i< ImageList.size(); i++) {
			cv::imshow("ImageList",ImageList[i]);
			cv::waitKey(3000);
		}
	}
	
#else

	if(true == getCalibrateSwitch()) {		
		addChessboardPoints( ImageList, boardSize );
		calibrate( imageSize );
		//calculateAverageErr(cameraMatrix,distCoeffs,rvecs, tvecs);

		setCalibrateSwitch(false);
	}
#endif

}

void IntrinsicMatrix::addPoints(const std::vector<cv::Point2f>&imageCorners, const std::vector<cv::Point3f>& objectCorners)
{
	// 2D image points from one view
	imagePoints.push_back(imageCorners);
	// corresponding 3D scene points
	objectPoints.push_back(objectCorners);
	return ;
}
int IntrinsicMatrix::addChessboardPoints( const std::vector< std::string > &filelist, cv::Size &boardSize)
{
#if 0
	std::vector< cv::Point2f> imageCorners;
	std::vector< cv::Point3f> objectCorners;

	/* push points of 3D into vector  */
	for(int i=0; i< boardSize.height; i++) {
		for(int j=0; j< boardSize.width; j++) {
			objectCorners.push_back(cv::Point3f(i,j ,0.0f));
		}
	}

	/* points of 2D image */
	cv::Mat image;
	int sucesses = 0;

	for( int i=0; i<filelist.size(); i++) {
		image = cv::imread(filelist[i], 0);
		bool found = cv::findChessboardCorners( image, boardSize, imageCorners);
		if(found == 0){
			cv::cornerSubPix( image, imageCorners, cv::Size(5,5), cv::Size(-1,-1), 
				cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1) );
		}
		else{
			break;
		}
		if(	imageCorners.size() == boardSize.area() ) {
			addPoints(imageCorners, objectCorners);
			sucesses ++;
		}
	}
	return sucesses;
#endif
}

int IntrinsicMatrix::addChessboardPoints( const std::vector< cv::Mat > &imageList, cv::Size &boardSize)
{
	std::vector< cv::Point2f> imageCorners;
	std::vector< cv::Point3f> objectCorners;

	imageCorners.clear();
	objectCorners.clear();
	
	/* push points of 3D into vector  */
	for(int i=0; i< boardSize.height; i++) {
		for(int j=0; j< boardSize.width; j++) {
			objectCorners.push_back(cv::Point3f(i*grid.width,j *grid.height,0.0f));
		}
	}
	
	/* points of 2D image */
	cv::Mat image ;
	int sucesses = 0;
	image.create(1080,1920,CV_8UC3);
	for(int i=0; i< imageList.size(); i++){
		image = imageList[i];		
		bool  patternfound = cv::findChessboardCorners( image, boardSize, imageCorners, 
						CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK );
		if(patternfound){
			Mat gray;
	              cv::cvtColor(image, gray, CV_RGB2GRAY);				
			 cv::cornerSubPix(gray, imageCorners, Size(11, 11), Size(-1, -1),
	            							cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));			 
			//addPoints(imageCorners, objectCorners);
			//sucesses++;
		}
		else{
			printf("\r\n[%s]:XXXXXXX  : Can't Find chess boards Corners !!!",__FUNCTION__);
			break;
		}	
		
		if(imageCorners.size() == boardSize.area() ) {
			addPoints(imageCorners, objectCorners);
			sucesses ++;
		}		
		
	}
	cout << "Total Image number = " << ImageList.size() << endl;
	cout << " Find Corners Sucesses Image number = " <<sucesses <<endl;
	return sucesses;

}

double IntrinsicMatrix::calibrate(cv::Size &imageSize)
{	
	flag = 0;
	cameraMatrix = cv::Mat(3,3,CV_32FC1, cv::Scalar::all(0));
	distCoeffs = cv::Mat(1,5, CV_32FC1,cv::Scalar::all(0));
	
	double ret =cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag);

	cout << "calibrateCamera() : return value = " << ret << endl;
	cout << cameraMatrix << endl;
	cout << distCoeffs << endl;
	
	return ret;
}

int IntrinsicMatrix::calculateAverageErr(const cv::Mat &cameraMatrix, const cv::Mat &distCofficent,const std::vector< cv::Mat > &R, const std::vector< cv::Mat > &T)
{
	std::vector< cv::Point2f > new_projected_points;
	for(int i=0; i<ImageList.size(); i++) {
		new_projected_points.clear();
		std::vector< cv::Point3f > one_objectPointsArray = objectPoints[i];;
		cv::projectPoints(one_objectPointsArray, R[i], T[i], cameraMatrix,distCofficent,new_projected_points);
		std::vector< cv::Point2f > old_projectedImg_points = imagePoints[i];
		cv::Mat new_projected_points_mat =cv::Mat(1,new_projected_points.size(),CV_32FC2);
		cv::Mat old_projectedImg_points_mat = cv::Mat(1,old_projectedImg_points.size(), CV_32FC2);
		for(int j=0; j<one_objectPointsArray.size(); j++) {
			new_projected_points_mat.at<cv::Vec2f>(0,j) = cv::Vec2f(new_projected_points[j].x,new_projected_points[j].y);
			old_projectedImg_points_mat.at<cv::Vec2f>(0,j)=cv::Vec2f(old_projectedImg_points[j].x, old_projectedImg_points[j].y);
			
		}
		m_dErr = cv::norm(new_projected_points_mat, old_projectedImg_points_mat);
		
		m_dErr = std::sqrt( m_dErr*m_dErr/m_iOneImgCornerCount);
		m_dTotal_Err  += m_dErr;
		cout << "\n============================================\n";
		cout << "Average Error = " << m_dTotal_Err<<endl;
	}


}


