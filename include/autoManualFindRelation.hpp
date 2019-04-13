/*
 * autoManualFindRelation.hpp
 *
 *  Created on: 2019年4月9日
 *      Author: alex
 */

#ifndef AUTOMANUALFINDRELATION_HPP_
#define AUTOMANUALFINDRELATION_HPP_

#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>


using namespace std;
using namespace cv;

namespace cr_automanualfindrelation{

typedef struct {
	cv::Point2i pixel;
}INVISABLEGRID_T;

typedef struct{
	bool markFlag;
	bool selectFlag;
	cv::Point2i pixel;
	cv::Point2i pos;
	double distance;
}FEATUREPOINT_T;


typedef void ( *pNOTIFYFUNC)(std::vector<FEATUREPOINT_T>& recommendPoints );


class CAutoManualFindRelation
{
public:
	CAutoManualFindRelation(int m_disWidth,int m_disHeight , int row , int col );
	virtual ~CAutoManualFindRelation();

	//for to be used
	void create(pNOTIFYFUNC	notifyFunc); //注册回调函数
	void autoFindPoints();//自动找特征点
	void cloneSrcImage(Mat &src);//获取截图用于找特征点
	void manualInsertRecommendPoints( cv::Point2i inPoint );//手动增加待选参考点
	void insertPos(  cv::Point2i inPos );//插入pos到选择的点
	void selectPoint(  cv::Point2i inPixel ); //选择点进行找pos
	void deletePos( cv::Point2i inPixel );//删除掉该点的pos信息

	bool readParams(std::vector<FEATUREPOINT_T>& getParam);//读取配置
	bool writeParams(void);//写配置

	int Point2getPos(const Point2i inPoint,Point2i &result ); //输入点获取POS
	void drawPoints( cv::Mat drawMat,	std::vector<FEATUREPOINT_T>& featurePoints , bool bDraw );

	//for test
	void drawInvisableGrid(cv::Mat drawMat , bool bDraw);//画出隐藏的网格
	void draw_subdiv( Mat& img ,vector<Vec6f> triangleList ,bool bdraw);//画剖分三角
	int draw_point_triangle( Mat& img , Point2i fp , vector<FEATUREPOINT_T> &back,Point2i &pos, bool bdraw );//插入点，画出pos和对应三角形的顶点
	void getPos( Point2i inPoint , Point2i& result );//插入点，返回pos和对应三角形的顶点信息
	void draw_subdiv_point( Mat& img, Point2i fp, Scalar color );//画点以及对应的剖分三角
	void getPoints( std::vector<FEATUREPOINT_T>& pointVec); //得到待选的参考点容器

	void getTriangleList(vector<Vec6f>& triangleList);


	//used only inside
	void handleKeyPoints( std::vector<KeyPoint>& keypoints );
	void generateInvisibleGrid();
	int getBlockIdAtInvisibleGrid( cv::Point2i inPoint );
	void selectPointInBlock();
	void clearVector2Init();
	void putPointsInBlock(std::vector<KeyPoint>& keypoints );
	void collectMarkedPoints();

	void updateSubdiv();
	void vertex2pos(vector<Point2i> &vertex);
	void InterpolationPos( Point2i inPoint , Point2i& result );
	void preprocessPos(std::vector<FEATUREPOINT_T>& calcPos);
	int findposInFpassembel(Point2f &fp , Point2i &pos);
	void insertVertexAndPosition(vector<FEATUREPOINT_T> insert);
	void getTriangleVertex( Point2f fp, vector<Point2i> &result );
	bool readParamsForTest();


	void calcNormalWay(Point2i inPoint,Point2i& result,vector<double>& dis);
	void calcTriArea(Point2i inPoint , vector<double>& dis , vector<double>& area);


	double getDistance(Point2i pointO, Point2i pointA);
	double getDist_P2L(Point2i pointP, Point2i pointA, Point2i pointB);
	void calcDistancePoint2Triangle(Point2i inPoint, vector<double>& dis);
	void getNear2LineUseTwoPoint2Calc(int flag,Point2i inPoint,Point2i& result);
	void initDivsubObj();
	void getHomography2estimateConer();

	void findThreeNearestPointInCanUsedPoints2estimate( std::vector<FEATUREPOINT_T> featurPoints );

private:
	pNOTIFYFUNC m_notifyFunc;
	std::vector<FEATUREPOINT_T> m_featurePoints ;
	std::vector<INVISABLEGRID_T> m_invisableGrid ;
	std::vector<FEATUREPOINT_T> m_canUsedPoints;
	std::vector<cv::KeyPoint>* m_blockVect;

	int m_gridx,m_gridy;
	int m_row,m_col;
	int m_disWidth,m_disHeight;
	Mat srcFrame;

	Subdiv2D* m_pSubdiv;
	Rect m_rect;
	std::vector<FEATUREPOINT_T> fpassemble;
	std::vector<FEATUREPOINT_T> m_calcPos;

	FileStorage m_readfs;
	FileStorage m_writefs;

	vector<Point2f> m_orgpointBK;
	cv::Point2i m_fpDrawTest;
	cv::Point2i m_testPixel;
};

}

#endif /* AUTOMANUALFINDRELATION_HPP_ */
