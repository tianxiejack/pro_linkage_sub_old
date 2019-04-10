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
}FEATUREPOINT_T;


typedef struct{
	Point2i ver;
	Point2i pos;
}POSITION_T;

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

	bool readParams(std::vector<POSITION_T>& getParam);//读取配置
	bool writeParams(void);//写配置

	void drawPoints( cv::Mat drawMat,	std::vector<FEATUREPOINT_T>& featurePoints , bool bDraw );

	//for test
	void drawInvisableGrid(cv::Mat drawMat , bool bDraw);//画出隐藏的网格
	void draw_subdiv( Mat& img ,bool bdraw);//画剖分三角
	int draw_point_triangle( Mat& img , Point2i fp , vector<POSITION_T> &back,Point2i &pos, bool bdraw );//插入点，画出pos和对应三角形的顶点
	void getPos( Point2i inPoint , vector<Point2i>& triVertex ,  vector<Point2i>& triPos , Point2i& result );//插入点，返回pos和对应三角形的顶点信息
	void draw_subdiv_point( Mat& img, Point2i fp, Scalar color );//画点以及对应的剖分三角
	void getPoints( std::vector<FEATUREPOINT_T>& pointVec); //得到待选的参考点容器

	//used only inside
	void handleKeyPoints( std::vector<KeyPoint>& keypoints );
	void generateInvisibleGrid();
	int getBlockIdAtInvisibleGrid( cv::Point2i inPoint );
	void selectPointInBlock();
	void clearVector2Init();
	void putPointsInBlock(std::vector<KeyPoint>& keypoints );
	void collectMarkedPoints();

	void updateSubdiv();
	void vertex2pos(vector<Point2i> &vertex , vector<Point2i> & getPos );
	void InterpolationPos( Point2i inPoint , vector<Point2i>& triVertex ,  vector<Point2i>& triPos , Point2i& result );
	void preprocessPos( vector<Point2i>& inpos );
	int findposInFpassembel(Point2f &fp , Point2i &pos);
	void insertVertexAndPosition(vector<POSITION_T> insert);
	void getTriangleVertex( Point2f fp, vector<Point2i> &result );
	int Point2getPos(const Point2i inPoint,Point2i &result );


private:
	pNOTIFYFUNC m_notifyFunc;
	std::vector<FEATUREPOINT_T> m_featurePoints ;
	std::vector<INVISABLEGRID_T> m_invisableGrid ;
	std::vector<POSITION_T> m_canUsedPoints;
	std::vector<cv::KeyPoint>* m_blockVect;

	int m_gridx,m_gridy;
	int m_row,m_col;
	int m_disWidth,m_disHeight;
	Mat srcFrame;

	Subdiv2D subdiv;
	Rect rect;
	std::vector<POSITION_T> fpassemble;

	FileStorage m_readfs;
	FileStorage m_writefs;
};

}

#endif /* AUTOMANUALFINDRELATION_HPP_ */
