
#include "autoManualFindRelation.hpp"

using namespace cr_automanualfindrelation;

#define CONFIG_AUTOMANUALFIND_FILE		"ConfigAutoManualFindFile.yml"

CAutoManualFindRelation::CAutoManualFindRelation(int disWidth, int disHeight,int row, int col) :m_disWidth(disWidth), m_disHeight(disHeight), m_row(row), m_col(col) 
{
	m_featurePoints.clear();
	m_invisableGrid.clear();
	m_gridx = m_gridy = 0;

	m_rect.x = 0;
	m_rect.y = 0;
	m_rect.width = disWidth;
	m_rect.height = disHeight;
	subdiv.initDelaunay(m_rect);
}

CAutoManualFindRelation::~CAutoManualFindRelation() 
{
	delete[] m_blockVect;
}

void CAutoManualFindRelation::create(pNOTIFYFUNC notifyFunc)
{
	m_notifyFunc = notifyFunc;
	m_blockVect = new std::vector<cv::KeyPoint>[m_row * m_col];
	generateInvisibleGrid();
}

void CAutoManualFindRelation::clearVector2Init()
{
	m_featurePoints.clear();
	for (int i = 0; i < m_row * m_col; i++) 
	{
		m_blockVect[i].clear();
	}
	return;
}

void CAutoManualFindRelation::generateInvisibleGrid()
{
	INVISABLEGRID_T tmpPoint;
	m_gridx = m_disWidth / m_row;
	m_gridy = m_disHeight / m_col;

	for (int j = 0; j < m_row; j++)
		for (int i = 0; i < m_col; i++) {
			tmpPoint.pixel.x = i * m_gridx;
			tmpPoint.pixel.y = j * m_gridy;
			m_invisableGrid.push_back(tmpPoint);
		}
	return;
}

void CAutoManualFindRelation::drawInvisableGrid(cv::Mat drawMat, bool bDraw) 
{
	Point p1, p2;
	CvScalar colour;
	int frcolor;

	if (bDraw)
		colour = {255,255,0,255};
		else
		colour = {0,0,0,0};

	for (int i = 0; i < m_row; i++) {
		p1.x = 0;
		p1.y = i * m_gridy;
		p2.x = m_disWidth;
		p2.y = i * m_gridy;
		cv::line(drawMat, p1, p2, colour, 3, 8, 0);
	}

	for (int i = 0; i < m_col; i++) {
		p1.x = i * m_gridx;
		p1.y = 0;
		p2.x = i * m_gridx;
		p2.y = m_disHeight;
		cv::line(drawMat, p1, p2, colour, 3, 8, 0);
	}
	return;
}

int CAutoManualFindRelation::getBlockIdAtInvisibleGrid(cv::Point2i inPoint)
{
	int blockId;
	int rowNo, colNo;

	rowNo = inPoint.x / m_gridx;
	colNo = inPoint.y / m_gridy;

	blockId = rowNo * m_col + colNo;
	return blockId;
}

void CAutoManualFindRelation::putPointsInBlock(std::vector<KeyPoint>& keypoints)
{
	int blockId = 0;
	for (int i = 0; i < keypoints.size(); i++)
	{
		blockId = getBlockIdAtInvisibleGrid(keypoints[i].pt);
		m_blockVect[blockId].push_back(keypoints[i]);
	}
	return;
}

void CAutoManualFindRelation::selectPointInBlock() 
{
	float maxResponse;
	int tmpj;
	FEATUREPOINT_T ptmp;

	for (int i = 0; i < m_row * m_col; i++) {
		maxResponse = 0;
		for (int j = 0; j < m_blockVect[i].size(); j++) {
			if (maxResponse < m_blockVect[i][j].response) {
				maxResponse = m_blockVect[i][j].response;
				ptmp.pixel = m_blockVect[i][j].pt;
			}
		}
		ptmp.markFlag = false;
		ptmp.selectFlag = false;
		m_featurePoints.push_back(ptmp);
	}

	//for(int i =0 ;i < m_featurePoints.size();i++ )
	//	printf("point = (%d , %d )\n" , m_featurePoints[i].pixel.x, m_featurePoints[i].pixel.y );

	return;
}

void CAutoManualFindRelation::handleKeyPoints(std::vector<KeyPoint>& keypoints) {
	FEATUREPOINT_T tmp;

	//特征点分布到块里
	putPointsInBlock(keypoints);

	//各个块剔除特征点
	selectPointInBlock();
	return;
}

void CAutoManualFindRelation::drawPoints(cv::Mat drawMat,std::vector<FEATUREPOINT_T>& featurePoints, bool bDraw)
{
	for (int i = 0; i < featurePoints.size(); i++) 
	{
		if (bDraw)
		{
			if( featurePoints[i].selectFlag )
				cv::circle(drawMat , featurePoints[i].pixel , 3 ,cvScalar(0,0,255,255), 2, 8, 0 );
			else if( featurePoints[i].markFlag )
				cv::circle(drawMat , featurePoints[i].pixel , 3 ,cvScalar(0,255,0,255), 2, 8, 0 );
			else
				cv::circle(drawMat , featurePoints[i].pixel , 3 ,cvScalar(255,0,0,255), 2, 8, 0 );
		} 
		else
			cv::circle(drawMat, featurePoints[i].pixel, 3, cvScalar(0, 0, 0, 0), 2,8, 0);
	}
	cv::circle(drawMat, cv::Point2i(100, 200), 3, cvScalar(0, 0, 0, 0), 2, 8, 0);
	return;
}

void CAutoManualFindRelation::autoFindPoints()
{
	std::vector<KeyPoint> keypoints;
	Ptr<FeatureDetector> detector = makePtr<ORB>(30000, 1.2, 8, 31, 0, 2,
			ORB::HARRIS_SCORE, 31);	//FeatureDetector::create ( "ORB" );
	detector->detect(srcFrame, keypoints);
	clearVector2Init();
	handleKeyPoints(keypoints);

	if (m_notifyFunc != NULL)
		(*m_notifyFunc)(m_featurePoints);

	printf("!!! keypoints = %d  \n", keypoints.size());
	return;
}

void CAutoManualFindRelation::cloneSrcImage(Mat &src)
{
	#if 1
		Mat yuyv;
		src.copyTo(yuyv);
		cvtColor(yuyv, srcFrame, CV_YUV2BGR_YUYV);
	#else
		src.copyTo(srcFrame);
	#endif
}

void CAutoManualFindRelation::manualInsertRecommendPoints(cv::Point2i inPoint) 
{
	FEATUREPOINT_T tmpPoint;
	tmpPoint.pixel = inPoint;
	tmpPoint.markFlag = false;
	m_featurePoints.push_back(tmpPoint);

	if (m_notifyFunc != NULL)
		(*m_notifyFunc)(m_featurePoints);

	return;
}

void CAutoManualFindRelation::getPoints(std::vector<FEATUREPOINT_T>& pointVec)
{
	pointVec.assign(m_featurePoints.begin(), m_featurePoints.end());
	return;
}

void CAutoManualFindRelation::selectPoint(cv::Point2i inPixel) 
{
	for (int i = 0; i < m_featurePoints.size(); i++) {
		if (m_featurePoints[i].selectFlag ) {
			m_featurePoints[i].selectFlag = false;
		}
	}

	for (int i = 0; i < m_featurePoints.size(); i++) {
		if (m_featurePoints[i].pixel == inPixel) {
			m_featurePoints[i].selectFlag = true;
		}
	}

	if (m_notifyFunc != NULL)
		(*m_notifyFunc)(m_featurePoints);

	return;
}

void CAutoManualFindRelation::insertPos(cv::Point2i inPos) 
{
	for (int i = 0; i < m_featurePoints.size(); i++) 
	{
		if (m_featurePoints[i].selectFlag == true) {
			m_featurePoints[i].pos = inPos;
			m_featurePoints[i].markFlag = true;
			m_featurePoints[i].selectFlag = false;
		}
	}
	collectMarkedPoints();

	if (m_notifyFunc != NULL)
		(*m_notifyFunc)(m_featurePoints);

	if (m_canUsedPoints.size() > 3)
		insertVertexAndPosition(m_canUsedPoints);

	return;
}

void CAutoManualFindRelation::collectMarkedPoints() 
{
	FEATUREPOINT_T tmp;
	m_canUsedPoints.clear();
	for (int i = 0; i < m_featurePoints.size(); i++) {
		if (m_featurePoints[i].markFlag) {
			tmp.pixel = m_featurePoints[i].pixel;
			tmp.pos = m_featurePoints[i].pos;
			m_canUsedPoints.push_back(tmp);
		}
	}

	insertVertexAndPosition(m_canUsedPoints) ;
	return;
}

void CAutoManualFindRelation::deletePos(cv::Point2i inPixel)
{
	for (int i = 0; i < m_featurePoints.size(); i++) {
		if (m_featurePoints[i].selectFlag == true) {
			m_featurePoints[i].pixel = inPixel;
			m_featurePoints[i].markFlag = false;
			m_featurePoints[i].selectFlag = false;
		}
	}

	if (m_notifyFunc != NULL)
		(*m_notifyFunc)(m_featurePoints);

	return;
}

void CAutoManualFindRelation::insertVertexAndPosition(vector<FEATUREPOINT_T> insert) 
{
	fpassemble.clear();
	fpassemble = insert;
	updateSubdiv();
	return;
}

void CAutoManualFindRelation::updateSubdiv() 
{
	for (std::vector<FEATUREPOINT_T>::iterator plist = fpassemble.begin();
			plist != fpassemble.end(); ++plist) {
		subdiv.insert(plist->pixel);
	}
}

int CAutoManualFindRelation::Point2getPos(const Point2i inPoint,Point2i &result) 
{
	int ret = 0;
	vector<Point2i> triVertex;

	getTriangleVertex(inPoint, triVertex);
	for (std::vector<Point2i>::iterator plist = triVertex.begin();plist != triVertex.end(); ++plist)
	{
		if (plist->x <= 0 || plist->x > m_rect.width || plist->y <= 0
				|| plist->y > m_rect.height)
			ret = -1;
		break;
	}

	if (-1 == ret)
		return ret;

	vertex2pos( triVertex );
	getPos(inPoint, triVertex, result);
	return ret;
}

void CAutoManualFindRelation::getTriangleVertex(Point2f fp,vector<Point2i> &result) 
{
	int e0 = 0, vertex = 0;
	Point2i tmp;
	subdiv.locate(fp, e0, vertex);
	if (e0 > 0) {
		int e = e0;
		do {
			Point2f org, dst;
			if (subdiv.edgeOrg(e, &org) > 0 && subdiv.edgeDst(e, &dst) > 0) {
				result.push_back(org);
			}
			e = subdiv.getEdge(e, Subdiv2D::NEXT_AROUND_LEFT);
		} while (e != e0);
	}
}

void CAutoManualFindRelation::vertex2pos( vector<Point2i> &vertex )
{
	FEATUREPOINT_T tmp;
	m_calcPos.clear();

	for (int i = 0; i < vertex.size(); i++)
	{
		for (std::vector<FEATUREPOINT_T>::iterator plist = fpassemble.begin();plist != fpassemble.end(); ++plist)
		{
			if (plist->pixel == vertex[i])
			{
				tmp.pixel = plist->pixel;
				tmp.pos = plist->pos;
				m_calcPos.push_back(tmp);
				break;
			}
		}
	}
	return;
}

static bool comp(const FEATUREPOINT_T &a, const FEATUREPOINT_T &b)
{
	unsigned int tmpa, tmpb;
	tmpa = a.pixel.x;
	tmpb = b.pixel.y;
	return tmpa < tmpb;
}

void CAutoManualFindRelation::preprocessPos()
{

	int min = 40000, max = 0;
	int sizeNum = m_calcPos.size();
	if (sizeNum)
		sort(m_calcPos.begin(), m_calcPos.end(), comp);

	if (abs(m_calcPos[2].pixel.x - m_calcPos[0].pixel.x) > 18000) {
		m_calcPos[0].pixel.x += 36000;
		if (m_calcPos[1].pixel.x < 18000)
			m_calcPos[1].pixel.x += 36000;
	}

	for (int j = 0; j < 3; j++) {
		if (m_calcPos[j].pixel.y > 32000) {
			m_calcPos[j].pixel.y = 32768 - m_calcPos[j].pixel.y;
		}
	}

	return;
}


void CAutoManualFindRelation::calcNormalWay(Point2i inPoint,vector<Point2i>& triVertex,Point2i& result)
{
	double d1, d2, d3;
	double f1, f2, f3, dtmp;

	d1 = getDistance(inPoint , m_calcPos[0].pixel);
	d2 = getDistance(inPoint , m_calcPos[1].pixel);
	d3 = getDistance(inPoint , m_calcPos[2].pixel);


	dtmp = 1 + d1 / d2 + d2 / d3;
	f1 = 1 / dtmp;
	f2 =  d1 / d2 * f1;
	f3 = 1 - f1 - f2;

	result.x = f1 * m_calcPos[0].pos.x + f2 * m_calcPos[1].pos.x + f3 * m_calcPos[2].pos.x;
	result.y = f1 * m_calcPos[0].pos.y + f2 * m_calcPos[1].pos.y + f3 * m_calcPos[2].pos.y;

	return ;
}


/***** 求两点间距离*****/
double CAutoManualFindRelation::getDistance(Point2i pointO, Point2i pointA)
{
	double distance;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);
    return distance;
}

/***** 点到直线的距离:P到AB的距离*****/
//P为线外一点，AB为线段两个端点
double CAutoManualFindRelation::getDist_P2L(Point2i pointP, Point2i pointA, Point2i pointB)
{
    //求直线方程
    int A = 0, B = 0, C = 0;
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = pointA.x*pointB.y - pointA.y*pointB.x;
    //代入点到直线距离公式
    double distance = 0;
    distance = ((double)abs(A*pointP.x + B*pointP.y + C)) / ((double)sqrtf(A*A + B*B));
    return distance;
}


void CAutoManualFindRelation::calcDistancePoint2Triangle(Point2i inPoint,vector<Point2i>& triVertex, vector<double>& dis)
{
	Point2i A,B;
	dis.clear();
	for(int i=0 ; i< 3 ;i++)
		dis.push_back(  getDist_P2L(inPoint, m_calcPos[i].pixel, m_calcPos[(i+1)%3].pixel) );
	return ;
}


void CAutoManualFindRelation::getNear2LineUseTwoPoint2Calc(int flag,Point2i inPoint,vector<Point2i>& triVertex,Point2i& result)
{
	double d1, d2;
	double f1;

	d1 = getDistance(inPoint, m_calcPos[flag].pixel);
	d2 = getDistance(inPoint, m_calcPos[(flag+1)%3].pixel);

	f1 = d2 /(d1+d2);

	result.x = f1 * m_calcPos[flag].pos.x + (1 - f1) * m_calcPos[(flag+1)%3].pos.x;
	result.y = f1 * m_calcPos[flag].pos.y + (1 - f1) * m_calcPos[(flag+1)%3].pos.y;

	return ;
}


void CAutoManualFindRelation::InterpolationPos(Point2i inPoint,vector<Point2i>& triVertex, Point2i& result)
{

	std::vector<double> getDis;
	int flag = 3;
	double min = 1000;
	calcDistancePoint2Triangle(inPoint,triVertex,getDis);

	for(int i=0 ; i < getDis.size(); i++ )
	{
		if( getDis[i] < 20 )
		{
			if(getDis[i] < min)
			{
				min = getDis[i];
				flag = i;
			}
		}
	}

	if(flag < 3)
		getNear2LineUseTwoPoint2Calc(flag,inPoint,triVertex,result);
	else
		calcNormalWay(inPoint,triVertex,result);

	result.x %= 36000;
	if (result.y < 0)
		result.y = 32768 - result.y;

	return;
}

void CAutoManualFindRelation::getPos(Point2i inPoint,vector<Point2i>& triVertex, Point2i& result)
{
	preprocessPos();
	InterpolationPos(inPoint, triVertex, result);
	return;
}

void CAutoManualFindRelation::draw_subdiv(Mat& img, bool bdraw)
{
	vector<Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);
	vector<Point> pt(3);
	CvScalar color;
	int linewidth = 1;
	if (bdraw)
		color = cvScalar(0, 100, 255, 255);
	else
	{
		color = cvScalar(0, 0, 0, 0);
		linewidth = 2 ;
	}

	for (size_t i = 0; i < triangleList.size(); i++)
	{
		Vec6f t = triangleList[i];
		pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		line(img, pt[0], pt[1], color, linewidth, CV_AA, 0);
		line(img, pt[1], pt[2], color, linewidth, CV_AA, 0);
		line(img, pt[2], pt[0], color, linewidth, CV_AA, 0);
	}
	return;
}

int CAutoManualFindRelation::draw_point_triangle(Mat& img, Point2i fp,vector<FEATUREPOINT_T> &back, Point2i &pos, bool bdraw) 
{
	int e0 = 0, vertex = 0;
	int num = 0;
	int ret = 0;
	int flag;
	int index;
	CvScalar color;
	vector<Point2f> orgpoint;
	Point2i tmppos;
	FEATUREPOINT_T tmpBack;

	//readParamsForTest();
	//fp.x = m_testPixel.x;
	//fp.y = m_testPixel.y;
	//printf("%s : line:%d     inPoint (%d , %d) \n",__func__,__LINE__,fp.x , fp.y);

	if (bdraw)
		color = cvScalar(255, 0, 255, 255);
	else
		color = cvScalar(0, 0, 0, 0);
	
	subdiv.locate(fp, e0, vertex);

	if (e0 > 0) {
		int e = e0;
		do {
			Point2f org, dst;
			if (subdiv.edgeOrg(e, &org) > 0 && subdiv.edgeDst(e, &dst) > 0) {
				orgpoint.push_back(org);
				if (org.x <= 0.00001 || org.x > m_rect.width || org.y <= 0.00001
						|| org.y > m_rect.height)
					ret = -1;
			}
			e = subdiv.getEdge(e, Subdiv2D::NEXT_AROUND_LEFT);
		} while (e != e0);
	}

	if (-1 == ret)
		return ret;

	for (int k = 0; k < 3; k++) {
		index = (k + 1) % 3;
		line(img, orgpoint[k], orgpoint[index], color, 3, CV_AA, 0);
	}

	draw_subdiv_point(img, fp, color);
	back.clear();
	for (int k = 0; k < 3; k++) {
		flag = findposInFpassembel(orgpoint[k], tmppos);

		if (-1 == flag)
			return -1;
		else {
			tmpBack.pixel = orgpoint[k];
			tmpBack.pos = tmppos;
			back.push_back(tmpBack);
		}
	}
	Point2getPos(fp, pos);
	return 0;
}

int CAutoManualFindRelation::findposInFpassembel(Point2f &fp, Point2i &pos) 
{
	for (std::vector<FEATUREPOINT_T>::iterator plist = fpassemble.begin();
			plist != fpassemble.end(); ++plist) {
		if (plist->pixel.x == fp.x && plist->pixel.y == fp.y) {
			pos.x = plist->pos.x;
			pos.y = plist->pos.y;
			return 0;
		}
	}
	return -1;
}

void CAutoManualFindRelation::draw_subdiv_point(Mat& img, Point2i fp,Scalar color) 
{
	circle(img, fp, 3, color, CV_FILLED, 8, 0);
}

bool CAutoManualFindRelation::readParams(std::vector<FEATUREPOINT_T>& getParam) 
{
	char paramName[40];
	memset(paramName, 0, sizeof(paramName));
	string cfgFile;
	cfgFile = CONFIG_AUTOMANUALFIND_FILE;

	m_readfs.open(cfgFile, FileStorage::READ);
	int size;
	FEATUREPOINT_T tmpPos;

	m_featurePoints.clear();
	getParam.clear();
	if (m_readfs.isOpened()) {
		sprintf(paramName, "vectorSize");
		m_readfs[paramName] >> size;
		for (int i = 0; i < size; i++) {
			sprintf(paramName, "points_markFlag_%d", i);
			m_readfs[paramName] >> tmpPos.markFlag;
			memset(paramName, 0, sizeof(paramName));

			sprintf(paramName, "points_selectFlag_%d", i);
			m_readfs[paramName] >> tmpPos.selectFlag;
			memset(paramName, 0, sizeof(paramName));

			sprintf(paramName, "points_pixel_x_%d", i);
			m_readfs[paramName] >> tmpPos.pixel.x;
			memset(paramName, 0, sizeof(paramName));

			sprintf(paramName, "points_pixel_y_%d", i);
			m_readfs[paramName] >> tmpPos.pixel.y;
			memset(paramName, 0, sizeof(paramName));

			sprintf(paramName, "points_pos_x_%d", i);
			m_readfs[paramName] >> tmpPos.pos.x;
			memset(paramName, 0, sizeof(paramName));

			sprintf(paramName, "points_pos_y_%d", i);
			m_readfs[paramName] >> tmpPos.pos.y;
			memset(paramName, 0, sizeof(paramName));

			m_featurePoints.push_back(tmpPos);
		}
		getParam = m_featurePoints;
		m_readfs.release();
		collectMarkedPoints();
		return true;
	}
	return false;
}

bool CAutoManualFindRelation::writeParams(void)
{
	char paramName[40];
	memset(paramName, 0, sizeof(paramName));
	string cfgFile;
	cfgFile = CONFIG_AUTOMANUALFIND_FILE;

	m_writefs.open(cfgFile, FileStorage::WRITE);

	if (m_writefs.isOpened()) {
		sprintf(paramName, "vectorSize");
		m_writefs << paramName << (int) m_featurePoints.size();

		for (int i = 0; i < m_featurePoints.size(); i++) {
			sprintf(paramName, "points_markFlag_%d", i);
			m_writefs << paramName << (int) m_featurePoints[i].markFlag;
			memset(paramName, 0, sizeof(paramName));

			sprintf(paramName, "points_selectFlag_%d", i);
			m_writefs << paramName << (int) m_featurePoints[i].selectFlag;
			memset(paramName, 0, sizeof(paramName));

			sprintf(paramName, "points_pixel_x_%d", i);
			m_writefs << paramName << (int) m_featurePoints[i].pixel.x;
			memset(paramName, 0, sizeof(paramName));

			sprintf(paramName, "points_pixel_y_%d", i);
			m_writefs << paramName << (int) m_featurePoints[i].pixel.y;
			memset(paramName, 0, sizeof(paramName));

			sprintf(paramName, "points_pos_x_%d", i);
			m_writefs << paramName << (int) m_featurePoints[i].pos.x;
			memset(paramName, 0, sizeof(paramName));

			sprintf(paramName, "points_pos_y_%d", i);
			m_writefs << paramName << (int) m_featurePoints[i].pos.y;
			memset(paramName, 0, sizeof(paramName));
		}
		m_writefs.release();
		return true;
	}

	return false;
}

bool CAutoManualFindRelation::readParamsForTest()
{
	char paramName[40];
	memset(paramName, 0, sizeof(paramName));
	string cfgFile;
	cfgFile = "ConfigForTest.yml";

	m_readfs.open(cfgFile, FileStorage::READ);
	int size;
	FEATUREPOINT_T tmpPos;

	if (m_readfs.isOpened()) {
		sprintf(paramName, "x");
		m_readfs[paramName] >> m_testPixel.x;
		sprintf(paramName, "y");
		m_readfs[paramName] >> m_testPixel.y;

		m_readfs.release();
		return true;
	}
	return false;
}
