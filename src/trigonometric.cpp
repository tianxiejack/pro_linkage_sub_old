/*
 * trigonometric.cpp
 *
 *  Created on: 2019年2月20日
 *      Author: alex
 */


#include "trigonometric.hpp"


using namespace cr_trigonometricInterpolation;

Trigonometric::Trigonometric(int width , int height)
{
	rect.x = 0 ;
	rect.y = 0;
	rect.width = width ;
	rect.height = height;
	 subdiv.initDelaunay(rect);
}

Trigonometric::~Trigonometric()
{
	fpassemble.clear();
}

void Trigonometric::insertVertexAndPosition(vector<position_t> insert)
{
	fpassemble.clear();
	fpassemble = insert;
	updateSubdiv();
	return ;
}

void Trigonometric::updateSubdiv()
{
	for(std::vector<position_t>::iterator plist = fpassemble.begin(); plist != fpassemble.end(); ++plist)
	{
		 subdiv.insert( plist->ver );
	}
}

int Trigonometric::Point2getPos(const Point2i inPoint,Point2i &result)
{
	int ret = 0;
	vector<Point2i> triVertex;
	vector<Point2i> triPos;

	getTriangleVertex( inPoint , triVertex);
	for(std::vector<Point2i>::iterator plist = triVertex.begin(); plist != triVertex.end(); ++plist)
	{
		if( plist->x <= 0 || plist->x >= rect.width || plist->y <= 0 || plist->y >= rect.height )
			ret = -1;
		break;
	}

	if( -1 == ret )
		return ret;

	vertex2pos( triVertex, triPos );

	getPos( inPoint , triVertex ,  triPos , result );

	return ret;
}

void Trigonometric::getTriangleVertex( Point2f fp, vector<Point2i> &result )
{
	 int e0=0, vertex=0;
	 Point2i tmp;
	 subdiv.locate(fp, e0, vertex);
	 if( e0 > 0 )
	 {
		 int e = e0;
		 do
		 {
			 Point2f org, dst;
			 if( subdiv.edgeOrg(e, &org) > 0 && subdiv.edgeDst(e, &dst) > 0 )
			 {
				 result.push_back(org);
			 }
			 e = subdiv.getEdge(e, Subdiv2D::NEXT_AROUND_LEFT);
		 }
		 while( e != e0 );
	 }
}

void Trigonometric::vertex2pos(vector<Point2i> &vertex, vector<Point2i> & getPos )
{
	 getPos.clear();
	 for(int i = 0 ; i< vertex.size(); i++)
	 {
		for(std::vector<position_t>::iterator plist = fpassemble.begin(); plist != fpassemble.end(); ++plist)
		{
			if( plist->ver == vertex[i] )
			{
				getPos.push_back( plist->pos );
				break;
			}
		}
	 }
	 return;
}

bool comp(const Point2i &a,const Point2i &b)
{
	unsigned int tmpa ,tmpb;
	tmpa = a.x;
	tmpb = b.x;
	return tmpa<tmpb;
}

void Trigonometric::preprocessPos( vector<Point2i>& inpos )
{
	int min = 40000 , max = 0;
	int sizeNum = inpos.size();
	if(sizeNum)
		sort(inpos.begin(),inpos.end(),comp);

	if(abs(inpos[2].x - inpos[0].x) > 18000)
	{
		inpos[0].x += 36000;
		if(inpos[1].x < 18000)
			inpos[1].x += 36000;
	}

	for(int j=0;j<3;j++)
	{
		if( inpos[j].y > 32000)
		{
			inpos[j].y = 32768 - inpos[j].y;
		}
	}

	return ;
}

void Trigonometric::InterpolationPos( Point2i inPoint , vector<Point2i>& triVertex ,  vector<Point2i>& triPos , Point2i result )
{
	 unsigned int d1, d2, d3;
	 double f1, f2, f3, dtmp;

	 d1 = pow((inPoint.x - triVertex[0].x),2) + pow((inPoint.y - triVertex[0].y),2);
	 d2 = pow((inPoint.x - triVertex[1].x),2) + pow((inPoint.y - triVertex[1].y),2);
	 d3 = pow((inPoint.x - triVertex[2].x),2) + pow((inPoint.y - triVertex[2].y),2);

	 dtmp = 1+ (double)d1/d2 + (double)d2/d3 ;
	 f1 = 1/dtmp;
	 f2 = (double)d1/d2*f1;
	 f3 = 1 - f1 - f2;

	 result.x = f1*triPos[0].x + f2*triPos[1].x + f3*triPos[2].x;
	 result.y = f1*triPos[0].y + f2*triPos[1].y + f3*triPos[2].y;

	 result.x %= 36000;
	 	if(result.y < 0)
	 		result.y = 32768 - result.y ;

	 return ;
}

void Trigonometric::getPos( Point2i inPoint , vector<Point2i>& triVertex ,  vector<Point2i>& triPos , Point2i result )
{
	preprocessPos( triPos );

	InterpolationPos( inPoint, triVertex, triPos, result );

	return ;
}
