/**********************************************************************
* Copyright(C),2009-2019,ChamRun Tech. Co., Ltd
* File name: GridMap.h
* Author: sunwj
* Version: 1.0.0
* Function: Use grid map to implement linkage by calculate each node's PTZ value of the map.
* Build Date: 2019.02.18
***********************************************************************/


#ifndef __GRID_MAP_H__
#define __GRID_MAP_H__

#include <iostream>
#include <vector>

using namespace std;


typedef struct __gridmapnode
{
	int pano,zoom;    // camera PTZ value
	int tilt;
	int coord_x,coord_y;// screen coordinates
	bool isCircle;
	unsigned char has_mark;
	
}GridMapNode;

typedef struct __markmapnode
{
	int x,y;
	bool isShow;	
}MarkMapNode;


class GridMap
{
public:
	GridMap();
	virtual ~GridMap();
	enum 
	{
		GRID_COLS = 10,
		GRID_ROWS = 10
	};
private:
	GridMapNode m_gridNodes[GRID_ROWS][GRID_COLS];


};

#endif // __GRID_MAP_H__


