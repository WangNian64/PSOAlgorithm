#pragma once
//
//  CAstar.h
//  Astar
//
//  Created by xujw on 15/4/9.
//  Copyright (c) 2015年 xujw. All rights reserved.
//
/*
	F:路径评分 = g+h
	G:走一格格子的花销
	H:当前格子到目标格子的估算花销

	上下左右走一格花销为10，斜着走一格花销为14，以方便计算
	即格子宽高为10 对角线为14
 */
#ifndef __Astar__CAstar__
#define __Astar__CAstar__

#include <stdio.h>
#include <iostream>
#include "Tools.h"
using namespace std;

enum class AType
{
	ATYPE_UNKNOWN,
	ATYPE_CLOSED,
	ATYPE_OPENED,
	ATYPE_BARRIER   //障碍
};

class APoint
{
public:
	__device__ APoint();
	~APoint();
	int rowIndex;
	int colIndex;
	double x;
	double y;
	AType type;		//类型:障碍、开放列表、关闭列表
	double f;		//f = g+h
	double g;
	double h;
	APoint* parent;
	bool operator == (const APoint& po)
	{
		if (x == po.x && y == po.y)
		{
			return true;
		}
		return false;
	}
	bool AEqualB()
	{

	}
	double CalcuPointDist(const APoint& point)
	{
		return abs(this->y - point.y) + abs(this->x - point.x);
	}
	__device__ double CalcuPointDist(const APoint& point, int i)
	{
		return abs(this->y - point.y) + abs(this->x - point.x);
	}
	__device__ void resetAPoint()
	{
		f = g = h = 0;
		type = AType::ATYPE_UNKNOWN;
		parent = nullptr;
	}
};
enum PathDirection
{
	Vertical,
	Horizon
};
class CAstar
{
public:
	int openList_MaxSize = 100;		//开放列表数组最大大小
	int closeList_MaxSize = 100;	//关闭列表数组最大大小
	int neighbourList_MaxSize = 4;	//周边节点列表数组最大大小

	int openList_CurSize = 0;//当前大小
	int closeList_CurSize = 0;
	int neighbourList_CurSize = 0;
	APoint** _openList;				//开放列表
	APoint** _closeList;			//关闭列表
	APoint** _neighbourList;		//周边节点
	APoint* _endPoint;
	APoint* _curPoint;

	APoint** _allPoints;//修改这个数据结构
	int pointRowNum;	//点的行数目
	int pointColNum;	//点的列数目

	PathDirection curPathDirect;
	__device__ CAstar();
	~CAstar();
	__device__ double CalcuPathLength(APoint* point);
	__device__ APoint* findWay(PathDirection beginDirect, int beginRowIndex, int beginColIndex, int endRowIndex, int endColIndex);
	__device__ void resetAStar();
	//bool SameDirect(APoint* curPoint, APoint* nextPoint);
private:
	__device__ double getF(APoint* point);
	__device__ double getH(APoint* point);
	__device__ double getE(APoint* curPoint, APoint* nextPoint, APoint* endPoint);
	__device__ APoint** getNeighboringPoint(APoint* point);
};




#endif 