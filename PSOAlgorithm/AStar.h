#pragma once
//
//  CAstar.h
//  Astar
//
//  Created by xujw on 15/4/9.
//  Copyright (c) 2015�� xujw. All rights reserved.
//
/*
	F:·������ = g+h
	G:��һ����ӵĻ���
	H:��ǰ���ӵ�Ŀ����ӵĹ��㻨��

	����������һ����Ϊ10��б����һ����Ϊ14���Է������
	�����ӿ��Ϊ10 �Խ���Ϊ14
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
	ATYPE_BARRIER   //�ϰ�
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
	AType type;		//����:�ϰ��������б��ر��б�
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
	int openList_MaxSize = 100;		//�����б���������С
	int closeList_MaxSize = 100;	//�ر��б���������С
	int neighbourList_MaxSize = 4;	//�ܱ߽ڵ��б���������С

	int openList_CurSize = 0;//��ǰ��С
	int closeList_CurSize = 0;
	int neighbourList_CurSize = 0;
	APoint** _openList;				//�����б�
	APoint** _closeList;			//�ر��б�
	APoint** _neighbourList;		//�ܱ߽ڵ�
	APoint* _endPoint;
	APoint* _curPoint;

	APoint** _allPoints;//�޸�������ݽṹ
	int pointRowNum;	//�������Ŀ
	int pointColNum;	//�������Ŀ

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