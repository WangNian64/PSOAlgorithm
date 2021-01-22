//
//  CAstar.cpp
//  Astar
//
//  Created by xujw on 15/4/9.
//  Copyright (c) 2015�� xujw. All rights reserved.
//
//  ����������һ����Ϊ10��б����һ����Ϊ14���Է������
//  �����ӿ��Ϊ10 �Խ���Ϊ14
#pragma once
#include "AStar.h"
//�Զ���������
//bool mySort(const APoint* p1, const APoint* p2)
//{
//    return p1->f < p2->f;
//}

__device__ APoint::APoint() :x(0)
, y(0)
, h(0)
, f(0)
, g(0)
, parent(nullptr)
, type(AType::ATYPE_UNKNOWN)
{
}
APoint::~APoint()
{
    if (parent) { delete[] parent; }
}


#pragma mark------CAstar-------

__device__ CAstar::CAstar() :_endPoint(nullptr), _curPoint(nullptr)
{
}

CAstar::~CAstar()
{
	if (_openList) { delete[] _openList; }
	if (_closeList) { delete[] _closeList; }
	if (_neighbourList) { delete[] _neighbourList; }
	openList_CurSize = 0;
	closeList_CurSize = 0;
	neighbourList_CurSize = 0;
}
//����һ��·�����ܳ���
__device__ double CAstar::CalcuPathLength(APoint* point)
{
    double length = 0.0;
    while (point->parent)
    {
        length += point->CalcuPointDist(*(point->parent), -1);
        point = point->parent;
    }
    return length;
}
__device__ void CAstar::resetAStar()
{
    for (int i = 0; i < pointRowNum; i++)
    {
         for (int j = 0; j < pointColNum; j++)
        {
            _allPoints[i * pointColNum + j]->resetAPoint();
        }
    }
	delete[] _openList;
	delete[] _closeList;
	delete[] _neighbourList;
	openList_CurSize = 0;
	closeList_CurSize = 0;
	neighbourList_CurSize = 0;
}
//Ҫ�ĳ�__device__��
__device__ APoint* CAstar::findWay(PathDirection beginDirect, int beginRowIndex, int beginColIndex, int endRowIndex, int endColIndex)
{
	curPathDirect = beginDirect;
	_endPoint = _allPoints[endRowIndex * pointColNum + endColIndex];//����ڴ����GPU�ڲ���
	APoint* beginPoint = _allPoints[beginRowIndex * pointColNum + beginColIndex];

	if (_endPoint->type == AType::ATYPE_BARRIER)
	{
		//cout << "�յ����ϰ�" << endl;
		return nullptr;
	}
	if (*_endPoint == *beginPoint)
	{
		_curPoint = beginPoint;
		_curPoint->parent = _endPoint;
		return _curPoint;
	}

	_openList[openList_CurSize++] = beginPoint;
	beginPoint->type = AType::ATYPE_OPENED;
	beginPoint->f = getF(beginPoint);
	//---------
	do
	{
		//��ȡ��Сֵ�Ľڵ�
		_curPoint = _openList[0];

		//ɾ����ǰ��Ľڵ�
		if (openList_CurSize == 1) {
			_openList = NULL;//���ҲҪ��
		}
		else {
			_openList = &_openList[1];
		}
		openList_CurSize--;

		_curPoint->type = AType::ATYPE_CLOSED;
		_closeList[closeList_CurSize++] = _curPoint;

		if (*_curPoint == *_endPoint)
		{
			//cout << "have find way" << endl;
			return _curPoint;
		}
		//��ȡ���ڵĽڵ�
		APoint** neVec = getNeighboringPoint(_curPoint);//
		for (int i = 0; i < neighbourList_CurSize; i++)
		{
			APoint* tmpoint = neVec[i];
			double tempG = _curPoint->g + tmpoint->CalcuPointDist(*_curPoint) + getE(_curPoint, tmpoint, _endPoint);
			if (tmpoint->type == AType::ATYPE_CLOSED)
			{
				continue;
			}
			//�Ƿ��ڿ����б���
			if (tmpoint->type != AType::ATYPE_OPENED)
			{
				tmpoint->parent = _curPoint;
				//����GHF
				tmpoint->g = tempG;
				//���·���
				//if (tmpoint->x == _curPoint->x)
				//{
				//    curPathDirect = PathDirection::Vertical;
				//}
				//if (tmpoint->y == _curPoint->y)
				//{
				//    curPathDirect = PathDirection::Horizon;
				//}
				tmpoint->h = getH(tmpoint);
				tmpoint->f = getF(tmpoint);
				//��ӵ������б���
				_openList[openList_CurSize++] = tmpoint;
				tmpoint->type = AType::ATYPE_OPENED;
			}
			else
			{
				//�Ѿ��ڿ����б���
				//if (tmpoint->g < _curPoint->g)
				//if (tmpoint->h < _curPoint->h)
				if (tempG < tmpoint->g)
				{
					tmpoint->parent = _curPoint;
					tmpoint->g = tempG;
					//���·���
					if (tmpoint->x == _curPoint->x)
					{
						curPathDirect = PathDirection::Vertical;
					}
					if (tmpoint->y == _curPoint->y)
					{
						curPathDirect = PathDirection::Horizon;
					}
					tmpoint->f = getF(tmpoint);
				}
			}
		}
		//���� Fֵ��С������ǰ��
		//��Ҫ�Լ�ʵ��
		//�鲢�������ȶ��ģ������ʵ��
		//stable_sort(_openList.begin(), _openList.end(), mySort);
		APoint** tempOpenList = new APoint * [openList_CurSize];
		StableSort_APoint(_openList, 0, openList_CurSize - 1, tempOpenList);

	} while (openList_CurSize > 0);


	//cout << "---can not find way---" << endl;
	return nullptr;
}
//bool CAstar::SameDirect(APoint* curPoint, APoint* nextPoint)
//{
//    return (curPathDirect == PathDirection::Vertical && nextPoint->x == curPoint->x)//ά��ԭ����
//        || (curPathDirect == PathDirection::Horizon && nextPoint->y == curPoint->y);
//}
//�õ�F=G+H+E(E��Ϊ�˶�·������΢�������ٹյ㣩
__device__ double CAstar::getF(APoint* point)
{
    return (point->g + getH(point));
}
//����H
__device__ double CAstar::getH(APoint* point)
{
    //�����ٳ��н������㷨
    return abs(_endPoint->y - point->y) + abs(_endPoint->x - point->x);
}
//����E
__device__ double CAstar::getE(APoint* curPoint, APoint* nextPoint, APoint* endPoint)
{
    //��һ���������ֱ�ߵ㣨���ǹյ㣩��E=0
    if (curPoint->parent == NULL)//��һ����
    {
        if (curPathDirect == PathDirection::Horizon)
        {
            return (nextPoint->y == curPoint->y) ? 0 : 2;
        }
        if (curPathDirect == PathDirection::Vertical)
        {
            return (nextPoint->x == curPoint->x) ? 0 : 2;
        }
    }
    if (nextPoint->x == curPoint->parent->x
        || nextPoint->y == curPoint->parent->y)//ά��ԭ����
    {
        return 0.0;
    }

    //�����յ�ĵ�
    if (nextPoint->x == endPoint->x || nextPoint->y == endPoint->y)
    {
        return 1.0;
    }

    return 2.0;
}
__device__ APoint** CAstar::getNeighboringPoint(APoint* point)//���ڽڵ�����4��
{
	neighbourList_CurSize = 0;//���neighbor
    //����ѡ����ݵ�ǰ�������������˳��
    if (curPathDirect == PathDirection::Vertical)//·�߷���ֱ���ȼ�鴹ֱ����
    {
        if (point->rowIndex < pointRowNum - 1)
        {
            if (_allPoints[(point->rowIndex + 1) * pointColNum + point->colIndex]->type != AType::ATYPE_BARRIER)
            {
                _neighbourList[neighbourList_CurSize++] = _allPoints[(point->rowIndex + 1) * pointColNum + point->colIndex];
            }
        }
        if (point->rowIndex > 0)
        {
            if (_allPoints[(point->rowIndex - 1) * pointColNum + point->colIndex]->type != AType::ATYPE_BARRIER)
            {
                _neighbourList[neighbourList_CurSize++] = _allPoints[(point->rowIndex - 1) * pointColNum + point->colIndex];
            }
        }
        if (point->colIndex < pointColNum - 1)
        {
            if (_allPoints[point->rowIndex * pointColNum + point->colIndex + 1]->type != AType::ATYPE_BARRIER)
            {
                _neighbourList[neighbourList_CurSize++] = _allPoints[point->rowIndex * pointColNum + point->colIndex + 1];
            }
        }
        if (point->colIndex > 0)
        {
            if (_allPoints[point->rowIndex * pointColNum + point->colIndex - 1]->type != AType::ATYPE_BARRIER)
            {
                _neighbourList[neighbourList_CurSize++] = _allPoints[point->rowIndex * pointColNum + point->colIndex - 1];
            }
        }
    }  
    else//ˮƽ�����ȼ��ˮƽ����Ľڵ�
    {
        if (point->colIndex < pointColNum - 1)
        {
            if (_allPoints[point->rowIndex * pointColNum + point->colIndex + 1]->type != AType::ATYPE_BARRIER)
            {
                _neighbourList[neighbourList_CurSize++] = _allPoints[point->rowIndex * pointColNum + point->colIndex + 1];
            }
        }
        if (point->colIndex > 0)
        {
            if (_allPoints[point->rowIndex * pointColNum + point->colIndex - 1]->type != AType::ATYPE_BARRIER)
            {
                _neighbourList[neighbourList_CurSize++] = _allPoints[point->rowIndex * pointColNum + point->colIndex - 1];
            }
        }
        if (point->rowIndex < pointColNum - 1)
        {
            if (_allPoints[(point->rowIndex + 1) * pointColNum + point->colIndex]->type != AType::ATYPE_BARRIER)
            {
                _neighbourList[neighbourList_CurSize++] = _allPoints[(point->rowIndex + 1) * pointColNum + point->colIndex];
            }
        }
        if (point->rowIndex > 0)
        {
            if (_allPoints[(point->rowIndex - 1) * pointColNum + point->colIndex]->type != AType::ATYPE_BARRIER)
            {
                _neighbourList[neighbourList_CurSize++] = _allPoints[(point->rowIndex - 1) * pointColNum + point->colIndex];
            }
        }
    }

    return _neighbourList;
}
