//
//  CAstar.cpp
//  Astar
//
//  Created by xujw on 15/4/9.
//  Copyright (c) 2015年 xujw. All rights reserved.
//
//  上下左右走一格花销为10，斜着走一格花销为14，以方便计算
//  即格子宽高为10 对角线为14
#pragma once
#include "AStar.h"
#include <algorithm>
//自定义排序函数
bool mySort(const APoint* p1, const APoint* p2)
{
    return p1->f < p2->f;
}

APoint::APoint() :x(0)
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

}


#pragma mark------CAstar-------

CAstar::CAstar() :_endPoint(nullptr)
, _curPoint(nullptr)
{
}

CAstar::~CAstar()
{
    _openList.clear();
    _closeList.clear();
    _neighbourList.clear();
    _allPoints.clear();
}
//计算一条路径的总长度
double CAstar::CalcuPathLength(APoint* point)
{
    double length = 0.0;
    while (point->parent)
    {
        length += point->CalcuPointDist(*(point->parent));
        point = point->parent;
    }
    return length;
}
void CAstar::resetAStar()
{
    for (int i = 0; i < _allPoints.size(); i++)
    {
        for (int j = 0; j < _allPoints[i].size(); j++)
        {
            _allPoints[i][j]->resetAPoint();
        }
    }
    vector<APoint*>().swap(_openList);
    vector<APoint*>().swap(_closeList);
    vector<APoint*>().swap(_neighbourList);
}
APoint* CAstar::findWay(int beginRowIndex, int beginColIndex, int endRowIndex, int endColIndex)
{
    _endPoint = _allPoints[endRowIndex][endColIndex];
    APoint* beginPoint = _allPoints[beginRowIndex][beginColIndex];
    if (_endPoint->type == AType::ATYPE_BARRIER)
    {
        cout << "终点是障碍" << endl;
        return nullptr;
    }
    if (*_endPoint == *beginPoint)
    {
        cout << "起始点相同" << endl;
        return nullptr;
    }

    _openList.push_back(beginPoint);
    beginPoint->type = AType::ATYPE_OPENED;
    beginPoint->f = getF(beginPoint);
    //---------
    do
    {
        //获取最小值的节点
        _curPoint = _openList[0];
        _openList.erase(_openList.begin());
        _curPoint->type = AType::ATYPE_CLOSED;
        _closeList.push_back(_curPoint);

        if (*_curPoint == *_endPoint)
        {
            //cout << "have find way" << endl;
            return _curPoint;
        }
        //获取相邻的节点
        vector<APoint*> neVec = getNeighboringPoint(_curPoint);
        for (int i = 0; i < neVec.size(); i++)
        {
            auto tmpoint = neVec[i];
            if (tmpoint->type == AType::ATYPE_CLOSED)
            {
                continue;
            }
            //是否在开放列表里
            if (tmpoint->type != AType::ATYPE_OPENED)
            {
                tmpoint->parent = _curPoint;
                tmpoint->g = _curPoint->g + tmpoint->CalcuPointDist(*_curPoint);////////////
                //计算H值
                tmpoint->h = getH(tmpoint);
                //添加到开放列表里
                _openList.push_back(tmpoint);
                tmpoint->type = AType::ATYPE_OPENED;
            }
            else
            {
                //已经在开放列表里
                if (tmpoint->h < _curPoint->h)
                {
                    tmpoint->parent = _curPoint;
                    tmpoint->g = _curPoint->g + tmpoint->CalcuPointDist(*_curPoint);////////////
                }
            }
        }
        //排序 F值最小的排在前面
        sort(_openList.begin(), _openList.end(), mySort);

    } while (_openList.size() > 0);


    cout << "---can not find way---" << endl;

    return nullptr;
}
//得到F=G+H
int CAstar::getF(APoint* point)
{
    return (point->g + getH(point));
}
//估算H
int CAstar::getH(APoint* point)
{
    //曼哈顿城市街区估算法
    return abs(_endPoint->y - point->y) + abs(_endPoint->x - point->x);
}

vector<APoint*> CAstar::getNeighboringPoint(APoint* point)
{
    _neighbourList.clear();
    if (point->rowIndex < _allPoints.size() - 1)
    {
        if (_allPoints[point->rowIndex + 1][point->colIndex]->type != AType::ATYPE_BARRIER)
        {
            _neighbourList.push_back(_allPoints[point->rowIndex + 1][point->colIndex]);
        }
    }
    if (point->rowIndex > 0)
    {
        if (_allPoints[point->rowIndex - 1][point->colIndex]->type != AType::ATYPE_BARRIER)
        {
            _neighbourList.push_back(_allPoints[point->rowIndex - 1][point->colIndex]);
        }
    }
    if (point->colIndex < _allPoints[0].size() - 1)
    {
        if (_allPoints[point->rowIndex][point->colIndex + 1]->type != AType::ATYPE_BARRIER)
        {
            _neighbourList.push_back(_allPoints[point->rowIndex][point->colIndex + 1]);
        }
    }
    if (point->colIndex > 0)
    {
        if (_allPoints[point->rowIndex][point->colIndex - 1]->type != AType::ATYPE_BARRIER)
        {
            _neighbourList.push_back(_allPoints[point->rowIndex][point->colIndex - 1]);
        }
    }

    return _neighbourList;
}
