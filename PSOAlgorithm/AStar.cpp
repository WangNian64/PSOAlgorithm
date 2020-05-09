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
#include "DevicePara.h"
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
APoint* CAstar::findWay(PathDirection beginDirect, int beginRowIndex, int beginColIndex, int endRowIndex, int endColIndex)
{
    curPathDirect = beginDirect;
    _endPoint = _allPoints[endRowIndex][endColIndex];
    APoint* beginPoint = _allPoints[beginRowIndex][beginColIndex];

    if (_endPoint->type == AType::ATYPE_BARRIER)
    {
        cout << "终点是障碍" << endl;
        return nullptr;
    }
    if (*_endPoint == *beginPoint)
    {
        _curPoint = beginPoint;
        _curPoint->parent = _endPoint;
        return _curPoint;
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
            double tempG = _curPoint->g + tmpoint->CalcuPointDist(*_curPoint) + getE(_curPoint, tmpoint, _endPoint);
            if (tmpoint->type == AType::ATYPE_CLOSED)
            {
                continue;
            }
            //是否在开放列表里
            if (tmpoint->type != AType::ATYPE_OPENED)
            {
                tmpoint->parent = _curPoint;
                //计算GHF
                tmpoint->g = tempG;
                //更新方向
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
                //添加到开放列表里
                _openList.push_back(tmpoint);
                tmpoint->type = AType::ATYPE_OPENED;
            }
            else
            {
                //已经在开放列表里
                //if (tmpoint->g < _curPoint->g)
                //if (tmpoint->h < _curPoint->h)
                if (tempG < tmpoint->g)
                {
                    tmpoint->parent = _curPoint;
                    tmpoint->g = tempG;
                    //更新方向
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
        //排序 F值最小的排在前面
        stable_sort(_openList.begin(), _openList.end(), mySort);

    } while (_openList.size() > 0);


    cout << "---can not find way---" << endl;

    return nullptr;
}
bool CAstar::SameDirect(APoint* curPoint, APoint* nextPoint)
{
    return (curPathDirect == PathDirection::Vertical && nextPoint->x == curPoint->x)//维持原方向
        || (curPathDirect == PathDirection::Horizon && nextPoint->y == curPoint->y);
}
//得到F=G+H+E(E是为了对路径进行微调，减少拐点）
double CAstar::getF(APoint* point)
{
    return (point->g + getH(point));
}
//估算H
double CAstar::getH(APoint* point)
{
    //曼哈顿城市街区估算法
    return abs(_endPoint->y - point->y) + abs(_endPoint->x - point->x);
}
//计算E
double CAstar::getE(APoint* curPoint, APoint* nextPoint, APoint* endPoint)
{
    //第一个点或者是直线点（不是拐点），E=0
    if (curPoint->parent == NULL)//第一个点
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
        || nextPoint->y == curPoint->parent->y)//维持原方向
    {
        return 0.0;
    }

    //拐向终点的点
    if (nextPoint->x == endPoint->x || nextPoint->y == endPoint->y)
    {
        return 1.0;
    }

    return 2.0;
}
vector<APoint*> CAstar::getNeighboringPoint(APoint* point)
{
    _neighbourList.clear();
    //可以选择根据当前方向调整点的添加顺序
    if (curPathDirect == PathDirection::Vertical)//路线方向垂直，先检查垂直方向
    {

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
    }  
    else//水平方向，先检查水平方向的节点
    {
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
    }

    return _neighbourList;
}
