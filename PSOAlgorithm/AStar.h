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
#include <vector>
#include <iostream>
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
    APoint();
    ~APoint();
    int rowIndex;
    int colIndex;
    double x;
    double y;
    AType type;   //类型:障碍、开放列表、关闭列表
    double f;  //f = g+h
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
    double CalcuPointDist(const APoint& point)
    {
        return abs(this->y - point.y) + abs(this->x - point.x);
    }
    void resetAPoint()
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
    vector<APoint*> _openList;      //开放列表
    vector<APoint*> _closeList;     //关闭列表
    vector<APoint*> _neighbourList; //周边节点
    APoint* _endPoint;
    APoint* _curPoint;
    vector< vector<APoint*> > _allPoints;
    PathDirection curPathDirect;
    CAstar();
    ~CAstar();
    double CalcuPathLength(APoint* point);
    APoint* findWay(int beginRowIndex, int beginColIndex, int endRowIndex, int endColIndex);
    void resetAStar();
    //    APoint* findWay(int beginX,int beginY,int endX,int endY);
private:
    double getF(APoint* point);
    double getH(APoint* point);
    double getE(APoint* curPoint, APoint* nextPoint, APoint* endPoint);
    vector<APoint*> getNeighboringPoint(APoint* point);
};




#endif /* defined(__Astar__CAstar__) */
