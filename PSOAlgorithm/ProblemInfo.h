#pragma once
#include <iostream>
using namespace std;

struct ProblemParas
{
	int DeviceSum;				//设备总数
	double* DeviceDist;			//设备之间的距离数组
	double* DeviceLengthArray;	//设备的长度数组
	double* DeviceWidthArray;	//设备的宽度数组
	double WorkshopLength;		//车间长度
	double WorkshopWidth;		//车间宽度
	double* DeviceSquareArray;	//设备面积数组
	double* UnitCostArray;		//设备之间单位距离的物流搬运成本数组
	int* TotalMatFlowArray;		//物流总量数组

	int* AdjRankArray;			//邻接等级数组
};
