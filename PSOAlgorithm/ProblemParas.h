#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

struct Size {
	double x;
	double y;
};
//成本计算相关参数
struct CostPara {
	double MatFlow;		//物流总量
	double UnitCost;	//单位物流量的成本
};
struct ProblemParas
{	
	int DeviceSum;					//设备总数
	Size* DeviceSizeArray;			//设备尺寸数组

	Size WorkshopSize;				//车间尺寸

	CostPara** CostParaArray;			//成本计算参数数组

	ProblemParas() {}
	ProblemParas(int deviceNum)
	{
		DeviceSum = deviceNum;
	}
};
