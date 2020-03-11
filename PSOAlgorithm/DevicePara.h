#pragma once
#include <iostream>
#include <vector>
#include "AStar.h"
using namespace std;
//表示一个坐标
struct Vector2
{
public:
	double x;
	double y;
	Vector2(){}
	Vector2(double x, double y)
	{
		this->x = x;
		this->y = y;
	}
};
//设备种类（暂时）
enum DeviceTypeEnum
{
	//立体库
	HighBayWarehouse,
	DenseWarehouse,
	MultiWarehouse,

	//输送机
	StraightConveryer,
	CentringConveyor,
	AlignConveyor,
	CurveConveyor,
	SpiralConveyor,
	LiftingConveyor,

	//顶升、万向轮、四轴机械手
	LiftingTransfer,
	UniversalWheelTransfer,
	FourJointArm,

	//RGV
	RGVSys,
};
//每种设备的独特参数
struct DeviceType
{
	DeviceTypeEnum typeName;//设备种类名
	vector<DeviceType> adjTypes;//邻接设备的类型
	int adjLowNum;//设备邻接设备数目下界
	int adjUpNum;//设备邻接设备数目上界
};
//出入口 类型
enum InoutType
{
	In,			//入口
	Out,		//出口
};

//出入口点的朝向
enum PointDirect
{
	Up = 1, Down = 2, Left = 3, Right = 4
};
//邻接点结构
struct AdjPoint
{
public:
	int index;//下标
	InoutType inoutType;//出入口类型
	Vector2 pos;		//出入口的位置
	PointDirect direct;//出入点的方向
	PointDirect GetDirect(string str)
	{
		PointDirect PD;
		if (str == "UP")
			PD = (PointDirect)1;
		if (str == "DOWN")
			PD = (PointDirect)2;
		if (str == "LEFT")
			PD = (PointDirect)3;
		if (str == "RIGHT")
			PD = (PointDirect)4;
		return PD;
	}
};
//设备朝向（顺时针转，分为默认/90/180/270）
enum DeviceDirect
{
	Default,
	Rotate90,
	Rotate180,
	Rotate270
};
//设备相关性(从0到5依次增大）
enum DeviceRelation
{
	X,U,O,I,E,A
};
//单个设备的参数
class DevicePara
{
public:
	int ID;				//设备ID
	//DeviceType type;	//设备种类相关参数 
	double workSpeed;	//加工/处理1单位物料的时间
	Vector2 size;		//设备尺寸（分别是x轴和y轴的长度）
	Vector2 axis;		//设备坐标
	DeviceDirect direct;//设备朝向
	double spaceLength;//空隙（为了实现距离约束）
	//出入口点的数组（会影响输送线的布局）
	vector<AdjPoint> adjPointsIn;//入口
	vector<AdjPoint> adjPointsOut;//出口
	vector<AdjPoint> usableAdjPointsIn;
	vector<AdjPoint> usableAdjPointsOut;
	DevicePara() {}
	~DevicePara() {
		vector<AdjPoint>().swap(adjPointsIn);
		vector<AdjPoint>().swap(adjPointsOut);
		vector<AdjPoint>().swap(usableAdjPointsIn);
		vector<AdjPoint>().swap(usableAdjPointsOut);
	}
};
//出入口相连的数据结构
struct PointLink
{
public:
	int device1Index;
	int device1PointIndex;
	int device2Index;
	int device2PointIndex;
	vector<Vector2> points;
	PointLink() {}
	PointLink(int device1Index, int device1PointIndex, int device2Index, int device2PointIndex, vector<Vector2> points)
	{
		this->device1Index = device1Index;
		this->device1PointIndex = device1PointIndex;
		this->device2Index = device2Index;
		this->device2PointIndex = device2PointIndex;
		this->points = points;
	}
};

//物料信息
struct CargoType
{
	string cargoName;	//物料名
	int deviceSum;		//经过的设备总数
	int* deviceList;	//设备队列
	double totalVolume;	//一段时间的总物流量
};

