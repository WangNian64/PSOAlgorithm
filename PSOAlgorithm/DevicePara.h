#pragma once
#include <iostream>
#include <vector>
using namespace std;
//表示一个坐标
struct Vector2
{
public:
	double x;
	double y;
};
//设备种类
enum DeviceTypeEnum
{
	HighBayWarehouse,
	DenseWarehouse,
	MultiWarehouse,

	StraightConveryer,
	CentringConveyor,
	AlignConveyor,
	CurveConveyor,
	SpiralConveyor,
	LiftingConveyor,

	LiftingTransfer,
	UniversalWheelTransfer,

	RGVSys,
};
//每种设备的独特参数
struct DeviceType
{
	DeviceTypeEnum ypeName;//设备种类名
	vector<DeviceType> adjTypes;//邻接设备的类型
	int adjLowNum;//设备邻接设备数目下界
	int adjUpNum;//设备邻接设备数目上界
};
//设备朝向
enum DeviceDirect
{
	Up,
	Down,
	Left,
	Right
};
//单个设备的参数
class DevicePara
{
public:
	int ID;				//设备ID
	DeviceType type;	//设备种类相关参数 
	double length;		//设备长度
	double width;		//设备宽度
	double axisX;		//设备X坐标
	double axisY;		//设备Y坐标
	DeviceDirect direct;//设备朝向

	vector<Vector2> adjPoints;//设备邻接点位置数组(根据设备大小和种类计算）

};
