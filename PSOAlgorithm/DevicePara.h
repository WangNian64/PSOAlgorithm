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
	Inout		//出入口
};
//邻接点结构
struct AdjPoint
{
	InoutType inoutType;//出入口类型
	Vector2 pos;//出入口的位置
};
//设备朝向
enum DeviceDirect
{
	Up,			//上
	Down,		//下
	Left,		//左
	Right		//右
};
//单个设备的参数
class DevicePara
{
public:
	int ID;				//设备ID
	//DeviceType type;	//设备种类相关参数 
	double workSpeed;	//输送/操作效率（xx秒，运输或者加工一个单位物料）
	double length;		//设备长度
	double width;		//设备宽度
	Vector2 axis;		//设备坐标
	DeviceDirect direct;//设备朝向

	//四个方向的出入口数组（会影响输送线的布局）
	vector<AdjPoint> leftAdjPoints;
	vector<AdjPoint> rightAdjPoints;
	vector<AdjPoint> upAdjPoints;
	vector<AdjPoint> downAdjPoints;

};
