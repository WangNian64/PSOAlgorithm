#pragma once
#include <iostream>
#include <vector>
#include "AStar.h"
using namespace std;
//��ʾһ������
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
//�豸���ࣨ��ʱ��
enum DeviceTypeEnum
{
	//�����
	HighBayWarehouse,
	DenseWarehouse,
	MultiWarehouse,

	//���ͻ�
	StraightConveryer,
	CentringConveyor,
	AlignConveyor,
	CurveConveyor,
	SpiralConveyor,
	LiftingConveyor,

	//�����������֡������е��
	LiftingTransfer,
	UniversalWheelTransfer,
	FourJointArm,

	//RGV
	RGVSys,
};
//ÿ���豸�Ķ��ز���
struct DeviceType
{
	DeviceTypeEnum typeName;//�豸������
	vector<DeviceType> adjTypes;//�ڽ��豸������
	int adjLowNum;//�豸�ڽ��豸��Ŀ�½�
	int adjUpNum;//�豸�ڽ��豸��Ŀ�Ͻ�
};
//����� ����
enum InoutType
{
	In,			//���
	Out,		//����
};

//����ڵ�ĳ���
enum PointDirect
{
	Up = 1, Right = 2, Down = 3, Left = 4
};
//�ڽӵ�ṹ
struct AdjPoint
{
public:
	int index;//�±�
	InoutType inoutType;//���������
	Vector2 pos;		//����ڵ�λ��
	PointDirect direct;//�����ķ���
	PointDirect GetDirect(string str)
	{
		PointDirect PD;
		if (str == "UP")
			PD = (PointDirect)1;
		if (str == "RIGHT")
			PD = (PointDirect)2;
		if (str == "DOWN")
			PD = (PointDirect)3;
		if (str == "LEFT")
			PD = (PointDirect)4;
		return PD;
	}
};
//�豸����˳ʱ��ת����ΪĬ��/90/180/270��
enum DeviceDirect
{
	Default,
	Rotate90,
	Rotate180,
	Rotate270
};
//�豸�����(��0��5��������
enum DeviceRelation
{
	X,U,O,I,E,A
};
//�����豸�Ĳ���
class DevicePara
{
public:
	int ID;				//�豸ID
	//DeviceType type;	//�豸������ز��� 
	double workSpeed;	//�ӹ�/����1��λ���ϵ�ʱ��
	Vector2 size;		//�豸�ߴ磨�ֱ���x���y��ĳ��ȣ�
	Vector2 axis;		//�豸����
	DeviceDirect direct;//�豸����
	double spaceLength;//��϶��Ϊ��ʵ�־���Լ����
	//����ڵ�����飨��Ӱ�������ߵĲ��֣�
	vector<AdjPoint> adjPointsIn;//���
	vector<AdjPoint> adjPointsOut;//����
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
//��������������ݽṹ
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
struct InoutPoint
{
	int pointDirect;//0��ʾ��ֱ��1��ʾˮƽ
	Vector2 pointAxis;//������
	InoutPoint() {}
	InoutPoint(int pointDirect, Vector2 pointAxis)
	{
		this->pointAxis = pointAxis;
		this->pointDirect = pointDirect;
	}
};
//������Ϣ
struct CargoType
{
	string cargoName;	//������
	int deviceSum;		//�������豸����
	int* deviceList;	//�豸����
	double totalVolume;	//һ��ʱ�����������
};

