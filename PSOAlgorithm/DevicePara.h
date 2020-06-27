#pragma once
#include <cmath>
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
	double Distance(Vector2 v) const
	{
		return sqrt(pow(this->x - v.x, 2) + pow(this->y - v.y, 2));
	}
	bool operator<(const Vector2& v) const
	{
		if (*this == v) {
			return false;
		}
		else
		{
			if (this->x == v.x)
			{
				return this->y < v.y;
			}
			else
			{
				return this->x < v.x;
			}
		}
	}
	bool operator!=(const Vector2& v) const
	{
		return !(*this == v);
	}
	bool operator>(const Vector2& v) const
	{
		return !(*this < v || *this == v);
	}
	bool operator==(const Vector2& v) const
	{
		return abs(this->x - v.x) <= 0.0001 && abs(this->y - v.y) <= 0.0001;
	}
};
struct Vector2Int
{
public:
	int x;
	int y;
	Vector2Int() {}
	Vector2Int(int x, int y)
	{
		this->x = x;
		this->y = y;
	}
	double Distance(Vector2Int v) const
	{
		return sqrt(pow((this->x - v.x)/10000.0f, 2) + pow((this->y - v.y)/10000.0f, 2));
	}
	bool operator<(const Vector2Int& v) const
	{
		if (*this == v) {
			return false;
		}
		else
		{
			if (this->x == v.x)
			{
				return this->y < v.y;
			}
			else
			{
				return this->x < v.x;
			}
		}
	}
	bool operator!=(const Vector2Int& v) const
	{
		return !(*this == v);
	}
	bool operator>(const Vector2Int& v) const
	{
		return !(*this < v || *this == v);
	}
	bool operator==(const Vector2Int& v) const
	{
		return this->x == v.x && this->y == v.y;
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

//�������߶εķ��򣺴�ֱ/ˮƽ
enum PathPointDirect
{
	Vert, Hori
};
//·�������Ϣ
struct PointInfo
{
	int vertDirNum;
	int horiDirNum;
	bool isKeep;//�Ƿ���
	PointInfo() = default;
	PointInfo(int vDirNum, int hDirNum, bool iK) 
	{
		vertDirNum = vDirNum;
		horiDirNum = hDirNum;
		isKeep = iK;
	}
};
//�豸�����(��0��5��������
enum DeviceRelation
{
	X,U,O,I,E,A
};
struct DeviceIDSize
{
	int ID;
	Vector2 size;
	DeviceIDSize(int id, Vector2 s) {
		ID = id;
		size = s;
	}
	bool operator<(const DeviceIDSize& rhs) const {
		return (this->size.x * this->size.y) < (rhs.size.x * rhs.size.y);
	}
};
//�����豸�Ĳ���
class DevicePara
{
public:
	int ID;					//�豸ID
	//DeviceType type;		//�豸������ز��� 
	double workSpeed;		//�ӹ�/����1��λ���ϵ�ʱ��
	Vector2 size;			//�豸�ߴ磨�ֱ���x���y��ĳ��ȣ�
	Vector2 axis;			//�豸����
	DeviceDirect direct;	//�豸����
	double spaceLength;		//��϶��Ϊ��ʵ�־���Լ����
	//����ڵ�����飨��Ӱ�������ߵĲ��֣�
	vector<AdjPoint> adjPointsIn;//���
	vector<AdjPoint> adjPointsOut;//����
	DevicePara() {}
	~DevicePara() {
		vector<AdjPoint>().swap(adjPointsIn);
		vector<AdjPoint>().swap(adjPointsOut);
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
struct DeviceLink
{
	int inDeviceIndex;
	int outDeviceIndex;
	int inPointIndex;
	int outPointIndex;
	DeviceLink() {
		inDeviceIndex = outDeviceIndex = inPointIndex = outPointIndex = 0;
	}
};
//������Ϣ
struct CargoType
{
	string cargoName;	//������
	int deviceSum;		//�������豸��Ŀ
	int linkSum;		//�豸��Ե���Ŀ
	DeviceLink* deviceLinkList;	//�豸�����б�
	double totalVolume;	//�����ϵ���������
};
//һС��·�������ݽṹ
struct SegPath
{
	Vector2Int p1;
	Vector2Int p2;
	PathPointDirect direct;
	SegPath(Vector2Int p1, Vector2Int p2)
	{
		this->p1 = p1;
		this->p2 = p2;
		if (abs(p1.x - p2.x) > abs(p1.y - p2.y)) {
			direct = PathPointDirect::Hori;
		}
		else {
			direct = PathPointDirect::Vert;
		}
	}
	bool operator<(const SegPath& sg) const
	{
		if ((this->p1 == sg.p1 && this->p2 == sg.p2) || (this->p1 == sg.p2 && this->p2 == sg.p1))
			return false;
		else
		{ 
			if (this->p1 == sg.p1)
			{
				return this->p2 < sg.p2;
			}
			else
			{
				return this->p1 < sg.p1;
			}
		}
	}
};
struct StraightConveyorInfo
{
	Vector2Int startPos;
	Vector2Int endPos;
	int startVnum;
	int startHnum;
	int endVnum;
	int endHnum;
	StraightConveyorInfo() = default;
	StraightConveyorInfo(Vector2Int sPos, Vector2Int ePos) {
		startPos = sPos;
		endPos = ePos;
	}
	bool operator<(const StraightConveyorInfo& rhs) const 
	{
		if (rhs.startPos == this->startPos && rhs.endPos == this->endPos) 
		{
			return false;
		}
		else
		{
			if (this->startPos == rhs.startPos)
			{
				return this->endPos < rhs.endPos;
			}
			else
			{
				return this->startPos < rhs.startPos;
			}
		}
	}
};