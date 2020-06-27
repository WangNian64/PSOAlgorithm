#pragma once
#include <cmath>
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
	Up = 1, Right = 2, Down = 3, Left = 4
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
		if (str == "RIGHT")
			PD = (PointDirect)2;
		if (str == "DOWN")
			PD = (PointDirect)3;
		if (str == "LEFT")
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

//点所在线段的方向：垂直/水平
enum PathPointDirect
{
	Vert, Hori
};
//路径点的信息
struct PointInfo
{
	int vertDirNum;
	int horiDirNum;
	bool isKeep;//是否保留
	PointInfo() = default;
	PointInfo(int vDirNum, int hDirNum, bool iK) 
	{
		vertDirNum = vDirNum;
		horiDirNum = hDirNum;
		isKeep = iK;
	}
};
//设备相关性(从0到5依次增大）
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
//单个设备的参数
class DevicePara
{
public:
	int ID;					//设备ID
	//DeviceType type;		//设备种类相关参数 
	double workSpeed;		//加工/处理1单位物料的时间
	Vector2 size;			//设备尺寸（分别是x轴和y轴的长度）
	Vector2 axis;			//设备坐标
	DeviceDirect direct;	//设备朝向
	double spaceLength;		//空隙（为了实现距离约束）
	//出入口点的数组（会影响输送线的布局）
	vector<AdjPoint> adjPointsIn;//入口
	vector<AdjPoint> adjPointsOut;//出口
	DevicePara() {}
	~DevicePara() {
		vector<AdjPoint>().swap(adjPointsIn);
		vector<AdjPoint>().swap(adjPointsOut);
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
struct InoutPoint
{
	int pointDirect;//0表示垂直，1表示水平
	Vector2 pointAxis;//点坐标
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
//物料信息
struct CargoType
{
	string cargoName;	//物料名
	int deviceSum;		//经过的设备数目
	int linkSum;		//设备配对的数目
	DeviceLink* deviceLinkList;	//设备连接列表
	double totalVolume;	//该物料的总物流量
};
//一小段路径的数据结构
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