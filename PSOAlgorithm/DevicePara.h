#pragma once
#include <cuda_runtime.h>
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
	//device构造函数
	__device__ Vector2(double x, double y, int i)//device构造函数
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
	__device__ Vector2Int(int x, int y, int i)
	{
		this->x = x;
		this->y = y;
	}
	__device__ double Distance(Vector2Int v) const
	{
		return sqrt(powf((this->x - v.x)/10000.0f, 2) + powf((this->y - v.y)/10000.0f, 2));
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
	Vector2Int pointAxis;//点的坐标
	int vertDirNum;//点的垂直连线数目
	int horiDirNum;//点的水平连线数目
	bool isKeep;//是否保留
	PointInfo();
	PointInfo(Vector2Int pointAxis, int vDirNum, int hDirNum, bool iK) 
	{
		this->pointAxis = pointAxis;
		vertDirNum = vDirNum;
		horiDirNum = hDirNum;
		isKeep = iK;
	}
	__device__ PointInfo(Vector2Int pointAxis, int vDirNum, int hDirNum, bool iK, int i)
	{
		this->pointAxis = pointAxis;
		vertDirNum = vDirNum;
		horiDirNum = hDirNum;
		isKeep = iK;
	}
	bool AEqualB(const PointInfo& obj)//A==B
	{
		return this->pointAxis == obj.pointAxis;
	}
	bool ABigB(const PointInfo& obj)//A>B
	{
		if (this->pointAxis == obj.pointAxis)
			return false;
		else//A!=B
		{
			if (this->pointAxis.x == obj.pointAxis.x)
			{
				return this->pointAxis.y > obj.pointAxis.y;
			}
			else
			{
				return this->pointAxis.x > obj.pointAxis.x;
			}
		}
	}
	bool ABigEqualB(const PointInfo& obj) //A>=B
	{
		return this->AEqualB(obj) || this->ABigB(obj);
	}
	bool ASmallB(const PointInfo& obj)//A<B
	{
		return !this->ABigEqualB(obj);
	}
	bool ASmallEqualB(const PointInfo& obj)//A<=B
	{
		return !this->ABigB(obj);
	}


	//device版本的函数
	__device__ bool AEqualB(const PointInfo& obj, int i)//A==B
	{
		return this->pointAxis == obj.pointAxis;
	}
	__device__ bool ABigB(const PointInfo& obj, int i)//A>B
	{
		if (this->pointAxis == obj.pointAxis)
			return false;
		else//A!=B
		{
			if (this->pointAxis.x == obj.pointAxis.x)
			{
				return this->pointAxis.y > obj.pointAxis.y;
			}
			else
			{
				return this->pointAxis.x > obj.pointAxis.x;
			}
		}
	}
	__device__ bool ABigEqualB(const PointInfo& obj, int i) //A>=B
	{
		return this->AEqualB(obj, i) || this->ABigB(obj, i);
	}
	__device__ bool ASmallB(const PointInfo& obj, int i)//A<B
	{
		return !this->ABigEqualB(obj, i);
	}
	__device__ bool ASmallEqualB(const PointInfo& obj, int i)//A<=B
	{
		return !this->ABigB(obj, i);
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
	DeviceIDSize() {}
	DeviceIDSize(int id, Vector2 s) {
		ID = id;
		size = s;
	}
	__device__ DeviceIDSize(int id, Vector2 s, int i) {
		ID = id;
		size = s;
	}
	bool operator<(const DeviceIDSize& rhs) const {
		return (this->size.x * this->size.y) < (rhs.size.x * rhs.size.y);//按照面积大小排序
	}
};
//单个设备的参数
class DevicePara
{
public:
	int ID;					//设备ID
	double workSpeed;		//加工/处理1单位物料的时间
	Vector2 size;			//设备尺寸（分别是x轴和y轴的长度）
	Vector2 axis;			//设备坐标
	DeviceDirect direct;	//设备朝向
	double spaceLength;		//空隙（为了实现距离约束）
	//出入口点的数组（会影响输送线的布局）
	int adjPInCount;
	int adjPOutCount;
	AdjPoint* adjPointsIn;	//入口
	AdjPoint* adjPointsOut;	//出口
	DevicePara() {}
	~DevicePara() {

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
	Vector2* points;//这个
	int pointNum;//点的数目

	PointLink() {}
	PointLink(int device1Index, int device1PointIndex, int device2Index, int device2PointIndex, Vector2* points, int pointNum)
	{
		this->device1Index = device1Index;
		this->device1PointIndex = device1PointIndex;
		this->device2Index = device2Index;
		this->device2PointIndex = device2PointIndex;
		this->points = points;
		this->pointNum = pointNum;
	}
	__device__ PointLink(int device1Index, int device1PointIndex, int device2Index, int device2PointIndex, Vector2* points, int pointNum, int i)
	{
		this->device1Index = device1Index;
		this->device1PointIndex = device1PointIndex;
		this->device2Index = device2Index;
		this->device2PointIndex = device2PointIndex;
		this->points = points;
		this->pointNum = pointNum;
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
	__device__ InoutPoint(int pointDirect, Vector2 pointAxis, int i)
	{
		this->pointAxis = pointAxis;
		this->pointDirect = pointDirect;
	}
};
struct DeviceLink
{
	int inDeviceIndex;//入口设备的index
	int outDeviceIndex;//出口设备的index
	int inPointIndex;//把这个含义改为总的inPoint的下标（现在都是一维的）
	int outPointIndex;//同上
	DeviceLink() {
		inDeviceIndex = outDeviceIndex = inPointIndex = outPointIndex = 0;
	}
};
//物料信息
struct CargoType
{
	int deviceSum;				//经过的设备数目
	int linkSum;				//设备配对的数目
	DeviceLink* deviceLinkList;	//设备连接列表
	double totalVolume;			//该物料的总物流量
};
//一小段路径的数据结构（起点，终点，连线的方向）
struct SegPath 
{
	Vector2Int p1;
	Vector2Int p2;
	PathPointDirect direct;
	SegPath() {}
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
	__device__ SegPath(Vector2Int p1, Vector2Int p2, int i)
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
	bool AEqualB(const SegPath& sg)//A==B
	{
		return (this->p1 == sg.p1 && this->p2 == sg.p2) || (this->p1 == sg.p2 && this->p2 == sg.p1);
	}
	bool ABigB(const SegPath& sg)//A>B
	{
		if ((this->p1 == sg.p1 && this->p2 == sg.p2) || (this->p1 == sg.p2 && this->p2 == sg.p1))
			return false;
		else//A!=B
		{
			if (this->p1 == sg.p1)
			{
				return this->p2 > sg.p2;
			}
			else
			{
				return this->p1 > sg.p1;
			}
		}
	}
	bool ABigEqualB(const SegPath& sg) //A>=B
	{
		return this->AEqualB(sg) || this->ABigB(sg);
	}
	bool ASmallB(const SegPath& sg)//A<B
	{
		return !this->ABigEqualB(sg);
	}
	bool ASmallEqualB(const SegPath& sg)//A<=B
	{
		return !this->ABigB(sg);
	}



	/////////__device__版本的
	__device__ bool AEqualB(const SegPath& sg, int i)//A==B
	{
		return (this->p1 == sg.p1 && this->p2 == sg.p2) || (this->p1 == sg.p2 && this->p2 == sg.p1);
	}
	__device__ bool ABigB(const SegPath& sg, int i)//A>B
	{
		if ((this->p1 == sg.p1 && this->p2 == sg.p2) || (this->p1 == sg.p2 && this->p2 == sg.p1))
			return false;
		else//A!=B
		{
			if (this->p1 == sg.p1)
			{
				return this->p2 > sg.p2;
			}
			else
			{
				return this->p1 > sg.p1;
			}
		}
	}
	__device__ bool ABigEqualB(const SegPath& sg, int i) //A>=B
	{
		return this->AEqualB(sg, i) || this->ABigB(sg, i);
	}
	__device__ bool ASmallB(const SegPath& sg, int i)//A<B
	{
		return !this->ABigEqualB(sg, i);
	}
	__device__ bool ASmallEqualB(const SegPath& sg, int i)//A<=B
	{
		return !this->ABigB(sg, i);
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
	StraightConveyorInfo();
	StraightConveyorInfo(Vector2Int sPos, Vector2Int ePos)
	{
		startPos = sPos;
		endPos = ePos;
	}
	__device__ StraightConveyorInfo(Vector2Int sPos, Vector2Int ePos, int i)
	{
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