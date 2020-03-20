#pragma once
#include "PSO.h"
#include "Tools.h"
#include "AStar.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#define PI 3.14159265358979
#define DOUBLE_MAX 1.7976931348623158e+308
#define DOUBLE_MIN 2.2250738585072014e-308
#define MAX_FITNESS 1000000.0
void FitnessFunction(Particle& particle, ProblemParas proParas, double* lowerBounds, double* upBounds);
double CalcuTotalArea(Particle& particle, ProblemParas proParas);
double CalcuDeviceDist(Vector2 pos1, Vector2 pos2);

int getRandomInPoint(DevicePara& device);
int getRandomOutPoint(DevicePara& device);
int FindAxisIndex(double axis, const vector<double>& axisList);

//顺时针旋转后的坐标
Vector2 Rotate(Vector2 pointPos, Vector2 centerPos, float rotateAngle);
//默认的适应度计算函数，可以替换
void FitnessFunction(Particle& particle, ProblemParas proParas, double* lowerBounds, double* upBounds)
{
	bool IsWorkable = true;//解是否可行
	double deviceDist = 0;
	particle.fitness_[0] = particle.fitness_[1] = 0;
	//for (int i = 0; i < particle.dim_; i+=2) {
	//	for (int j = 0; j < particle.dim_; j+=2) {
	//		deviceDist = abs(particle.position_[i] - particle.position_[j])
	//			+ abs(particle.position_[i + 1] - particle.position_[j + 1]);
	//		if (deviceDist < proParas.minDistArray[i / 2][j / 2])
	//		{
	//			particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
	//			IsWorkable = false;
	//			break;
	//		}
	//		//添加成本作为适应度值（考虑最小距离）
	//		//particle.fitness_[0] += deviceDist * proParas.costParaArray[i / 2][j / 2].UnitCost * proParas.costParaArray[i / 2][j / 2].MatFlow;
	//	}
	//}

#pragma region 深拷贝一份设备信息
	DevicePara* copyDeviceParas = new DevicePara[proParas.DeviceSum];
	for (int i = 0; i < proParas.DeviceSum; i++)
	{
		copyDeviceParas[i].ID = proParas.deviceParaList[i].ID;
		copyDeviceParas[i].direct = proParas.deviceParaList[i].direct;
		copyDeviceParas[i].axis = proParas.deviceParaList[i].axis;
		copyDeviceParas[i].size = proParas.deviceParaList[i].size;
		copyDeviceParas[i].spaceLength = proParas.deviceParaList[i].spaceLength;
		copyDeviceParas[i].workSpeed = proParas.deviceParaList[i].workSpeed;
		copyDeviceParas[i].adjPointsIn = proParas.deviceParaList[i].adjPointsIn;
		copyDeviceParas[i].adjPointsOut = proParas.deviceParaList[i].adjPointsOut;
		copyDeviceParas[i].usableAdjPointsIn = proParas.deviceParaList[i].usableAdjPointsIn;
		copyDeviceParas[i].usableAdjPointsOut = proParas.deviceParaList[i].usableAdjPointsOut;
	}
#pragma endregion

#pragma region 根据设备朝向，调整设备尺寸xy和出入口坐标
	for (int i = 2; i < particle.dim_; i += 3)
	{
		//double转int，转换为Direction，然后根据朝向重新计算设备尺寸和出入口
		//Rotate90或者ROtate270，尺寸的x和y互换
		//出入口按照顺时针算，旋转角=Direction*90(正好对应0,90,180,270）
		DeviceDirect curDirect = (DeviceDirect)(int)particle.position_[i];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)
		{
			double temp = copyDeviceParas[i / 3].size.x;
			copyDeviceParas[i / 3].size.x = copyDeviceParas[i / 3].size.y;
			copyDeviceParas[i / 3].size.y = temp;
		}
		//重新计算旋转后的出入口坐标
		//Vector2 deviceCenterPos(particle.position_[i - 2], particle.position_[i - 1]);
		Vector2 deviceCenterPos(0,0);
		double rotateAngle = curDirect * 90;
		int newDirect = 0;
		for (AdjPoint &point : copyDeviceParas[i / 3].adjPointsIn)
		{
			point.pos = Rotate(point.pos, deviceCenterPos, rotateAngle);
			newDirect = point.direct + (int)curDirect;
			point.direct = (newDirect == 4) ? (PointDirect)4 : (PointDirect)(newDirect % 4);
		}
		for (AdjPoint &point : copyDeviceParas[i / 3].adjPointsOut)
		{
			point.pos = Rotate(point.pos, deviceCenterPos, rotateAngle);
			newDirect = point.direct + (int)curDirect;
			point.direct = (newDirect == 4) ? (PointDirect)4 : (PointDirect)(newDirect % 4);
		}
		for (AdjPoint &point : copyDeviceParas[i / 3].usableAdjPointsIn)
		{
			point.pos = Rotate(point.pos, deviceCenterPos, rotateAngle);
			newDirect = point.direct + (int)curDirect;
			point.direct = (newDirect == 4) ? (PointDirect)4 : (PointDirect)(newDirect % 4);
		}
		for (AdjPoint &point : copyDeviceParas[i / 3].usableAdjPointsOut)
		{
			point.pos = Rotate(point.pos, deviceCenterPos, rotateAngle);
			newDirect = point.direct + (int)curDirect;
			point.direct = (newDirect == 4) ? (PointDirect)4 : (PointDirect)(newDirect % 4);
		}
	}

#pragma endregion

#pragma region 检查设备是否重叠
	double outSizeLength, outSizeWidth;
	for (int i = 0; i < particle.dim_; i += 3) {
		outSizeLength = 0.5 * copyDeviceParas[i / 3].size.x + copyDeviceParas[i / 3].spaceLength;
		outSizeWidth = 0.5 * copyDeviceParas[i / 3].size.y + copyDeviceParas[i / 3].spaceLength;
		double firstLowX = particle.position_[i] - outSizeLength;
		double firstUpX = particle.position_[i] + outSizeLength;
		double firstLowY = particle.position_[i + 1] - outSizeWidth;
		double firstUpY = particle.position_[i + 1] + outSizeWidth;
		for (int j = i + 3; j < particle.dim_; j += 3) {
			outSizeLength = 0.5 * copyDeviceParas[j / 3].size.x + copyDeviceParas[j / 3].spaceLength;
			outSizeWidth = 0.5 * copyDeviceParas[j / 3].size.y + copyDeviceParas[j / 3].spaceLength;
			double secondLowX = particle.position_[j] - outSizeLength;
			double secondUpX = particle.position_[j] + outSizeLength;
			double secondLowY = particle.position_[j + 1] - outSizeWidth;
			double secondUpY = particle.position_[j + 1] + outSizeWidth;
			if (IsRangeOverlap(firstLowX, firstUpX, secondLowX, secondUpX) && IsRangeOverlap(firstLowY, firstUpY, secondLowY, secondUpY)) {
				particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
				IsWorkable = false;
				return;
			}
		}
	}
#pragma endregion


#pragma region 当前布局是可行解，进行布线

	if (IsWorkable == true)
	{


#pragma region 根据设备坐标和出入口坐标构造路径点图

		vector< vector<APoint*> > pathPointMap;
		vector<double> horizonAxisList;
		vector<double> verticalAxisList;
		//先对出入口点水平和垂直进行分类(注意加上偏移量)
		for (int i = 0; i < proParas.DeviceSum; i++)
		{
			for (AdjPoint p : copyDeviceParas[i].adjPointsIn)
			{
				//cout << p.pos.x << "," << p.pos.y << endl;
				if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//上下
				{
					horizonAxisList.push_back(p.pos.x + particle.position_[i * 3]);
				} else {//左右
					verticalAxisList.push_back(p.pos.y + particle.position_[i * 3 + 1]);
				}
			}
			for (AdjPoint p : copyDeviceParas[i].adjPointsOut)
			{
				//cout << p.pos.x << "," << p.pos.y << endl;
				if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//上下
				{
					horizonAxisList.push_back(p.pos.x + particle.position_[i * 3]);
				}
				else {//左右
					verticalAxisList.push_back(p.pos.y + particle.position_[i * 3 + 1]);
				}
			}
		}
		//cout << endl;
		//仓库入口2个点
		horizonAxisList.push_back(proParas.entrancePos.x);
		verticalAxisList.push_back(proParas.entrancePos.y);
		//存下每个设备坐标的四个范围（作为后面障碍点的范围）
		double* DeviceLowXList = new double[proParas.DeviceSum];
		double* DeviceHighXList = new double[proParas.DeviceSum];
		double* DeviceLowYList = new double[proParas.DeviceSum];
		double* DeviceHighYList = new double[proParas.DeviceSum];
		//每个设备周围的4个点
		for (int i = 0; i < particle.dim_; i += 3) {
			outSizeLength = 0.5 * copyDeviceParas[i / 3].size.x + copyDeviceParas[i / 3].spaceLength;
			outSizeWidth = 0.5 * copyDeviceParas[i / 3].size.y + copyDeviceParas[i / 3].spaceLength;
			double LowX = particle.position_[i] - outSizeLength;
			double HighX = particle.position_[i] + outSizeLength;
			double LowY = particle.position_[i + 1] - outSizeWidth;
			double HighY = particle.position_[i + 1] + outSizeWidth;

			verticalAxisList.push_back(LowY);
			verticalAxisList.push_back(HighY);
			horizonAxisList.push_back(LowX);
			horizonAxisList.push_back(HighX);
			//防止路径进入设备内部
			horizonAxisList.push_back(particle.position_[i]);
			verticalAxisList.push_back(particle.position_[i + 1]);

			//每个设备的四个范围
			DeviceLowXList[i / 3] = LowX;
			DeviceHighXList[i / 3] = HighX;
			DeviceLowYList[i / 3] = LowY;
			DeviceHighYList[i / 3] = HighY;
		}
		//对这些点的坐标按照从小到大排序
		sort(verticalAxisList.begin(), verticalAxisList.end());
		sort(horizonAxisList.begin(), horizonAxisList.end());

		//存下所有的障碍点的下标
		vector<int> barrierRowIndexs;
		vector<int> barrierColIndexs;
		//用这些坐标去组成路径点map
		for (int i = 0; i < verticalAxisList.size(); i++)
		{
			vector<APoint*> tmp;
			for (int j = 0; j < horizonAxisList.size(); j++)
			{
				APoint* p = new APoint();
				p->x = horizonAxisList[j];
				p->y = verticalAxisList[i];
				for (int k = 0; k < proParas.DeviceSum; k++)
				{
					//if ((p->x > DeviceLowXList[k] && p->x < DeviceHighXList[k])
					//	&& (p->y > DeviceLowYList[k] && p->y < DeviceHighYList[k]))
					if (p->x - DeviceLowXList[k] >= 0.0001 && DeviceHighXList[k] - p->x >= 0.0001
						&& p->y - DeviceLowYList[k] >= 0.0001 && DeviceHighYList[k] - p->y >= 0.0001)
					{
						p->type = AType::ATYPE_BARRIER;
						//障碍点在图中的下标
						barrierRowIndexs.push_back(i);
						barrierColIndexs.push_back(j);
					}
				}
				p->rowIndex = i;
				p->colIndex = j;
				tmp.push_back(p);
			}
			pathPointMap.push_back(tmp);
		}

#pragma endregion

#pragma region 寻路

		auto star = new CAstar();
		star->_allPoints = pathPointMap;
		int beginRowIndex, beginColIndex, endRowIndex, endColIndex;


		double totalTime = 0.0;
		vector<PointLink>().swap(particle.pointLinks);

		for (int i = 0; i < proParas.CargoTypeNum; i++)
		{
			CargoType curCargoType = proParas.cargoTypeList[i];

			for (int j = 0; j < proParas.cargoTypeList[i].deviceSum; j++)//遍历物料经过的设备列表
			{
				int forwardDeviceIndex, curDeviceIndex;
				int forwardOutIndex, curInIndex;
				double device1PosX, device1PosY, device2PosX, device2PosY;
				double initDevice1PosX, initDevice1PosY, initDevice2PosX, initDevice2PosY;
				//距离
				double deviceDistance = 0.0;
				if (j == 0)//入口
				{
					//第一个设备的距离应该是和入口的距离
					//入口+一个设备的入口点
					curDeviceIndex = proParas.cargoTypeList[i].deviceList[j] - 1;
					curInIndex = getRandomInPoint(copyDeviceParas[curDeviceIndex]);
					forwardOutIndex = 0;
					forwardDeviceIndex = -1;
					device1PosX = proParas.entrancePos.x;
					device1PosY = proParas.entrancePos.y;
					device2PosX = copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.x + particle.position_[curDeviceIndex * 3];
					device2PosY = copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.y + particle.position_[curDeviceIndex * 3 + 1];

					initDevice1PosX = device1PosX;
					initDevice1PosY = device1PosY;
					initDevice2PosX = device2PosX;
					initDevice2PosY = device2PosY;
					//得到设备周围的点
					switch (copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].direct)
					{
					case PointDirect::Up:
						device2PosY += copyDeviceParas[curDeviceIndex].spaceLength;
						break;
					case PointDirect::Down:
						device2PosY -= copyDeviceParas[curDeviceIndex].spaceLength;
						break;
					case PointDirect::Left:
						device2PosX -= copyDeviceParas[curDeviceIndex].spaceLength;
						break;
					case PointDirect::Right:
						device2PosX += copyDeviceParas[curDeviceIndex].spaceLength;
						break;
					}
				}
				else
				{
					forwardDeviceIndex = proParas.cargoTypeList[i].deviceList[j - 1] - 1;
					curDeviceIndex = proParas.cargoTypeList[i].deviceList[j] - 1;
					forwardOutIndex = getRandomOutPoint(copyDeviceParas[forwardDeviceIndex]);
					curInIndex = getRandomInPoint(copyDeviceParas[curDeviceIndex]);

					device1PosX = copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].pos.x + particle.position_[forwardDeviceIndex * 3];
					device1PosY = copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].pos.y + particle.position_[forwardDeviceIndex * 3 + 1];
					device2PosX = copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.x + particle.position_[curDeviceIndex * 3];
					device2PosY = copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.y + particle.position_[curDeviceIndex * 3 + 1];

					initDevice1PosX = device1PosX;
					initDevice1PosY = device1PosY;
					initDevice2PosX = device2PosX;
					initDevice2PosY = device2PosY;
					//得到设备周围的点
					switch (copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].direct)
					{
					case PointDirect::Up:
						device1PosY += copyDeviceParas[forwardDeviceIndex].spaceLength;
						break;
					case PointDirect::Down:
						device1PosY -= copyDeviceParas[forwardDeviceIndex].spaceLength;
						break;
					case PointDirect::Left:
						device1PosX -= copyDeviceParas[forwardDeviceIndex].spaceLength;
						break;
					case PointDirect::Right:
						device1PosX += copyDeviceParas[forwardDeviceIndex].spaceLength;
						break;
					}
					switch (copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].direct)
					{
					case PointDirect::Up:
						device2PosY += copyDeviceParas[curDeviceIndex].spaceLength;
						break;
					case PointDirect::Down:
						device2PosY -= copyDeviceParas[curDeviceIndex].spaceLength;
						break;
					case PointDirect::Left:
						device2PosX -= copyDeviceParas[curDeviceIndex].spaceLength;
						break;
					case PointDirect::Right:
						device2PosX += copyDeviceParas[curDeviceIndex].spaceLength;
						break;
					}
				}

				//计算最短路径
				beginRowIndex = FindAxisIndex(device1PosY, verticalAxisList);
				beginColIndex = FindAxisIndex(device1PosX, horizonAxisList);
				endRowIndex = FindAxisIndex(device2PosY, verticalAxisList);
				endColIndex = FindAxisIndex(device2PosX, horizonAxisList);
				//得到路径，path是第一个节点
				APoint* path = star->findWay(beginRowIndex, beginColIndex, endRowIndex, endColIndex);
				if (path == nullptr)
				{
					cout << endl;
				}
				//根据路径计算长度
				deviceDistance = star->CalcuPathLength(path);
				//路径保存下来
				vector<Vector2> points;
				Vector2 endP(initDevice2PosX, initDevice2PosY);
				points.push_back(endP);
				while (path)
				{
					Vector2 tempP(path->x, path->y);
					points.push_back(tempP);
					path = path->parent;
				}
				Vector2 startP(initDevice1PosX, initDevice1PosY);
				points.push_back(startP);

				PointLink pointLink(forwardDeviceIndex, forwardOutIndex, curDeviceIndex, curInIndex, points);
				particle.pointLinks.push_back(pointLink);


				//计算输送时间(物料总量 * 路线长度 * 输送效率)
				totalTime += curCargoType.totalVolume * deviceDistance * proParas.conveySpeed;
				//计算设备处理时间(物料总量 * 处理效率)
				//totalTime += curCargoType.totalVolume * curDevice.workSpeed;

				star->resetAStar();
				//给障碍点重新标记
				for (int i = 0; i < barrierRowIndexs.size(); i++)
				{
					star->_allPoints[barrierRowIndexs[i]][barrierColIndexs[i]]->type = AType::ATYPE_BARRIER;
				}
			}
		}
#pragma endregion

		//设置适应度值
		particle.fitness_[0] = totalTime;
		particle.fitness_[1] = CalcuTotalArea(particle, proParas);

#pragma region 析构
		delete[] copyDeviceParas;
		delete[] DeviceLowXList;
		delete[] DeviceHighXList;
		delete[] DeviceLowYList;
		delete[] DeviceHighYList;
		vector<int>().swap(barrierRowIndexs);
		vector<int>().swap(barrierColIndexs);
		vector< vector<APoint*> >().swap(pathPointMap);//这样释放内存是不够的
		for (int i = 0; i < pathPointMap.size(); i++)
		{
			for (int j = 0; j < pathPointMap[i].size(); j++)
			{
				delete pathPointMap[i][j];
			}
		}

		vector<double>().swap(horizonAxisList);
		vector<double>().swap(verticalAxisList);
#pragma endregion

	}
#pragma endregion

	return;
}
//顺时针旋转后的坐标
Vector2 Rotate(Vector2 pointPos, Vector2 centerPos, float rotateAngle)
{
	float xx = (pointPos.x - centerPos.x) * cos(rotateAngle * (PI / 180)) + (pointPos.y - centerPos.y) * sin(rotateAngle * (PI / 180)) + centerPos.x;
	float yy = -(pointPos.x - centerPos.x) * sin(rotateAngle * (PI / 180)) + (pointPos.y - centerPos.y) * cos(rotateAngle * (PI / 180)) + centerPos.y;
	Vector2 result(xx, yy);
	return result;
}
int FindAxisIndex(double axis, const vector<double>& axisList)
{
	//用二分法更快
	int low = 0;
	int high = axisList.size() - 1;
	int result = 0;
	while (low <= high)
	{
		int middle = (low + high) >> 1;
		if (abs(axisList[middle] - axis) <= 0.0001)
		{
			result = middle;
			break;
		}
		else if (axisList[middle] > axis)
		{
			high = middle - 1;
		}
		else if (axisList[middle] < axis)
		{
			low = middle + 1;
		}
	}
	return result;
}
int getRandomInPoint(DevicePara& device)
{
	if (device.usableAdjPointsIn.size() == 0) {
		return -1;
	}
	int randomIndex = rand() % device.usableAdjPointsIn.size();//产生随机小数
	int resultIndex = device.usableAdjPointsIn[randomIndex].index;
	device.usableAdjPointsIn.erase(device.usableAdjPointsIn.begin() + randomIndex);
	return resultIndex;
}
int getRandomOutPoint(DevicePara& device)
{
	if (device.usableAdjPointsOut.size() == 0) {
		return -1;
	}
	int randomIndex = rand() % device.usableAdjPointsOut.size();//产生随机小数
	int resultIndex = device.usableAdjPointsOut[randomIndex].index;
	device.usableAdjPointsOut.erase(device.usableAdjPointsOut.begin() + randomIndex);
	return resultIndex;
}
//计算曼哈段距离
double CalcuDeviceDist(Vector2 pos1, Vector2 pos2)
{
	return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y);
}
//计算占地面积
double CalcuTotalArea(Particle& particle, ProblemParas proParas) {
	double area = 0;
	double min_X, min_Y, max_X, max_Y;
	int min_X_index, min_Y_index, max_X_index, max_Y_index;
	min_X = min_Y = DOUBLE_MAX;
	max_X = max_Y = DOUBLE_MIN;
	min_X_index = min_Y_index = max_X_index = max_Y_index = 0;
	for (int i = 0; i < particle.dim_; i += 2) {
		if (particle.position_[i] - proParas.deviceParaList[i / 2].size.x * 0.5 < min_X) {
			min_X = particle.position_[i] - proParas.deviceParaList[i / 2].size.x * 0.5;
			min_X_index = i / 2;
		}
		if (particle.position_[i + 1] - proParas.deviceParaList[i / 2].size.y * 0.5 < min_Y) {
			min_Y = particle.position_[i + 1] - proParas.deviceParaList[i / 2].size.y * 0.5;
			min_Y_index = i / 2;
		}
		if (particle.position_[i] + proParas.deviceParaList[i / 2].size.x * 0.5 > max_X) {
			max_X = particle.position_[i] + proParas.deviceParaList[i / 2].size.x * 0.5;
			max_X_index = i / 2;
		}
		if (particle.position_[i + 1] + proParas.deviceParaList[i / 2].size.y * 0.5 > max_Y) {
			max_Y = particle.position_[i + 1] + proParas.deviceParaList[i / 2].size.y * 0.5;
			max_Y_index = i / 2;
		}
	}
	//计算总面积
	area = (max_X - min_X) * (max_Y - min_Y);
	return area;
}
