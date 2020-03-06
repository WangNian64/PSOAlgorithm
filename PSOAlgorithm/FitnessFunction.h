#pragma once
#include "PSO.h"
#include "Tools.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#define PI 3.1415926
#define DOUBLE_MAX 1.7976931348623158e+308
#define DOUBLE_MIN 2.2250738585072014e-308
#define MAX_FITNESS 1000000.0
void FitnessFunction(Particle & particle, ProblemParas proParas);
double CalcuTotalArea(Particle & particle, ProblemParas proParas);
double CalcuDeviceDist(Vector2 pos1, Vector2 pos2);

int getRandomInPoint(DevicePara& device);
int getRandomOutPoint(DevicePara& device);
//默认的适应度计算函数，可以替换
void FitnessFunction(Particle & particle, ProblemParas proParas)
{
	bool IsWorkable = true;
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
	//如果重叠，加入惩罚
	for (int i = 0; i < particle.dim_; i += 2) {
		double firstLowX = particle.position_[i] - 0.5 * proParas.deviceParaList[i / 2].size.x;
		double firstUpX = particle.position_[i] + 0.5 * proParas.deviceParaList[i / 2].size.x;
		double firstLowY = particle.position_[i + 1] - 0.5 * proParas.deviceParaList[i / 2].size.y;
		double firstUpY = particle.position_[i + 1] + 0.5 * proParas.deviceParaList[i / 2].size.y;
		for (int j = i + 2; j < particle.dim_; j += 2) {
			double secondLowX = particle.position_[j] - 0.5 * proParas.deviceParaList[j / 2].size.x;
			double secondUpX = particle.position_[j] + 0.5 * proParas.deviceParaList[j / 2].size.x;
			double secondLowY = particle.position_[j + 1] - 0.5 * proParas.deviceParaList[j / 2].size.y;
			double secondUpY = particle.position_[j + 1] + 0.5 * proParas.deviceParaList[j / 2].size.y;
			if (IsRangeOverlap(firstLowX, firstUpX, secondLowX, secondUpX) && IsRangeOverlap(firstLowY, firstUpY, secondLowY, secondUpY)) {
				particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
				IsWorkable = false;
				return;
			}
		}
	}

	//只要当前布局是可行解，就进行布线
	if (IsWorkable == true)
	{
		DevicePara* copyDeviceParas = new DevicePara[proParas.DeviceSum];
		for (int i = 0; i < proParas.DeviceSum; i++)
		{
			copyDeviceParas[i].ID = proParas.deviceParaList[i].ID;
			copyDeviceParas[i].direct = proParas.deviceParaList[i].direct;
			copyDeviceParas[i].axis = proParas.deviceParaList[i].axis;
			copyDeviceParas[i].size = proParas.deviceParaList[i].size;
			copyDeviceParas[i].workSpeed = proParas.deviceParaList[i].workSpeed;
			copyDeviceParas[i].adjPointsIn = proParas.deviceParaList[i].adjPointsIn;
			copyDeviceParas[i].adjPointsOut = proParas.deviceParaList[i].adjPointsOut;
			copyDeviceParas[i].usableAdjPointsIn = proParas.deviceParaList[i].usableAdjPointsIn;
			copyDeviceParas[i].usableAdjPointsOut = proParas.deviceParaList[i].usableAdjPointsOut;
		}

		//for (int i = 0; i < proParas.DeviceSum; i++)
		//{
		//	cout << copyDeviceParas[i].usableAdjPointsIn.size() << " "
		//		<< copyDeviceParas[i].usableAdjPointsOut.size() << endl;
		//}
		double totalTime = 0.0;
		particle.pointLinks = new PointLink[100];
		particle.pointLinkSum = 0;
		for (int i = 0; i < proParas.CargoTypeNum; i++)
		{
			CargoType curCargoType = proParas.cargoTypeList[i];

			for (int j = 0; j < proParas.cargoTypeList[i].deviceSum; j++)//遍历物料经过的设备列表
			{
				int forwardDeviceIndex, curDeviceIndex;
				int forwardOutIndex, curInIndex;
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
					Vector2 device1Pos(proParas.entrancePos.x, proParas.entrancePos.y);
					Vector2 device2Pos(copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.x + particle.position_[curDeviceIndex * 2],
						copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.y + particle.position_[curDeviceIndex * 2 + 1]);
					deviceDistance = CalcuDeviceDist(device1Pos, device2Pos);
					//更新已连接的点数组
					PointLink pointLink(forwardDeviceIndex, forwardOutIndex, curDeviceIndex, curInIndex);
					particle.pointLinks[particle.pointLinkSum++] = pointLink;
				}
				else 
				{
					forwardDeviceIndex = proParas.cargoTypeList[i].deviceList[j - 1] - 1;
					curDeviceIndex = proParas.cargoTypeList[i].deviceList[j] - 1;
					forwardOutIndex = getRandomOutPoint(copyDeviceParas[forwardDeviceIndex]);
					curInIndex = getRandomInPoint(copyDeviceParas[curDeviceIndex]);

					Vector2 device1Pos(copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].pos.x + particle.position_[forwardDeviceIndex * 2],
						copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].pos.y + particle.position_[forwardDeviceIndex * 2 + 1]);
					Vector2 device2Pos(copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.x + particle.position_[curDeviceIndex * 2],
						copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.y + particle.position_[curDeviceIndex * 2 + 1]);
					deviceDistance = CalcuDeviceDist(device1Pos, device2Pos);

					PointLink pointLink(forwardDeviceIndex, forwardOutIndex, curDeviceIndex, curInIndex);
					particle.pointLinks[particle.pointLinkSum++] = pointLink;
				}
				//计算输送时间(物料总量 * 路线长度 * 输送效率)
				totalTime += curCargoType.totalVolume * deviceDistance * proParas.conveySpeed;
				
				//计算设备处理时间(物料总量 * 处理效率)
				//totalTime += curCargoType.totalVolume * curDevice.workSpeed;
			}
		}
		particle.fitness_[0] = totalTime;
		//面积也作为适应度值
		particle.fitness_[1] = CalcuTotalArea(particle, proParas);

		//for (int i = 0; i < proParas.DeviceSum; i++)
		//{
		//	cout << copyDeviceParas[i].usableAdjPointsIn.size() << " "
		//		<< copyDeviceParas[i].usableAdjPointsOut.size() << endl;
		//}
		//cout << endl;
	}
	
	return;
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
double CalcuTotalArea(Particle & particle, ProblemParas proParas) {
	double area = 0;
	double min_X, min_Y, max_X, max_Y;
	int min_X_index, min_Y_index, max_X_index, max_Y_index;
	min_X = min_Y = DOUBLE_MAX;
	max_X = max_Y = DOUBLE_MIN;
	min_X_index = min_Y_index = max_X_index = max_Y_index = 0;
	for (int i = 0; i < particle.dim_; i+=2) {
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
