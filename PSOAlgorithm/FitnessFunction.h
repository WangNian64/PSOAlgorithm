#pragma once
#include "PSO.h"
#include "Tools.h"
#include "AStar.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <map>
//Up = 1, Right = 2, Down = 3, Left = 4
//一共10种情况：上下，左右，下下，上上，左左，右右，
//上左，上右，下左，下右
//横竖的顺序都是上下左右
int pointDirectArray[5][5] = {
		  {-1, -1, -1, -1, -1},
		  //上 右 下 左
		  /*上*/{-1, 1, 2, 3, 4},
		  /*右*/{-1, 5, 6, 7, 8},
		  /*下*/{-1, 9, 10, 11, 12},
		  /*左*/{-1, 13, 14, 15, 16},
};
#pragma region 判断两个坐标的上下或者左右关系
bool IsInLeft2Out(Vector2 inPos, Vector2 outPos)
{
	return inPos.x <= outPos.x;
}
bool IsInRight2Out(Vector2 inPos, Vector2 outPos)
{
	return inPos.x >= outPos.x;
}
bool IsInUp2Out(Vector2 inPos, Vector2 outPos)
{
	return inPos.y >= outPos.y;
}
bool IsInDown2Out(Vector2 inPos, Vector2 outPos)
{
	return inPos.y <= outPos.y;
}

#pragma endregion
void FitnessFunction(int curIterNum, int maxIterNum, vector<BestPathInfo>& bestPathInfoList, ProblemParas proParas, Particle& particle);
double CalcuTotalArea(Particle& particle, ProblemParas proParas);
double CalcuDeviceDist(Vector2 pos1, Vector2 pos2);

int FindAxisIndex(double axis, const vector<double>& axisList);

//顺时针旋转后的坐标
Vector2 Rotate(Vector2 pointPos, Vector2 centerPos, float rotateAngle);
//对一个数字*10000然后四舍五入到int
int Multi10000ToInt(double num);

Vector2Int Multi10000ToInt(Vector2 v);
//默认的适应度计算函数，可以替换
void FitnessFunction(int curIterNum, int maxIterNum, vector<BestPathInfo>& bestPathInfoList, ProblemParas proParas, Particle& particle)
{
	double punishValue1 = 0;
	double punishValue2 = 0;
	bool IsDeviceOverlap = false;//是否重叠
	double deviceDist = 0;
	particle.fitness_[0] = particle.fitness_[1] = 0;

	#pragma region 深拷贝一份设备参数
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
			swap(copyDeviceParas[i / 3].size.x, copyDeviceParas[i / 3].size.y);
		}
		//重新计算旋转后的出入口坐标
		Vector2 deviceCenterPos(0, 0);
		double rotateAngle = curDirect * 90;
		int newDirect = 0;
		for (AdjPoint& point : copyDeviceParas[i / 3].adjPointsIn)
		{
			point.pos = Rotate(point.pos, deviceCenterPos, rotateAngle);
			newDirect = point.direct + (int)curDirect;
			point.direct = (newDirect == 4) ? (PointDirect)4 : (PointDirect)(newDirect % 4);
		}
		for (AdjPoint& point : copyDeviceParas[i / 3].adjPointsOut)
		{
			point.pos = Rotate(point.pos, deviceCenterPos, rotateAngle);
			newDirect = point.direct + (int)curDirect;
			point.direct = (newDirect == 4) ? (PointDirect)4 : (PointDirect)(newDirect % 4);
		}
	}
	#pragma endregion

	#pragma region 检查设备是否重叠
	//如果重叠，进行调整
	//降低标准会发生什么？
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
				IsDeviceOverlap = true;
				//cout << curIterNum << ":" << "重叠" << endl;
				return;
			}
		}
	}
	#pragma endregion

	#pragma region 如果设备重叠，调整设备位置，否则直接布线
	//if (IsDeviceOverlap == true) {
	//	//步骤：
	//	//只要有一个设备有重叠，就要处理
	//	while ()
	//}
	//在检测一次设备位置，看是否重叠

	if (IsDeviceOverlap == false)
	{
		#pragma region 对齐方案1：对齐设备中心点x和y
		////if (curIterNum == 199)
		////{
		//for (int i = 0; i < proParas.DeviceSum; i++)
		//{
		//	for (int j = 0; j < proParas.DeviceSum; j++)
		//	{
		//		if (i != j)
		//		{
		//			if (abs(particle.position_[3 * i] - particle.position_[3 * j]) <= 1)
		//			{
		//				particle.position_[3 * i] = particle.position_[3 * j]
		//					= (particle.position_[3 * i] + particle.position_[3 * j]) * 0.5f;
		//			}
		//			if (abs(particle.position_[3 * i + 1] - particle.position_[3 * j + 1]) <= 1)
		//			{
		//				particle.position_[3 * i + 1] = particle.position_[3 * j + 1]
		//					= (particle.position_[3 * i + 1] + particle.position_[3 * j + 1]) * 0.5f;
		//			}
		//		}
		//	}
		//}
		////对齐入口
		//for (int i = 0; i < proParas.DeviceSum; i++)
		//{
		//	if (abs(particle.position_[3 * i] - proParas.entrancePos.x) <= 1)
		//	{
		//		particle.position_[3 * i] = proParas.entrancePos.x;
		//	}
		//	if (abs(particle.position_[3 * i + 1] - proParas.entrancePos.y) <= 1)
		//	{
		//		particle.position_[3 * i + 1] = proParas.entrancePos.y;
		//	}
		//}
		////}
		#pragma endregion

		#pragma region 对齐方案2：检测设备出入口点坐标并进行对齐操作
		//遍历所有cargoTypeList
		for (int j = 0; j < proParas.CargoTypeNum; j++)
		{
			for (int k = 0; k < proParas.cargoTypeList[j].linkSum; k++)
			{
				int outDeviceIndex = proParas.cargoTypeList[j].deviceLinkList[k].outDeviceIndex;
				int inDeviceIndex = proParas.cargoTypeList[j].deviceLinkList[k].inDeviceIndex;
				int outPointIndex = proParas.cargoTypeList[j].deviceLinkList[k].outPointIndex;
				int inPointIndex = proParas.cargoTypeList[j].deviceLinkList[k].inPointIndex;
				AdjPoint outPoint, inPoint;
				//特殊情况1：仓库入口
				if (outDeviceIndex == -1)
				{
					inPoint = copyDeviceParas[inDeviceIndex].adjPointsIn[inPointIndex];
					Vector2 inPointTPos(inPoint.pos.x + particle.position_[inDeviceIndex * 3],
						inPoint.pos.y + particle.position_[inDeviceIndex * 3 + 1]);
					if (inPointTPos.x != proParas.entrancePos.x && abs(inPointTPos.x - proParas.entrancePos.x) < proParas.conveyMinDist)
					{
						//只能修改in，不能修改入口
						double moveLength = inPointTPos.x - proParas.entrancePos.x;
						particle.position_[inDeviceIndex * 3] -= moveLength;

					}
					else if (inPointTPos.y != proParas.entrancePos.y && abs(inPointTPos.y - proParas.entrancePos.y) < proParas.conveyMinDist)
					{
						double moveLength = inPointTPos.y - proParas.entrancePos.y;
						particle.position_[inDeviceIndex * 3 + 1] -= moveLength;
					}
				}
				else if (inDeviceIndex == -2)//特殊情况2：仓库出口
				{
					outPoint = copyDeviceParas[outDeviceIndex].adjPointsOut[outPointIndex];
					Vector2 outPointTPos(outPoint.pos.x + particle.position_[outDeviceIndex * 3],
						outPoint.pos.y + particle.position_[outDeviceIndex * 3 + 1]);
					if (outPointTPos.x != proParas.exitPos.x && abs(outPointTPos.x - proParas.exitPos.x) < proParas.conveyMinDist)
					{
						//只能修改out，不能修改出口
						double moveLength = outPointTPos.x - proParas.exitPos.x;
						particle.position_[outDeviceIndex * 3] -= moveLength;
					}
					else if (outPointTPos.y != proParas.exitPos.y && abs(outPointTPos.y - proParas.exitPos.y) < proParas.conveyMinDist)
					{
						double moveLength = outPointTPos.y - proParas.exitPos.y;
						particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
					}
				}
				else//其他情况
				{
					outPoint = copyDeviceParas[outDeviceIndex].adjPointsOut[outPointIndex];
					inPoint = copyDeviceParas[inDeviceIndex].adjPointsIn[inPointIndex];
					//在考虑了设备坐标的情况下对比
					Vector2 outPointTPos(outPoint.pos.x + particle.position_[outDeviceIndex * 3],
						outPoint.pos.y + particle.position_[outDeviceIndex * 3 + 1]);
					Vector2 inPointTPos(inPoint.pos.x + particle.position_[inDeviceIndex * 3],
						inPoint.pos.y + particle.position_[inDeviceIndex * 3 + 1]);
					if (outPointTPos.x != inPointTPos.x && abs(outPointTPos.x - inPointTPos.x) < proParas.conveyMinDist)
					{
						//x坐标接近
						double moveLength = (outPointTPos.x - inPointTPos.x) * 0.5;
						particle.position_[outDeviceIndex * 3] -= moveLength;
						particle.position_[inDeviceIndex * 3] += moveLength;

					}
					else if (outPointTPos.y != inPointTPos.y && abs(outPointTPos.y - inPointTPos.y) < proParas.conveyMinDist)
					{
						//y坐标接近
						double moveLength = (outPointTPos.y - inPointTPos.y) * 0.5;
						particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
						particle.position_[inDeviceIndex * 3 + 1] += moveLength;
					}
				}
			}
		}
		#pragma endregion

		#pragma region 对齐方案3：根据配对的出入口点的朝向分情况优化
		//遍历所有cargoTypeList
		//if 有一个是出口为i设备，且不是最后一个，那么就可以拿出这一对出入口点
		for (int j = 0; j < proParas.CargoTypeNum; j++)
		{
			for (int k = 0; k < proParas.cargoTypeList[j].linkSum; k++)
			{
				int outDeviceIndex = proParas.cargoTypeList[j].deviceLinkList[k].outDeviceIndex;
				int inDeviceIndex = proParas.cargoTypeList[j].deviceLinkList[k].inDeviceIndex;
				int outPointIndex = proParas.cargoTypeList[j].deviceLinkList[k].outPointIndex;
				int inPointIndex = proParas.cargoTypeList[j].deviceLinkList[k].inPointIndex;
				AdjPoint outPoint, inPoint;
				PointDirect outPointDirect, inPointDirect;
				//特殊情况1：仓库入口
				if (outDeviceIndex == -1)
				{
					inPoint = copyDeviceParas[inDeviceIndex].adjPointsIn[inPointIndex];
					Vector2 inPointTPos(inPoint.pos.x + particle.position_[inDeviceIndex * 3],
						inPoint.pos.y + particle.position_[inDeviceIndex * 3 + 1]);
					if (inPointTPos.x != proParas.entrancePos.x && abs(inPointTPos.x - proParas.entrancePos.x) < proParas.conveyMinDist)
					{
						//只能修改in，不能修改入口
						double moveLength = inPointTPos.x - proParas.entrancePos.x;
						particle.position_[inDeviceIndex * 3] -= moveLength;

					}
					else if (inPointTPos.y != proParas.entrancePos.y && abs(inPointTPos.y - proParas.entrancePos.y) < proParas.conveyMinDist)
					{
						double moveLength = inPointTPos.y - proParas.entrancePos.y;
						particle.position_[inDeviceIndex * 3 + 1] -= moveLength;
					}
				}
				else if (inDeviceIndex == -2)//特殊情况2：仓库出口
				{
					outPoint = copyDeviceParas[outDeviceIndex].adjPointsOut[outPointIndex];
					Vector2 outPointTPos(outPoint.pos.x + particle.position_[outDeviceIndex * 3],
						outPoint.pos.y + particle.position_[outDeviceIndex * 3 + 1]);
					if (outPointTPos.x != proParas.exitPos.x && abs(outPointTPos.x - proParas.exitPos.x) < proParas.conveyMinDist)
					{
						//只能修改out，不能修改出口
						double moveLength = outPointTPos.x - proParas.exitPos.x;
						particle.position_[outDeviceIndex * 3] -= moveLength;
					}
					else if (outPointTPos.y != proParas.exitPos.y && abs(outPointTPos.y - proParas.exitPos.y) < proParas.conveyMinDist)
					{
						double moveLength = outPointTPos.y - proParas.exitPos.y;
						particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
					}
				}
				else//其他情况
				{
					outPoint = copyDeviceParas[outDeviceIndex].adjPointsOut[outPointIndex];
					inPoint = copyDeviceParas[inDeviceIndex].adjPointsIn[inPointIndex];
					//考虑点的朝向
					outPointDirect = outPoint.direct;
					inPointDirect = inPoint.direct;
					//出入口的真实坐标
					Vector2 outPointTPos(outPoint.pos.x + particle.position_[outDeviceIndex * 3],
						outPoint.pos.y + particle.position_[outDeviceIndex * 3 + 1]);
					Vector2 inPointTPos(inPoint.pos.x + particle.position_[inDeviceIndex * 3],
						inPoint.pos.y + particle.position_[inDeviceIndex * 3 + 1]);
					//出入口设备四个方向不考虑外层的边界
					Vector2 outDeviceUpPos(particle.position_[outDeviceIndex * 3], particle.position_[outDeviceIndex * 3 + 1] + 0.5 * copyDeviceParas[outDeviceIndex].size.y);
					Vector2 outDeviceDownPos(particle.position_[outDeviceIndex * 3], particle.position_[outDeviceIndex * 3 + 1] - 0.5 * copyDeviceParas[outDeviceIndex].size.y);
					Vector2 outDeviceLeftPos(particle.position_[outDeviceIndex * 3] - 0.5 * copyDeviceParas[outDeviceIndex].size.x, particle.position_[outDeviceIndex * 3 + 1]);
					Vector2 outDeviceRightPos(particle.position_[outDeviceIndex * 3] + 0.5 * copyDeviceParas[outDeviceIndex].size.x, particle.position_[outDeviceIndex * 3 + 1]);

					Vector2 inDeviceUpPos(particle.position_[inDeviceIndex * 3], particle.position_[inDeviceIndex * 3 + 1] + 0.5 * copyDeviceParas[inDeviceIndex].size.y);
					Vector2 inDeviceDownPos(particle.position_[inDeviceIndex * 3], particle.position_[inDeviceIndex * 3 + 1] - 0.5 * copyDeviceParas[inDeviceIndex].size.y);
					Vector2 inDeviceLeftPos(particle.position_[inDeviceIndex * 3] - 0.5 * copyDeviceParas[inDeviceIndex].size.x, particle.position_[inDeviceIndex * 3 + 1]);
					Vector2 inDeviceRightPos(particle.position_[inDeviceIndex * 3] + 0.5 * copyDeviceParas[inDeviceIndex].size.x, particle.position_[inDeviceIndex * 3 + 1]);


					//出入口设备四个方向的边界
					Vector2 outUpPos(outDeviceUpPos.x, outDeviceUpPos.y + proParas.convey2DeviceDist);
					Vector2 outDownPos(outDeviceDownPos.x, outDeviceDownPos.y - proParas.convey2DeviceDist);
					Vector2 outLeftPos(outDeviceLeftPos.x - proParas.convey2DeviceDist, outDeviceLeftPos.y);
					Vector2 outRightPos(outDeviceRightPos.x + proParas.convey2DeviceDist, outDeviceRightPos.y);

					Vector2 inUpPos(inDeviceUpPos.x, inDeviceUpPos.y + proParas.convey2DeviceDist);
					Vector2 inDownPos(inDeviceDownPos.x, inDeviceDownPos.y - proParas.convey2DeviceDist);
					Vector2 inLeftPos(inDeviceLeftPos.x - proParas.convey2DeviceDist, inDeviceLeftPos.x);
					Vector2 inRightPos(outDeviceRightPos.x + proParas.convey2DeviceDist, outDeviceRightPos.y);


					//要用到的各种比较值
					double inR2outXDist, out2inLXDist, in2outLXDist, outR2inXDist;
					double outU2inDYDist, inR2outLXDist, outR2inLXDist;
					double outL2inLXDist, inU2outUYDist, inU2outDYDist, outU2inYDist;
					double out2inDYDist, inU2outYDist, in2outDYDist;
					//先判断两者的方位
					//加一个判断二者方位的操作
					Vector2 inDevicePos(particle.position_[inDeviceIndex * 3], particle.position_[inDeviceIndex * 3 + 1]);
					Vector2 outDevicePos(particle.position_[outDeviceIndex * 3], particle.position_[outDeviceIndex * 3 + 1]);
					bool inLeft2OutBool, inRight2OutBool, inUp2OutBool, inDown2OutBool;

					int OutPosIndex_X, OutPosIndex_Y, InPosIndex_X, InPosIndex_Y;
					OutPosIndex_X = outDeviceIndex * 3; OutPosIndex_Y = outDeviceIndex * 3 + 1;
					InPosIndex_X = inDeviceIndex * 3; InPosIndex_Y = inDeviceIndex * 3 + 1;
					double moveLength = 0.0;
					//用表驱动法，一共16种
					//Up = 1, Right = 2, Down = 3, Left = 4
					//      	   //上 右 下 左
					//  /*上*/{-1, 1, 2, 3, 4},
					//	/*右*/{-1, 5, 6, 7, 8},
					//	/*下*/{-1, 9, 10, 11, 12},
					//	/*左*/{-1, 13, 14, 15, 16},
					switch (pointDirectArray[outPointDirect][inPointDirect])
					{
					case 1://上上
					{
						//1.两者在y上过于接近,让两者在y上对齐
						if (outPointTPos.y != inPointTPos.y && abs(outPointTPos.y - inPointTPos.y) < proParas.conveyMinDist)
						{
							moveLength = (outPointTPos.y - inPointTPos.y) * 0.5;
							particle.position_[OutPosIndex_Y] -= moveLength;
							particle.position_[InPosIndex_Y] += moveLength;
							break;
						}
						//2.入口设备在出口设备左/右上角
						inUp2OutBool = IsInUp2Out(inDownPos, outUpPos);
						if (inUp2OutBool)
						{
							inR2outXDist = inRightPos.x - outPointTPos.x;
							if (inR2outXDist > 0 && inR2outXDist < proParas.conveyMinDist)
							{
								moveLength = inR2outXDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
							out2inLXDist = outPointTPos.x - inLeftPos.x;
							if (out2inLXDist > 0 && out2inLXDist < proParas.conveyMinDist)
							{
								moveLength = out2inLXDist * 0.5;
								particle.position_[OutPosIndex_X] -= moveLength;
								particle.position_[InPosIndex_X] += moveLength;
								break;
							}
						}
						//3.入口设备在出口设备左/右下角
						inDown2OutBool = IsInDown2Out(inUpPos, outDownPos);
						if (inDown2OutBool)
						{
							in2outLXDist = inPointTPos.x - outLeftPos.x;
							if (in2outLXDist > 0 && in2outLXDist < proParas.conveyMinDist)
							{
								moveLength = in2outLXDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
							outR2inXDist = outRightPos.x - inPointTPos.x;
							if (outR2inXDist > 0 && outR2inXDist < proParas.conveyMinDist)
							{
								moveLength = outR2inXDist * 0.5;
								particle.position_[OutPosIndex_X] -= moveLength;
								particle.position_[InPosIndex_X] += moveLength;
								break;
							}
						}
						break;
					}
					case 11://下下
					{
						//1.两者在y上过于接近,让两者在y上对齐
						if (outPointTPos.y != inPointTPos.y && abs(outPointTPos.y - inPointTPos.y) < proParas.conveyMinDist)//这个待定
						{
							moveLength = (outPointTPos.y - inPointTPos.y) * 0.5;
							particle.position_[OutPosIndex_Y] -= moveLength;
							particle.position_[InPosIndex_Y] += moveLength;
							break;
						}
						inUp2OutBool = IsInUp2Out(inDownPos, outUpPos);
						//2.入口设备在出口设备左/右上角
						if (inUp2OutBool)
						{
							in2outLXDist = inPointTPos.x - outLeftPos.x;
							if (in2outLXDist > 0 && in2outLXDist < proParas.conveyMinDist)
							{
								moveLength = in2outLXDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
							outR2inXDist = outRightPos.x - inPointTPos.x;
							if (outR2inXDist > 0 && outR2inXDist < proParas.conveyMinDist)
							{
								moveLength = outR2inXDist * 0.5;
								particle.position_[OutPosIndex_X] -= moveLength;
								particle.position_[InPosIndex_X] += moveLength;
								break;
							}
						}
						//3.入口设备在出口设备左/右下角
						inDown2OutBool = IsInDown2Out(inUpPos, outDownPos);
						if (inDown2OutBool)
						{
							inR2outXDist = inRightPos.x - outPointTPos.x;
							if (inR2outXDist > 0 && inR2outXDist < proParas.conveyMinDist)
							{
								moveLength = inR2outXDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
							out2inLXDist = outPointTPos.x - inLeftPos.x;
							if (out2inLXDist > 0 && out2inLXDist < proParas.conveyMinDist)
							{
								moveLength = out2inLXDist * 0.5;
								particle.position_[OutPosIndex_X] -= moveLength;
								particle.position_[InPosIndex_X] += moveLength;
								break;
							}
						}
						break;
					}
					case 16://左左
					{
						//1.两者在x上过于接近,让两者在x上对齐
						if (outPointTPos.x != inPointTPos.x && abs(outPointTPos.x - inPointTPos.x) < proParas.conveyMinDist)
						{
							moveLength = (outPointTPos.x - inPointTPos.x) * 0.5;
							particle.position_[OutPosIndex_X] -= moveLength;
							particle.position_[InPosIndex_X] += moveLength;
							break;
						}
						//2.in在out左边
						inLeft2OutBool = IsInLeft2Out(inRightPos, outLeftPos);
						if (inLeft2OutBool)
						{
							//in在out左上
							out2inDYDist = outPointTPos.y - inDownPos.y;
							if (out2inDYDist > 0 && out2inDYDist < proParas.conveyMinDist)
							{
								moveLength = out2inDYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
							//in在out左下
							inU2outYDist = inUpPos.y - outPointTPos.y;
							if (inU2outYDist > 0 && inU2outYDist < proParas.conveyMinDist)
							{
								moveLength = inU2outYDist * 0.5;
								particle.position_[OutPosIndex_Y] += moveLength;
								particle.position_[InPosIndex_Y] -= moveLength;
								break;
							}
						}
						//3.in在out右边
						inRight2OutBool = IsInRight2Out(inLeftPos, outRightPos);
						if (inRight2OutBool)
						{
							//in在out右上
							outU2inYDist = outUpPos.y - inPointTPos.y;
							if (outU2inYDist > 0 && outU2inYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
							//in在out右下
							in2outDYDist = inPointTPos.y - outDownPos.y;
							if (in2outDYDist > 0 && in2outDYDist < proParas.conveyMinDist)
							{
								moveLength = in2outDYDist * 0.5;
								particle.position_[OutPosIndex_Y] += moveLength;
								particle.position_[InPosIndex_Y] -= moveLength;
								break;
							}
						}
						break;
					}
					case 6://右右
					{
						//1.两者在x上过于接近,让两者在x上对齐
						if (outPointTPos.x != inPointTPos.x && abs(outPointTPos.x - inPointTPos.x) < proParas.conveyMinDist)
						{
							moveLength = (outPointTPos.x - inPointTPos.x) * 0.5;
							particle.position_[OutPosIndex_X] -= moveLength;
							particle.position_[InPosIndex_X] += moveLength;
							break;
						}
						//2.in在out右边
						inRight2OutBool = IsInRight2Out(inLeftPos, outRightPos);
						if (inRight2OutBool)
						{
							//in在out右上
							out2inDYDist = outPointTPos.y - inDownPos.y;
							if (out2inDYDist > 0 && out2inDYDist < proParas.conveyMinDist)
							{
								moveLength = out2inDYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
							//in在out右下
							inU2outYDist = inUpPos.y - outPointTPos.y;
							if (inU2outYDist > 0 && inU2outYDist < proParas.conveyMinDist)
							{
								moveLength = inU2outYDist * 0.5;
								particle.position_[OutPosIndex_Y] += moveLength;
								particle.position_[InPosIndex_Y] -= moveLength;
								break;
							}
						}
						//3.in在out左边
						inLeft2OutBool = IsInLeft2Out(inRightPos, outLeftPos);
						if (inLeft2OutBool)
						{
							//in在out左上
							outU2inYDist = outUpPos.y - inPointTPos.y;
							if (outU2inYDist > 0 && outU2inYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
							//in在out左下
							in2outDYDist = inPointTPos.y - outDownPos.y;
							if (in2outDYDist > 0 && in2outDYDist < proParas.conveyMinDist)
							{
								moveLength = in2outDYDist * 0.5;
								particle.position_[OutPosIndex_Y] += moveLength;
								particle.position_[InPosIndex_Y] -= moveLength;
								break;
							}
						}
						break;
					}
					case 3://上下
					{
						//1.in在out上方，且在x上很接近
						inUp2OutBool = IsInUp2Out(inDownPos, outUpPos);
						if (inUp2OutBool)
						{
							if (outPointTPos.x != inPointTPos.x && abs(outPointTPos.x - inPointTPos.x) < proParas.conveyMinDist)
							{
								moveLength = (outPointTPos.x - inPointTPos.x) * 0.5;
								particle.position_[OutPosIndex_X] -= moveLength;
								particle.position_[InPosIndex_X] += moveLength;
								break;
							}
						}
						//2.入口设备在出口设备左/右上角
						inUp2OutBool = IsInUp2Out(inDeviceDownPos, outDeviceUpPos);
						if (inUp2OutBool)//左右可以用一种方式计算
						{
							outU2inDYDist = outUpPos.y - inDownPos.y;
							if (outU2inDYDist > 0 && outU2inDYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inDYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
						}
						//入口设备在出口设备下面
						inDown2OutBool = IsInDown2Out(inUpPos, outDownPos);
						if (inDown2OutBool)
						{
							//3.入口设备在出口设备下面/分为左右
							inR2outLXDist = inRightPos.x - outLeftPos.x;
							if (inR2outLXDist > 0 && inR2outLXDist < proParas.conveyMinDist)
							{
								moveLength = inR2outLXDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
							outR2inLXDist = outRightPos.x - inLeftPos.x;
							if (outR2inLXDist > 0 && outR2inLXDist < proParas.conveyMinDist)
							{
								moveLength = outR2inLXDist * 0.5;
								particle.position_[OutPosIndex_X] -= moveLength;
								particle.position_[InPosIndex_X] += moveLength;
								break;
							}
						}
						break;
					}
					case 14://左右
					{
						//1.in在out左边，且在y上很接近
						inLeft2OutBool = IsInLeft2Out(inRightPos, outLeftPos);
						if (inLeft2OutBool)
						{
							if (outPointTPos.y != inPointTPos.y && abs(outPointTPos.y - inPointTPos.y) < proParas.conveyMinDist)
							{
								moveLength = (outPointTPos.y - inPointTPos.y) * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
						}
						//2.in在out左边
						inLeft2OutBool = IsInLeft2Out(inDeviceRightPos, outDeviceLeftPos);
						if (inLeft2OutBool)
						{
							inR2outLXDist = inRightPos.x - outLeftPos.x;
							if (inR2outLXDist > 0 && inR2outLXDist < proParas.conveyMinDist)
							{
								moveLength = inR2outLXDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
						}
						//3.in在out右边，分为右上和右下
						inRight2OutBool = IsInRight2Out(inLeftPos, outRightPos);
						if (inRight2OutBool)
						{
							//in在out右下
							inU2outDYDist = inUpPos.y - outDownPos.y;
							if (inU2outDYDist > 0 && inU2outDYDist < proParas.conveyMinDist)
							{
								moveLength = inU2outDYDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
							//in在out右上
							outU2inDYDist = outUpPos.y - inDownPos.y;
							if (outU2inDYDist > 0 && outU2inDYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inDYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
						}
						break;
					}
					case 4://上左(先不考虑可能的情况)
					{
						inLeft2OutBool = IsInLeft2Out(inRightPos, outLeftPos);
						if (inLeft2OutBool)
						{
							//1.in在out左上
							outU2inDYDist = outUpPos.y - inDownPos.y;
							if (outU2inDYDist > 0 && outU2inDYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inDYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
						}
						inUp2OutBool = IsInUp2Out(inDownPos, outUpPos);
						if (inUp2OutBool)
						{
							//2.in在out右上
							out2inLXDist = outPointTPos.x - inLeftPos.x;
							if (out2inLXDist > 0 && out2inLXDist < proParas.conveyMinDist)
							{
								moveLength = out2inLXDist * 0.5;
								particle.position_[OutPosIndex_X] -= moveLength;
								particle.position_[InPosIndex_X] += moveLength;
								break;
							}
						}
						inRight2OutBool = IsInRight2Out(inLeftPos, outRightPos);
						if (inRight2OutBool)
						{
							//3.in在out右上一点
							outU2inYDist = outUpPos.y - inPointTPos.y;
							if (outU2inYDist > 0 && outU2inYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
						}
						inDown2OutBool = IsInDown2Out(inUpPos, outDownPos);
						if (inDown2OutBool)
						{
							//4.in在out右下角
							outR2inLXDist = outRightPos.x - inLeftPos.x;
							if (outR2inLXDist > 0 && outR2inLXDist < proParas.conveyMinDist)
							{
								moveLength = outR2inLXDist * 0.5;
								particle.position_[OutPosIndex_X] -= moveLength;
								particle.position_[InPosIndex_X] += moveLength;
								break;
							}
						}
						break;
					}
					case 2://上右
					{
						//in在out上
						inUp2OutBool = IsInUp2Out(inDownPos, outUpPos);
						if (inUp2OutBool)
						{
							//1.in在out左上
							inR2outXDist = inRightPos.x - outPointTPos.x;
							if (inR2outXDist > 0 && inR2outXDist < proParas.conveyMinDist)
							{
								moveLength = inR2outXDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
						}
						//in在out右
						inRight2OutBool = IsInRight2Out(inLeftPos, outRightPos);//这个计算也可以优化，提前算好
						if (inRight2OutBool)
						{
							//2.in在out右上
							outU2inDYDist = outUpPos.y - inDownPos.y;
							if (outU2inDYDist > 0 && outU2inDYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inDYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
						}
						//in在out左
						inLeft2OutBool = IsInLeft2Out(inRightPos, outLeftPos);
						if (inLeft2OutBool)
						{
							//3.in在out左上
							outU2inYDist = outUpPos.y - inPointTPos.y;
							if (outU2inYDist > 0 && outU2inYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
						}
						//in在out下
						inDown2OutBool = IsInDown2Out(inUpPos, outDownPos);
						if (inDown2OutBool)
						{
							//3.in在out左下
							inR2outLXDist = inRightPos.x - outLeftPos.x;
							if (inR2outLXDist > 0 && inR2outLXDist < proParas.conveyMinDist)
							{
								moveLength = inR2outLXDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
						}
						break;
					}
					default:
						break;
					}
				}
			}
		}
		#pragma endregion

		#pragma region 计算出入口点的集合坐标
		vector<InoutPoint> tempInoutPoints;
		for (int i = 0; i < proParas.DeviceSum; i++)
		{
			for (AdjPoint point : copyDeviceParas[i].adjPointsIn)
			{
				InoutPoint ioPoint;
				ioPoint.pointDirect = point.direct;
				Vector2 axis(point.pos.x + particle.position_[3 * i], point.pos.y + particle.position_[3 * i + 1]);
				ioPoint.pointAxis = axis;
				tempInoutPoints.push_back(ioPoint);
			}
			for (AdjPoint point : copyDeviceParas[i].adjPointsOut)
			{
				InoutPoint ioPoint;
				ioPoint.pointDirect = point.direct;
				Vector2 axis(point.pos.x + particle.position_[3 * i], point.pos.y + particle.position_[3 * i + 1]);
				ioPoint.pointAxis = axis;
				tempInoutPoints.push_back(ioPoint);
			}
		}

		#pragma endregion

		#pragma region 根据设备坐标和出入口坐标构造路径点图
		vector< vector<APoint*> > pathPointMap;
		vector<double> horizonAxisList;
		vector<double> verticalAxisList;
		//先对出入口点水平和垂直进行分类(注意加上偏移量)
		for (int i = 0; i < proParas.DeviceSum; i++)
		{
			for (AdjPoint p : copyDeviceParas[i].adjPointsIn)
			{
				if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//上下
				{
					horizonAxisList.push_back(p.pos.x + particle.position_[i * 3]);
				}
				else {//左右
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
		//仓库入口2个点&出口
		horizonAxisList.push_back(proParas.entrancePos.x);
		verticalAxisList.push_back(proParas.entrancePos.y);

		horizonAxisList.push_back(proParas.exitPos.x);
		verticalAxisList.push_back(proParas.exitPos.y);
		//存下每个设备坐标的四个范围（作为后面障碍点的范围）
		double* DeviceLowXList = new double[proParas.DeviceSum];
		double* DeviceHighXList = new double[proParas.DeviceSum];
		double* DeviceLowYList = new double[proParas.DeviceSum];
		double* DeviceHighYList = new double[proParas.DeviceSum];
		//每个设备周围的4个点
		for (int i = 0; i < particle.dim_; i += 3) {
			//outSizeLength = 0.5 * copyDeviceParas[i / 3].size.x + copyDeviceParas[i / 3].spaceLength;
			outSizeLength = 0.5 * copyDeviceParas[i / 3].size.x + proParas.convey2DeviceDist;
			//outSizeWidth = 0.5 * copyDeviceParas[i / 3].size.y + copyDeviceParas[i / 3].spaceLength;
			outSizeWidth = 0.5 * copyDeviceParas[i / 3].size.y + proParas.convey2DeviceDist;
			double LowX = particle.position_[i] - outSizeLength;
			double HighX = particle.position_[i] + outSizeLength;
			double LowY = particle.position_[i + 1] - outSizeWidth;
			double HighY = particle.position_[i + 1] + outSizeWidth;

			verticalAxisList.push_back(LowY);
			verticalAxisList.push_back(HighY);
			horizonAxisList.push_back(LowX);
			horizonAxisList.push_back(HighX);

			//每个设备的四个范围
			DeviceLowXList[i / 3] = LowX;
			DeviceHighXList[i / 3] = HighX;
			DeviceLowYList[i / 3] = LowY;
			DeviceHighYList[i / 3] = HighY;

			//防止路径进入设备内部
			horizonAxisList.push_back(particle.position_[i]);
			verticalAxisList.push_back(particle.position_[i + 1]);

		}
		//对这些点的坐标按照从小到大排序
		sort(verticalAxisList.begin(), verticalAxisList.end());
		sort(horizonAxisList.begin(), horizonAxisList.end());
		//只保留不重复的点
		auto unique_end1 = unique(verticalAxisList.begin(), verticalAxisList.end());
		verticalAxisList.erase(unique_end1, verticalAxisList.end());
		auto unique_end2 = unique(horizonAxisList.begin(), horizonAxisList.end());
		horizonAxisList.erase(unique_end2, horizonAxisList.end());

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
					if (p->x - DeviceLowXList[k] >= 0.01 && DeviceHighXList[k] - p->x >= 0.01
						&& p->y - DeviceLowYList[k] >= 0.01 && DeviceHighYList[k] - p->y >= 0.01)
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
		//vector<PointLink>().swap(particle.pointLinks);

		vector<PointLink> copyPLinks;
		for (int i = 0; i < proParas.CargoTypeNum; i++)
		{
			CargoType curCargoType = proParas.cargoTypeList[i];

			for (int j = 0; j < proParas.cargoTypeList[i].linkSum; j++)//遍历物料经过的设备列表
			{
				PathDirection pathBeginDirect;
				PathDirection pathEndDirect;
				int forwardDeviceIndex, curDeviceIndex;//设备1和设备2ID
				int forwardOutIndex, curInIndex;//出入口的下标
				double device1PosX, device1PosY, device2PosX, device2PosY;//设备周围的四个点
				double initDevice1PosX, initDevice1PosY, initDevice2PosX, initDevice2PosY;//保存未增加包围边的坐标

				double deviceDistance = 0.0;//距离

				forwardDeviceIndex = proParas.cargoTypeList[i].deviceLinkList[j].outDeviceIndex;
				curDeviceIndex = proParas.cargoTypeList[i].deviceLinkList[j].inDeviceIndex;

				forwardOutIndex = proParas.cargoTypeList[i].deviceLinkList[j].outPointIndex;
				curInIndex = proParas.cargoTypeList[i].deviceLinkList[j].inPointIndex;
				if (forwardDeviceIndex == -1)//说明是仓库入口
				{
					/*forwardOutIndex = 0;
					curInIndex = proParas.cargoTypeList[i].deviceLinkList[j].inPointIndex;*/

					//开头和结尾点的朝向方向
					pathBeginDirect = PathDirection::Vertical;
					pathEndDirect = (copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].direct % 2 == 0)
						? PathDirection::Horizon : PathDirection::Vertical;

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
						//device2PosY += copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosY += proParas.convey2DeviceDist;
						break;
					case PointDirect::Down:
						//device2PosY -= copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosY -= proParas.convey2DeviceDist;
						break;
					case PointDirect::Left:
						//device2PosX -= copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosX -= proParas.convey2DeviceDist;
						break;
					case PointDirect::Right:
						//device2PosX += copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosX += proParas.convey2DeviceDist;
						break;
					}
				}
				else if (curDeviceIndex == -2)//说明是出口
				{
					//forwardOutIndex = proParas.cargoTypeList[i].deviceLinkList[j].outPointIndex;
					//curInIndex = 0;

					//开头和结尾点的朝向方向
					pathBeginDirect = (copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].direct % 2 == 0)
						? PathDirection::Horizon : PathDirection::Vertical;
					pathEndDirect = PathDirection::Vertical;

					device1PosX = copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].pos.x + particle.position_[forwardDeviceIndex * 3];
					device1PosY = copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].pos.y + particle.position_[forwardDeviceIndex * 3 + 1];
					device2PosX = proParas.exitPos.x;
					device2PosY = proParas.exitPos.y;

					initDevice1PosX = device1PosX;
					initDevice1PosY = device1PosY;
					initDevice2PosX = device2PosX;
					initDevice2PosY = device2PosY;
					//得到设备周围的点
					switch (copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].direct)
					{
					case PointDirect::Up:
						//device1PosY += copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosY += proParas.convey2DeviceDist;
						break;
					case PointDirect::Down:
						//device1PosY -= copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosY -= proParas.convey2DeviceDist;
						break;
					case PointDirect::Left:
						//device1PosX -= copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosX -= proParas.convey2DeviceDist;
						break;
					case PointDirect::Right:
						//device1PosX += copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosX += proParas.convey2DeviceDist;
						break;
					}
				}
				else//普通
				{
					//forwardOutIndex = proParas.cargoTypeList[i].deviceLinkList[j].outPointIndex - 1;
					//curInIndex = proParas.cargoTypeList[i].deviceLinkList[j].inPointIndex - 1;

					//开头和结尾点的朝向方向
					pathBeginDirect = (copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].direct % 2 == 0)
						? PathDirection::Horizon : PathDirection::Vertical;
					pathEndDirect = (copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].direct % 2 == 0)
						? PathDirection::Horizon : PathDirection::Vertical;

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
						//device1PosY += copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosY += proParas.convey2DeviceDist;
						break;
					case PointDirect::Down:
						//device1PosY -= copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosY -= proParas.convey2DeviceDist;
						break;
					case PointDirect::Left:
						//device1PosX -= copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosX -= proParas.convey2DeviceDist;
						break;
					case PointDirect::Right:
						//device1PosX += copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosX += proParas.convey2DeviceDist;
						break;
					}
					switch (copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].direct)
					{
					case PointDirect::Up:
						//device2PosY += copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosY += proParas.convey2DeviceDist;
						break;
					case PointDirect::Down:
						//device2PosY -= copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosY -= proParas.convey2DeviceDist;
						break;
					case PointDirect::Left:
						//device2PosX -= copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosX -= proParas.convey2DeviceDist;
						break;
					case PointDirect::Right:
						//device2PosX += copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosX += proParas.convey2DeviceDist;
						break;
					}
				}
				//计算最短路径
				beginRowIndex = FindAxisIndex(device1PosY, verticalAxisList);
				beginColIndex = FindAxisIndex(device1PosX, horizonAxisList);
				endRowIndex = FindAxisIndex(device2PosY, verticalAxisList);
				endColIndex = FindAxisIndex(device2PosX, horizonAxisList);

				//得到路径，path是第一个节点
				APoint* path = star->findWay(pathBeginDirect, beginRowIndex, beginColIndex, endRowIndex, endColIndex);
				//不可行的解，直接退出
				if (path == nullptr)
				{
					particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
					return;
				}
				//根据路径计算长度
				deviceDistance = star->CalcuPathLength(path);


				#pragma region 计算路径（只带上起始点 + 路径中的转弯点）
				//路径保存下来
				vector<Vector2> points1;

				Vector2 endP1(initDevice2PosX, initDevice2PosY);
				points1.push_back(endP1);
				APoint* copyPath = path;
				while (copyPath)
				{
					Vector2 tempP(copyPath->x, copyPath->y);
					points1.push_back(tempP);
					copyPath = copyPath->parent;
				}
				Vector2 startP1(initDevice1PosX, initDevice1PosY);
				points1.push_back(startP1);

				PointLink pointLink1(forwardDeviceIndex, forwardOutIndex, curDeviceIndex, curInIndex, points1);
				copyPLinks.push_back(pointLink1);


				//保存路径（只保存每段的起始点）
				//if (curIterNum != 0) 
				//vector<Vector2> points;
				//Vector2 endP(initDevice2PosX, initDevice2PosY);//从终点开始计算
				//points.push_back(endP);

				//PathDirection pathCurDirect = pathEndDirect;//因为这个路径是反向的

				//Vector2 lastP(path->x, path->y);//从最后一个点开始
				//points.push_back(lastP);
				//path = path->parent;
				//while (path)
				//{
				//	Vector2 curP(path->x, path->y);
				//	//只有转弯的点才会被加入路径
				//	if (pathCurDirect == PathDirection::Horizon && curP.x == lastP.x)
				//	{
				//		//if (curIterNum != 0)
				//			//if ((lastP.x != points.back().x || lastP.y != points.back().y)
				//			//	&& CalcuDeviceDist(lastP, points.back()) < proParas.conveyMinDist)
				//			//{
				//			//	//punishValue1 = 200 * (curIterNum + 1);
				//			//	//punishValue2 = 40 * (curIterNum + 1);
				//			//	particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
				//			//	return;
				//			//}
				//		points.push_back(lastP);
				//		pathCurDirect = PathDirection::Vertical;
				//	}
				//	else if (pathCurDirect == PathDirection::Vertical && curP.y == lastP.y)
				//	{
				//		//if (curIterNum != 0) {
				//			//if ((lastP.x != points.back().x || lastP.y != points.back().y)
				//			//	&& CalcuDeviceDist(lastP, points.back()) < proParas.conveyMinDist)
				//			//{
				//			//	//punishValue1 = 200 * (curIterNum + 1);
				//			//	//punishValue2 = 40 * (curIterNum + 1);
				//			//	particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
				//			//	return;
				//			//}
				//		points.push_back(lastP);
				//		pathCurDirect = PathDirection::Horizon;
				//	}
				//	path = path->parent;
				//	lastP = curP;
				//}
				////if (curIterNum != 0)
				//	//if (CalcuDeviceDist(lastP, points.back()) < proParas.conveyMinDist)
				//	//{
				//	//	//punishValue1 = 200 * (curIterNum + 1);
				//	//	//punishValue2 = 40 * (curIterNum + 1);
				//	//	particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
				//	//	return;
				//	//}
				//points.push_back(lastP);
				//Vector2 startP(initDevice1PosX, initDevice1PosY);
				//points.push_back(startP);

				//PointLink pointLink(forwardDeviceIndex, forwardOutIndex, curDeviceIndex, curInIndex, points);
				//particle.pointLinks.push_back(pointLink);
				#pragma endregion



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

		#pragma region 将布局结果转化为输送机参数(同时加强一下输送线最短距离约束)

		//particle.strConveyorList.clear();
		//particle.curveConveyorList.clear();
		//particle.pathPointInfoMap.clear();
		set<StraightConveyorInfo> tempStrConveyorList;//直线输送机信息列表
		set<Vector2Int> tempCurveConveyorList;//转弯输送机信息列表

		set<SegPath> segPathSet;
		for (PointLink pl : copyPLinks)//过滤所有重复的路线段
		{
			for (int i = pl.points.size() - 1; i > 0; i--)
			{
				Vector2Int p1 = Multi10000ToInt(pl.points[i]);
				Vector2Int p2 = Multi10000ToInt(pl.points[i - 1]);
				if (p1 != p2)//坐标不能一样
				{
					SegPath temp(p1, p2);
					segPathSet.insert(temp);
				}
			}
		}
		map<Vector2Int, PointInfo> pathPointInfoMap;//路径点信息map
		//遍历set，计算出所有set中 点的出入度&点所在路线的垂直水平数目&是否保留 信息
		for (SegPath sp : segPathSet)
		{
			//在map中找sp的p1
			if (pathPointInfoMap.count(sp.p1)) {
				//找到，更新信息
				Vector2Int startP = sp.p1;
				if (sp.direct == PathPointDirect::Vert) {
					++pathPointInfoMap[startP].vertDirNum;
				}
				else {
					++pathPointInfoMap[startP].horiDirNum;
				}
			}
			else {
				if (sp.direct == PathPointDirect::Vert) {
					pathPointInfoMap.insert({ sp.p1, PointInfo(1, 0, false) });
				}
				else {
					pathPointInfoMap.insert({ sp.p1, PointInfo(0, 1, false) });
				}
			}

			//在map中找sp的p2
			if (pathPointInfoMap.count(sp.p2)) {
				//找到，更新信息
				Vector2Int endP = sp.p2;
				if (sp.direct == PathPointDirect::Vert) {
					++pathPointInfoMap[endP].vertDirNum;
				}
				else {
					++pathPointInfoMap[endP].horiDirNum;
				}
			}
			else {
				if (sp.direct == PathPointDirect::Vert) {
					pathPointInfoMap.insert({ sp.p2, PointInfo(1, 0, false) });
				}
				else {
					pathPointInfoMap.insert({ sp.p2, PointInfo(0, 1, false) });
				}
			}
		}

		for (auto it = pathPointInfoMap.begin(); it != pathPointInfoMap.end(); it++)
		{
			if ((it->second.horiDirNum == 1 && it->second.vertDirNum == 0)
				|| (it->second.horiDirNum == 0 && it->second.vertDirNum == 1)) {
				it->second.isKeep = true;
			}
			if (it->second.horiDirNum >= 1 && it->second.vertDirNum >= 1) {
				it->second.isKeep = true;
			}
		}
		for (PointLink pl : copyPLinks) {
			StraightConveyorInfo tempStrInfo;
			tempStrInfo.startPos = Multi10000ToInt(pl.points[pl.points.size() - 1]);//开头
			tempStrInfo.startVnum = pathPointInfoMap[tempStrInfo.startPos].vertDirNum;
			tempStrInfo.startHnum = pathPointInfoMap[tempStrInfo.startPos].horiDirNum;
			for (int i = pl.points.size() - 2; i >= 0; i--) {
				Vector2Int p = Multi10000ToInt(pl.points[i]);
				if (pathPointInfoMap[p].isKeep == true) {//这里会出现重复的
					//先更新直线输送机
					if (tempStrInfo.startPos != p) {
						tempStrInfo.endPos = p;
						//if (curIterNum != 0)
						if (tempStrInfo.startPos.Distance(tempStrInfo.endPos) < proParas.conveyMinDist)
						{
							//cout << "输送线太短" << endl;
							//punishValue1 = 150 * (curIterNum + 1);
							//punishValue2 = 30 * (curIterNum + 1);
							particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
							return;
						}
						tempStrInfo.endVnum = pathPointInfoMap[tempStrInfo.endPos].vertDirNum;
						tempStrInfo.endHnum = pathPointInfoMap[tempStrInfo.endPos].horiDirNum;
						tempStrConveyorList.insert(tempStrInfo);
						tempStrInfo.startPos = p;
						tempStrInfo.startVnum = pathPointInfoMap[tempStrInfo.startPos].vertDirNum;
						tempStrInfo.startHnum = pathPointInfoMap[tempStrInfo.startPos].horiDirNum;
					}
					//只要不是始终点，都需要更新转弯输送机
					if (!(pathPointInfoMap[p].horiDirNum == 1 && pathPointInfoMap[p].vertDirNum == 0)
						&& !(pathPointInfoMap[p].horiDirNum == 0 && pathPointInfoMap[p].vertDirNum == 1)) {
						tempCurveConveyorList.insert(p);
					}
				}
			}
		}
		#pragma endregion

		#pragma region 计算目标函数值
		//遍历直线和转弯输送机的set，得到输送机的总成本
		double conveyorTotalCost = 0.0;
		for (StraightConveyorInfo sci : tempStrConveyorList)
		{
			conveyorTotalCost += proParas.strConveyorUnitCost * sci.startPos.Distance(sci.endPos);
		}
		conveyorTotalCost += proParas.curveConveyorUnitCost * tempCurveConveyorList.size();
		//设置适应度值
		cout << totalTime << "," << conveyorTotalCost << endl;

		//particle.fitness_[0] = 1000;
		particle.fitness_[0] = totalTime;
		//particle.fitness_[0] = totalTime + punishValue1;

		//particle.fitness_[1] = CalcuTotalArea(particle, proParas);
		//particle.fitness_[1] = 1000;
		particle.fitness_[1] = conveyorTotalCost;
		//particle.fitness_[1] = conveyorTotalCost + punishValue2;

		//根据适应度是否升级选择更新BestPathInfoList
		for (int i = 0; i < bestPathInfoList.size(); ++i) {
			if (particle.fitness_[i] < bestPathInfoList[i].curBestFitnessVal) {//需要更新
				bestPathInfoList[i].inoutPoints = tempInoutPoints;
				bestPathInfoList[i].strConveyorList = tempStrConveyorList;
				bestPathInfoList[i].curveConveyorList = tempCurveConveyorList;
				bestPathInfoList[i].curBestFitnessVal = particle.fitness_[i];
			}
		}
		#pragma endregion

		delete[] DeviceLowXList;
		delete[] DeviceHighXList;
		delete[] DeviceLowYList;
		delete[] DeviceHighYList;
	}
	delete[] copyDeviceParas;
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
	min_X = min_Y = INT_MAX;
	max_X = max_Y = INT_MAX;
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
//先乘以10000，然后四舍五入到Int
int Multi10000ToInt(double num)
{
	return round(num * 10000);
}
Vector2Int Multi10000ToInt(Vector2 v)
{
	return Vector2Int(Multi10000ToInt(v.x), Multi10000ToInt(v.y));
}