#pragma once
#include "PSO.h"
#include "Tools.h"
#include "AStar.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <map>
#define PI 3.14159265358979
#define DOUBLE_MAX 1.7976931348623158e+308
#define DOUBLE_MIN 2.2250738585072014e-308

#define MAX_FITNESS 10000000.0
//Up = 1, Right = 2, Down = 3, Left = 4
//һ��10����������£����ң����£����ϣ��������ң�
//�������ң���������
//������˳������������
int pointDirectArray[5][5] = {
	      {-1, -1, -1, -1, -1},
		       //�� �� �� ��
	/*��*/{-1, 1, 2, 3, 4},
	/*��*/{-1, 5, 6, 7, 8},
	/*��*/{-1, 9, 10, 11, 12},
	/*��*/{-1, 13, 14, 15, 16},
};
//�ж�������������»������ҹ�ϵ
bool IsInLeft2Out(Vector2 inPos, Vector2 outPos)
{
	return inPos.x <= outPos.x;
}
bool IsInUp2Out(Vector2 inPos, Vector2 outPos)
{
	return inPos.y >= outPos.y;
}

void FitnessFunction(int curIterNum, int maxIterNum, Particle& particle, ProblemParas proParas, double* lowerBounds, double* upBounds);
double CalcuTotalArea(Particle& particle, ProblemParas proParas);
double CalcuDeviceDist(Vector2 pos1, Vector2 pos2);

int FindAxisIndex(double axis, const vector<double>& axisList);

//˳ʱ����ת�������
Vector2 Rotate(Vector2 pointPos, Vector2 centerPos, float rotateAngle);
//��һ������*10000Ȼ���������뵽int
int Multi10000ToInt(double num);

Vector2Int Multi10000ToInt(Vector2 v);
//Ĭ�ϵ���Ӧ�ȼ��㺯���������滻
void FitnessFunction(int curIterNum, int maxIterNum, Particle& particle, ProblemParas proParas, double* lowerBounds, double* upBounds)
{
	bool IsWorkable = true;//���Ƿ����
	double deviceDist = 0;
	particle.fitness_[0] = particle.fitness_[1] = 0;

	#pragma region ���һ���豸����
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

	#pragma region �����豸���򣬵����豸�ߴ�xy�ͳ��������
	for (int i = 2; i < particle.dim_; i += 3)
	{
		//doubleתint��ת��ΪDirection��Ȼ����ݳ������¼����豸�ߴ�ͳ����
		//Rotate90����ROtate270���ߴ��x��y����
		//����ڰ���˳ʱ���㣬��ת��=Direction*90(���ö�Ӧ0,90,180,270��
		DeviceDirect curDirect = (DeviceDirect)(int)particle.position_[i];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)
		{
			swap(copyDeviceParas[i / 3].size.x, copyDeviceParas[i / 3].size.y);
		}
		//���¼�����ת��ĳ��������
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

	#pragma region ����豸�Ƿ��ص�
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
				//cout << "�ص�" << endl;
				return;
			}
		}
	}
	#pragma endregion

	#pragma region ��ǰ�����ǿ��н⣬���в���

	if (IsWorkable == true)
	{
		#pragma region ���뷽��1�������豸���ĵ�x��y
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
		////�������
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

		#pragma region ���뷽��2������豸����ڵ����겢���ж������
		//��������cargoTypeList
		//if ��һ���ǳ���Ϊi�豸���Ҳ������һ������ô�Ϳ����ó���һ�Գ���ڵ�
		for (int j = 0; j < proParas.CargoTypeNum; j++)
		{
			for (int k = 0; k < proParas.cargoTypeList[j].linkSum; k++)
			{
				int outDeviceIndex = proParas.cargoTypeList[j].deviceLinkList[k].outDeviceIndex;
				int inDeviceIndex = proParas.cargoTypeList[j].deviceLinkList[k].inDeviceIndex;
				int outPointIndex = proParas.cargoTypeList[j].deviceLinkList[k].outPointIndex;
				int inPointIndex = proParas.cargoTypeList[j].deviceLinkList[k].inPointIndex;
				AdjPoint outPoint, inPoint;
				//�������1���ֿ����
				if (outDeviceIndex == -1)
				{
					inPoint = copyDeviceParas[inDeviceIndex].adjPointsIn[inPointIndex];
					Vector2 inPointTPos(inPoint.pos.x + particle.position_[inDeviceIndex * 3],
						inPoint.pos.y + particle.position_[inDeviceIndex * 3 + 1]);
					if (inPointTPos.x != proParas.entrancePos.x && abs(inPointTPos.x - proParas.entrancePos.x) < proParas.conveyMinDist)
					{
						//ֻ���޸�in�������޸����
						double moveLength = inPointTPos.x - proParas.entrancePos.x;
						particle.position_[inDeviceIndex * 3] -= moveLength;

					}
					else if (inPointTPos.y != proParas.entrancePos.y && abs(inPointTPos.y - proParas.entrancePos.y) < proParas.conveyMinDist)
					{
						double moveLength = inPointTPos.y - proParas.entrancePos.y;
						particle.position_[inDeviceIndex * 3 + 1] -= moveLength;
					}
				}
				else if (inDeviceIndex == -2)//�������2���ֿ����
				{
					outPoint = copyDeviceParas[outDeviceIndex].adjPointsOut[outPointIndex];
					Vector2 outPointTPos(outPoint.pos.x + particle.position_[outDeviceIndex * 3],
						outPoint.pos.y + particle.position_[outDeviceIndex * 3 + 1]);
					if (outPointTPos.x != proParas.exitPos.x && abs(outPointTPos.x - proParas.exitPos.x) < proParas.conveyMinDist)
					{
						//ֻ���޸�out�������޸ĳ���
						double moveLength = outPointTPos.x - proParas.exitPos.x;
						particle.position_[outDeviceIndex * 3] -= moveLength;
					}
					else if (outPointTPos.y != proParas.exitPos.y && abs(outPointTPos.y - proParas.exitPos.y) < proParas.conveyMinDist)
					{
						double moveLength = outPointTPos.y - proParas.exitPos.y;
						particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
					}
				}
				else//�������
				{
					outPoint = copyDeviceParas[outDeviceIndex].adjPointsOut[outPointIndex];
					inPoint = copyDeviceParas[inDeviceIndex].adjPointsIn[inPointIndex];
					//�ڿ������豸���������¶Ա�
					Vector2 outPointTPos(outPoint.pos.x + particle.position_[outDeviceIndex * 3],
						outPoint.pos.y + particle.position_[outDeviceIndex * 3 + 1]);
					Vector2 inPointTPos(inPoint.pos.x + particle.position_[inDeviceIndex * 3],
						inPoint.pos.y + particle.position_[inDeviceIndex * 3 + 1]);
					if (outPointTPos.x != inPointTPos.x && abs(outPointTPos.x - inPointTPos.x) < proParas.conveyMinDist)
					{
						//x����ӽ�
						double moveLength = (outPointTPos.x - inPointTPos.x) * 0.5;
						particle.position_[outDeviceIndex * 3] -= moveLength;
						particle.position_[inDeviceIndex * 3] += moveLength;

					}
					else if (outPointTPos.y != inPointTPos.y && abs(outPointTPos.y - inPointTPos.y) < proParas.conveyMinDist)
					{
						//y����ӽ�
						double moveLength = (outPointTPos.y - inPointTPos.y) * 0.5;
						particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
						particle.position_[inDeviceIndex * 3 + 1] += moveLength;
					}
				}
			}
		}
		#pragma endregion

		#pragma region ���뷽��3��������Եĳ���ڵ�ĳ��������Ż�
		//��������cargoTypeList
		//if ��һ���ǳ���Ϊi�豸���Ҳ������һ������ô�Ϳ����ó���һ�Գ���ڵ�
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
				//�������1���ֿ����
				if (outDeviceIndex == -1)
				{
					inPoint = copyDeviceParas[inDeviceIndex].adjPointsIn[inPointIndex];
					Vector2 inPointTPos(inPoint.pos.x + particle.position_[inDeviceIndex * 3],
						inPoint.pos.y + particle.position_[inDeviceIndex * 3 + 1]);
					if (inPointTPos.x != proParas.entrancePos.x && abs(inPointTPos.x - proParas.entrancePos.x) < proParas.conveyMinDist)
					{
						//ֻ���޸�in�������޸����
						double moveLength = inPointTPos.x - proParas.entrancePos.x;
						particle.position_[inDeviceIndex * 3] -= moveLength;

					}
					else if (inPointTPos.y != proParas.entrancePos.y && abs(inPointTPos.y - proParas.entrancePos.y) < proParas.conveyMinDist)
					{
						double moveLength = inPointTPos.y - proParas.entrancePos.y;
						particle.position_[inDeviceIndex * 3 + 1] -= moveLength;
					}
				}
				else if (inDeviceIndex == -2)//�������2���ֿ����
				{
					outPoint = copyDeviceParas[outDeviceIndex].adjPointsOut[outPointIndex];
					Vector2 outPointTPos(outPoint.pos.x + particle.position_[outDeviceIndex * 3],
						outPoint.pos.y + particle.position_[outDeviceIndex * 3 + 1]);
					if (outPointTPos.x != proParas.exitPos.x && abs(outPointTPos.x - proParas.exitPos.x) < proParas.conveyMinDist)
					{
						//ֻ���޸�out�������޸ĳ���
						double moveLength = outPointTPos.x - proParas.exitPos.x;
						particle.position_[outDeviceIndex * 3] -= moveLength;
					}
					else if (outPointTPos.y != proParas.exitPos.y && abs(outPointTPos.y - proParas.exitPos.y) < proParas.conveyMinDist)
					{
						double moveLength = outPointTPos.y - proParas.exitPos.y;
						particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
					}
				}
				else//�������
				{
					outPoint = copyDeviceParas[outDeviceIndex].adjPointsOut[outPointIndex];
					inPoint = copyDeviceParas[inDeviceIndex].adjPointsIn[inPointIndex];
					//���ǵ�ĳ���
					outPointDirect = outPoint.direct;
					inPointDirect = inPoint.direct;
					//����ڵ���ʵ����
					Vector2 outPointTPos(outPoint.pos.x + particle.position_[outDeviceIndex * 3],
						outPoint.pos.y + particle.position_[outDeviceIndex * 3 + 1]);
					Vector2 inPointTPos(inPoint.pos.x + particle.position_[inDeviceIndex * 3],
						inPoint.pos.y + particle.position_[inDeviceIndex * 3 + 1]);
					//������豸�ĸ�����ı߽�
					Vector2 outUpPos(particle.position_[outDeviceIndex * 3], particle.position_[outDeviceIndex * 3 + 1] + 0.5 * copyDeviceParas[outDeviceIndex].size.y + proParas.convey2DeviceDist);
					Vector2 outDownPos(particle.position_[outDeviceIndex * 3], particle.position_[outDeviceIndex * 3 + 1] - 0.5 * copyDeviceParas[outDeviceIndex].size.y - proParas.convey2DeviceDist);
					Vector2 outLeftPos(particle.position_[outDeviceIndex * 3] - 0.5 * copyDeviceParas[outDeviceIndex].size.x - proParas.convey2DeviceDist, particle.position_[outDeviceIndex * 3 + 1]);
					Vector2 outRightPos(particle.position_[outDeviceIndex * 3] + 0.5 * copyDeviceParas[outDeviceIndex].size.x + proParas.convey2DeviceDist, particle.position_[outDeviceIndex * 3 + 1]);

					Vector2 inUpPos(particle.position_[inDeviceIndex * 3], particle.position_[inDeviceIndex * 3 + 1] + 0.5 * copyDeviceParas[inDeviceIndex].size.y + proParas.convey2DeviceDist);
					Vector2 inDownPos(particle.position_[inDeviceIndex * 3], particle.position_[inDeviceIndex * 3 + 1] - 0.5 * copyDeviceParas[inDeviceIndex].size.y - proParas.convey2DeviceDist);
					Vector2 inLeftPos(particle.position_[inDeviceIndex * 3] - 0.5 * copyDeviceParas[inDeviceIndex].size.x - proParas.convey2DeviceDist, particle.position_[inDeviceIndex * 3 + 1]);
					Vector2 inRightPos(particle.position_[inDeviceIndex * 3] + 0.5 * copyDeviceParas[inDeviceIndex].size.x + proParas.convey2DeviceDist, particle.position_[inDeviceIndex * 3 + 1]);

					//Ҫ�õ��ĸ��ֱȽ�ֵ
					double inR2outXDist, out2inLXDist, in2outLXDist, outR2inXDist;
					double outU2InDYDist, inR2outLXDist, outR2inLXDist;
					double outUp2inDYDist, outL2inLXDist, inU2outUYDist, outU2inYDist;
					//���ж����ߵķ�λ
					//��һ���ж϶��߷�λ�Ĳ���
					Vector2 inDevicePos(particle.position_[inDeviceIndex * 3], particle.position_[inDeviceIndex * 3 + 1]);
					Vector2 outDevicePos(particle.position_[outDeviceIndex * 3], particle.position_[outDeviceIndex * 3 + 1]);
					bool inLeft2OutBool = IsInLeft2Out(inDevicePos, outDevicePos);
					bool inUp2OutBool = IsInUp2Out(inDevicePos, outDevicePos);

					//һ��10����������£����ң����£����ϣ��������ң��������ң���������
					//in��outӦ���ǿ��Ի�����
					//�ñ���������һ��16��
					//Up = 1, Right = 2, Down = 3, Left = 4
					//      	   //�� �� �� ��
					//  /*��*/{-1, 1, 2, 3, 4},
					//	/*��*/{ -1, 5, 6, 7, 8 },
					//	/*��*/{ -1, 9, 10, 11, 12 },
					//	/*��*/{ -1, 13, 14, 15, 16 },
					switch (pointDirectArray[outPointDirect][inPointDirect])
					{
					case 1://����
					{
						//1.������y�Ϲ��ڽӽ�,��������y�϶���
						if (outPointTPos.y != inPointTPos.y && abs(outPointTPos.y - inPointTPos.y) < proParas.conveyMinDist)
						{
							double moveLength = (outPointTPos.y - inPointTPos.y) * 0.5;
							particle.position_[outDeviceIndex * 3] -= moveLength;
							particle.position_[inDeviceIndex * 3] += moveLength;
						}
						else
						{
							//2.����豸�ڳ����豸��/���Ͻ�(�ֿ����㣩
							if (inUp2OutBool)
							{
								inR2outXDist = inRightPos.x - outPointTPos.x;
								if (inR2outXDist > 0 && inR2outXDist < proParas.conveyMinDist)
								{
									double moveLength = inR2outXDist * 0.5;
									particle.position_[outDeviceIndex * 3] += moveLength;
									particle.position_[inDeviceIndex * 3] -= moveLength;
								}
								out2inLXDist = outPointTPos.x - inLeftPos.x;
								if (out2inLXDist > 0 && out2inLXDist < proParas.conveyMinDist)
								{
									double moveLength = out2inLXDist * 0.5;
									particle.position_[outDeviceIndex * 3] -= moveLength;
									particle.position_[inDeviceIndex * 3] += moveLength;
								}
							}
							else
							{
								//3.����豸�ڳ����豸�·�(��Ϊ����)�ԳƵ�
								in2outLXDist = inPointTPos.x - outLeftPos.x;
								if (in2outLXDist > 0 && in2outLXDist < proParas.conveyMinDist)
								{
									double moveLength = in2outLXDist * 0.5;
									particle.position_[outDeviceIndex * 3] += moveLength;
									particle.position_[inDeviceIndex * 3] -= moveLength;
								}
								outR2inXDist = outRightPos.x - inPointTPos.x;
								if (outR2inXDist > 0 && outR2inXDist < proParas.conveyMinDist)
								{
									double moveLength = outR2inXDist * 0.5;
									particle.position_[outDeviceIndex * 3] -= moveLength;
									particle.position_[inDeviceIndex * 3] += moveLength;
								}
							}
						}
						break;
					}
					case 11://����
					{
						//1.������y�Ϲ��ڽӽ�,��������y�϶���

					}
					case 3://����
					{
						//1.������x�Ϲ��ڽӽ�,��������x�϶���
						if (outPointTPos.x != inPointTPos.x && abs(outPointTPos.x - inPointTPos.x) < proParas.conveyMinDist)
						{
							double moveLength = (outPointTPos.x - inPointTPos.x) * 0.5;
							particle.position_[outDeviceIndex * 3] -= moveLength;
							particle.position_[inDeviceIndex * 3] += moveLength;
						}
						else
						{
							//2.����豸�ڳ����豸��/���Ͻ� ���߷�����
							if (inUp2OutBool)
							{
								outU2InDYDist = outUpPos.y - inDownPos.y;
								if (outU2InDYDist > 0 && outU2InDYDist < proParas.conveyMinDist)
								{
									double moveLength = outU2InDYDist * 0.5;
									particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
									particle.position_[inDeviceIndex * 3 + 1] += moveLength;
								}
							}
							else
							{
								//3.����豸�ڳ����豸����/��Ϊ����
								inR2outLXDist = inRightPos.x - outLeftPos.x;
								if (inR2outLXDist > 0 && inR2outLXDist < proParas.conveyMinDist)
								{
									double moveLength = inR2outLXDist * 0.5;
									particle.position_[outDeviceIndex * 3] += moveLength;
									particle.position_[inDeviceIndex * 3] -= moveLength;
								}
								outR2inLXDist = outRightPos.x - inLeftPos.x;
								if (outR2inLXDist > 0 && outR2inLXDist < proParas.conveyMinDist)
								{
									double moveLength = outR2inLXDist * 0.5;
									particle.position_[outDeviceIndex * 3] -= moveLength;
									particle.position_[inDeviceIndex * 3] += moveLength;
								}
							}
						}
						break;
					}
					case 4://����
					{
						if (inLeft2OutBool)
						{
							if (inUp2OutBool)
							{
								//1.in��out����
								outUp2inDYDist = outUpPos.y - inDownPos.y;
								if (outUp2inDYDist > 0 && outUp2inDYDist < proParas.conveyMinDist)
								{
									double moveLength = outUp2inDYDist * 0.5;
									particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
									particle.position_[inDeviceIndex * 3 + 1] += moveLength;
								}
							}
							else
							{
								//5.in��out����
								outL2inLXDist = outLeftPos.x - inLeftPos.x;
								if (outL2inLXDist > 0 && outL2inLXDist < proParas.conveyMinDist)
								{
									double moveLength = outL2inLXDist * 0.5;
									particle.position_[outDeviceIndex * 3] -= moveLength;
									particle.position_[inDeviceIndex * 3] += moveLength;
								}
							}
							//3.in��Out���
							double inU2outUYDist = inUpPos.y - outUpPos.y;
							if (inU2outUYDist > 0 && inU2outUYDist < proParas.conveyMinDist)
							{
								double moveLength = inU2outUYDist * 0.5;
								particle.position_[outDeviceIndex * 3 + 1] += moveLength;
								particle.position_[inDeviceIndex * 3 + 1] -= moveLength;
							}
						}
						else
						{
							if (inUp2OutBool)
							{
								//2.in��out����
								out2inLXDist = outPointTPos.x - inLeftPos.x;
								if (out2inLXDist > 0 && out2inLXDist < proParas.conveyMinDist)
								{
									double moveLength = out2inLXDist * 0.5;
									particle.position_[outDeviceIndex * 3] -= moveLength;
									particle.position_[inDeviceIndex * 3] += moveLength;
								}
							}
							else
							{
								//6.in��out����
								outR2inLXDist = outRightPos.x - inLeftPos.x;
								if (outR2inLXDist > 0 && outR2inLXDist < proParas.conveyMinDist)
								{
									double moveLength = outR2inLXDist * 0.5;
									particle.position_[outDeviceIndex * 3] -= moveLength;
									particle.position_[inDeviceIndex * 3] += moveLength;
								}
							}
							//4.in��out�ұ�
							outU2inYDist = outUpPos.y - inPointTPos.y;
							if (outU2inYDist > 0 && outU2inYDist < proParas.conveyMinDist)
							{
								double moveLength = outU2inYDist * 0.5;
								particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
								particle.position_[inDeviceIndex * 3 + 1] += moveLength;
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

		#pragma region �������ڵ�ļ�������
		vector<InoutPoint>().swap(particle.inoutPoints);
		for (int i = 0; i < proParas.DeviceSum; i++)
		{
			for (AdjPoint point : copyDeviceParas[i].adjPointsIn)
			{
				InoutPoint ioPoint;
				ioPoint.pointDirect = point.direct;
				Vector2 axis(point.pos.x + particle.position_[3 * i], point.pos.y + particle.position_[3 * i + 1]);
				ioPoint.pointAxis = axis;
				particle.inoutPoints.push_back(ioPoint);
			}
			for (AdjPoint point : copyDeviceParas[i].adjPointsOut)
			{
				InoutPoint ioPoint;
				ioPoint.pointDirect = point.direct;
				Vector2 axis(point.pos.x + particle.position_[3 * i], point.pos.y + particle.position_[3 * i + 1]);
				ioPoint.pointAxis = axis;
				particle.inoutPoints.push_back(ioPoint);
			}
		}

		#pragma endregion

		#pragma region �����豸����ͳ�������깹��·����ͼ
		vector< vector<APoint*> > pathPointMap;
		vector<double> horizonAxisList;
		vector<double> verticalAxisList;
		//�ȶԳ���ڵ�ˮƽ�ʹ�ֱ���з���(ע�����ƫ����)
		for (int i = 0; i < proParas.DeviceSum; i++)
		{
			for (AdjPoint p : copyDeviceParas[i].adjPointsIn)
			{
				if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//����
				{
					horizonAxisList.push_back(p.pos.x + particle.position_[i * 3]);
				}
				else {//����
					verticalAxisList.push_back(p.pos.y + particle.position_[i * 3 + 1]);
				}
			}
			for (AdjPoint p : copyDeviceParas[i].adjPointsOut)
			{
				//cout << p.pos.x << "," << p.pos.y << endl;
				if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//����
				{
					horizonAxisList.push_back(p.pos.x + particle.position_[i * 3]);
				}
				else {//����
					verticalAxisList.push_back(p.pos.y + particle.position_[i * 3 + 1]);
				}
			}
		}
		//�ֿ����2����&����
		horizonAxisList.push_back(proParas.entrancePos.x);
		verticalAxisList.push_back(proParas.entrancePos.y);

		horizonAxisList.push_back(proParas.exitPos.x);
		verticalAxisList.push_back(proParas.exitPos.y);
		//����ÿ���豸������ĸ���Χ����Ϊ�����ϰ���ķ�Χ��
		double* DeviceLowXList = new double[proParas.DeviceSum];
		double* DeviceHighXList = new double[proParas.DeviceSum];
		double* DeviceLowYList = new double[proParas.DeviceSum];
		double* DeviceHighYList = new double[proParas.DeviceSum];
		//ÿ���豸��Χ��4����
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

			//ÿ���豸���ĸ���Χ
			DeviceLowXList[i / 3] = LowX;
			DeviceHighXList[i / 3] = HighX;
			DeviceLowYList[i / 3] = LowY;
			DeviceHighYList[i / 3] = HighY;

			//��ֹ·�������豸�ڲ�
			horizonAxisList.push_back(particle.position_[i]);
			verticalAxisList.push_back(particle.position_[i + 1]);

		}
		//����Щ������갴�մ�С��������
		sort(verticalAxisList.begin(), verticalAxisList.end());
		sort(horizonAxisList.begin(), horizonAxisList.end());
		//ֻ�������ظ��ĵ�
		auto unique_end1 = unique(verticalAxisList.begin(), verticalAxisList.end());
		verticalAxisList.erase(unique_end1, verticalAxisList.end());
		auto unique_end2 = unique(horizonAxisList.begin(), horizonAxisList.end());
		horizonAxisList.erase(unique_end2, horizonAxisList.end());

		//�������е��ϰ�����±�
		vector<int> barrierRowIndexs;
		vector<int> barrierColIndexs;
		//����Щ����ȥ���·����map
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
						//�ϰ�����ͼ�е��±�
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

		#pragma region Ѱ·

		auto star = new CAstar();
		star->_allPoints = pathPointMap;
		int beginRowIndex, beginColIndex, endRowIndex, endColIndex;

		double totalTime = 0.0;
		vector<PointLink>().swap(particle.pointLinks);


		vector<PointLink> copyPLinks;
		for (int i = 0; i < proParas.CargoTypeNum; i++)
		{
			CargoType curCargoType = proParas.cargoTypeList[i];

			for (int j = 0; j < proParas.cargoTypeList[i].linkSum; j++)//�������Ͼ������豸�б�
			{
				PathDirection pathBeginDirect;
				PathDirection pathEndDirect;
				int forwardDeviceIndex, curDeviceIndex;//�豸1���豸2ID
				int forwardOutIndex, curInIndex;//����ڵ��±�
				double device1PosX, device1PosY, device2PosX, device2PosY;//�豸��Χ���ĸ���
				double initDevice1PosX, initDevice1PosY, initDevice2PosX, initDevice2PosY;//����δ���Ӱ�Χ�ߵ�����

				double deviceDistance = 0.0;//����

				forwardDeviceIndex = proParas.cargoTypeList[i].deviceLinkList[j].outDeviceIndex;
				curDeviceIndex = proParas.cargoTypeList[i].deviceLinkList[j].inDeviceIndex;

				forwardOutIndex = proParas.cargoTypeList[i].deviceLinkList[j].outPointIndex;
				curInIndex = proParas.cargoTypeList[i].deviceLinkList[j].inPointIndex;
				if (forwardDeviceIndex == -1)//˵���ǲֿ����
				{
					/*forwardOutIndex = 0;
					curInIndex = proParas.cargoTypeList[i].deviceLinkList[j].inPointIndex;*/

					//��ͷ�ͽ�β��ĳ�����
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
					//�õ��豸��Χ�ĵ�
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
				else if (curDeviceIndex == -2)//˵���ǳ���
				{
					//forwardOutIndex = proParas.cargoTypeList[i].deviceLinkList[j].outPointIndex;
					//curInIndex = 0;

					//��ͷ�ͽ�β��ĳ�����
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
					//�õ��豸��Χ�ĵ�
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
				else//��ͨ
				{
					//forwardOutIndex = proParas.cargoTypeList[i].deviceLinkList[j].outPointIndex - 1;
					//curInIndex = proParas.cargoTypeList[i].deviceLinkList[j].inPointIndex - 1;

					//��ͷ�ͽ�β��ĳ�����
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
					//�õ��豸��Χ�ĵ�
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
				//�������·��
				beginRowIndex = FindAxisIndex(device1PosY, verticalAxisList);
				beginColIndex = FindAxisIndex(device1PosX, horizonAxisList);
				endRowIndex = FindAxisIndex(device2PosY, verticalAxisList);
				endColIndex = FindAxisIndex(device2PosX, horizonAxisList);
				//ʵ�ʵ�index��Ҫ��һ������
				/*if (forwardDeviceIndex != -1)
				{
					���ݳ��ڵ�ķ����ó�ʼ���indexƫ��һ����λ
					switch (copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].direct)
					{
					case PointDirect::Up:
						if (beginRowIndex < verticalAxisList.size() - 1)
							beginRowIndex++;
						break;
					case PointDirect::Down:
						if (beginRowIndex > 0)
							beginRowIndex--;
						break;
					case PointDirect::Left:
						if (beginColIndex > 0)
							beginColIndex--;
						break;
					case PointDirect::Right:
						if (beginColIndex < horizonAxisList.size() - 1)
							beginColIndex++;
						break;
					}
				}
				if (curDeviceIndex != -2)
				{
					������ڵ�ķ����ó�ʼ���indexƫ��һ����λ
					switch (copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].direct)
					{
					case PointDirect::Up:
						if (endRowIndex < verticalAxisList.size() - 1)
							endRowIndex++;
						break;
					case PointDirect::Down:
						if (endRowIndex > 0)
							endRowIndex--;
						break;
					case PointDirect::Left:
						if (endColIndex > 0)
							endColIndex--;
						break;
					case PointDirect::Right:
						if (endColIndex < horizonAxisList.size() - 1)
							endColIndex++;
						break;
					}
				}*/
				//�õ�·����path�ǵ�һ���ڵ�
				APoint* path = star->findWay(pathBeginDirect, beginRowIndex, beginColIndex, endRowIndex, endColIndex);
				//�����еĽ⣬ֱ���˳�
				if (path == nullptr)
				{
					particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
					return;
				}
				//����·�����㳤��
				deviceDistance = star->CalcuPathLength(path);


				#pragma region ����·����ֻ������ʼ�� + ·���е�ת��㣩
				//·����������
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


				//����·����ֻ����ÿ�ε���ʼ�㣩
				vector<Vector2> points;
				Vector2 endP(initDevice2PosX, initDevice2PosY);//���յ㿪ʼ����
				points.push_back(endP);

				PathDirection pathCurDirect = pathEndDirect;//��Ϊ���·���Ƿ����

				Vector2 lastP(path->x, path->y);//�����һ���㿪ʼ
				points.push_back(lastP);
				path = path->parent;
				while (path)
				{
					Vector2 curP(path->x, path->y);
					//ֻ��ת��ĵ�Żᱻ����·��
					if (pathCurDirect == PathDirection::Horizon && curP.x == lastP.x)
					{
						//if ((lastP.x != points.back().x || lastP.y != points.back().y)
						//	&& CalcuDeviceDist(lastP, points.back()) < proParas.conveyMinDist)
						//{
						//	particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
						//	return;
						//}
						points.push_back(lastP);
						pathCurDirect = PathDirection::Vertical;
					}
					else if (pathCurDirect == PathDirection::Vertical && curP.y == lastP.y)
					{
						//if ((lastP.x != points.back().x || lastP.y != points.back().y)
						//	&& CalcuDeviceDist(lastP, points.back()) < proParas.conveyMinDist)
						//{
						//	particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
						//	return;
						//}
						points.push_back(lastP);
						pathCurDirect = PathDirection::Horizon;
					}
					path = path->parent;
					lastP = curP;
				}
				if (CalcuDeviceDist(lastP, points.back()) < proParas.conveyMinDist) 
				{
					particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
					return;
				}
				points.push_back(lastP);
				Vector2 startP(initDevice1PosX, initDevice1PosY);
				points.push_back(startP);

				PointLink pointLink(forwardDeviceIndex, forwardOutIndex, curDeviceIndex, curInIndex, points);
				particle.pointLinks.push_back(pointLink);
				#pragma endregion
				


				//��������ʱ��(�������� * ·�߳��� * ����Ч��)
				totalTime += curCargoType.totalVolume * deviceDistance * proParas.conveySpeed;
				//�����豸����ʱ��(�������� * ����Ч��)
				//totalTime += curCargoType.totalVolume * curDevice.workSpeed;

				star->resetAStar();
				//���ϰ������±��
				for (int i = 0; i < barrierRowIndexs.size(); i++)
				{
					star->_allPoints[barrierRowIndexs[i]][barrierColIndexs[i]]->type = AType::ATYPE_BARRIER;
				}
			}
		}
		#pragma endregion

		#pragma region �����ֽ��ת��Ϊ���ͻ�����(ͬʱ��ǿһ����������̾���Լ��)
		particle.strConveyorList.clear();
		particle.curveConveyorList.clear();
		//particle.pathPointInfoMap.clear();
		set<SegPath> segPathSet;
		for (PointLink pl : copyPLinks)//���������ظ���·�߶�
		{
			for (int i = pl.points.size() - 1; i > 0; i--)
			{
				Vector2Int p1 = Multi10000ToInt(pl.points[i]);
				Vector2Int p2 = Multi10000ToInt(pl.points[i - 1]);
				if (p1 != p2)//���겻��һ��
				{
					SegPath temp(p1, p2);
					segPathSet.insert(temp);
				}
			}
		}
		map<Vector2Int, PointInfo> pathPointInfoMap;//·������Ϣmap
		//����set�����������set�� ��ĳ����&������·�ߵĴ�ֱˮƽ��Ŀ&�Ƿ��� ��Ϣ
		for (SegPath sp : segPathSet)
		{
			//��map����sp��p1
			if (pathPointInfoMap.count(sp.p1)) {
				//�ҵ���������Ϣ
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

			//��map����sp��p2
			if (pathPointInfoMap.count(sp.p2)) {
				//�ҵ���������Ϣ
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
			tempStrInfo.startPos = Multi10000ToInt(pl.points[pl.points.size() - 1]);//��ͷ
			tempStrInfo.startVnum = pathPointInfoMap[tempStrInfo.startPos].vertDirNum;
			tempStrInfo.startHnum = pathPointInfoMap[tempStrInfo.startPos].horiDirNum;
			for (int i = pl.points.size() - 2; i >= 0; i--) {
				Vector2Int p = Multi10000ToInt(pl.points[i]);
				if (pathPointInfoMap[p].isKeep == true) {//���������ظ���
					//�ȸ���ֱ�����ͻ�
					if (tempStrInfo.startPos != p) {
						tempStrInfo.endPos = p;
						if (curIterNum >= maxIterNum / 2) {
							if (tempStrInfo.startPos.Distance(tempStrInfo.endPos) < proParas.conveyMinDist)
							{
								particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
								return;
							}
						}
						tempStrInfo.endVnum = pathPointInfoMap[tempStrInfo.endPos].vertDirNum;
						tempStrInfo.endHnum = pathPointInfoMap[tempStrInfo.endPos].horiDirNum;
						particle.strConveyorList.insert(tempStrInfo);
						tempStrInfo.startPos = p;
						tempStrInfo.startVnum = pathPointInfoMap[tempStrInfo.startPos].vertDirNum;
						tempStrInfo.startHnum = pathPointInfoMap[tempStrInfo.startPos].horiDirNum;
					}
					//ֻҪ����ʼ�յ㣬����Ҫ����ת�����ͻ�
					if (!(pathPointInfoMap[p].horiDirNum == 1 && pathPointInfoMap[p].vertDirNum == 0)
						&& !(pathPointInfoMap[p].horiDirNum == 0 && pathPointInfoMap[p].vertDirNum == 1)) {
						particle.curveConveyorList.insert(p);
					}
				}
			}
		}
		#pragma endregion

		#pragma region ����Ŀ�꺯��ֵ

		//����ֱ�ߺ�ת�����ͻ���set���õ����ͻ����ܳɱ�
		double conveyorTotalCost = 0.0;
		for (StraightConveyorInfo sci : particle.strConveyorList)
		{
			conveyorTotalCost += proParas.strConveyorUnitCost * sci.startPos.Distance(sci.endPos);
		}
		conveyorTotalCost += proParas.curveConveyorUnitCost * particle.curveConveyorList.size();
		//������Ӧ��ֵ
		//cout << conveyorTotalCost << "," << totalTime << endl;

		particle.fitness_[0] = conveyorTotalCost;
		//particle.fitness_[0] = 1000;

		//particle.fitness_[1] = CalcuTotalArea(particle, proParas);
		//particle.fitness_[1] = totalTime;
		particle.fitness_[1] = 1000;
		#pragma endregion

	}
	#pragma endregion
	return;
}
//˳ʱ����ת�������
Vector2 Rotate(Vector2 pointPos, Vector2 centerPos, float rotateAngle)
{
	float xx = (pointPos.x - centerPos.x) * cos(rotateAngle * (PI / 180)) + (pointPos.y - centerPos.y) * sin(rotateAngle * (PI / 180)) + centerPos.x;
	float yy = -(pointPos.x - centerPos.x) * sin(rotateAngle * (PI / 180)) + (pointPos.y - centerPos.y) * cos(rotateAngle * (PI / 180)) + centerPos.y;
	Vector2 result(xx, yy);
	return result;
}
int FindAxisIndex(double axis, const vector<double>& axisList)
{
	//�ö��ַ�����
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
//���������ξ���
double CalcuDeviceDist(Vector2 pos1, Vector2 pos2)
{
	return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y);
}
//����ռ�����
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
	//���������
	area = (max_X - min_X) * (max_Y - min_Y);
	return area;
}
//�ȳ���10000��Ȼ���������뵽Int
int Multi10000ToInt(double num)
{
	return round(num * 10000);
}
Vector2Int Multi10000ToInt(Vector2 v)
{
	return Vector2Int(Multi10000ToInt(v.x), Multi10000ToInt(v.y));
}