#pragma once
#include "PSO.h"
#include "AStar.h"
#include <algorithm>
#include <math.h>
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
#pragma region �ж�������������»������ҹ�ϵ
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
void FitnessFunction(int curIterNum, int maxIterNum, BestPathInfo* bestPathInfoList, ProblemParas proParas, Particle& particle);
double CalcuTotalArea(Particle& particle, DevicePara* copyDeviceParas);
double CalcuDeviceDist(Vector2 pos1, Vector2 pos2);

int FindAxisIndex(double axis, const vector<double>& axisList);

//˳ʱ����ת�������
Vector2 Rotate(Vector2 pointPos, Vector2 centerPos, float rotateAngle);
//��һ������*10000Ȼ���������뵽int
int Multi10000ToInt(double num);

Vector2Int Multi10000ToInt(Vector2 v);
//Ĭ�ϵ���Ӧ�ȼ��㺯���������滻
void FitnessFunction(int curIterNum, int maxIterNum, BestPathInfo* bestPathInfoList, ProblemParas proParas, Particle& particle)
{
	double punishValue1 = 0;
	double punishValue2 = 0;
	bool IsDeviceOverlap = false;//�Ƿ��ص�
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
		for (int pointIndex = 0; pointIndex < copyDeviceParas[i / 3].adjPInCount; ++pointIndex)
		{
			AdjPoint& point = copyDeviceParas[i / 3].adjPointsIn[pointIndex];
			point.pos = Rotate(point.pos, deviceCenterPos, rotateAngle);
			newDirect = point.direct + (int)curDirect;
			point.direct = (newDirect == 4) ? (PointDirect)4 : (PointDirect)(newDirect % 4);
		}
		for (int pointIndex = 0; pointIndex < copyDeviceParas[i / 3].adjPOutCount; ++pointIndex)
		{
			AdjPoint& point = copyDeviceParas[i / 3].adjPointsOut[pointIndex];
			point.pos = Rotate(point.pos, deviceCenterPos, rotateAngle);
			newDirect = point.direct + (int)curDirect;
			point.direct = (newDirect == 4) ? (PointDirect)4 : (PointDirect)(newDirect % 4);
		}
	}
#pragma endregion

#pragma region ����豸�Ƿ��ص�
	//����ص������е���
	//���ͱ�׼�ᷢ��ʲô��
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
				//particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
				IsDeviceOverlap = true;
				//cout << curIterNum << ":" << "�ص�" << endl;
				//return;
			}
		}
	}
#pragma endregion

#pragma region ����豸�ص��������豸λ�ã����������ص�������ֱ�Ӳ���

#pragma region �����豸λ��
	if (IsDeviceOverlap == true) {
		//1.��һ���豸�ĳߴ�����洢�����豸
		//2.ÿ�δӵ�һ���豸��ʼ�����Ҵӱ��Ϊ1���豸��ʼ����Ƿ�͸��豸�ص�
		//3.����ص���ִ�����²�����
		//	a.�涨�豸ֻ���������ƶ�
		//  b.�ֱ������豸����/���ƶ��ľ���
		//	c.ԭ��ѡ���ƶ��������豸�ĳߴ�x֮��/�ߴ�y֮��������ڳ���xy�ߴ������С�ķ����ƶ�
		//Ŀǰ���ƶ���������λ�ã��Ȳ��������
		//4.�ƶ�֮���´λ��Ǵӵ�һ���豸��ʼ��⣬ֱ�����еĶ����ص�
		//5.Ȼ�����������豸λ�õİ�����Σ����С�ڳ���ߴ磬�ƶ��������ڵ�һ�����λ��
		//Ȼ���������ƶ�ǰ���λ�ò��޸������豸��λ��
		//�ñ�����������������ر��ף���������ά��һ����İ������
		int deviceIDSizeCount = proParas.DeviceSum;
		DeviceIDSize* deviceIDSizeList = new DeviceIDSize[proParas.DeviceSum];//�����豸��С�����ID����
		//���������ṹ���豸���꣬��Ϊ���ܻ��޸�ʧ��
		double* particlePosList = new double[particle.dim_];
		for (int i = 0; i < particle.dim_; ++i) {
			particlePosList[i] = particle.position_[i];
		}
		for (int i = 0; i < deviceIDSizeCount; ++i) {
			deviceIDSizeList[i] = DeviceIDSize(i, copyDeviceParas[i].size);
		}
		//sort(deviceIDSizeList.begin(), deviceIDSizeList.end());//����������Լ�д
		DeviceIDSize_Sort(deviceIDSizeList, 0, deviceIDSizeCount - 1);//�����豸�ĳߴ�����

		double outSizeLength1, outSizeWidth1;
		double outSizeLength2, outSizeWidth2;
		int firstID, secondID;
		int maxIter = 1000;//��ֹ��ѭ��
		int curIter = 0;
		bool tooMuch = false;
		for (int i = 0; i < deviceIDSizeCount; ++i) {
			//����������豸�Ƿ�����ص�
			firstID = deviceIDSizeList[i].ID;
			outSizeLength1 = 0.5 * copyDeviceParas[firstID].size.x + copyDeviceParas[firstID].spaceLength;
			outSizeWidth1 = 0.5 * copyDeviceParas[firstID].size.y + copyDeviceParas[firstID].spaceLength;
			double firstLowX = particlePosList[3 * firstID] - outSizeLength1;
			double firstUpX = particlePosList[3 * firstID] + outSizeLength1;
			double firstLowY = particlePosList[3 * firstID + 1] - outSizeWidth1;
			double firstUpY = particlePosList[3 * firstID + 1] + outSizeWidth1;
			for (int j = 0; j < deviceIDSizeCount;) {
				++curIter;
				if (curIter > maxIter) {
					particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
					return;
				}
				//cout << j << endl;
				secondID = deviceIDSizeList[j].ID;
				if (firstID != secondID) {
					outSizeLength2 = 0.5 * copyDeviceParas[secondID].size.x + copyDeviceParas[secondID].spaceLength;
					outSizeWidth2 = 0.5 * copyDeviceParas[secondID].size.y + copyDeviceParas[secondID].spaceLength;
					double secondLowX = particlePosList[3 * secondID] - outSizeLength2;
					double secondUpX = particlePosList[3 * secondID] + outSizeLength2;
					double secondLowY = particlePosList[3 * secondID + 1] - outSizeWidth2;
					double secondUpY = particlePosList[3 * secondID + 1] + outSizeWidth2;
					if (IsRangeOverlap(firstLowX, firstUpX, secondLowX, secondUpX)
						&& IsRangeOverlap(firstLowY, firstUpY, secondLowY, secondUpY)) {
						j = 0;//ֻҪ�����ص��ľ�Ҫ���¿�ʼ�ж�
						//�ص��ˣ��ƶ��豸�����ݱ���ȷ�������ƶ�
						//������Բ�һ���ܺã���Ҫ��֤
						double rateLeft = (outSizeLength1 + outSizeLength2) * 2 / proParas.workShopLength;
						double rateDown = (outSizeWidth1 + outSizeWidth2) * 2 / proParas.workShopWidth;
						if (rateLeft < rateDown) {//˵��Ӧ�������ƶ�
							particlePosList[firstID * 3] = secondLowX - outSizeLength1 - 0.1;
						}
						else {//�����ƶ�
							particlePosList[firstID * 3 + 1] = secondLowY - outSizeWidth1 - 0.1;
						}
						firstLowX = particlePosList[3 * firstID] - outSizeLength1;
						firstUpX = particlePosList[3 * firstID] + outSizeLength1;
						firstLowY = particlePosList[3 * firstID + 1] - outSizeWidth1;
						firstUpY = particlePosList[3 * firstID + 1] + outSizeWidth1;
					}
					else {
						++j;
					}
				}
				else {
					++j;
				}
			}
		}
		//5.Ȼ�����������豸λ�õİ�����Σ����С�ڳ���ߴ磬�ƶ��������ڵ�һ�����λ��
		//ʵ�֣�����������豸���ĸ�����ı߽�
		double min_X, min_Y, max_X, max_Y;
		min_X = min_Y = INT_MAX;
		max_X = max_Y = -INT_MAX;
		for (int i = 0; i < proParas.DeviceSum; ++i) {
			double outSizeLength = copyDeviceParas[i].size.x * 0.5 + copyDeviceParas[i].spaceLength;
			double outSizeWidth = copyDeviceParas[i].size.y * 0.5 + copyDeviceParas[i].spaceLength;
			min_X = min(min_X, particlePosList[3 * i] - outSizeLength);
			max_X = max(max_X, particlePosList[3 * i] + outSizeLength);
			min_Y = min(min_Y, particlePosList[3 * i + 1] - outSizeWidth);
			max_Y = max(max_Y, particlePosList[3 * i + 1] + outSizeWidth);
		}
		Vector2 oriRectAxis((max_X + min_X) / 2.0, (max_Y + min_Y) / 2.0);//�ܰ�����ε���������
		Vector2 newRectAxis;
		if ((max_X - min_X) <= proParas.workShopLength
			&& (max_Y - min_Y) <= proParas.workShopWidth) {//�������С�ڳ����С
			//��������������һ������
			double rectLowX = 0 + (max_X - min_X) * 0.5;
			double rectHighX = proParas.workShopLength - (max_X - min_X) * 0.5;
			double rectLowY = 0 + (max_Y - min_Y) * 0.5;
			double rectHighY = proParas.workShopWidth - (max_Y - min_Y) * 0.5;
			newRectAxis.x = GetDoubleRand() * (rectHighX - rectLowX) + rectLowX;
			newRectAxis.y = GetDoubleRand() * (rectHighY - rectLowY) + rectLowY;
			//���ݰ�����ε�����仯���޸������豸������
			double deviceOffsetX = newRectAxis.x - oriRectAxis.x;
			double deviceOffsetY = newRectAxis.y - oriRectAxis.y;
			for (int i = 0; i < proParas.DeviceSum; ++i) {
				particlePosList[3 * i] += deviceOffsetX;
				particlePosList[3 * i + 1] += deviceOffsetY;
			}
			IsDeviceOverlap = false;
			//�޸�postion����Ҫ�޸���������
			for (int i = 0; i < proParas.DeviceSum; i++) {
				particle.position_[i * 3] = particlePosList[3 * i];
				particle.position_[i * 3 + 1] = particlePosList[3 * i + 1];
			}
		}
		else {
			IsDeviceOverlap = true;
			particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
			return;
		}
	}
#pragma endregion

	if (IsDeviceOverlap == false)
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
					//������豸�ĸ����򲻿������ı߽�
					Vector2 outDeviceUpPos(particle.position_[outDeviceIndex * 3], particle.position_[outDeviceIndex * 3 + 1] + 0.5 * copyDeviceParas[outDeviceIndex].size.y);
					Vector2 outDeviceDownPos(particle.position_[outDeviceIndex * 3], particle.position_[outDeviceIndex * 3 + 1] - 0.5 * copyDeviceParas[outDeviceIndex].size.y);
					Vector2 outDeviceLeftPos(particle.position_[outDeviceIndex * 3] - 0.5 * copyDeviceParas[outDeviceIndex].size.x, particle.position_[outDeviceIndex * 3 + 1]);
					Vector2 outDeviceRightPos(particle.position_[outDeviceIndex * 3] + 0.5 * copyDeviceParas[outDeviceIndex].size.x, particle.position_[outDeviceIndex * 3 + 1]);

					Vector2 inDeviceUpPos(particle.position_[inDeviceIndex * 3], particle.position_[inDeviceIndex * 3 + 1] + 0.5 * copyDeviceParas[inDeviceIndex].size.y);
					Vector2 inDeviceDownPos(particle.position_[inDeviceIndex * 3], particle.position_[inDeviceIndex * 3 + 1] - 0.5 * copyDeviceParas[inDeviceIndex].size.y);
					Vector2 inDeviceLeftPos(particle.position_[inDeviceIndex * 3] - 0.5 * copyDeviceParas[inDeviceIndex].size.x, particle.position_[inDeviceIndex * 3 + 1]);
					Vector2 inDeviceRightPos(particle.position_[inDeviceIndex * 3] + 0.5 * copyDeviceParas[inDeviceIndex].size.x, particle.position_[inDeviceIndex * 3 + 1]);


					//������豸�ĸ�����ı߽�
					Vector2 outUpPos(outDeviceUpPos.x, outDeviceUpPos.y + proParas.convey2DeviceDist);
					Vector2 outDownPos(outDeviceDownPos.x, outDeviceDownPos.y - proParas.convey2DeviceDist);
					Vector2 outLeftPos(outDeviceLeftPos.x - proParas.convey2DeviceDist, outDeviceLeftPos.y);
					Vector2 outRightPos(outDeviceRightPos.x + proParas.convey2DeviceDist, outDeviceRightPos.y);

					Vector2 inUpPos(inDeviceUpPos.x, inDeviceUpPos.y + proParas.convey2DeviceDist);
					Vector2 inDownPos(inDeviceDownPos.x, inDeviceDownPos.y - proParas.convey2DeviceDist);
					Vector2 inLeftPos(inDeviceLeftPos.x - proParas.convey2DeviceDist, inDeviceLeftPos.x);
					Vector2 inRightPos(outDeviceRightPos.x + proParas.convey2DeviceDist, outDeviceRightPos.y);


					//Ҫ�õ��ĸ��ֱȽ�ֵ
					double inR2outXDist, out2inLXDist, in2outLXDist, outR2inXDist;
					double outU2inDYDist, inR2outLXDist, outR2inLXDist;
					double outL2inLXDist, inU2outUYDist, inU2outDYDist, outU2inYDist;
					double out2inDYDist, inU2outYDist, in2outDYDist;
					//���ж����ߵķ�λ
					//��һ���ж϶��߷�λ�Ĳ���
					Vector2 inDevicePos(particle.position_[inDeviceIndex * 3], particle.position_[inDeviceIndex * 3 + 1]);
					Vector2 outDevicePos(particle.position_[outDeviceIndex * 3], particle.position_[outDeviceIndex * 3 + 1]);
					bool inLeft2OutBool, inRight2OutBool, inUp2OutBool, inDown2OutBool;

					int OutPosIndex_X, OutPosIndex_Y, InPosIndex_X, InPosIndex_Y;
					OutPosIndex_X = outDeviceIndex * 3; OutPosIndex_Y = outDeviceIndex * 3 + 1;
					InPosIndex_X = inDeviceIndex * 3; InPosIndex_Y = inDeviceIndex * 3 + 1;
					double moveLength = 0.0;
					//�ñ���������һ��16��
					//Up = 1, Right = 2, Down = 3, Left = 4
					//      	   //�� �� �� ��
					//  /*��*/{-1, 1, 2, 3, 4},
					//	/*��*/{-1, 5, 6, 7, 8},
					//	/*��*/{-1, 9, 10, 11, 12},
					//	/*��*/{-1, 13, 14, 15, 16},
					switch (pointDirectArray[outPointDirect][inPointDirect])
					{
					case 1://����
					{
						//1.������y�Ϲ��ڽӽ�,��������y�϶���
						if (outPointTPos.y != inPointTPos.y && abs(outPointTPos.y - inPointTPos.y) < proParas.conveyMinDist)
						{
							moveLength = (outPointTPos.y - inPointTPos.y) * 0.5;
							particle.position_[OutPosIndex_Y] -= moveLength;
							particle.position_[InPosIndex_Y] += moveLength;
							break;
						}
						//2.����豸�ڳ����豸��/���Ͻ�
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
						//3.����豸�ڳ����豸��/���½�
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
					case 11://����
					{
						//1.������y�Ϲ��ڽӽ�,��������y�϶���
						if (outPointTPos.y != inPointTPos.y && abs(outPointTPos.y - inPointTPos.y) < proParas.conveyMinDist)//�������
						{
							moveLength = (outPointTPos.y - inPointTPos.y) * 0.5;
							particle.position_[OutPosIndex_Y] -= moveLength;
							particle.position_[InPosIndex_Y] += moveLength;
							break;
						}
						inUp2OutBool = IsInUp2Out(inDownPos, outUpPos);
						//2.����豸�ڳ����豸��/���Ͻ�
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
						//3.����豸�ڳ����豸��/���½�
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
					case 16://����
					{
						//1.������x�Ϲ��ڽӽ�,��������x�϶���
						if (outPointTPos.x != inPointTPos.x && abs(outPointTPos.x - inPointTPos.x) < proParas.conveyMinDist)
						{
							moveLength = (outPointTPos.x - inPointTPos.x) * 0.5;
							particle.position_[OutPosIndex_X] -= moveLength;
							particle.position_[InPosIndex_X] += moveLength;
							break;
						}
						//2.in��out���
						inLeft2OutBool = IsInLeft2Out(inRightPos, outLeftPos);
						if (inLeft2OutBool)
						{
							//in��out����
							out2inDYDist = outPointTPos.y - inDownPos.y;
							if (out2inDYDist > 0 && out2inDYDist < proParas.conveyMinDist)
							{
								moveLength = out2inDYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
							//in��out����
							inU2outYDist = inUpPos.y - outPointTPos.y;
							if (inU2outYDist > 0 && inU2outYDist < proParas.conveyMinDist)
							{
								moveLength = inU2outYDist * 0.5;
								particle.position_[OutPosIndex_Y] += moveLength;
								particle.position_[InPosIndex_Y] -= moveLength;
								break;
							}
						}
						//3.in��out�ұ�
						inRight2OutBool = IsInRight2Out(inLeftPos, outRightPos);
						if (inRight2OutBool)
						{
							//in��out����
							outU2inYDist = outUpPos.y - inPointTPos.y;
							if (outU2inYDist > 0 && outU2inYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
							//in��out����
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
					case 6://����
					{
						//1.������x�Ϲ��ڽӽ�,��������x�϶���
						if (outPointTPos.x != inPointTPos.x && abs(outPointTPos.x - inPointTPos.x) < proParas.conveyMinDist)
						{
							moveLength = (outPointTPos.x - inPointTPos.x) * 0.5;
							particle.position_[OutPosIndex_X] -= moveLength;
							particle.position_[InPosIndex_X] += moveLength;
							break;
						}
						//2.in��out�ұ�
						inRight2OutBool = IsInRight2Out(inLeftPos, outRightPos);
						if (inRight2OutBool)
						{
							//in��out����
							out2inDYDist = outPointTPos.y - inDownPos.y;
							if (out2inDYDist > 0 && out2inDYDist < proParas.conveyMinDist)
							{
								moveLength = out2inDYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
							//in��out����
							inU2outYDist = inUpPos.y - outPointTPos.y;
							if (inU2outYDist > 0 && inU2outYDist < proParas.conveyMinDist)
							{
								moveLength = inU2outYDist * 0.5;
								particle.position_[OutPosIndex_Y] += moveLength;
								particle.position_[InPosIndex_Y] -= moveLength;
								break;
							}
						}
						//3.in��out���
						inLeft2OutBool = IsInLeft2Out(inRightPos, outLeftPos);
						if (inLeft2OutBool)
						{
							//in��out����
							outU2inYDist = outUpPos.y - inPointTPos.y;
							if (outU2inYDist > 0 && outU2inYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
							//in��out����
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
					case 3://����
					{
						//1.in��out�Ϸ�������x�Ϻܽӽ�
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
						//2.����豸�ڳ����豸��/���Ͻ�
						inUp2OutBool = IsInUp2Out(inDeviceDownPos, outDeviceUpPos);
						if (inUp2OutBool)//���ҿ�����һ�ַ�ʽ����
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
						//����豸�ڳ����豸����
						inDown2OutBool = IsInDown2Out(inUpPos, outDownPos);
						if (inDown2OutBool)
						{
							//3.����豸�ڳ����豸����/��Ϊ����
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
					case 14://����
					{
						//1.in��out��ߣ�����y�Ϻܽӽ�
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
						//2.in��out���
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
						//3.in��out�ұߣ���Ϊ���Ϻ�����
						inRight2OutBool = IsInRight2Out(inLeftPos, outRightPos);
						if (inRight2OutBool)
						{
							//in��out����
							inU2outDYDist = inUpPos.y - outDownPos.y;
							if (inU2outDYDist > 0 && inU2outDYDist < proParas.conveyMinDist)
							{
								moveLength = inU2outDYDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
							//in��out����
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
					case 4://����(�Ȳ����ǿ��ܵ����)
					{
						inLeft2OutBool = IsInLeft2Out(inRightPos, outLeftPos);
						if (inLeft2OutBool)
						{
							//1.in��out����
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
							//2.in��out����
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
							//3.in��out����һ��
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
							//4.in��out���½�
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
					case 2://����
					{
						//in��out��
						inUp2OutBool = IsInUp2Out(inDownPos, outUpPos);
						if (inUp2OutBool)
						{
							//1.in��out����
							inR2outXDist = inRightPos.x - outPointTPos.x;
							if (inR2outXDist > 0 && inR2outXDist < proParas.conveyMinDist)
							{
								moveLength = inR2outXDist * 0.5;
								particle.position_[OutPosIndex_X] += moveLength;
								particle.position_[InPosIndex_X] -= moveLength;
								break;
							}
						}
						//in��out��
						inRight2OutBool = IsInRight2Out(inLeftPos, outRightPos);//�������Ҳ�����Ż�����ǰ���
						if (inRight2OutBool)
						{
							//2.in��out����
							outU2inDYDist = outUpPos.y - inDownPos.y;
							if (outU2inDYDist > 0 && outU2inDYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inDYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
						}
						//in��out��
						inLeft2OutBool = IsInLeft2Out(inRightPos, outLeftPos);
						if (inLeft2OutBool)
						{
							//3.in��out����
							outU2inYDist = outUpPos.y - inPointTPos.y;
							if (outU2inYDist > 0 && outU2inYDist < proParas.conveyMinDist)
							{
								moveLength = outU2inYDist * 0.5;
								particle.position_[OutPosIndex_Y] -= moveLength;
								particle.position_[InPosIndex_Y] += moveLength;
								break;
							}
						}
						//in��out��
						inDown2OutBool = IsInDown2Out(inUpPos, outDownPos);
						if (inDown2OutBool)
						{
							//3.in��out����
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

#pragma region �������ڵ�ļ�������
		//Ȼ������е�inoutPoint��ֵ
		InoutPoint* tempInoutPoints = new InoutPoint[proParas.inoutPointCount];
		int ioPIndex = 0;
		for (int i = 0; i < proParas.DeviceSum; i++)
		{
			for (int pointIndex = 0; pointIndex < copyDeviceParas[i].adjPInCount; ++pointIndex)
			{
				AdjPoint& point = copyDeviceParas[i].adjPointsIn[pointIndex];
				InoutPoint ioPoint;
				ioPoint.pointDirect = point.direct;
				Vector2 axis(point.pos.x + particle.position_[3 * i], point.pos.y + particle.position_[3 * i + 1]);
				ioPoint.pointAxis = axis;
				tempInoutPoints[ioPIndex++] = ioPoint;
			}
			for (int pointIndex = 0; pointIndex < copyDeviceParas[i].adjPOutCount; ++pointIndex)
			{
				AdjPoint& point = copyDeviceParas[i].adjPointsOut[pointIndex];
				InoutPoint ioPoint;
				ioPoint.pointDirect = point.direct;
				Vector2 axis(point.pos.x + particle.position_[3 * i], point.pos.y + particle.position_[3 * i + 1]);
				ioPoint.pointAxis = axis;
				tempInoutPoints[ioPIndex++] = ioPoint;
			}
		}

#pragma endregion

#pragma region �����豸����ͳ�������깹��·����ͼ
		double* horizonAxisList = new double[proParas.horiPointCount];
		double* verticalAxisList = new double[proParas.vertPointCount];
		int curHoriIndex = 0;
		int curVertIndex = 0;
		//�ȶԳ���ڵ�ˮƽ�ʹ�ֱ���з���(ע�����ƫ����)
		//�����ˮƽ�ʹ�ֱ����ڵ����Ŀ��һ����horiCount��vertCount��
		for (int i = 0; i < proParas.DeviceSum; i++)
		{
			for (int pointIndex = 0; pointIndex < copyDeviceParas[i].adjPInCount; pointIndex++)
			{
				AdjPoint& p = copyDeviceParas[i].adjPointsIn[pointIndex];
				if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//����
				{
					horizonAxisList[curHoriIndex++] = p.pos.x + particle.position_[i * 3];
				}
				else {//����
					verticalAxisList[curVertIndex++] = p.pos.y + particle.position_[i * 3 + 1];
				}
			}
			for (int pointIndex = 0; pointIndex < copyDeviceParas[i].adjPOutCount; pointIndex++)
			{
				AdjPoint& p = copyDeviceParas[i].adjPointsOut[pointIndex];
				if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//����
				{
					horizonAxisList[curHoriIndex++] = p.pos.x + particle.position_[i * 3];
				}
				else {//����
					verticalAxisList[curVertIndex++] = p.pos.y + particle.position_[i * 3 + 1];
				}
			}
		}
		//�ֿ����2����&���ڣ�horiCount��vertCount+=2��
		horizonAxisList[curHoriIndex++] = proParas.entrancePos.x;
		verticalAxisList[curVertIndex++] = proParas.entrancePos.y;

		horizonAxisList[curHoriIndex++] = proParas.exitPos.x;
		verticalAxisList[curVertIndex++] = proParas.exitPos.y;
		//����ÿ���豸������ĸ���Χ����Ϊ�����ϰ���ķ�Χ��
		double* DeviceLowXList = new double[proParas.DeviceSum];
		double* DeviceHighXList = new double[proParas.DeviceSum];
		double* DeviceLowYList = new double[proParas.DeviceSum];
		double* DeviceHighYList = new double[proParas.DeviceSum];
		//ÿ���豸��Χ��4����
		for (int i = 0; i < particle.dim_; i += 3) {
			outSizeLength = 0.5 * copyDeviceParas[i / 3].size.x + proParas.convey2DeviceDist;
			outSizeWidth = 0.5 * copyDeviceParas[i / 3].size.y + proParas.convey2DeviceDist;
			double LowX = particle.position_[i] - outSizeLength;
			double HighX = particle.position_[i] + outSizeLength;
			double LowY = particle.position_[i + 1] - outSizeWidth;
			double HighY = particle.position_[i + 1] + outSizeWidth;

			verticalAxisList[curVertIndex++] = LowY;
			verticalAxisList[curVertIndex++] = HighY;
			horizonAxisList[curHoriIndex++] = LowX;
			horizonAxisList[curHoriIndex++] = HighX;

			//ÿ���豸���ĸ���Χ
			DeviceLowXList[i / 3] = LowX;
			DeviceHighXList[i / 3] = HighX;
			DeviceLowYList[i / 3] = LowY;
			DeviceHighYList[i / 3] = HighY;

			//��ֹ·�������豸�ڲ�
			horizonAxisList[curHoriIndex++] = particle.position_[i];
			verticalAxisList[curVertIndex++] = particle.position_[i + 1];

		}
		//��һ��������Щ��
		//����Щ������갴�մ�С��������
		//sort������Ҫ�Լ�ʵ�֣�����ʹ��cuda�⺯����
		Double_Sort(horizonAxisList, 0, proParas.horiPointCount - 1);
		Double_Sort(verticalAxisList, 0, proParas.vertPointCount - 1);
		//ֻ�������ظ��ĵ㣨�Լ�ʵ�֣�

		int uniqueHoriPCount = proParas.horiPointCount;
		int uniqueVertPCount = proParas.vertPointCount;
		int unique_end1 = Double_Unique(horizonAxisList, 0, uniqueHoriPCount - 1);
		uniqueHoriPCount = unique_end1;
		//verticalAxisList.erase(unique_end1, verticalAxisList.end());
		int unique_end2 = Double_Unique(verticalAxisList, 0, uniqueVertPCount - 1);
		uniqueVertPCount = unique_end2;
		//horizonAxisList.erase(unique_end2, horizonAxisList.end());

		//�����е��ϰ�����±�
		int barrierRowNum = 200;//��һ���̶���С�����ڴ�
		int* barrierRowIndexList = new int[barrierRowNum];
		int barrierColNum = 200;
		int* barrierColIndexList = new int[barrierColNum];
		int totalBarRowNum = 0;//��¼barrier��ʵ������Ŀ
		int totalBarColNum = 0;//��¼barrier��ʵ������Ŀ
		//vector<int> barrierRowIndexList;
		//vector<int> barrierColIndexList;

		//����Щ����ȥ���·����map��map�Ƕ�ά�ģ��൱�ڶ�ά�����
		//��һά�����ά
		//horiNum��ӦcolNum��vertNum��ӦrowNum
		int pathColNum = uniqueHoriPCount;
		int pathRowNum = uniqueVertPCount;
		APoint** pathPointMap = new APoint*[pathColNum * pathRowNum];
		
		for (int i = 0; i < pathColNum * pathRowNum; i++)
		{
			pathPointMap[i] = new APoint();
			int rowIndex = i / pathColNum;//��Ӧ�����i
			int colIndex = i % pathColNum;//��Ӧ�����j
			pathPointMap[i]->x = horizonAxisList[colIndex];
			pathPointMap[i]->y = verticalAxisList[rowIndex];
			//���������豸�����Ƿ��к�������ص��ģ�ʵ�ֱ���ϰ��㣩
			for (int k = 0; k < proParas.DeviceSum; k++)
			{
				if (pathPointMap[i]->x - DeviceLowXList[k] >= 0.01 && DeviceHighXList[k] - pathPointMap[i]->x >= 0.01
					&& pathPointMap[i]->y - DeviceLowYList[k] >= 0.01 && DeviceHighYList[k] - pathPointMap[i]->y >= 0.01)
				{
					pathPointMap[i]->type = AType::ATYPE_BARRIER;
					//�ϰ�����ͼ�е��±�
					barrierRowIndexList[totalBarRowNum++] = rowIndex;
					barrierColIndexList[totalBarColNum++] = colIndex;
				}
			}
			pathPointMap[i]->colIndex = colIndex;//��¼�µ���·����map�е��±�
			pathPointMap[i]->rowIndex = rowIndex;
		}
#pragma endregion

#pragma region Ѱ·
		auto star = new CAstar();
		star->_allPoints = pathPointMap;
		star->pointColNum = pathColNum;
		star->pointRowNum = pathRowNum;
		int beginRowIndex, beginColIndex, endRowIndex, endColIndex;

		double totalTime = 0.0;
		//�����е�·����Ϣ
		PointLink* copyPLinks = new PointLink[proParas.totalLinkSum];//��ʱÿ��·����50����
		int curLinkIndex = 0;//��ǰ��link�±�
		int totalLinkPointSum = 0;//����link�����е����Ŀ��ÿ��link�кܶ�㣩
		for (int i = 0; i < proParas.CargoTypeNum; i++)//��������
		{
			CargoType curCargoType = proParas.cargoTypeList[i];

			//ÿ�ֻ�����ܾ�������豸��Ҳ����һ��Type��Ӧ���link
			for (int j = 0; j < proParas.cargoTypeList[i].linkSum; j++)
			{
				PathDirection pathBeginDirect;
				PathDirection pathEndDirect;
				int forwardDeviceIndex, curDeviceIndex;//�豸1���豸2ID
				int forwardOutIndex, curInIndex;//����ڵ��±�
				double device1PosX, device1PosY, device2PosX, device2PosY;//�豸��Χ���ĸ���
				double initDevice1PosX, initDevice1PosY, initDevice2PosX, initDevice2PosY;//����δ���Ӱ�Χ�ߵ�����

				double deviceDistance = 0.0;//����
				double outDSizeL, inDSizeL;

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
					outDSizeL = 0.0;
					inDSizeL = pathEndDirect == PathDirection::Horizon ? (0.5 * copyDeviceParas[curDeviceIndex].size.x) : (0.5 * copyDeviceParas[curDeviceIndex].size.y);

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
						device2PosY += proParas.convey2DeviceDist;
						break;
					case PointDirect::Down:
						device2PosY -= proParas.convey2DeviceDist;
						break;
					case PointDirect::Left:
						device2PosX -= proParas.convey2DeviceDist;
						break;
					case PointDirect::Right:
						device2PosX += proParas.convey2DeviceDist;
						break;
					}
				}
				else if (curDeviceIndex == -2)//˵���ǳ���
				{
					//��ͷ�ͽ�β��ĳ�����
					pathBeginDirect = (copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].direct % 2 == 0)
						? PathDirection::Horizon : PathDirection::Vertical;
					pathEndDirect = PathDirection::Vertical;

					outDSizeL = pathBeginDirect == PathDirection::Horizon ? (0.5 * copyDeviceParas[forwardDeviceIndex].size.x) : (0.5 * copyDeviceParas[forwardDeviceIndex].size.y);
					inDSizeL = 0.0;

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


					outDSizeL = pathBeginDirect == PathDirection::Horizon ? (0.5 * copyDeviceParas[forwardDeviceIndex].size.x) : (0.5 * copyDeviceParas[forwardDeviceIndex].size.y);
					inDSizeL = pathEndDirect == PathDirection::Horizon ? (0.5 * copyDeviceParas[curDeviceIndex].size.x) : (0.5 * copyDeviceParas[curDeviceIndex].size.y);

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
				beginRowIndex = FindAxisIndex(device1PosY, verticalAxisList, uniqueVertPCount);
				beginColIndex = FindAxisIndex(device1PosX, horizonAxisList, uniqueHoriPCount);
				endRowIndex = FindAxisIndex(device2PosY, verticalAxisList, uniqueVertPCount);
				endColIndex = FindAxisIndex(device2PosX, horizonAxisList, uniqueHoriPCount);

				//�õ�·����path�ǵ�һ���ڵ�
				APoint* path = star->findWay(pathBeginDirect, beginRowIndex, beginColIndex, endRowIndex, endColIndex);
				//�����еĽ⣬ֱ���˳�
				if (path == nullptr)
				{
					particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
					return;
				}
				//����·�����㳤��
				deviceDistance = star->CalcuPathLength(path) + outDSizeL + inDSizeL;


#pragma region ����·����ֻ������ʼ�� + ·���е�ת��㣩
				//·����������
				//���·���еĵ㻹û�м򻯣��ȸ�ÿ��·�����ô�СΪ50
				Vector2* points1 = new Vector2[proParas.fixedLinkPointSum];
				int points1Index = 0;
				Vector2 endP1(initDevice2PosX, initDevice2PosY);
				points1[points1Index++] = endP1;
				APoint* copyPath = path;
				while (copyPath)
				{
					Vector2 tempP(copyPath->x, copyPath->y);
					points1[points1Index++] = tempP;
					copyPath = copyPath->parent;
				}
				Vector2 startP1(initDevice1PosX, initDevice1PosY);
				points1[points1Index++] = startP1;//points1Index��ĳһ��link��ʵ�ʵ����Ŀ
				totalLinkPointSum += points1Index;//ͳ��link�����е�ʵ�ʵ����Ŀ������֮�����ռ䣩

				PointLink pointLink1(forwardDeviceIndex, forwardOutIndex, curDeviceIndex, curInIndex, points1, points1Index);
				copyPLinks[curLinkIndex++] = pointLink1;

				//Vector2 lastP(path->x, path->y);//�����һ���㿪ʼ
				//points.push_back(lastP);
				//path = path->parent;
				//while (path)
				//{
				//	Vector2 curP(path->x, path->y);
				//	//ֻ��ת��ĵ�Żᱻ����·��
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



//��������ʱ��(�������� * ·�߳��� * ����Ч��)
				totalTime += curCargoType.totalVolume * deviceDistance * proParas.conveySpeed;
				//�����豸����ʱ��(�������� * ����Ч��)
				//totalTime += curCargoType.totalVolume * curDevice.workSpeed;

				star->resetAStar();
				//���ϰ������±��
				for (int i = 0; i < totalBarRowNum; i++)
				{
					star->_allPoints[barrierRowIndexList[i] * pathColNum + barrierColIndexList[i]]->type = AType::ATYPE_BARRIER;
				}
			}
		}
#pragma endregion

#pragma region �����ֽ��ת��Ϊ���ͻ�����(ͬʱ��ǿһ����������̾���Լ��)
		//ֱ�����ͻ���Ϣ�б�
		int tempStrConveyorList_PointSum = proParas.fixedUniqueLinkPointSum * proParas.totalLinkSum;//20����
		StraightConveyorInfo* tempStrConveyorList = new StraightConveyorInfo[tempStrConveyorList_PointSum];

		//ת�����ͻ���Ϣ�б�
		int tempCurveConveyorList_PointSum = proParas.fixedUniqueLinkPointSum * proParas.totalLinkSum;//20����
		Vector2Int* tempCurveConveyorList = new Vector2Int[tempCurveConveyorList_PointSum];
		 
		//ֻ����ת��������
		int segPathSet_PointSum = proParas.fixedLinkPointSum * proParas.totalLinkSum;//50����
		int segPathSet_CurIndex = 0;
		SegPath* segPathSet = new SegPath[segPathSet_PointSum];

		//////���������ظ���·�߶Σ���ȡsegPathSet//////
		//1.��copyPLinks������е�ŵ�seg������
		for (int i = 0; i < proParas.totalLinkSum; i++)
		{
			for (int j = copyPLinks[i].pointNum - 1; j > 0; j--)
			{
				Vector2Int p1 = Multi10000ToInt(copyPLinks[i].points[j]);
				Vector2Int p2 = Multi10000ToInt(copyPLinks[i].points[j - 1]);
				if (p1 != p2)//���겻��һ��
				{
					SegPath temp(p1, p2);
					segPathSet[segPathSet_CurIndex++] = temp;
				}
			}
		}
		//2.��seg�����������+ȥ��
		SegPath_Sort(segPathSet, 0, segPathSet_CurIndex - 1);
		//ȥ�غ�ĵ����Ŀ
		int segPathSet_UniqueSum = SegPath_Unique(segPathSet, 0, segPathSet_CurIndex - 1);

		//////����set�������set�����е� ������·�ߵĴ�ֱˮƽ��Ŀ�����Ƴ���ȣ�&�Ƿ��� ��Ϣ//////
		//��Щ��Ϣ�浽pathPointInfoMap��
		//map<Vector2Int, PointInfo> pathPointInfoMap;


		//·������Ϣmap(ÿ���������:ÿ�������Ϣ)
		int pathPointInfoMap_PointSum = segPathSet_UniqueSum * 2;//ÿ��seg��Ӧ������
		PointInfo* pathPointInfoMap = new PointInfo[pathPointInfoMap_PointSum];
		int pathPointInfoMap_CurIndex = 0;//��ǰ�±�

		//ͬ���ģ�ֻ�����������map
		//���ʵ�֣�
		//1.�ֽ�segPath�����еĵ�ŵ�PointInfo��
		for (int segIndex = 0; segIndex < segPathSet_UniqueSum; segIndex++)
		{
			if (segPathSet[segIndex].direct == PathPointDirect::Vert) {
				pathPointInfoMap[pathPointInfoMap_CurIndex++] = PointInfo(segPathSet[segIndex].p1, 1, 0, false);
				pathPointInfoMap[pathPointInfoMap_CurIndex++] = PointInfo(segPathSet[segIndex].p2, 1, 0, false);
			}
			else
			{
				pathPointInfoMap[pathPointInfoMap_CurIndex++] = PointInfo(segPathSet[segIndex].p1, 0, 1, false);
				pathPointInfoMap[pathPointInfoMap_CurIndex++] = PointInfo(segPathSet[segIndex].p2, 0, 1, false);
			}
		}
		//2.�����е��������Ŀ���ǽ�������ͬ�ĵ�ۼ���һ��
		PointInfo_Sort(pathPointInfoMap, 0, pathPointInfoMap_CurIndex - 1);
		//3.������������飬����ÿ�����vertNum��horiNum��Ŀ,˳��ȥ��
		int pathPointInfoMap_UniqueSum = PointInfo_CalcuAndUnique(pathPointInfoMap, 0, pathPointInfoMap_CurIndex - 1);
		

		//4.���ݵ��vertNum��horiNum���ж�ÿ�����Ƿ񱻱���
		for (int i = 0; i < pathPointInfoMap_UniqueSum; i++)
		{
			if ((pathPointInfoMap[i].horiDirNum == 1 && pathPointInfoMap[i].vertDirNum == 0)
				|| (pathPointInfoMap[i].horiDirNum == 0 && pathPointInfoMap[i].vertDirNum == 1)) {
				pathPointInfoMap[i].isKeep = true;
			}
			if (pathPointInfoMap[i].horiDirNum >= 1 && pathPointInfoMap[i].vertDirNum >= 1) {
				pathPointInfoMap[i].isKeep = true;
			}
		}

		//��pathPointInfoMap�еõ�tempStrConveyorList��tempCurveConveyorList
		for (int i = 0; i < proParas.totalLinkSum; i++) {
			StraightConveyorInfo tempStrInfo;
			tempStrInfo.startPos = Multi10000ToInt(copyPLinks[i].points[copyPLinks[i].pointNum - 1]);//��ͷ
			tempStrInfo.startVnum = pathPointInfoMap[tempStrInfo.startPos].vertDirNum;
			tempStrInfo.startHnum = pathPointInfoMap[tempStrInfo.startPos].horiDirNum;
			for (int j = copyPLinks[i].pointNum - 2; j >= 0; j--) {
				Vector2Int p = Multi10000ToInt(copyPLinks[i].points[j]);
				if (pathPointInfoMap[p].isKeep == true) {//���������ظ���
					//�ȸ���ֱ�����ͻ�
					if (tempStrInfo.startPos != p) {
						tempStrInfo.endPos = p;
						//if (curIterNum != 0)
						if (tempStrInfo.startPos.Distance(tempStrInfo.endPos) < proParas.conveyMinDist)
						{
							//cout << "������̫��" << endl;
							//punishValue1 = 150 * (curIterNum + 1);
							//punishValue2 = 30 * (curIterNum + 1);
							particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
							return;
						}
						tempStrInfo.endVnum = pathPointInfoMap[tempStrInfo.endPos].vertDirNum;
						tempStrInfo.endHnum = pathPointInfoMap[tempStrInfo.endPos].horiDirNum;
						//����ط�����ȥ�ص�
						//���û��set��ֻ��������+ȥ�ظ���
						tempStrConveyorList.insert(tempStrInfo);////
						tempStrInfo.startPos = p;
						tempStrInfo.startVnum = pathPointInfoMap[tempStrInfo.startPos].vertDirNum;
						tempStrInfo.startHnum = pathPointInfoMap[tempStrInfo.startPos].horiDirNum;
					}
					//ֻҪ����ʼ�յ㣬����Ҫ����ת�����ͻ�
					if (!(pathPointInfoMap[p].horiDirNum == 1 && pathPointInfoMap[p].vertDirNum == 0)
						&& !(pathPointInfoMap[p].horiDirNum == 0 && pathPointInfoMap[p].vertDirNum == 1)) {
						tempCurveConveyorList.insert(p);////
					}
				}
			}
		}
#pragma endregion

#pragma region ����Ŀ�꺯��ֵ
		//����ֱ�ߺ�ת�����ͻ���set���õ����ͻ����ܳɱ�
		double conveyorTotalCost = 0.0;
		for (StraightConveyorInfo sci : tempStrConveyorList)
		{
			conveyorTotalCost += proParas.strConveyorUnitCost * sci.startPos.Distance(sci.endPos);
		}
		conveyorTotalCost += proParas.curveConveyorUnitCost * tempCurveConveyorList.size();
		//������Ӧ��ֵ

		//particle.fitness_[0] = 1000;
		particle.fitness_[0] = totalTime;
		//particle.fitness_[0] = totalTime + punishValue1;

		//particle.fitness_[1] = CalcuTotalArea(particle, copyDeviceParas);//ռ�������
		//particle.fitness_[1] = 1000;
		particle.fitness_[1] = conveyorTotalCost;
		//particle.fitness_[1] = conveyorTotalCost + punishValue2;

		cout << particle.fitness_[0] << "," << particle.fitness_[1] << endl;

		//������Ӧ���Ƿ�����ѡ�����BestPathInfoList
		for (int i = 0; i < particle.fitnessCount; ++i) {
			if (particle.fitness_[i] < bestPathInfoList[i].curBestFitnessVal) {//��Ҫ����
				bestPathInfoList[i].inoutPoints = tempInoutPoints;
				bestPathInfoList[i].inoutPSize = proParas.inoutPointCount;
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
//˳ʱ����ת�������
Vector2 Rotate(Vector2 pointPos, Vector2 centerPos, float rotateAngle)
{
	float xx = (pointPos.x - centerPos.x) * cos(rotateAngle * (PI / 180)) + (pointPos.y - centerPos.y) * sin(rotateAngle * (PI / 180)) + centerPos.x;
	float yy = -(pointPos.x - centerPos.x) * sin(rotateAngle * (PI / 180)) + (pointPos.y - centerPos.y) * cos(rotateAngle * (PI / 180)) + centerPos.y;
	Vector2 result(xx, yy);
	return result;
}
int FindAxisIndex(double axis, const double* axisList, int axisCount)
{
	//�ö��ַ�����
	int low = 0;
	int high = axisCount - 1;
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
double CalcuTotalArea(Particle& particle, DevicePara* copyDeviceParas) {
	double area = 0;
	double min_X, min_Y, max_X, max_Y;
	min_X = min_Y = INT_MAX;
	max_X = max_Y = -INT_MAX;
	for (int i = 0; i < particle.dim_; i += 3) {
		double outSizeLength = copyDeviceParas[i / 3].size.x * 0.5 + copyDeviceParas[i / 3].spaceLength;
		double outSizeWidth = copyDeviceParas[i / 3].size.y * 0.5 + copyDeviceParas[i / 3].spaceLength;
		min_X = min(min_X, particle.position_[i] - outSizeLength);
		max_X = max(max_X, particle.position_[i] + outSizeLength);
		min_Y = min(min_Y, particle.position_[i + 1] - outSizeWidth);
		max_Y = max(max_Y, particle.position_[i + 1] + outSizeWidth);
	}
	//���������
	area = (max_X - min_X) * (max_Y - min_Y);
	return area;
}
//�ȳ���10000��Ȼ���������뵽Int
int Multi10000ToInt(double num)
{
	//ʹ���Զ����Round����
	return MyRound(num * 10000);
}
Vector2Int Multi10000ToInt(Vector2 v)
{
	return Vector2Int(Multi10000ToInt(v.x), Multi10000ToInt(v.y));
}