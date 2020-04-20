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
void FitnessFunction(int curIterNum, Particle& particle, ProblemParas proParas, double* lowerBounds, double* upBounds);
double CalcuTotalArea(Particle& particle, ProblemParas proParas);
double CalcuDeviceDist(Vector2 pos1, Vector2 pos2);

int getRandomInPoint(DevicePara& device);
int getRandomOutPoint(DevicePara& device);
int FindAxisIndex(double axis, const vector<double>& axisList);

//˳ʱ����ת�������
Vector2 Rotate(Vector2 pointPos, Vector2 centerPos, float rotateAngle);
//Ĭ�ϵ���Ӧ�ȼ��㺯���������滻
void FitnessFunction(int curIterNum, Particle& particle, ProblemParas proParas, double* lowerBounds, double* upBounds)
{
	bool IsWorkable = true;//���Ƿ����
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
	//		//��ӳɱ���Ϊ��Ӧ��ֵ��������С���룩
	//		//particle.fitness_[0] += deviceDist * proParas.costParaArray[i / 2][j / 2].UnitCost * proParas.costParaArray[i / 2][j / 2].MatFlow;
	//	}
	//}

#pragma region ���һ���豸��Ϣ
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

#pragma region �����豸���򣬵����豸�ߴ�xy�ͳ��������
	for (int i = 2; i < particle.dim_; i += 3)
	{
		//doubleתint��ת��ΪDirection��Ȼ����ݳ������¼����豸�ߴ�ͳ����
		//Rotate90����ROtate270���ߴ��x��y����
		//����ڰ���˳ʱ���㣬��ת��=Direction*90(���ö�Ӧ0,90,180,270��
		DeviceDirect curDirect = (DeviceDirect)(int)particle.position_[i];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)
		{
			double temp = copyDeviceParas[i / 3].size.x;
			copyDeviceParas[i / 3].size.x = copyDeviceParas[i / 3].size.y;
			copyDeviceParas[i / 3].size.y = temp;
		}
		//���¼�����ת��ĳ��������
		//Vector2 deviceCenterPos(particle.position_[i - 2], particle.position_[i - 1]);
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
		for (AdjPoint& point : copyDeviceParas[i / 3].usableAdjPointsIn)
		{
			point.pos = Rotate(point.pos, deviceCenterPos, rotateAngle);
			newDirect = point.direct + (int)curDirect;
			point.direct = (newDirect == 4) ? (PointDirect)4 : (PointDirect)(newDirect % 4);
		}
		for (AdjPoint& point : copyDeviceParas[i / 3].usableAdjPointsOut)
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
				return;
			}
		}
	}
#pragma endregion

#pragma region ��ǰ�����ǿ��н⣬���в���

	if (IsWorkable == true)
	{
		#pragma region �Ƚ��ж������
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

		#pragma region ����豸����ڵ����겢�������	
		double alignMinDist = 1.0f;
		for (int i = -1; i < proParas.DeviceSum; i++)
		{
			//��������cargoTypeList
			//if ��һ���ǳ���Ϊi�豸���Ҳ������һ������ô�Ϳ����ó���һ�Գ���ڵ�
			for (int j = 0; j < proParas.CargoTypeNum; j++)
			{
				for (int k = 0; k < proParas.cargoTypeList[j].deviceSum - 1; k++)
				{
					//�ҵ�ĳ�������豸��i��
					if (proParas.cargoTypeList[j].deviceList[k] - 1 == i)
					{
						int outDeviceIndex = i;
						int inDeviceIndex = proParas.cargoTypeList[j].deviceList[k + 1] - 1;
						//�������1���ֿ����
						if (outDeviceIndex == -1)
						{
							for (AdjPoint& inPoint : copyDeviceParas[inDeviceIndex].adjPointsIn)
							{
								Vector2 inPointTPos(inPoint.pos.x + particle.position_[inDeviceIndex * 3],
									inPoint.pos.y + particle.position_[inDeviceIndex * 3 + 1]);
								if (inPointTPos.x!=proParas.entrancePos.x && abs(inPointTPos.x - proParas.entrancePos.x) <= alignMinDist)
								{
									//ֻ���޸�in�������޸����
									double moveLength = inPointTPos.x - proParas.entrancePos.x;
									particle.position_[inDeviceIndex * 3] -= moveLength;

								}
								else if (inPointTPos.y!=proParas.entrancePos.y && abs(inPointTPos.y - proParas.entrancePos.y) <= alignMinDist)
								{
									double moveLength = inPointTPos.y - proParas.entrancePos.y;
									particle.position_[inDeviceIndex * 3 + 1] -= moveLength;
								}
							}
						}
						else if (inDeviceIndex == -2)//�������2���ֿ����
						{
							for (AdjPoint& outPoint : copyDeviceParas[outDeviceIndex].adjPointsOut)
							{
								Vector2 outPointTPos(outPoint.pos.x + particle.position_[outDeviceIndex * 3],
									outPoint.pos.y + particle.position_[outDeviceIndex * 3 + 1]);
								if (outPointTPos.x!=proParas.exitPos.x && abs(outPointTPos.x - proParas.exitPos.x) <= alignMinDist)
								{
									//ֻ���޸�out�������޸ĳ���
									double moveLength = outPointTPos.x - proParas.exitPos.x;
									particle.position_[outDeviceIndex * 3] -= moveLength;
								} 
								else if (outPointTPos.y!=proParas.exitPos.y && abs(outPointTPos.y - proParas.exitPos.y) <= alignMinDist)
								{
									double moveLength = outPointTPos.y - proParas.exitPos.y;
									particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
								}
							}
						} 
						else//�������
						{
							for (AdjPoint& outPoint : copyDeviceParas[outDeviceIndex].adjPointsOut)
							{
								for (AdjPoint& inPoint : copyDeviceParas[inDeviceIndex].adjPointsIn)
								{
									//�ڿ������豸���������¶Ա�
									Vector2 outPointTPos(outPoint.pos.x + particle.position_[outDeviceIndex * 3],
										outPoint.pos.y + particle.position_[outDeviceIndex * 3 + 1]);
									Vector2 inPointTPos(inPoint.pos.x + particle.position_[inDeviceIndex * 3],
										inPoint.pos.y + particle.position_[inDeviceIndex * 3 + 1]);
									if (outPointTPos.x!=inPointTPos.x && abs(outPointTPos.x - inPointTPos.x) <= alignMinDist)
									{
										//x����ӽ�
										double moveLength = (outPointTPos.x - inPointTPos.x) * 0.5;
										particle.position_[outDeviceIndex * 3] -= moveLength;
										particle.position_[inDeviceIndex * 3] += moveLength;

									}
									else if (outPointTPos.y!=inPointTPos.y && abs(outPointTPos.y - inPointTPos.y) <= alignMinDist)
									{
										//y����ӽ�
										double moveLength = (outPointTPos.y - inPointTPos.y) * 0.5;
										particle.position_[outDeviceIndex * 3 + 1] -= moveLength;
										particle.position_[inDeviceIndex * 3 + 1] += moveLength;
									}
								}
							}
						}
					}
				}
			}
		}
		#pragma endregion

		//�������ڵ�ļ�������
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
#pragma region �����豸����ͳ�������깹��·����ͼ

		vector< vector<APoint*> > pathPointMap;
		vector<double> horizonAxisList;
		vector<double> verticalAxisList;
		//�ȶԳ���ڵ�ˮƽ�ʹ�ֱ���з���(ע�����ƫ����)
		for (int i = 0; i < proParas.DeviceSum; i++)
		{
			for (AdjPoint p : copyDeviceParas[i].adjPointsIn)
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
		//cout << endl;
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
			outSizeLength = 0.5 * copyDeviceParas[i / 3].size.x + proParas.spaceLength;
			//outSizeWidth = 0.5 * copyDeviceParas[i / 3].size.y + copyDeviceParas[i / 3].spaceLength;
			outSizeWidth = 0.5 * copyDeviceParas[i / 3].size.y + proParas.spaceLength;
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
					//if ((p->x > DeviceLowXList[k] && p->x < DeviceHighXList[k])
					//	&& (p->y > DeviceLowYList[k] && p->y < DeviceHighYList[k]))
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



		for (int i = 0; i < proParas.CargoTypeNum; i++)
		{
			CargoType curCargoType = proParas.cargoTypeList[i];

			for (int j = 0; j < proParas.cargoTypeList[i].deviceSum - 1; j++)//�������Ͼ������豸�б�
			{
				PathDirection pathBeginDirect;
				int forwardDeviceIndex, curDeviceIndex;//�豸1���豸2ID(������λ�ò�1)
				int forwardOutIndex, curInIndex;//����ڵ��±�
				double device1PosX, device1PosY, device2PosX, device2PosY;//�豸��Χ���ĸ���
				double initDevice1PosX, initDevice1PosY, initDevice2PosX, initDevice2PosY;//����δ���Ӱ�Χ�ߵ�����

				double deviceDistance = 0.0;//����

				forwardDeviceIndex = proParas.cargoTypeList[i].deviceList[j] - 1;
				curDeviceIndex = proParas.cargoTypeList[i].deviceList[j + 1] - 1;
				//cout << forwardDeviceIndex << ", " << curDeviceIndex << endl;
				if (forwardDeviceIndex == -1)//˵�������
				{
					pathBeginDirect = PathDirection::Vertical;
					forwardOutIndex = 0;
					device1PosX = proParas.entrancePos.x;
					device1PosY = proParas.entrancePos.y;

					curInIndex = getRandomInPoint(copyDeviceParas[curDeviceIndex]);
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
						device2PosY += proParas.spaceLength;
						break;
					case PointDirect::Down:
						//device2PosY -= copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosY -= proParas.spaceLength;
						break;
					case PointDirect::Left:
						//device2PosX -= copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosX -= proParas.spaceLength;
						break;
					case PointDirect::Right:
						//device2PosX += copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosX += proParas.spaceLength;
						break;
					}
				}
				else if (curDeviceIndex == -2)//˵���ǳ���
				{
					pathBeginDirect = PathDirection::Vertical;
					forwardOutIndex = getRandomOutPoint(copyDeviceParas[forwardDeviceIndex]);
					device1PosX = copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].pos.x + particle.position_[forwardDeviceIndex * 3];
					device1PosY = copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].pos.y + particle.position_[forwardDeviceIndex * 3 + 1];

					curInIndex = 0;
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
						device1PosY += proParas.spaceLength;
						break;
					case PointDirect::Down:
						//device1PosY -= copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosY -= proParas.spaceLength;
						break;
					case PointDirect::Left:
						//device1PosX -= copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosX -= proParas.spaceLength;
						break;
					case PointDirect::Right:
						//device1PosX += copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosX += proParas.spaceLength;
						break;
					}
				}
				else//��ͨ
				{

					forwardOutIndex = getRandomOutPoint(copyDeviceParas[forwardDeviceIndex]);
					curInIndex = getRandomInPoint(copyDeviceParas[curDeviceIndex]);
					switch (copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].direct)
					{
					case PointDirect::Up:
						pathBeginDirect = PathDirection::Vertical;
						break;
					case PointDirect::Down:
						pathBeginDirect = PathDirection::Vertical;
						break;
					case PointDirect::Left:
						pathBeginDirect = PathDirection::Horizon;
						break;
					case PointDirect::Right:
						pathBeginDirect = PathDirection::Horizon;
						break;
					}

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
						device1PosY += proParas.spaceLength;
						break;
					case PointDirect::Down:
						//device1PosY -= copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosY -= proParas.spaceLength;
						break;
					case PointDirect::Left:
						//device1PosX -= copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosX -= proParas.spaceLength;
						break;
					case PointDirect::Right:
						//device1PosX += copyDeviceParas[forwardDeviceIndex].spaceLength;
						device1PosX += proParas.spaceLength;
						break;
					}
					switch (copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].direct)
					{
					case PointDirect::Up:
						//device2PosY += copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosY += proParas.spaceLength;
						break;
					case PointDirect::Down:
						//device2PosY -= copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosY -= proParas.spaceLength;
						break;
					case PointDirect::Left:
						//device2PosX -= copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosX -= proParas.spaceLength;
						break;
					case PointDirect::Right:
						//device2PosX += copyDeviceParas[curDeviceIndex].spaceLength;
						device2PosX += proParas.spaceLength;
						break;
					}
				}
				//if (j == 0)//���
				//{
				//	//��һ���豸�ľ���Ӧ���Ǻ���ڵľ���
				//	//���+һ���豸����ڵ�
				//	curDeviceIndex = proParas.cargoTypeList[i].deviceList[j] - 1;
				//	curInIndex = getRandomInPoint(copyDeviceParas[curDeviceIndex]);
				//	device1PosX = proParas.entrancePos.x;
				//	device1PosY = proParas.entrancePos.y;
				//	device2PosX = copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.x + particle.position_[curDeviceIndex * 3];
				//	device2PosY = copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.y + particle.position_[curDeviceIndex * 3 + 1];

				//	initDevice1PosX = device1PosX;
				//	initDevice1PosY = device1PosY;
				//	initDevice2PosX = device2PosX;
				//	initDevice2PosY = device2PosY;
				//	//�õ��豸��Χ�ĵ�
				//	switch (copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].direct)
				//	{
				//	case PointDirect::Up:
				//		device2PosY += copyDeviceParas[curDeviceIndex].spaceLength;
				//		break;
				//	case PointDirect::Down:
				//		device2PosY -= copyDeviceParas[curDeviceIndex].spaceLength;
				//		break;
				//	case PointDirect::Left:
				//		device2PosX -= copyDeviceParas[curDeviceIndex].spaceLength;
				//		break;
				//	case PointDirect::Right:
				//		device2PosX += copyDeviceParas[curDeviceIndex].spaceLength;
				//		break;
				//	}
				//}
				//else
				//{
				//	forwardDeviceIndex = proParas.cargoTypeList[i].deviceList[j - 1] - 1;
				//	curDeviceIndex = proParas.cargoTypeList[i].deviceList[j] - 1;
				//	forwardOutIndex = getRandomOutPoint(copyDeviceParas[forwardDeviceIndex]);
				//	curInIndex = getRandomInPoint(copyDeviceParas[curDeviceIndex]);

				//	device1PosX = copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].pos.x + particle.position_[forwardDeviceIndex * 3];
				//	device1PosY = copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].pos.y + particle.position_[forwardDeviceIndex * 3 + 1];
				//	device2PosX = copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.x + particle.position_[curDeviceIndex * 3];
				//	device2PosY = copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].pos.y + particle.position_[curDeviceIndex * 3 + 1];

				//	initDevice1PosX = device1PosX;
				//	initDevice1PosY = device1PosY;
				//	initDevice2PosX = device2PosX;
				//	initDevice2PosY = device2PosY;
				//	//�õ��豸��Χ�ĵ�
				//	switch (copyDeviceParas[forwardDeviceIndex].adjPointsOut[forwardOutIndex].direct)
				//	{
				//	case PointDirect::Up:
				//		device1PosY += copyDeviceParas[forwardDeviceIndex].spaceLength;
				//		break;
				//	case PointDirect::Down:
				//		device1PosY -= copyDeviceParas[forwardDeviceIndex].spaceLength;
				//		break;
				//	case PointDirect::Left:
				//		device1PosX -= copyDeviceParas[forwardDeviceIndex].spaceLength;
				//		break;
				//	case PointDirect::Right:
				//		device1PosX += copyDeviceParas[forwardDeviceIndex].spaceLength;
				//		break;
				//	}
				//	switch (copyDeviceParas[curDeviceIndex].adjPointsIn[curInIndex].direct)
				//	{
				//	case PointDirect::Up:
				//		device2PosY += copyDeviceParas[curDeviceIndex].spaceLength;
				//		break;
				//	case PointDirect::Down:
				//		device2PosY -= copyDeviceParas[curDeviceIndex].spaceLength;
				//		break;
				//	case PointDirect::Left:
				//		device2PosX -= copyDeviceParas[curDeviceIndex].spaceLength;
				//		break;
				//	case PointDirect::Right:
				//		device2PosX += copyDeviceParas[curDeviceIndex].spaceLength;
				//		break;
				//	}
				//}

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
				if (path == nullptr)
				{
					cout << endl;
				}
				//����·�����㳤��
				deviceDistance = star->CalcuPathLength(path);
				//·����������
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

		//������Ӧ��ֵ
		particle.fitness_[0] = totalTime;
		//particle.fitness_[1] = CalcuTotalArea(particle, proParas);
		particle.fitness_[1] = 100;//�������˳���֮�����û��������

#pragma region ����
		//delete[] copyDeviceParas;
		//delete[] DeviceLowXList;
		//delete[] DeviceHighXList;
		//delete[] DeviceLowYList;
		//delete[] DeviceHighYList;
		//vector<int>().swap(barrierRowIndexs);
		//vector<int>().swap(barrierColIndexs);
		//vector< vector<APoint*> >().swap(pathPointMap);//�����ͷ��ڴ��ǲ�����
		//for (int i = 0; i < pathPointMap.size(); i++)
		//{
		//	for (int j = 0; j < pathPointMap[i].size(); j++)
		//	{
		//		delete pathPointMap[i][j];
		//	}
		//}

		//vector<double>().swap(horizonAxisList);
		//vector<double>().swap(verticalAxisList);
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
int getRandomInPoint(DevicePara& device)
{
	if (device.usableAdjPointsIn.size() == 0) {
		return -1;
	}
	int randomIndex = rand() % device.usableAdjPointsIn.size();//�������С��
	int resultIndex = device.usableAdjPointsIn[randomIndex].index;
	device.usableAdjPointsIn.erase(device.usableAdjPointsIn.begin() + randomIndex);
	return resultIndex;
}
int getRandomOutPoint(DevicePara& device)
{
	if (device.usableAdjPointsOut.size() == 0) {
		return -1;
	}
	int randomIndex = rand() % device.usableAdjPointsOut.size();//�������С��
	int resultIndex = device.usableAdjPointsOut[randomIndex].index;
	device.usableAdjPointsOut.erase(device.usableAdjPointsOut.begin() + randomIndex);
	return resultIndex;
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
