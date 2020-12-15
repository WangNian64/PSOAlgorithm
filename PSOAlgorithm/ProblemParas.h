#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include "DevicePara.h"
#include "Tools.h"
#include "Graph.h"
using namespace std;
//�ɱ��������
struct CostPara {
	double MatFlow;		//��������
	double UnitCost;	//��λ�������ĳɱ�
};
struct ProblemParas
{	
	int DeviceSum;						//�豸����
	DevicePara* deviceParaList;			//�豸�����б�

	int inoutPointCount;				//����ڵ������Ŀ

	int horiPointCount = 0;				//δȥ��ǰ����ˮƽ����ĵ����Ŀ
	int vertPointCount = 0;				//δȥ��ǰ���д�ֱ����ĵ����Ŀ

	double workShopLength;				//���䳤��
	double workShopWidth;				//������

	Vector2 entrancePos;				//�ֿ��������	
	Vector2 exitPos;					//�ֿ��������

	//���ϲ����б�
	int CargoTypeNum;					//����������Ŀ
	CargoType* cargoTypeList;			//���������б�
	int totalLinkSum;					//�ܵ���������Ŀ
	
	int fixedLinkPointSum = 50;			//ÿһ��link�Ĺ̶�����ĿΪ50(û��ȥ��֮ǰ��)
	int fixedUniqueLinkPointSum = 20;	//ȥ�غ��ÿһ��link�Ĺ̶�����ĿΪ20
	//���ͻ�����
	double convey2DeviceDist;//���ͻ����豸�ľ��루Ѱ·��ʱ��Ҫ���ǣ�
	double conveyWidth;//���ͻ����
	double conveyMinLength;//���ͻ���̳���
	double conveySpeed;//���ͻ������ٶ�
	double strConveyorUnitCost;//��λֱ�����ͻ��ɱ�
	double curveConveyorUnitCost;//����ת�����ͻ��ɱ�

	double conveyMinDist;//������������֮�����̾���
	//��ǰ�Ĳ���
	//CostPara** costParaArray;		//�ɱ������������(�����������������ɱ�)
	//double** minDistArray;			//�豸��С��������
	//DeviceRelation** deviceRelateArray;//�豸���������

	ProblemParas() {}
	ProblemParas(ifstream& inputFile)
	{
		//��ȡ����
		string line;
		if (inputFile) // �и��ļ�
		{
			#pragma region �⼸������Ŀǰ����
			//costParaArray = new CostPara * [DeviceSum];
			//for (int i = 0; i < DeviceSum; i++) {
			//	costParaArray[i] = new CostPara[DeviceSum];
			//}

			//minDistArray = new double* [DeviceSum];
			//for (int i = 0; i < DeviceSum; i++) {
			//	minDistArray[i] = new double[DeviceSum];
			//}

			//deviceRelateArray = new DeviceRelation * [DeviceSum];
			//for (int i = 0; i < DeviceSum; i++) {
			//	deviceRelateArray[i] = new DeviceRelation[DeviceSum];
			//}
			#pragma endregion

			#pragma region ����ߴ�	
			getline(inputFile, line);//��һ��
			getline(inputFile, line);
			vector<string> shopSizeStr = split(line, ",");
			workShopLength = atof(shopSizeStr[0].c_str());
			workShopWidth = atof(shopSizeStr[1].c_str());
			#pragma endregion

			#pragma region �ֿ��������	
			getline(inputFile, line);//��һ��
			getline(inputFile, line);
			vector<string> enterPosStr = split(line, ",");
			entrancePos.x = atof(enterPosStr[0].c_str());
			entrancePos.y = atof(enterPosStr[1].c_str());
			#pragma endregion

			#pragma region �ֿ��������	
			getline(inputFile, line);//��һ��
			getline(inputFile, line);
			vector<string> exitPosStr = split(line, ",");
			exitPos.x = atof(exitPosStr[0].c_str());
			exitPos.y = atof(exitPosStr[1].c_str());
			#pragma endregion

			#pragma region �豸��ز���
			getline(inputFile, line);//��һ��
			getline(inputFile, line);
			DeviceSum = atoi(line.c_str());//�豸��Ŀ

			deviceParaList = new DevicePara[DeviceSum];
			
			inoutPointCount = 0;
			for (int i = 0; i < DeviceSum; i++)
			{
				//��temp vector��ת
				vector<AdjPoint> tempAdjPInList;
				vector<AdjPoint> tempAdjPOutList;

				getline(inputFile, line);
				vector<string> strSplit = split(line, " ");

				deviceParaList[i].ID = atoi(strSplit[0].c_str());
				deviceParaList[i].workSpeed = atof(strSplit[1].c_str());
				vector<string> sizeStr = split(strSplit[2], ",");
				deviceParaList[i].size.x = atof(sizeStr[0].c_str());
				deviceParaList[i].size.y = atof(sizeStr[1].c_str());

				deviceParaList[i].spaceLength = atof(strSplit[3].c_str());
				//�ڽӵ��λ��
				if (strSplit[4] != "null")
				{
					vector<string> adjStrList = split(strSplit[4], "|");//�����ÿ����
					int InIndex, OutIndex;
					InIndex = OutIndex = 0;
					for (int k = 0; k < adjStrList.size(); k++)
					{
						vector<string> adjStr = split(adjStrList[k], ",");//�ٴη���
						AdjPoint adjPoint;
						adjPoint.direct = adjPoint.GetDirect(adjStr[0]);
						adjPoint.inoutType = (InoutType)(atoi(adjStr[1].c_str()) - 1);
						adjPoint.pos.x = atof(adjStr[2].c_str());
						adjPoint.pos.y = atof(adjStr[3].c_str());
						if (adjPoint.inoutType == 0) {
							adjPoint.index = InIndex++;
							tempAdjPInList.push_back(adjPoint);
						} else {
							adjPoint.index = OutIndex++;
							tempAdjPOutList.push_back(adjPoint);
						}
					}
					//��temp�����ݴ浽*������
					deviceParaList[i].adjPInCount = tempAdjPInList.size();
					deviceParaList[i].adjPointsIn = new AdjPoint[deviceParaList[i].adjPInCount];
					for (int index = 0; index < deviceParaList[i].adjPInCount; index++) 
					{
						deviceParaList[i].adjPointsIn[index] = tempAdjPInList[index];
					}


					deviceParaList[i].adjPOutCount = tempAdjPOutList.size();
					deviceParaList[i].adjPointsOut = new AdjPoint[deviceParaList[i].adjPOutCount];
					for (int index = 0; index < deviceParaList[i].adjPOutCount; index++)
					{
						deviceParaList[i].adjPointsOut[index] = tempAdjPOutList[index];
					}

					//�ۼ�inoutPoint��Ŀ
					inoutPointCount = inoutPointCount + deviceParaList[i].adjPInCount + deviceParaList[i].adjPOutCount;
				}
			}


			#pragma region ����ˮƽ�ʹ�ֱ�����Ŀ��δ�����غϵĵ㣩
			//���ˮƽ�ʹ�ֱ����ڵ����Ŀ��һ����horiCount��vertCount��
			for (int i = 0; i < DeviceSum; i++)
			{
				for (int pointIndex = 0; pointIndex < deviceParaList[i].adjPInCount; pointIndex++)
				{
					AdjPoint& p = deviceParaList[i].adjPointsIn[pointIndex];
					if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//����
					{
						horiPointCount++;
					}
					else {//����
						vertPointCount++;
					}
				}
				for (int pointIndex = 0; pointIndex < deviceParaList[i].adjPOutCount; pointIndex++)
				{
					AdjPoint& p = deviceParaList[i].adjPointsOut[pointIndex];
					if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//����
					{
						horiPointCount++;
					}
					else {//����
						vertPointCount++;
					}
				}
			}
			//�ֿ����&����2���㣨horiCount��vertCount+=2��
			horiPointCount += 2;
			vertPointCount += 2;
			//ÿ���豸������ĸ���Χ
			horiPointCount = horiPointCount + 4 * DeviceSum;
			vertPointCount = vertPointCount + 4 * DeviceSum;
			#pragma endregion

			
			#pragma endregion

			#pragma region �����߲���
			getline(inputFile, line);//��һ��
			getline(inputFile, line);
			vector<string> conveyInfoStr = split(line, " ");
			conveyWidth = atof(conveyInfoStr[0].c_str());
			conveySpeed = atof(conveyInfoStr[1].c_str());
			conveyMinLength = atof(conveyInfoStr[2].c_str());
			convey2DeviceDist = atof(conveyInfoStr[3].c_str());
			strConveyorUnitCost = atof(conveyInfoStr[4].c_str());
			curveConveyorUnitCost = atof(conveyInfoStr[5].c_str());
			conveyMinDist = conveyMinLength + conveyWidth;
			#pragma endregion

			#pragma region ���ϲ���
			getline(inputFile, line);//��һ��
			getline(inputFile, line);
			CargoTypeNum = atoi(line.c_str());//����������Ŀ

			cargoTypeList = new CargoType[CargoTypeNum];
			for (int i = 0; i < CargoTypeNum; i++)
			{
				getline(inputFile, line);
				vector<string> strSplit = split(line, " ");
				cargoTypeList[i].cargoName = "";
				cargoTypeList[i].linkSum = atoi(strSplit[0].c_str());
				cargoTypeList[i].deviceSum = cargoTypeList[i].linkSum + 1;
				cargoTypeList[i].deviceLinkList = new DeviceLink[cargoTypeList[i].linkSum];

				totalLinkSum += cargoTypeList[i].linkSum;//�ۼ�
				for (int j = 0; j < cargoTypeList[i].linkSum; j++)
				{
					vector<string> deviceLinkStr = split(strSplit[j + 1], "-");
					//��Ϊ��ںͳ���
					if (deviceLinkStr[0] == "ENTER")//˵���ǲֿ����
					{
						cargoTypeList[i].deviceLinkList[j].outDeviceIndex = -1;
					}
					else
					{
						vector<string> devicePointStr = split(deviceLinkStr[0], ",");
						cargoTypeList[i].deviceLinkList[j].outDeviceIndex = atoi(devicePointStr[0].c_str()) - 1;
						cargoTypeList[i].deviceLinkList[j].outPointIndex = atoi(devicePointStr[1].c_str()) - 1;
					}
					if (deviceLinkStr[1] == "EXIT")//˵���ǲֿ����
					{
						cargoTypeList[i].deviceLinkList[j].inDeviceIndex = -2;
					}
					else
					{
						vector<string> devicePointStr = split(deviceLinkStr[1], ",");
						cargoTypeList[i].deviceLinkList[j].inDeviceIndex = atoi(devicePointStr[0].c_str()) - 1;
						cargoTypeList[i].deviceLinkList[j].inPointIndex = atoi(devicePointStr[1].c_str()) - 1;
					}
					//cout << cargoTypeList[i].deviceLinkList[j].outDeviceIndex << "," << cargoTypeList[i].deviceLinkList[j].outPointIndex
					//	<< "," << cargoTypeList[i].deviceLinkList[j].inDeviceIndex << "," << cargoTypeList[i].deviceLinkList[j].inPointIndex
					//	<< endl;
				}
				cargoTypeList[i].totalVolume = atof(strSplit[strSplit.size() - 1].c_str());
				//cout << endl;
			}
			//cout << endl;
			#pragma endregion




			#pragma region ��λ����������ɱ�����
			//getline(inputFile, line);//��һ��
			//for (int i = 0; i < DeviceSum; i++)
			//{
			//	getline(inputFile, line);
			//	vector<string> strSplit = split(line, ",");
			//	for (int j = 0; j < DeviceSum; j++)
			//	{
			//		costParaArray[i][j].UnitCost = atof(strSplit[j].c_str());
			//	}
			//}
			#pragma endregion

			#pragma region ����������
			//getline(inputFile, line);//��һ��
			//for (int i = 0; i < DeviceSum; i++) {
			//	getline(inputFile, line);
			//	vector<string> strSplit = split(line, ",");
			//	for (int j = 0; j < DeviceSum; j++)
			//	{
			//		costParaArray[i][j].MatFlow = atof(strSplit[j].c_str());
			//	}
			//}
			#pragma endregion

			#pragma region �豸��С�������� 
			//getline(inputFile, line);//��һ��
			//for (int i = 0; i < DeviceSum; i++) {
			//	getline(inputFile, line);
			//	vector<string> strSplit = split(line, ",");
			//	for (int j = 0; j < DeviceSum; j++)
			//	{
			//		minDistArray[i][j] = atof(strSplit[j].c_str());
			//	}
			//}
			#pragma endregion

			#pragma region �豸�������������
			//getline(inputFile, line);//��һ��
			//for (int i = 0; i < DeviceSum; i++) {
			//	getline(inputFile, line);
			//	vector<string> strSplit = split(line, ",");
			//	for (int j = 0; j < DeviceSum; j++)
			//	{
			//		deviceRelateArray[i][j] = (DeviceRelation)atoi(strSplit[j].c_str());
			//	}
			//}
			#pragma endregion

		}
		else // û�и��ļ�
		{
			cout << "no such file" << endl;
		}

		#pragma region �����豸ͼ�ṹ(�ƺ�û��ʲô��)
		////�㼯
		//for (int i = 0; i < DeviceSum; i++)
		//{
		//	deviceGraph.InsertVertex(i);
		//}
		////��
		//for (int i = 0; i < CargoTypeNum; i++)
		//{
		//	for (int j = 0; j < cargoTypeList[i].deviceSum - 1; j++)
		//	{
		//		int firstEdge = cargoTypeList[i].deviceList[j] - 1;//�豸���-1
		//		int secondEdge = cargoTypeList[i].deviceList[j + 1] - 1;
		//		deviceGraph.InsertEdge(firstEdge, secondEdge);
		//	}
		//}
		//deviceGraph.ShowGraph();
		#pragma endregion
	
	}

	//ProblemParas(const ProblemParas & para)
	//{
	//	this->DeviceSum = para.DeviceSum;
	//	this->deviceParaList = new DevicePara[this->DeviceSum];
	//	for (int i = 0; i < DeviceSum; i++)
	//	{
	//		this->deviceParaList[i] = para.deviceParaList[i];
	//	}
	//	//����ߴ�
	//	this->workShopLength = para.workShopLength;
	//  this->workShopWidth = para.workShopWidth;
	//	//�ֿ��������
	//	this->entrancePos = para.entrancePos;
	//	this->costParaArray = new CostPara * [this->DeviceSum];
	//	this->minDistArray = new double * [this->DeviceSum];
	//	this->deviceRelateArray = new DeviceRelation * [this->DeviceSum];
	//	for (int i = 0; i < this->DeviceSum; i++)
	//	{
	//		this->costParaArray[i] = new CostPara[this->DeviceSum];
	//		this->minDistArray[i] = new double[this->DeviceSum];
	//		this->deviceRelateArray[i] = new DeviceRelation[this->DeviceSum];
	//	}
	//	for (int i = 0; i < this->DeviceSum; i++)
	//	{
	//		for (int j = 0; j < this->DeviceSum; j++)
	//		{
	//			this->costParaArray[i][j] = para.costParaArray[i][j];
	//			this->minDistArray[i][j] = para.minDistArray[i][j];
	//			this->deviceRelateArray[i][j] = para.deviceRelateArray[i][j];
	//		}
	//	}
	//	//���ϲ����б�
	//	this->CargoTypeNum = para.CargoTypeNum;
	//	this->cargoTypeList = new CargoType[this->CargoTypeNum];

	//	this->deviceGraph = para.deviceGraph;//�豸����ͼ�ṹ

	//	this->conveySpeed = para.conveySpeed;//���ͻ������ٶ�
	//	this->spaceLength = para.spaceLength;//���豸�������һ��ǣ�Ϊ�˸��õľ���Լ����
	//}
};
