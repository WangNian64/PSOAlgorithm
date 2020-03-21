#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include "DevicePara.h"
#include "Tools.h"
#include "Graph.h"
using namespace std;
//�豸�ߴ�
struct Size {
	double x;
	double y;
};
//�ɱ��������
struct CostPara {
	double MatFlow;		//��������
	double UnitCost;	//��λ�������ĳɱ�
};
struct ProblemParas
{	

	int DeviceSum;					//�豸����
	DevicePara* deviceParaList;		//�豸�����б�

	//����ߴ�
	double workShopLength;
	double workShopWidth;
	
	//�ֿ��������
	Vector2 entrancePos;		
	//�ֿ��������
	Vector2 exitPos;
	CostPara** costParaArray;		//�ɱ������������(�����������������ɱ�)

	double** minDistArray;			//�豸��С��������
	DeviceRelation** deviceRelateArray;//�豸���������

	//���ϲ����б�
	int CargoTypeNum;
	CargoType* cargoTypeList;
	
	Graph deviceGraph;//�豸����ͼ�ṹ

	double conveySpeed;//���ͻ������ٶ�
	double spaceLength;//���豸�������һ��ǣ�Ϊ�˸��õľ���Լ����
	ProblemParas() {}
	ProblemParas(int deviceSum, int cargoTypeNum)
	{
		conveySpeed = 1;//���䵥λ���뵥λ���ϵ�ʱ��
		spaceLength = 0.5;

		#pragma region ����ռ�
		DeviceSum = deviceSum;
		CargoTypeNum = cargoTypeNum;
		deviceParaList = new DevicePara[DeviceSum];

		costParaArray = new CostPara * [DeviceSum];
		for (int i = 0; i < DeviceSum; i++) {
			costParaArray[i] = new CostPara[DeviceSum];
		}

		minDistArray = new double* [DeviceSum];
		for (int i = 0; i < DeviceSum; i++) {
			minDistArray[i] = new double[DeviceSum];
		}

		deviceRelateArray = new DeviceRelation * [DeviceSum];
		for (int i = 0; i < DeviceSum; i++) {
			deviceRelateArray[i] = new DeviceRelation[DeviceSum];
		}

		cargoTypeList = new CargoType[CargoTypeNum];

		#pragma endregion

		//��ȡ����
		ifstream fileIn("../../InputPara.txt");
		string line;
		if (fileIn) // �и��ļ�
		{
			#pragma region ����ߴ�	
			getline(fileIn, line);//��һ��
			getline(fileIn, line);
			vector<string> shopSizeStr = split(line, ",");
			workShopLength = atof(shopSizeStr[0].c_str());
			workShopWidth = atof(shopSizeStr[1].c_str());
			#pragma endregion

			#pragma region �ֿ��������	
			getline(fileIn, line);//��һ��
			getline(fileIn, line);
			vector<string> enterPosStr = split(line, ",");
			entrancePos.x = atof(enterPosStr[0].c_str());
			entrancePos.y = atof(enterPosStr[1].c_str());
			#pragma endregion

			#pragma region �ֿ��������	
			getline(fileIn, line);//��һ��
			getline(fileIn, line);
			vector<string> exitPosStr = split(line, ",");
			exitPos.x = atof(exitPosStr[0].c_str());
			exitPos.y = atof(exitPosStr[1].c_str());
			#pragma endregion

			#pragma region �豸��ز���
			getline(fileIn, line);//��һ��
			for (int i = 0; i < DeviceSum; i++)
			{
				getline(fileIn, line);
				vector<string> strSplit = split(line, " ");

				deviceParaList[i].ID = atoi(strSplit[0].c_str());
				deviceParaList[i].workSpeed = atof(strSplit[1].c_str());
				vector<string> sizeStr = split(strSplit[2], ",");
				deviceParaList[i].size.x = atof(sizeStr[0].c_str());
				deviceParaList[i].size.y = atof(sizeStr[1].c_str());
				//�豸��Χ�İ��ߣ�Ϊ��ʵ�־���Լ����
				deviceParaList[i].spaceLength = atof(strSplit[3].c_str());
				//�ڽӵ��λ��
				//��ʽ˵����1,2,0|2,4,0��0,1,2,����������ڡ����ڣ�2,0�ǵ�����꣩
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
							deviceParaList[i].adjPointsIn.push_back(adjPoint);
							deviceParaList[i].usableAdjPointsIn.push_back(adjPoint);
						} else {
							adjPoint.index = OutIndex++;
							deviceParaList[i].adjPointsOut.push_back(adjPoint);
							deviceParaList[i].usableAdjPointsOut.push_back(adjPoint);
						}
					}
				}
			}
			#pragma endregion

			#pragma region ���ϲ���
			getline(fileIn, line);//��һ��
			for (int i = 0; i < CargoTypeNum; i++)
			{
				getline(fileIn, line);
				vector<string> strSplit = split(line, " ");
				cargoTypeList[i].cargoName = "";
				cargoTypeList[i].deviceSum = atoi(strSplit[0].c_str());
				vector<string> deviceStrList = split(strSplit[1], "-");//����1-3-5-6�ָ�
				cargoTypeList[i].deviceList = new int[cargoTypeList[i].deviceSum];
				for (int j = 0; j < cargoTypeList[i].deviceSum; j++)
				{
					if (deviceStrList[j] == "ENTER")//˵���ǳ���
					{
						cargoTypeList[i].deviceList[j] = 0;//��ڵ������ʾ
					}
					else if (deviceStrList[j] == "EXIT")//˵���ǳ���
					{
						cargoTypeList[i].deviceList[j] = -1;//���ڵ������ʾ
					}
					else
					{
						cargoTypeList[i].deviceList[j] = atoi(deviceStrList[j].c_str());//��ͨ�豸
					}
					cout << cargoTypeList[i].deviceList[j] << ", " << endl;
				}
				cout << endl;
				cargoTypeList[i].totalVolume = atof(strSplit[2].c_str());
			}
			#pragma endregion

			#pragma region ��λ����������ɱ�����
			getline(fileIn, line);//��һ��
			for (int i = 0; i < DeviceSum; i++)
			{
				getline(fileIn, line);
				vector<string> strSplit = split(line, ",");
				for (int j = 0; j < DeviceSum; j++)
				{
					costParaArray[i][j].UnitCost = atof(strSplit[j].c_str());
				}
			}
			#pragma endregion

			#pragma region ����������
			getline(fileIn, line);//��һ��
			for (int i = 0; i < DeviceSum; i++) {
				getline(fileIn, line);
				vector<string> strSplit = split(line, ",");
				for (int j = 0; j < DeviceSum; j++)
				{
					costParaArray[i][j].MatFlow = atof(strSplit[j].c_str());
				}
			}
			#pragma endregion

			#pragma region �豸��С�������� 
			getline(fileIn, line);//��һ��
			for (int i = 0; i < DeviceSum; i++) {
				getline(fileIn, line);
				vector<string> strSplit = split(line, ",");
				for (int j = 0; j < DeviceSum; j++)
				{
					minDistArray[i][j] = atof(strSplit[j].c_str());
					if (i != j) {
						minDistArray[i][j]++;
					}
				}
			}
			#pragma endregion

			#pragma region �豸���������
			//getline(fileIn, line);//��һ��
			//for (int i = 0; i < DeviceSum; i++) {
			//	getline(fileIn, line);
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

		#pragma region �����豸ͼ�ṹ
		//�㼯
		for (int i = 0; i < DeviceSum; i++)
		{
			deviceGraph.InsertVertex(i);
		}
		//��
		for (int i = 0; i < CargoTypeNum; i++)
		{
			for (int j = 0; j < cargoTypeList[i].deviceSum - 1; j++)
			{
				int firstEdge = cargoTypeList[i].deviceList[j] - 1;//�豸���-1
				int secondEdge = cargoTypeList[i].deviceList[j + 1] - 1;
				deviceGraph.InsertEdge(firstEdge, secondEdge);
			}
		}
		deviceGraph.ShowGraph();
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
	//    this->workShopWidth = para.workShopWidth;

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
