#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include "DevicePara.h"
#include "Tools.h"
#include "Graph.h"
using namespace std;
//设备尺寸
struct Size {
	double x;
	double y;
};
//成本计算参数
struct CostPara {
	double MatFlow;		//物流总量
	double UnitCost;	//单位物流量的成本
};
struct ProblemParas
{	

	int DeviceSum;					//设备总数
	DevicePara* deviceParaList;		//设备参数列表

	//车间尺寸
	double workShopLength;
	double workShopWidth;
	
	//仓库入口坐标
	Vector2 entrancePos;		
	//仓库出口坐标
	Vector2 exitPos;
	CostPara** costParaArray;		//成本计算参数数组(包括物流量和物流成本)

	double** minDistArray;			//设备最小距离数组
	DeviceRelation** deviceRelateArray;//设备相关性数组

	//物料参数列表
	int CargoTypeNum;
	CargoType* cargoTypeList;
	
	Graph deviceGraph;//设备连的图结构

	double conveySpeed;//输送机输送速度
	double spaceLength;//给设备外层再套一层壳（为了更好的距离约束）
	ProblemParas() {}
	ProblemParas(int deviceSum, int cargoTypeNum)
	{
		conveySpeed = 1;//运输单位距离单位物料的时间
		spaceLength = 0.5;

		#pragma region 分配空间
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

		//读取参数
		ifstream fileIn("../../InputPara.txt");
		string line;
		if (fileIn) // 有该文件
		{
			#pragma region 车间尺寸	
			getline(fileIn, line);//空一行
			getline(fileIn, line);
			vector<string> shopSizeStr = split(line, ",");
			workShopLength = atof(shopSizeStr[0].c_str());
			workShopWidth = atof(shopSizeStr[1].c_str());
			#pragma endregion

			#pragma region 仓库入口坐标	
			getline(fileIn, line);//空一行
			getline(fileIn, line);
			vector<string> enterPosStr = split(line, ",");
			entrancePos.x = atof(enterPosStr[0].c_str());
			entrancePos.y = atof(enterPosStr[1].c_str());
			#pragma endregion

			#pragma region 仓库出口坐标	
			getline(fileIn, line);//空一行
			getline(fileIn, line);
			vector<string> exitPosStr = split(line, ",");
			exitPos.x = atof(exitPosStr[0].c_str());
			exitPos.y = atof(exitPosStr[1].c_str());
			#pragma endregion

			#pragma region 设备相关参数
			getline(fileIn, line);//空一行
			for (int i = 0; i < DeviceSum; i++)
			{
				getline(fileIn, line);
				vector<string> strSplit = split(line, " ");

				deviceParaList[i].ID = atoi(strSplit[0].c_str());
				deviceParaList[i].workSpeed = atof(strSplit[1].c_str());
				vector<string> sizeStr = split(strSplit[2], ",");
				deviceParaList[i].size.x = atof(sizeStr[0].c_str());
				deviceParaList[i].size.y = atof(sizeStr[1].c_str());
				//设备周围的包边（为了实现距离约束）
				deviceParaList[i].spaceLength = atof(strSplit[3].c_str());
				//邻接点的位置
				//格式说明：1,2,0|2,4,0（0,1,2,代表这是入口、出口，2,0是点的坐标）
				if (strSplit[4] != "null")
				{
					vector<string> adjStrList = split(strSplit[4], "|");//分离出每个点
					int InIndex, OutIndex;
					InIndex = OutIndex = 0;
					for (int k = 0; k < adjStrList.size(); k++)
					{
						vector<string> adjStr = split(adjStrList[k], ",");//再次分离
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

			#pragma region 物料参数
			getline(fileIn, line);//空一行
			for (int i = 0; i < CargoTypeNum; i++)
			{
				getline(fileIn, line);
				vector<string> strSplit = split(line, " ");
				cargoTypeList[i].cargoName = "";
				cargoTypeList[i].deviceSum = atoi(strSplit[0].c_str());
				vector<string> deviceStrList = split(strSplit[1], "-");//遇到1-3-5-6分割
				cargoTypeList[i].deviceList = new int[cargoTypeList[i].deviceSum];
				for (int j = 0; j < cargoTypeList[i].deviceSum; j++)
				{
					if (deviceStrList[j] == "ENTER")//说明是出口
					{
						cargoTypeList[i].deviceList[j] = 0;//入口的特殊表示
					}
					else if (deviceStrList[j] == "EXIT")//说明是出口
					{
						cargoTypeList[i].deviceList[j] = -1;//出口的特殊表示
					}
					else
					{
						cargoTypeList[i].deviceList[j] = atoi(deviceStrList[j].c_str());//普通设备
					}
					cout << cargoTypeList[i].deviceList[j] << ", " << endl;
				}
				cout << endl;
				cargoTypeList[i].totalVolume = atof(strSplit[2].c_str());
			}
			#pragma endregion

			#pragma region 单位距离的物流成本数组
			getline(fileIn, line);//空一行
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

			#pragma region 物流量数组
			getline(fileIn, line);//空一行
			for (int i = 0; i < DeviceSum; i++) {
				getline(fileIn, line);
				vector<string> strSplit = split(line, ",");
				for (int j = 0; j < DeviceSum; j++)
				{
					costParaArray[i][j].MatFlow = atof(strSplit[j].c_str());
				}
			}
			#pragma endregion

			#pragma region 设备最小距离数组 
			getline(fileIn, line);//空一行
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

			#pragma region 设备相关性数组
			//getline(fileIn, line);//空一行
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
		else // 没有该文件
		{
			cout << "no such file" << endl;
		}

		#pragma region 构造设备图结构
		//点集
		for (int i = 0; i < DeviceSum; i++)
		{
			deviceGraph.InsertVertex(i);
		}
		//边
		for (int i = 0; i < CargoTypeNum; i++)
		{
			for (int j = 0; j < cargoTypeList[i].deviceSum - 1; j++)
			{
				int firstEdge = cargoTypeList[i].deviceList[j] - 1;//设备序号-1
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

	//	//车间尺寸
	//	this->workShopLength = para.workShopLength;
	//    this->workShopWidth = para.workShopWidth;

	//	//仓库入口坐标
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
	//	//物料参数列表
	//	this->CargoTypeNum = para.CargoTypeNum;
	//	this->cargoTypeList = new CargoType[this->CargoTypeNum];

	//	this->deviceGraph = para.deviceGraph;//设备连的图结构

	//	this->conveySpeed = para.conveySpeed;//输送机输送速度
	//	this->spaceLength = para.spaceLength;//给设备外层再套一层壳（为了更好的距离约束）
	//}
};
