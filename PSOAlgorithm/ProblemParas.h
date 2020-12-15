#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include "DevicePara.h"
#include "Tools.h"
#include "Graph.h"
using namespace std;
//成本计算参数
struct CostPara {
	double MatFlow;		//物流总量
	double UnitCost;	//单位物流量的成本
};
struct ProblemParas
{	
	int DeviceSum;						//设备总数
	DevicePara* deviceParaList;			//设备参数列表

	int inoutPointCount;				//出入口点的总数目

	int horiPointCount = 0;				//未去重前所有水平方向的点的数目
	int vertPointCount = 0;				//未去重前所有垂直方向的点的数目

	double workShopLength;				//车间长度
	double workShopWidth;				//车间宽度

	Vector2 entrancePos;				//仓库入口坐标	
	Vector2 exitPos;					//仓库出口坐标

	//物料参数列表
	int CargoTypeNum;					//货物类型数目
	CargoType* cargoTypeList;			//货物类型列表
	int totalLinkSum;					//总的连接线数目
	
	int fixedLinkPointSum = 50;			//每一条link的固定点数目为50(没有去重之前的)
	int fixedUniqueLinkPointSum = 20;	//去重后的每一条link的固定点数目为20
	//输送机参数
	double convey2DeviceDist;//输送机到设备的距离（寻路的时候要考虑）
	double conveyWidth;//输送机宽度
	double conveyMinLength;//输送机最短长度
	double conveySpeed;//输送机输送速度
	double strConveyorUnitCost;//单位直线输送机成本
	double curveConveyorUnitCost;//单个转弯输送机成本

	double conveyMinDist;//输送线两个点之间的最短距离
	//以前的参数
	//CostPara** costParaArray;		//成本计算参数数组(包括物流量和物流成本)
	//double** minDistArray;			//设备最小距离数组
	//DeviceRelation** deviceRelateArray;//设备相关性数组

	ProblemParas() {}
	ProblemParas(ifstream& inputFile)
	{
		//读取参数
		string line;
		if (inputFile) // 有该文件
		{
			#pragma region 这几个参数目前不用
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

			#pragma region 车间尺寸	
			getline(inputFile, line);//空一行
			getline(inputFile, line);
			vector<string> shopSizeStr = split(line, ",");
			workShopLength = atof(shopSizeStr[0].c_str());
			workShopWidth = atof(shopSizeStr[1].c_str());
			#pragma endregion

			#pragma region 仓库入口坐标	
			getline(inputFile, line);//空一行
			getline(inputFile, line);
			vector<string> enterPosStr = split(line, ",");
			entrancePos.x = atof(enterPosStr[0].c_str());
			entrancePos.y = atof(enterPosStr[1].c_str());
			#pragma endregion

			#pragma region 仓库出口坐标	
			getline(inputFile, line);//空一行
			getline(inputFile, line);
			vector<string> exitPosStr = split(line, ",");
			exitPos.x = atof(exitPosStr[0].c_str());
			exitPos.y = atof(exitPosStr[1].c_str());
			#pragma endregion

			#pragma region 设备相关参数
			getline(inputFile, line);//空一行
			getline(inputFile, line);
			DeviceSum = atoi(line.c_str());//设备数目

			deviceParaList = new DevicePara[DeviceSum];
			
			inoutPointCount = 0;
			for (int i = 0; i < DeviceSum; i++)
			{
				//用temp vector中转
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
				//邻接点的位置
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
							tempAdjPInList.push_back(adjPoint);
						} else {
							adjPoint.index = OutIndex++;
							tempAdjPOutList.push_back(adjPoint);
						}
					}
					//将temp的数据存到*数组中
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

					//累加inoutPoint数目
					inoutPointCount = inoutPointCount + deviceParaList[i].adjPInCount + deviceParaList[i].adjPOutCount;
				}
			}


			#pragma region 计算水平和垂直点的数目（未考虑重合的点）
			//求出水平和垂直出入口点的数目（一部分horiCount和vertCount）
			for (int i = 0; i < DeviceSum; i++)
			{
				for (int pointIndex = 0; pointIndex < deviceParaList[i].adjPInCount; pointIndex++)
				{
					AdjPoint& p = deviceParaList[i].adjPointsIn[pointIndex];
					if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//上下
					{
						horiPointCount++;
					}
					else {//左右
						vertPointCount++;
					}
				}
				for (int pointIndex = 0; pointIndex < deviceParaList[i].adjPOutCount; pointIndex++)
				{
					AdjPoint& p = deviceParaList[i].adjPointsOut[pointIndex];
					if (p.direct == PointDirect::Up || p.direct == PointDirect::Down)//上下
					{
						horiPointCount++;
					}
					else {//左右
						vertPointCount++;
					}
				}
			}
			//仓库入口&出口2个点（horiCount和vertCount+=2）
			horiPointCount += 2;
			vertPointCount += 2;
			//每个设备坐标的四个范围
			horiPointCount = horiPointCount + 4 * DeviceSum;
			vertPointCount = vertPointCount + 4 * DeviceSum;
			#pragma endregion

			
			#pragma endregion

			#pragma region 输送线参数
			getline(inputFile, line);//空一行
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

			#pragma region 物料参数
			getline(inputFile, line);//空一行
			getline(inputFile, line);
			CargoTypeNum = atoi(line.c_str());//物料类型数目

			cargoTypeList = new CargoType[CargoTypeNum];
			for (int i = 0; i < CargoTypeNum; i++)
			{
				getline(inputFile, line);
				vector<string> strSplit = split(line, " ");
				cargoTypeList[i].cargoName = "";
				cargoTypeList[i].linkSum = atoi(strSplit[0].c_str());
				cargoTypeList[i].deviceSum = cargoTypeList[i].linkSum + 1;
				cargoTypeList[i].deviceLinkList = new DeviceLink[cargoTypeList[i].linkSum];

				totalLinkSum += cargoTypeList[i].linkSum;//累加
				for (int j = 0; j < cargoTypeList[i].linkSum; j++)
				{
					vector<string> deviceLinkStr = split(strSplit[j + 1], "-");
					//分为入口和出口
					if (deviceLinkStr[0] == "ENTER")//说明是仓库入口
					{
						cargoTypeList[i].deviceLinkList[j].outDeviceIndex = -1;
					}
					else
					{
						vector<string> devicePointStr = split(deviceLinkStr[0], ",");
						cargoTypeList[i].deviceLinkList[j].outDeviceIndex = atoi(devicePointStr[0].c_str()) - 1;
						cargoTypeList[i].deviceLinkList[j].outPointIndex = atoi(devicePointStr[1].c_str()) - 1;
					}
					if (deviceLinkStr[1] == "EXIT")//说明是仓库出口
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




			#pragma region 单位距离的物流成本数组
			//getline(inputFile, line);//空一行
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

			#pragma region 物流量数组
			//getline(inputFile, line);//空一行
			//for (int i = 0; i < DeviceSum; i++) {
			//	getline(inputFile, line);
			//	vector<string> strSplit = split(line, ",");
			//	for (int j = 0; j < DeviceSum; j++)
			//	{
			//		costParaArray[i][j].MatFlow = atof(strSplit[j].c_str());
			//	}
			//}
			#pragma endregion

			#pragma region 设备最小距离数组 
			//getline(inputFile, line);//空一行
			//for (int i = 0; i < DeviceSum; i++) {
			//	getline(inputFile, line);
			//	vector<string> strSplit = split(line, ",");
			//	for (int j = 0; j < DeviceSum; j++)
			//	{
			//		minDistArray[i][j] = atof(strSplit[j].c_str());
			//	}
			//}
			#pragma endregion

			#pragma region 设备物流相关性数组
			//getline(inputFile, line);//空一行
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
		else // 没有该文件
		{
			cout << "no such file" << endl;
		}

		#pragma region 构造设备图结构(似乎没有什么用)
		////点集
		//for (int i = 0; i < DeviceSum; i++)
		//{
		//	deviceGraph.InsertVertex(i);
		//}
		////边
		//for (int i = 0; i < CargoTypeNum; i++)
		//{
		//	for (int j = 0; j < cargoTypeList[i].deviceSum - 1; j++)
		//	{
		//		int firstEdge = cargoTypeList[i].deviceList[j] - 1;//设备序号-1
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
	//	//车间尺寸
	//	this->workShopLength = para.workShopLength;
	//  this->workShopWidth = para.workShopWidth;
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
