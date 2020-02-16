#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include "Tools.h"
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
	Size* DeviceSizeArray;			//设备尺寸数组

	Size WorkshopSize;				//车间尺寸

	CostPara** CostParaArray;		//成本计算参数数组

	double** MinDistArray;			//设备最小距离数组
	ProblemParas() {}
	ProblemParas(int deviceNum)
	{
		DeviceSum = deviceNum;

		DeviceSizeArray = new Size[deviceNum];

		CostParaArray = new CostPara * [DeviceSum];
		for (int i = 0; i < DeviceSum; i++) {
			CostParaArray[i] = new CostPara[DeviceSum];
		}

		MinDistArray = new double * [DeviceSum];
		for (int i = 0; i < DeviceSum; i++) {
			MinDistArray[i] = new double[DeviceSum];
		}




		ifstream fileIn("../../Para.txt");
		string line;
		if (fileIn) // 有该文件
		{
			//车间尺寸
			getline(fileIn, line);//空一行
			getline(fileIn, line);
			vector<string> shopSizeStr = split(line, ",");
			WorkshopSize.x = atof(shopSizeStr[0].c_str());
			WorkshopSize.y = atof(shopSizeStr[1].c_str());


			//设备尺寸
			getline(fileIn, line);//空一行
			getline(fileIn, line);
			vector<string> strSplit = split(line, " ");
			for (int i = 0; i < DeviceSum; i++)
			{
				vector<string> deviceSizeStr = split(strSplit[i], ",");
				DeviceSizeArray[i].x = atof(deviceSizeStr[0].c_str());
				DeviceSizeArray[i].y = atof(deviceSizeStr[1].c_str());
			}


			//单位距离成本数组
			getline(fileIn, line);//空一行
			for (int i = 0; i < DeviceSum; i++) {
				getline(fileIn, line);
				vector<string> strSplit = split(line, ",");
				for (int j = 0; j < DeviceSum; j++)
				{
					CostParaArray[i][j].UnitCost = atof(strSplit[j].c_str());
				}
			}

			//物流总量数组
			getline(fileIn, line);//空一行
			for (int i = 0; i < DeviceSum; i++) {
				getline(fileIn, line);
				vector<string> strSplit = split(line, ",");
				for (int j = 0; j < DeviceSum; j++)
				{
					CostParaArray[i][j].MatFlow = atof(strSplit[j].c_str());
				}
			}

			//设备最小距离数组
			getline(fileIn, line);//空一行
			for (int i = 0; i < DeviceSum; i++) {
				getline(fileIn, line);
				vector<string> strSplit = split(line, ",");
				for (int j = 0; j < DeviceSum; j++)
				{
					MinDistArray[i][j] = atof(strSplit[j].c_str());
					if (i != j) {
						MinDistArray[i][j] += 1;
					}
				}
				cout << endl;
			}
		}
		else // 没有该文件
		{
			cout << "no such file" << endl;
		}
	}
};
