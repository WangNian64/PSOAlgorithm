#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <ctime>
#include "PSO.h"
#include "FitnessFunction.h"
//保存结果到txt
//void SaveLayoutResults(PSOOptimizer psooptimizer, double* result) {
//	ofstream OutFile;
//	//存每一次的适应度值#include <time.h>
//
//
//
//	OutFile.open("../../IterateResult.txt");
//	for (int i = 0; i < psooptimizer.max_iter_num_; i++)
//	{
//		string line = to_string(1.0 / result[i]) + "\n";
//		OutFile << line;
//	}
//	OutFile.close();
//	//存最后的布局结果
//	OutFile.open("../../LayoutResult.txt");
//	for (int i = 0; i < psooptimizer.dim_; i += 2) {
//		string line = to_string(psooptimizer.all_best_position_[i]) + "," + to_string(psooptimizer.all_best_position_[i + 1]) + "\n";
//		OutFile << line;
//	}
//	OutFile.close();
//}
int main()
{
	#pragma region 设置PSO参数
	int deviceNum = 6;	
	int cargoTypeNum = 3;
	ProblemParas proParas(deviceNum, cargoTypeNum);//初始化所有设备相关参数

	int dim = deviceNum * 3;						// 总维度=设备数*3(x,y,朝向)
	PSOPara psopara(dim);							// dim是变量维度
	psopara.mesh_div_count = 2;						// 网格划分数目
	psopara.problemParas = proParas;				// 布局问题的参数
	psopara.particle_num_ = 100;					// 粒子个数
	psopara.max_iter_num_ = 400;					// 最大迭代次数
	psopara.fitness_count_ = 2;						// 适应度数目
	psopara.archive_max_count = 200;				// archive数组的最大数目
	psopara.SetDt(1.0);								// 时间步长
	psopara.SetWstart(0.9);							// 初始权重
	psopara.SetWend(0.4);							// 结束权重
	psopara.SetC1(1.49445);							// 加速度因子1
	psopara.SetC2(1.49445);							// 加速度因子2
	psopara.SetLowBound(0, 0, DeviceDirect::Default);				// position的搜索范围下限
	psopara.SetUpBound(proParas.workShopLength, proParas.workShopWidth, DeviceDirect::Rotate270);// position的搜索范围上限
	//
	#pragma endregion

	#pragma region 调用PSO算法，并输出结果
	
	clock_t startTime, endTime;//记录调用时间
	startTime = clock();//计时开始
	#pragma region 初始化

	PSOOptimizer psooptimizer(&psopara, FitnessFunction);//构造函数

	std::srand((unsigned int)time(0));


	psooptimizer.InitialAllParticles();//初始化所有粒子
	psooptimizer.InitialArchiveList();//初始化Archive存档
	psooptimizer.InitGbest();//初始化全局最优 //这个地方要注意min和max

	#pragma endregion

	#pragma region 迭代更新粒子&存每一次的适应度值
	ofstream OutFile;
	OutFile.open("../../archiveList.txt");
	for (int i = 0; i < psooptimizer.max_iter_num_; i++)
	{
		cout << i << endl;
		psooptimizer.UpdateAllParticles();//更新所有粒子的位置和速度
		psooptimizer.UpdatePbest();//更新pbest
		psooptimizer.UpdateArchiveList();//更新外部存档集合
		psooptimizer.UpdateGbest();//更新gbest

		//存储每次迭代的Archive集合
		OutFile << to_string(i) + "\n";
		for (auto it = psooptimizer.archive_list.begin(); it != psooptimizer.archive_list.end(); it++)
		{
			string line = to_string(it->fitness_[0]) + ", " + to_string(it->fitness_[1]) + "\n";
			OutFile << line;
		}
		OutFile << "\n";
	}
	OutFile.close();
	#pragma endregion
	endTime = clock();
	cout << "迭代" << psopara.max_iter_num_ << "次的最终用时:" << static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	#pragma region 保存设备尺寸&最终布局结果&连线点的坐标
	OutFile.open("../../FinalResult.txt");


	#pragma region 记录最终布局结果
	int resultIndex = 0;
	int minConveyValue = INTMAX_MAX;
	int minAreaVaule = INTMAX_MAX;
	//优先选运输效率最高的
	for (int i = 0; i < psooptimizer.archive_list.size(); i++)
	{
		if (psooptimizer.archive_list[i].fitness_[0] < minConveyValue)
		{
			minConveyValue = psooptimizer.archive_list[i].fitness_[0];
			resultIndex = i;
		}
	}
	//优先选面积最小的
	//for (int i = 0; i < psooptimizer.archive_list.size(); i++)
	//{
	//	if (psooptimizer.archive_list[i].fitness_[1] < minConveyValue)
	//	{
	//		minAreaVaule = psooptimizer.archive_list[i].fitness_[1];
	//		resultIndex = i;
	//	}
	//}

	for (int i = 0; i < dim; i += 3)
	{
		string line = to_string(psooptimizer.archive_list[resultIndex].position_[i]) +
			"," + to_string(psooptimizer.archive_list[resultIndex].position_[i + 1]) + "\n";
		OutFile << line;
	}
	#pragma endregion


	#pragma region 记录设备尺寸
	for (int i = 2; i < dim; i += 3)
	{
		DeviceDirect direct = (DeviceDirect)(int)psooptimizer.archive_list[resultIndex].position_[i];
		string line = "";
		if (direct == DeviceDirect::Rotate90 || direct == DeviceDirect::Rotate270)
		{
			line = to_string(psooptimizer.problemParas.deviceParaList[i / 3].size.y) + "," +
				to_string(psooptimizer.problemParas.deviceParaList[i / 3].size.x);
		} else {
			line = to_string(psooptimizer.problemParas.deviceParaList[i / 3].size.x) + "," +
				to_string(psooptimizer.problemParas.deviceParaList[i / 3].size.y);
		}
		OutFile << line + "\n";
	}
	#pragma endregion
	#pragma region 记录出入口路径
	vector<PointLink> p = psooptimizer.archive_list[resultIndex].pointLinks;
	for (int i = 0; i < p.size(); i++)
	{
		cout << psooptimizer.archive_list[resultIndex].pointLinks[i].device1Index << endl;
	}
	for (int i = 0; i < p.size(); i++)
	{
		string s1, s2;
		DevicePara device1, device2;
		s1 = to_string(p[i].device1Index) + " " + to_string(p[i].device2Index);
		//计算s2
		for (int j = 0; j < p[i].points.size(); j++)
		{
			s2 += to_string(p[i].points[j].x) + "," + to_string(p[i].points[j].y);
			if (j != p[i].points.size() - 1)
			{
				s2 += "|";
			}
		}
		string line = s1 + " " + s2 + "\n";
		OutFile << line;
	}
	#pragma endregion

	OutFile.close();
	#pragma endregion
	
	//最后的结果存到txt中(布局结果和每一次迭代的适应度值）
	//SaveLayoutResults(psooptimizer, result);

	#pragma endregion

	system("pause");
}
