#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <ctime>
#include "FitnessFunction.h"
int main()
{
	int problemSum = 1;//问题的数目
	int testSum = 1;//每个实验跑的实验次数
	for (int curProblem = 0; curProblem < problemSum; curProblem++)//跑多个问题
	{
		cout << "跑第" + to_string(curProblem + 1) + "个问题" << endl;

		#pragma region 设置PSO参数
		ifstream inputFile;
		inputFile.open("../../InputParas/InputPara" + to_string(curProblem + 1) + ".txt");

		ProblemParas proParas(inputFile);					//初始化所有设备相关参数

		int dim = proParas.DeviceSum * 3;					// 总维度=设备数*3(x,y,朝向)
		PSOPara psopara(dim);								// dim是变量维度
		psopara.mesh_div_count = 4;							// 网格划分数目
		psopara.problemParas = proParas;					// 布局问题的参数
		psopara.particle_num_ =	100;						// 粒子个数
		psopara.max_iter_num_ = 400;						// 最大迭代次数
		psopara.fitness_count_ = 2;							// 适应度数目
		psopara.archive_max_count = 100;					// archive数组的最大数目
		psopara.SetDt(1.0);									// 时间步长
		psopara.SetWstart(0.9);								// 初始权重
		psopara.SetWend(0.4);								// 结束权重
		psopara.SetC1(1.49445);								// 加速度因子1
		psopara.SetC2(1.49445);								// 加速度因子2
		psopara.SetLowBound(0, 0, DeviceDirect::Default);	// position的搜索范围下限

		//不要让设备朝向取到最大值，只能取到3.几
		psopara.SetUpBound(proParas.workShopLength, proParas.workShopWidth, DeviceDirect::Rotate270 + 1);// position的搜索范围上限
		#pragma endregion

		#pragma region GPU申请变量

		#pragma endregion

		#pragma region CPU->GPU

		#pragma endregion


		#pragma region 调用PSO算法，并输出结果
		PSOOptimizer psooptimizer(&psopara);//构造函数
		std::srand(GetRamdonSeed());

		string curProblemFolderName = "Problem" + to_string(curProblem + 1);
		for (int curTest = 0; curTest < testSum; curTest++) {//每个问题跑多次
			clock_t startTime, endTime;//记录调用时间

			startTime = clock();//计时开始
			#pragma region 初始化
			psooptimizer.InitialAllParticles();//初始化所有粒子
			psooptimizer.InitialArchiveList();//初始化Archive存档
			psooptimizer.InitGbest();//初始化全局最优
			#pragma endregion

			#pragma region 迭代更新粒子&存每一次的适应度值
			//目标1的值放在archiveList1中，目标2的值放在archiveList2中
			//第n次实验放到文件里去
			//文件夹的名字叫Testn
			ofstream OutFile;
			ofstream OutFile1;
			string curTestFolderName = "Test" + to_string(curTest + 1);
			OutFile.open("../../Results/" + curProblemFolderName + "/" + curTestFolderName + "/archiveList1.txt");
			OutFile1.open("../../Results/" + curProblemFolderName + "/" + curTestFolderName + "/archiveList2.txt");
			for (int i = 0; i < psooptimizer.max_iter_num_; i++)//开始并行操作
			{
				cout << (i + 1) << endl;
				psooptimizer.UpdateAllParticles();//更新所有粒子的位置和速度 并行
				psooptimizer.UpdatePbest();//更新pbest 并行
				//这里有一个从GPU到CPU的过程
				psooptimizer.UpdateArchiveList();//更新外部存档集合 非并行
				psooptimizer.UpdateGbest();//更新gbest 非并行

				//存储每次迭代的Archive集合
				//OutFile << to_string(i) + "\n";
				//cout << psooptimizer.archive_list.size() << endl;
				double minFitness1, minFitness2;
				minFitness1 = minFitness2 = INT_MAX;
				for (auto it = psooptimizer.archive_list.begin(); it != psooptimizer.archive_list.end(); it++)
				{
					minFitness1 = min(minFitness1, it->fitness_[0]);
					//string line = to_string(it->fitness_[0]) + "\n";
					//OutFile << line;
				}
				string f1line = to_string(minFitness1) + "\n";
				OutFile << f1line;
				cout << f1line << endl;
				//OutFile << "\n";

				for (auto it = psooptimizer.archive_list.begin(); it != psooptimizer.archive_list.end(); it++)
				{
					minFitness2 = min(minFitness2, it->fitness_[1]);
					//string line = to_string(it->fitness_[1]) + "\n";
					//OutFile1 << line;
				}
				string f2line = to_string(minFitness2) + "\n";
				OutFile1 << f2line;
				cout << f2line << endl;
				//OutFile1 << "\n";
			}
			OutFile.close();
			OutFile1.close();
			#pragma endregion

			endTime = clock();
			cout << "迭代" << psopara.max_iter_num_ << "次的最终用时:" << static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

			#pragma region 保存设备尺寸&最终布局结果&连线点的坐标
			OutFile.open("../../Results/" + curProblemFolderName + "/" + curTestFolderName + "/FinalResult.txt");
			#pragma region 记录最终布局结果
			int resultIndex = 0;
			int minHandleCost = INT_MAX;
			int minConveyValue = INT_MAX;
			//优先选物料运输成本最低的
			for (int i = 0; i < psooptimizer.archive_list.size(); i++)
			{
				if (psooptimizer.archive_list[i].fitness_[0] < minHandleCost)
				{
					minHandleCost = psooptimizer.archive_list[i].fitness_[0];
					resultIndex = i;
				}
			}
			//优先选输送机成本低的
			//for (int i = 0; i < psooptimizer.archive_list.size(); i++)
			//{
			//	if (psooptimizer.archive_list[i].fitness_[1] < minConveyValue)
			//	{
			//		minConveyValue = psooptimizer.archive_list[i].fitness_[0];
			//		resultIndex = i;
			//	}
			//}

			for (int i = 0; i < dim; i += 3)
			{
				OutFile /*<< fixed << setprecision(1)*/ << psooptimizer.archive_list[resultIndex].position_[i];
				OutFile << ",";
				OutFile /*<< fixed << setprecision(1)*/ << psooptimizer.archive_list[resultIndex].position_[i + 1];
				//string line = to_string(psooptimizer.archive_list[resultIndex].position_[i]) +
				//	"," + to_string(psooptimizer.archive_list[resultIndex].position_[i + 1]) + "\n";
				//OutFile << line;
				OutFile << "\n";
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
				}
				else {
					line = to_string(psooptimizer.problemParas.deviceParaList[i / 3].size.x) + "," +
						to_string(psooptimizer.problemParas.deviceParaList[i / 3].size.y);
				}
				OutFile << line + "\n";
			}
			#pragma endregion

			int fitnessIndex = 0;
			#pragma region 记录出入口坐标（旋转之后的，不带设备坐标）//也要改，待定
			InoutPoint* ioPoints = psooptimizer.bestPathInfoList[fitnessIndex].inoutPoints;
			int ioPointsSize = psooptimizer.bestPathInfoList[fitnessIndex].inoutPSize;
			OutFile << to_string(ioPointsSize) + "\n";//出入口数目
			for (int i = 0; i < ioPointsSize; i++)
			{
				if (ioPoints[i].pointDirect == PointDirect::Up || ioPoints[i].pointDirect == PointDirect::Down)
				{
					OutFile << "Vertical ";
				}
				else
				{
					OutFile << "Horizon ";
				}
				OutFile << ioPoints[i].pointAxis.x;
				OutFile << " ";
				OutFile << ioPoints[i].pointAxis.y;
				OutFile << "\n";
			}
			#pragma endregion

			#pragma region 记录出入口路径
			////先存每种货物的路径条数
			//string line = "";
			//for (int i = 0; i < proParas.CargoTypeNum; i++)
			//{
			//	line += to_string(proParas.cargoTypeList[i].deviceSum - 1);
			//	if (i != proParas.CargoTypeNum - 1)
			//	{
			//		line += " ";
			//	}
			//}
			//OutFile << line << "\n";

			//vector<PointLink> p = psooptimizer.archive_list[resultIndex].pointLinks;
			//for (int i = 0; i < p.size(); i++)
			//{
			//	string s1, s2;
			//	DevicePara device1, device2;

			//	//s1 = to_string(p[i].device1Index) + " " + to_string(p[i].device2Index);
			//	OutFile << to_string(p[i].device1Index) + " " + to_string(p[i].device2Index) + " ";
			//	//计算s2
			//	for (int j = 0; j < p[i].points.size(); j++)
			//	{
			//		OutFile /*<< fixed << setprecision(1)*/ << p[i].points[j].x;
			//		OutFile << ",";
			//		OutFile /*<< fixed << setprecision(1)*/ << p[i].points[j].y;
			//		//s2 += to_string(p[i].points[j].x) + "," + to_string(p[i].points[j].y);
			//		if (j != p[i].points.size() - 1)
			//		{
			//			//s2 += "|";
			//			OutFile << "|";
			//		}
			//	}
			//	OutFile << "\n";
			//	//string line = s1 + " " + s2 + "\n";
			//	//OutFile << line;
			//}
			#pragma endregion

			#pragma region 记录直线输送机和转弯输送机参数
			set<StraightConveyorInfo> strInfoList = psooptimizer.bestPathInfoList[fitnessIndex].strConveyorList;
			set<Vector2Int> curveInfoList = psooptimizer.bestPathInfoList[fitnessIndex].curveConveyorList;
			OutFile << strInfoList.size() << "\n";
			for (StraightConveyorInfo sci : strInfoList)
			{
				OutFile << to_string(sci.startPos.x) << "," << to_string(sci.startPos.y)
					<< ";" << to_string(sci.endPos.x) << "," << to_string(sci.endPos.y)
					<< ";" << to_string(sci.startHnum) << ";" << to_string(sci.startVnum)
					<< ";" << to_string(sci.endHnum) << ";" << to_string(sci.endVnum)
					<< "\n";
			}
			OutFile << curveInfoList.size() << "\n";
			for (Vector2Int v : curveInfoList)
			{
				OutFile << to_string(v.x) << "," << to_string(v.y) << "\n";
			}


			//set<SegPath> segPathSet = psooptimizer.archive_list[resultIndex].segPathSet;
			//OutFile << segPathSet.size() << "\n";
			//for (SegPath sp : segPathSet)
			//{
			//	OutFile << to_string(sp.p1.x) << "," << to_string(sp.p1.y)
			//			<< ";" << to_string(sp.p2.x) << "," << to_string(sp.p2.y) << "\n";
			//}

			//map<Vector2, PointInfo> pathPointInfoMap = psooptimizer.archive_list[resultIndex].pathPointInfoMap;
			//OutFile << pathPointInfoMap.size() << "\n";
			//for (auto it = pathPointInfoMap.begin(); it != pathPointInfoMap.end(); it++) 
			//{
			//	OutFile << to_string(it->first.x) << "," << to_string(it->first.y)
			//		<< ";" << to_string(it->second.horiDirNum) << ";" << to_string(it->second.vertDirNum) << "\n";
			//}
			#pragma endregion

			OutFile.close();
			#pragma endregion

		}


		#pragma endregion

	}

	system("pause");
}
