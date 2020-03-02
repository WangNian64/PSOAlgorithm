#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
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
	//布局相关参数
	int deviceNum = 6;	
	ProblemParas proParas(deviceNum);




	#pragma region 设置PSO参数
	int dim = deviceNum * 2;				// 总维度=设备数*2
	PSOPara psopara(dim);					// dim是变量维度
	psopara.mesh_div_count = 4;				// 网格划分数目
	psopara.problemParas = proParas;		// 布局问题的参数
	psopara.particle_num_ = 100;			// 粒子个数
	psopara.max_iter_num_ = 2000;			// 最大迭代次数
	psopara.fitness_count_ = 2;				// 适应度数目
	psopara.archive_max_count = 200;		// archive数组的最大数目
	psopara.SetDt(1.0);						// 时间步长
	psopara.SetWstart(0.9);					// 初始权重
	psopara.SetWend(0.4);					// 结束权重
	psopara.SetC1(1.49445);					// 加速度因子1
	psopara.SetC2(1.49445);					// 加速度因子2
	psopara.SetLowBound(0, 0);				// position的搜索范围下限
	psopara.SetUpBound(proParas.WorkshopSize.x, proParas.WorkshopSize.y);// position的搜索范围上限
	#pragma endregion

	#pragma region 调用PSO算法，并输出结果

	#pragma region 初始化

	PSOOptimizer psooptimizer(&psopara, FitnessFunction);//构造函数

	std::srand((unsigned int)time(0));


	psooptimizer.InitialAllParticles();//初始化所有粒子
	psooptimizer.InitialArchiveList();//初始化Archive存档
	psooptimizer.InitGbest();//初始化全局最优

	#pragma endregion

	//迭代更新粒子
	ofstream OutFile;
	//存每一次的适应度值
	OutFile.open("../archiveList.txt");
	for (int i = 0; i < psooptimizer.max_iter_num_; i++)
	{
		psooptimizer.UpdateAllParticles();//更新所有粒子的位置和速度
		psooptimizer.UpdatePbest();//更新pbest
		psooptimizer.UpdateArchiveList();//更新外部存档集合
		psooptimizer.UpdateGbest();//更新gbest

		//存储每次的Archive集合
		OutFile << to_string(i) + "\n";
		for (auto it = psooptimizer.archive_list.begin(); it != psooptimizer.archive_list.end(); it++)
		{
			string line = to_string(it->fitness_[0]) + ", " + to_string(it->fitness_[1]) + "\n";
			OutFile << line;
		}
		OutFile << "\n";
	}
	OutFile.close();
	//最后的结果存到txt中(布局结果和每一次迭代的适应度值）
	//SaveLayoutResults(psooptimizer, result);

	#pragma endregion

	system("pause");
}
