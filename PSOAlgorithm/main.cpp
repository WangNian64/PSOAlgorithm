#include <iostream>
#include <fstream>
#include <string>
#include "PSO.h"
#include "FitnessFunction.h"
#include <time.h>
//字符串分割函数
vector<string> split(const string& str, const string& pattern)
{
	vector<string> res;
	if ("" == str)
		return res;

	string strs = str + pattern;

	size_t pos = strs.find(pattern);
	size_t size = strs.size();

	while (pos != string::npos)
	{
		string x = strs.substr(0, pos);
		res.push_back(x);//stoi(x)转整型
		strs = strs.substr(pos + 1, size);
		pos = strs.find(pattern);
	}
	return res;
}
int main()
{
	#pragma region 设置算法参数


	// 设备数
	int deviceNum = 4;	

	#pragma region 设备尺寸数组
	ProblemParas proParas(deviceNum);
	proParas.DeviceSizeArray = new Size[deviceNum];
	ifstream fileIn("Para.txt");
	string line;
	if (fileIn) // 有该文件
	{
		getline(fileIn, line);
		vector<string> strSplit = split(line, " ");
		for (int i = 0; i < proParas.DeviceSum; i++)
		{
			vector<string> deviceSizeStr = split(strSplit[i], ",");
			proParas.DeviceSizeArray[i].x = atof(deviceSizeStr[0].c_str());
			proParas.DeviceSizeArray[i].y = atof(deviceSizeStr[1].c_str());
			cout << proParas.DeviceSizeArray[i].x << "," << proParas.DeviceSizeArray[i].y << endl;
		}
	}
	else // 没有该文件
	{
		cout << "no such file" << endl;
	}
	#pragma endregion

	#pragma region 车间尺寸
	proParas.WorkshopSize.x = 20.0;
	proParas.WorkshopSize.y = 20.0;
	#pragma endregion

	#pragma region 成本计算参数
	proParas.CostParaArray = new CostPara * [proParas.DeviceSum];
	for (int i = 0; i < proParas.DeviceSum; i++) {
		proParas.CostParaArray[i] = new CostPara[proParas.DeviceSum];
	}
	if (fileIn) // 有该文件
	{
		getline(fileIn, line);//空一行
		for (int i = 0; i < proParas.DeviceSum; i++) {
			getline(fileIn, line);
			cout << line << endl;
			vector<string> strSplit = split(line, " ");
			for (int j = 0; j < proParas.DeviceSum; j++)
			{
				vector<string> costParaStr = split(strSplit[j], ",");
				proParas.CostParaArray[i][j].MatFlow = atof(costParaStr[0].c_str());
				proParas.CostParaArray[i][j].UnitCost = atof(costParaStr[1].c_str());
				cout << proParas.CostParaArray[i][j].MatFlow << 
					"," << proParas.CostParaArray[i][j].UnitCost << endl;
			}
		}
	}
	else // 没有该文件
	{
		cout << "no such file" << endl;
	}
	#pragma endregion

	#pragma region PSO参数
	int dim = deviceNum * 2;		// 总维度=设备数*2
	PSOPara psopara(dim, true);		// 2为变量维度，true表示有搜索上下限
	psopara.particle_num_ = 20;		// 粒子个数
	psopara.max_iter_num_ = 300;	// 最大迭代次数
	psopara.SetDt(1.0);				// 时间步长
	psopara.SetWstart(0.9);			// 初始权重
	psopara.SetWend(0.4);			// 结束权重
	psopara.SetC1(1.49445);			// 加速度因子1
	psopara.SetC2(1.49445);			// 加速度因子2
	psopara.SetLowBound(0);			// position搜索范围下限
	psopara.SetUpBound(20);			// position搜索范围上限
	psopara.problemParas = proParas;// 布局问题的参数
	#pragma endregion


	#pragma endregion

	PSOOptimizer psooptimizer(&psopara, FitnessFunction);//构造函数

	std::srand((unsigned int)time(0));
	psooptimizer.InitialAllParticles();//初始化所有粒子
	double fitness = psooptimizer.all_best_fitness_;
	double* result = new double[psooptimizer.max_iter_num_];//初始化存储每一代最优粒子的数组

	//迭代更新粒子
	for (int i = 0; i < psooptimizer.max_iter_num_; i++)
	{
		psooptimizer.UpdateAllParticles();//更新所有粒子的位置和速度
		result[i] = psooptimizer.all_best_fitness_;//记录下这一代的最优粒子
		cout << "第" << i << "次迭代结果：" << endl;
		for (int j = 0; j < dim; j+=2) {
			cout << "X" << (j + 1) / 2 << ": "<< psooptimizer.all_best_position_[j] << ", " << "Y" << (j + 1) / 2 << ": " << psooptimizer.all_best_position_[j + 1] << endl;
		}
		cout << ", fitness = " << result[i] << std::endl;
	}

	system("pause");
}