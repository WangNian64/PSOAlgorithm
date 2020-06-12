#pragma once
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <random>
#include "ProblemParas.h"

// 适应度是越大越好还是越小越好
#define MINIMIZE_FITNESS
//#define MAXIMIZE_FITNESS

struct PSOPara
{
	int dim_;									// 参数维度（position和velocity的维度）
	int fitness_count_;							// 适应度数目
	int particle_num_;							// 粒子个数
	int max_iter_num_;							// 最大迭代次数

	int mesh_div_count;
	double* dt_ = nullptr;						// 时间步长
	double* wstart_ = nullptr;					// 初始权重
	double* wend_ = nullptr;					// 终止权重
	double* C1_ = nullptr;						// 加速度因子1
	double* C2_ = nullptr;						// 加速度因子2

	double* lower_bound_ = nullptr;				// position搜索范围下限
	double* upper_bound_ = nullptr;				// position搜索范围上限
	double* range_interval_ = nullptr;			// position搜索区间长度

	int results_dim_ = 0;						// results的维度

	int archive_max_count;						// pareto最优解数组的最大值
	ProblemParas problemParas;					//和粒子对应的设备布局参数

	PSOPara() {}

	PSOPara(int dim)
	{
		dim_ = dim;

		dt_ = new double[dim_];
		wstart_ = new double[dim_];
		wend_ = new double[dim_];
		C1_ = new double[dim_];
		C2_ = new double[dim_];

		lower_bound_ = new double[dim_];
		upper_bound_ = new double[dim_];
		range_interval_ = new double[dim_];

		ProblemParas problemParas();
	}

	// 析构函数：释放堆内存
	~PSOPara()
	{
		if (lower_bound_) { delete[]lower_bound_; }
		if (upper_bound_) { delete[]upper_bound_; }
		if (range_interval_) { delete[]range_interval_; }
		if (dt_) { delete[]dt_; }
		if (wstart_) { delete[]wstart_; }
		if (wend_) { delete[]wend_; }
		if (C1_) { delete[]C1_; }
		if (C2_) { delete[]C2_; }
	}

	// 设置物流问题相关参数
	void SetProblemParas(ProblemParas paras) {
		problemParas = paras;
	}

	// 设置时间步长
	void SetDt(double stepLength) {
		for (int i = 0; i < dim_; i++) {
			dt_[i] = stepLength;
		}
	}

	// 设置wstart_
	void SetWstart(double startWeight) {
		for (int i = 0; i < dim_; i++) {
			wstart_[i] = startWeight;
		}
	}

	// 设置wend_
	void SetWend(double endWeight) {
		for (int i = 0; i < dim_; i++) {
			wend_[i] = endWeight;
		}
	}

	// 设置C1
	void SetC1(double c1) {
		for (int i = 0; i < dim_; i++) {
			C1_[i] = c1;
		}
	}

	// 设置C2
	void SetC2(double c2) {
		for (int i = 0; i < dim_; i++) {
			C2_[i] = c2;
		}
	}

	// 设置low_bound
	void SetLowBound(double lowBoundX, double lowBoundY, int lowBoundDirect) {
		for (int i = 0; i < dim_; i++) {
			if (i % 3 == 0)
				lower_bound_[i] = lowBoundX + problemParas.deviceParaList[i / 3].size.x * 0.5 + problemParas.deviceParaList[i / 3].spaceLength;
			else if (i % 3 == 1)
				lower_bound_[i] = lowBoundY + problemParas.deviceParaList[i / 3].size.y * 0.5 + problemParas.deviceParaList[i / 3].spaceLength;
			else
				lower_bound_[i] = lowBoundDirect;
		}
	}

	// 设置upper_bound
	void SetUpBound(double upBoundX, double upBoundY, int upBoundDirect) {
		for (int i = 0; i < dim_; i++) {
			if (i % 3 == 0)
				upper_bound_[i] = upBoundX - problemParas.deviceParaList[i / 3].size.x * 0.5 - problemParas.deviceParaList[i / 3].spaceLength;
			else if (i % 3 == 1)
				upper_bound_[i] = upBoundY - problemParas.deviceParaList[i / 3].size.y * 0.5 - problemParas.deviceParaList[i / 3].spaceLength;
			else
				upper_bound_[i] = upBoundDirect;
		}
	}
};

//粒子结构体
struct Particle
{
	int dim_;							// 参数维度（position和velocity的维度）

	int fitnessCount;					//适应度的个数
	double* fitness_ = nullptr;			//适应度数组
	double* position_ = nullptr;		//粒子位置数组
	double* velocity_ = nullptr;		//粒子速度数组

	double* best_position_ = nullptr;	//粒子的个体最优位置数组
	double* best_fitness_ = nullptr;	//粒子的个体最优适应度数组

	//vector<PointLink> pointLinks; //最终路径集合
	vector<InoutPoint> inoutPoints;//出入口集合
	set<StraightConveyorInfo> strConveyorList;//直线输送机信息列表
	set<Vector2Int> curveConveyorList;//转弯输送机信息列表
	//map<Vector2Int, PointInfo> pathPointInfoMap;//路径点信息map
	//set<SegPath> segPathSet;
	//int pointLinkSum = 0;//路径的数目
	Particle() {}
	Particle(const Particle& particle)//拷贝构造函数
	{
		this->dim_ = particle.dim_;
		this->fitnessCount = particle.fitnessCount;
		this->position_ = new double[this->dim_];
		this->velocity_ = new double[this->dim_];
		this->best_position_ = new double[this->dim_];

		this->fitness_ = new double[this->fitnessCount];
		this->best_fitness_ = new double[this->fitnessCount];
		for (int i = 0; i < this->fitnessCount; i++)
		{
			this->fitness_[i] = particle.fitness_[i];
			this->best_fitness_[i] = particle.best_fitness_[i];
		}
		for (int i = 0; i < this->dim_; i++)
		{
			this->position_[i] = particle.position_[i];
			this->velocity_[i] = particle.velocity_[i];
			this->best_position_[i] = particle.best_position_[i];
		}
		//this->pointLinks = particle.pointLinks;
		this->inoutPoints = particle.inoutPoints;
		this->strConveyorList = particle.strConveyorList;
		this->curveConveyorList = particle.curveConveyorList;
		//this->pathPointInfoMap = particle.pathPointInfoMap;
	}
	~Particle()
	{

	}
};

typedef void (*ComputeFitness)(int curIterNum, int maxIterNum, Particle& particle, ProblemParas proParas, double* lowerBounds, double* upBounds);

class PSOOptimizer
{
public:
	int particle_num_;								// 粒子个数
	int max_iter_num_;								// 最大迭代次数
	int curr_iter_;									// 当前迭代次数

	int dim_;										// 参数维度（position和velocity的维度）
	int fitness_count;								// 适应度数目
	int meshDivCount;								// 网格等分因子（默认为10）
	Particle* particles_ = nullptr;					// 所有粒子

	double* upper_bound_ = nullptr;					// position搜索范围上限
	double* lower_bound_ = nullptr;					// position搜索范围下限
	double* range_interval_ = nullptr;				// position搜索区间长度

	double* dt_ = nullptr;							// 时间步长
	double* wstart_ = nullptr;						// 初始权重
	double* wend_ = nullptr;						// 终止权重
	double* w_ = nullptr;							// 当前迭代权重
	double* C1_ = nullptr;							// 加速度因子
	double* C2_ = nullptr;							// 加速度因子

	double** all_best_fitness_ = nullptr;			// 全局最优粒子的适应度数组 100x2
	double** all_best_position_ = nullptr;			// 全局最优粒子的position 100x12

	ProblemParas problemParas;						// 布局问题参数

	ComputeFitness fitness_fun_ = nullptr;			// 适应度函数

	//MOPSO相关参数
	vector<Particle> archive_list;					// 存放pareto非劣解的数组
	int archiveMaxCount;							// archive数组的最大数目

public:
	// 默认构造函数
	PSOOptimizer() {}

	// 构造函数
	PSOOptimizer(PSOPara* pso_para, ComputeFitness fitness_fun);

	// 析构函数
	~PSOOptimizer();

	// 初始化所有粒子参数
	void InitialAllParticles();
	// 初始化第i个粒子参数
	void InitialParticle(int i);

	// 初始化Archive数组
	void InitialArchiveList();
	// 更新Archive数组
	void UpdateArchiveList();

	// 初始化全局最优
	void InitGbest();

	// 获取双精度随机数（默认精度为0.1）
	double GetDoubleRand(int N = 9);

	//(返回一个0 - N包括N的int）
	int GetIntRand(int N);

	// 计算该粒子的适应度值
	void GetFitness(Particle& particle);

	// 更新所有粒子参数
	void UpdateAllParticles();

	// 更新第i个粒子
	void UpdateParticle(int i);

	// 更新Pbest
	void UpdatePbest();
	// 更新Gbest
	void UpdateGbest();
	// 比较两个粒子的适应度，判断是否完全支配，从而计算出pbest
	bool ComparePbest(double* fitness, double* pbestFitness);
	// 获取当前迭代的权重
	void GetInertialWeight();

};