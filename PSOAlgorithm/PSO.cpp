#pragma once
#include "Pareto.h"
#include "Archive.h"
#include "FitnessFunction.h"
#include "Tools.h"
#include <ctime>
// 构造函数(初始化各种算法的参数，给数组分配空间)
PSOOptimizer::PSOOptimizer(PSOPara* pso_para, ProblemParas& problemParas)
{
	particle_num_ = pso_para->particle_num_;
	max_iter_num_ = pso_para->max_iter_num_;
	dim_ = pso_para->dim_;
	fitness_count = pso_para->fitness_count_;
	curr_iter_ = 0;

	dt_ = pso_para->dt_;
	wstart_ = pso_para->wstart_;
	wend_ = pso_para->wend_;
	C1_ = pso_para->C1_;
	C2_ = pso_para->C2_;


	meshDivCount = pso_para->mesh_div_count;
	archiveMaxCount = pso_para->archive_max_count;



	//给CPU粒子分配空间
	position_CPU = (double*)malloc(sizeof(double) * particle_num_ * dim_);
	velocity_CPU = (double*)malloc(sizeof(double) * particle_num_ * dim_);
	best_position_CPU = (double*)malloc(sizeof(double) * particle_num_ * dim_);

	fitness_CPU = (double*)malloc(sizeof(double) * particle_num_ * fitness_count);
	best_fitness_CPU = (double*)malloc(sizeof(double) * particle_num_ * fitness_count);
	//给GPU粒子分配空间
	cudaMalloc((void**)& position_GPU, sizeof(double) * particle_num_ * dim_);
	cudaMalloc((void**)& velocity_GPU, sizeof(double) * particle_num_ * dim_);
	cudaMalloc((void**)& best_position_GPU, sizeof(double) * particle_num_ * dim_);

	cudaMalloc((void**)& fitness_GPU, sizeof(double) * particle_num_ * fitness_count);
	cudaMalloc((void**)& best_fitness_GPU, sizeof(double) * particle_num_ * fitness_count);



	//初始化随机数种子
	cudaMalloc(&devStates, particle_num_ * sizeof(curandState));
	initRandomGenerator <<< 1, particle_num_ >>> (devStates, unsigned(time(NULL)));//time作为随机数种子

	//给随机数数组分配GPU空间
	cudaMalloc((void**)& randomNumList, sizeof(double) * particle_num_);


	//GPU内存分配
	cudaMalloc((void**)& upper_bound_, sizeof(double) * dim_);
	cudaMalloc((void**)& lower_bound_, sizeof(double) * dim_);
	cudaMalloc((void**)& range_interval_, sizeof(double) * dim_);
	//数据CPU->GPU
	cudaMemcpy(upper_bound_, pso_para->upper_bound_, sizeof(double) * dim_, cudaMemcpyHostToDevice);
	cudaMemcpy(lower_bound_, pso_para->lower_bound_, sizeof(double) * dim_, cudaMemcpyHostToDevice);
	cudaMemcpy(range_interval_, pso_para->range_interval_, sizeof(double) * dim_, cudaMemcpyHostToDevice);


	//CPU版本的bound
	lower_bound_CPU = new double[dim_];
	upper_bound_CPU = new double[dim_];


	//原本是二维的，降为一维
	cudaMalloc((void**)& all_best_position_, sizeof(double) * particle_num_ * dim_);
	cudaMalloc((void**)& all_best_fitness_, sizeof(double) * particle_num_ * fitness_count);


	this->problemParas = problemParas;//CPU端的
	//this->bestPathInfoList = new BestPathInfo[fitness_count];//默认初始化(这也是个问题，因为list内部还有子结构也是*数组


	//给problemParas的参数赋值
	DeviceSum = problemParas.DeviceSum;									//设备总数

	horiPointCount = problemParas.horiPointCount;						//未去重前所有水平方向的点的数目
	vertPointCount = problemParas.vertPointCount;						//未去重前所有垂直方向的点的数目

	workShopLength = problemParas.workShopLength;						//车间长度
	workShopWidth = problemParas.workShopWidth;							//车间宽度

	entrancePos = problemParas.entrancePos;								//仓库入口坐标	
	exitPos = problemParas.exitPos;										//仓库出口坐标

	//物料参数列表
	CargoTypeNum = problemParas.CargoTypeNum;							//货物类型数目
	totalLinkSum = problemParas.totalLinkSum;							//总的连接线数目

	fixedLinkPointSum = problemParas.fixedLinkPointSum;					//每一条link的固定点数目为50(没有去重之前的)
	fixedUniqueLinkPointSum = problemParas.fixedUniqueLinkPointSum;		//去重后的每一条link的固定点数目为20
	//输送机参数
	convey2DeviceDist = problemParas.convey2DeviceDist;//输送机到设备的距离（寻路的时候要考虑）
	conveyWidth = problemParas.conveyWidth;//输送机宽度
	conveyMinLength = problemParas.conveyMinLength;//输送机最短长度
	conveySpeed = problemParas.conveySpeed;//输送机输送速度
	strConveyorUnitCost = problemParas.strConveyorUnitCost;//单位直线输送机成本
	curveConveyorUnitCost = problemParas.curveConveyorUnitCost;//单个转弯输送机成本
	conveyMinDist = problemParas.conveyMinDist;//输送线两个点之间的最短距离

	totalInPoint = problemParas.totalInPoint;
	totalOutPoint = problemParas.totalOutPoint;
	//DevicePara* deviceParaList;										//设备参数列表
	//数目：DeviceSum
	//GPU内存分配
	cudaMalloc((void**)& ID, sizeof(int)* DeviceSum);
	cudaMalloc((void**)& workSpeed, sizeof(double)* DeviceSum);
	cudaMalloc((void**)& size, sizeof(Vector2)* DeviceSum);
	cudaMalloc((void**)& axis, sizeof(Vector2)* DeviceSum);
	cudaMalloc((void**)& direct, sizeof(DeviceDirect)* DeviceSum);
	cudaMalloc((void**)& spaceLength, sizeof(double)* DeviceSum);
	//出入口点的数组（会影响输送线的布局）
	cudaMalloc((void**)& adjPInCount, sizeof(int)* DeviceSum);
	cudaMalloc((void**)& adjPOutCount, sizeof(int)* DeviceSum);

	cudaMalloc((void**)& adjPointsIn, sizeof(AdjPoint)* totalInPoint);
	cudaMalloc((void**)& adjPointsOut, sizeof(AdjPoint)* totalOutPoint);
	//CPU赋值到GPU
	cudaMemcpy(ID, problemParas.ID, sizeof(int) * DeviceSum, cudaMemcpyHostToDevice);
	cudaMemcpy(workSpeed, problemParas.workSpeed, sizeof(double) * DeviceSum, cudaMemcpyHostToDevice);
	cudaMemcpy(size, problemParas.size, sizeof(Vector2) * DeviceSum, cudaMemcpyHostToDevice);
	cudaMemcpy(axis, problemParas.axis, sizeof(Vector2) * DeviceSum, cudaMemcpyHostToDevice);
	cudaMemcpy(direct, problemParas.direct, sizeof(DeviceDirect) * DeviceSum, cudaMemcpyHostToDevice);
	cudaMemcpy(spaceLength, problemParas.spaceLength, sizeof(double) * DeviceSum, cudaMemcpyHostToDevice);

	cudaMemcpy(adjPInCount, problemParas.adjPInCount, sizeof(int) * DeviceSum, cudaMemcpyHostToDevice);
	cudaMemcpy(adjPOutCount, problemParas.adjPOutCount, sizeof(int) * DeviceSum, cudaMemcpyHostToDevice);
	cudaMemcpy(accumAdjPInCount, problemParas.accumAdjPInCount, sizeof(int) * DeviceSum, cudaMemcpyHostToDevice);
	cudaMemcpy(accumAdjPOutCount, problemParas.accumAdjPOutCount, sizeof(int) * DeviceSum, cudaMemcpyHostToDevice);

	cudaMemcpy(adjPointsIn, problemParas.adjPointsIn, sizeof(int) * totalInPoint, cudaMemcpyHostToDevice);
	cudaMemcpy(adjPointsOut, problemParas.adjPointsOut, sizeof(int) * totalOutPoint, cudaMemcpyHostToDevice);



	//CargoType* cargoTypeList;			//货物类型列表
	//数目：CargoTypeNum
	CargoTypeNum = problemParas.CargoTypeNum;						//货物类型数目
	totalLinkSum = problemParas.totalLinkSum;						//总的连接线数目

	cudaMalloc((void**)& deviceSum, sizeof(int)* CargoTypeNum);
	cudaMalloc((void**)& linkSum, sizeof(int)* CargoTypeNum);
	cudaMalloc((void**)& totalVolume, sizeof(double)* CargoTypeNum);

	cudaMalloc((void**)& deviceLinkList, sizeof(DeviceLink)* totalLinkSum);
	//CPU赋值到GPU
	cudaMemcpy(deviceSum, problemParas.deviceSum, sizeof(int)* CargoTypeNum, cudaMemcpyHostToDevice);
	cudaMemcpy(linkSum, problemParas.linkSum, sizeof(double)* CargoTypeNum, cudaMemcpyHostToDevice);
	cudaMemcpy(deviceLinkList, problemParas.deviceLinkList, sizeof(DeviceLink)* totalLinkSum, cudaMemcpyHostToDevice);
	cudaMemcpy(totalVolume, problemParas.totalVolume, sizeof(double)* CargoTypeNum, cudaMemcpyHostToDevice);
}

PSOOptimizer::~PSOOptimizer()//析构函数都需要修改
{
	if (upper_bound_) { delete[]upper_bound_; }
	if (lower_bound_) { delete[]lower_bound_; }
	if (range_interval_) { delete[]range_interval_; }
	if (all_best_position_) { delete[]all_best_position_; }
}

// 初始化所有粒子（没有更新全局最佳）
//CPU
void PSOOptimizer::InitialAllParticles()
{
	//在CPU中初始化所有粒子
	for (int i = 0; i < particle_num_; ++i)
	{
		InitialParticle(fitness_CPU, position_CPU, velocity_CPU, best_position_CPU, best_fitness_CPU, i);
	}
	
	//粒子从CPU->GPU
	cudaMemcpy(position_GPU, position_CPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyHostToDevice);
	cudaMemcpy(velocity_GPU, velocity_CPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyHostToDevice);
	cudaMemcpy(best_position_GPU, best_position_CPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyHostToDevice);
	cudaMemcpy(fitness_GPU, fitness_CPU, sizeof(double) * particle_num_ * fitness_count, cudaMemcpyHostToDevice);
	cudaMemcpy(best_fitness_GPU, best_fitness_CPU, sizeof(double) * particle_num_ * fitness_count, cudaMemcpyHostToDevice);
	//计算Fitness值 GPU
	GetFitness(dim_, fitness_count, fitness_CPU, position_CPU, velocity_CPU, best_position_CPU, best_fitness_CPU);
	//初始化pbest和gbest CPU
	for (int i = 0; i < particle_num_; ++i)
	{
		// 初始化个体最优位置
		for (int j = 0; j < dim_; j++)
		{
			best_position_CPU[i * dim_ + j] = position_CPU[i * dim_ + j];
		}
		// 初始化粒子个体历史最佳
		for (int j = 0; j < fitness_count; j++)
		{
			best_fitness_CPU[i * fitness_count + j] = fitness_CPU[i * fitness_count + j];
		}
	}

}

// 初始化Archive数组
void PSOOptimizer::InitialArchiveList()
{
	//同样，先进行一次深拷贝
	vector<Particle> particleList(particle_num_, Particle());
	for (int i = 0; i < particle_num_; i++)
	{
		particleList[i].position_ = new double[dim_];
		particleList[i].velocity_ = new double[dim_];
		particleList[i].best_position_ = new double[dim_];
		for (int j = 0; j < dim_; j++)
		{
			particleList[i].position_[j] = position_CPU[i * dim_ + j];
			particleList[i].velocity_[j] = velocity_CPU[i * dim_ + j];
			particleList[i].best_position_[j] = best_position_CPU[i * dim_ + j];
		}

		particleList[i].fitness_ = new double[fitness_count];
		particleList[i].best_fitness_ = new double[fitness_count];
		for (int j = 0; j < fitness_count; j++)
		{
			particleList[i].fitness_[j] = fitness_CPU[i * fitness_count + j];
			particleList[i].best_fitness_[j] = best_fitness_CPU[i * fitness_count + j];
		}
	}
	//vector<Particle> particleList(this->particles_GPU, this->particles_GPU + this->particle_num_);
	Pareto initPareto(particleList);
	this->archive_list = initPareto.GetPareto();
}

// 更新Archive数组 这个不是每个粒子都计算，也就是说，只需要一个线程就可以
// CPU
void PSOOptimizer::UpdateArchiveList() 
{
	//GPU->CPU
	cudaMemcpy(position_CPU, position_GPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyDeviceToHost);
	cudaMemcpy(velocity_CPU, velocity_GPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyDeviceToHost);
	cudaMemcpy(best_position_CPU, best_position_GPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyDeviceToHost);
	cudaMemcpy(fitness_CPU, fitness_GPU, sizeof(double) * particle_num_ * fitness_count, cudaMemcpyDeviceToHost);
	cudaMemcpy(best_fitness_CPU, best_fitness_GPU, sizeof(double) * particle_num_ * fitness_count, cudaMemcpyDeviceToHost);

	//首先，计算当前粒子群的pareto边界，将边界粒子加入到存档archiving中
	//进行一次深拷贝
	vector<Particle> particleList(particle_num_, Particle());
	for (int i = 0; i < particle_num_; i++)
	{
		particleList[i].position_ = new double[dim_];
		particleList[i].velocity_ = new double[dim_];
		particleList[i].best_position_ = new double[dim_];
		for (int j = 0; j < dim_; j++)
		{
			particleList[i].position_[j] = position_CPU[i * dim_ + j];
			particleList[i].velocity_[j] = velocity_CPU[i * dim_ + j];
			particleList[i].best_position_[j] = best_position_CPU[i * dim_ + j];
		}

		particleList[i].fitness_ = new double[fitness_count];
		particleList[i].best_fitness_ = new double[fitness_count];
		for (int j = 0; j < fitness_count; j++)
		{
			particleList[i].fitness_[j] = fitness_CPU[i * fitness_count + j];
			particleList[i].best_fitness_[j] = best_fitness_CPU[i * fitness_count + j];
		}
	}
	//vector<Particle> particleList(this->particles_CPU, this->particles_CPU + particle_num_);
	Pareto pareto1(particleList);
	vector<Particle> curParetos = pareto1.GetPareto();
	//其次，在存档中根据支配关系进行第二轮筛选，将非边界粒子去除
	vector<Particle> newParetos;
	curParetos.insert(curParetos.end(), this->archive_list.begin(), this->archive_list.end());//合并cur和原Archive
	Pareto pareto2(curParetos);
	vector<Particle> curArchives = pareto2.GetPareto();

	this->archive_list = curArchives;
}

// 初始化全局最优
void PSOOptimizer::InitGbest()
{
	GetGbest getG(this->archive_list, this->meshDivCount, this->lower_bound_, this->upper_bound_, this->dim_, this->particle_num_);
	Particle* gbestList = getG.getGbest();
	for (int i = 0; i < particle_num_; i++)
	{
		for (int j = 0; j < fitness_count; j++)
		{
			this->all_best_fitness_[i * fitness_count + j] = gbestList[i].best_fitness_[j];
		}
		for (int k = 0; k < dim_; k++)
		{
			this->all_best_position_[i * dim_ + k] = gbestList[i].best_position_[k];
		}
	}
}

void PSOOptimizer::GetFitness(int dim, int fitnessCount, double* fitness_GPU, double* position_GPU, double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU)
{
	FitnessFunction <<<blockSum, threadsPerBlock>>> (curr_iter_, max_iter_num_, bestPathInfoList, problemParas,
		dim, fitnessCount, fitness_GPU, position_GPU, velocity_GPU, best_position_GPU, best_fitness_GPU);
}

//先改第一个
void PSOOptimizer::UpdateAllParticles()
{
	//计算当前代的惯性系数
	double temp = curr_iter_ / (double)max_iter_num_;
	//temp *= temp;//系数变化
	w_ = wstart_ - (wstart_ - wend_) * temp;
	//更新当前代所有粒子
	//参数只能从外面传进去
	UpdateParticle<<<blockSum, threadsPerBlock>>>(curr_iter_, max_iter_num_, dim_, w_, C1_, C2_, dt_,
			bestPathInfoList, particles_GPU, randomNumList, range_interval_, upper_bound_, lower_bound_, all_best_position_, 
			size, spaceLength, workShopLength, workShopWidth);//这个是并行的

	//计算更新后粒子的适应度值数组
	//也要改成GPU并行的
	FitnessFunction<<<blockSum, threadsPerBlock>>>(curIterNum, maxIterNum, bestPathInfoList, problemParas,
		dim, fitnessCount, fitness_GPU, position_GPU, velocity_GPU, best_position_GPU, best_fitness_GPU);//这个也要改

	curr_iter_++;
}

//更新粒子&计算适应度
static __global__ void UpdateParticle(int curIterNum, int maxIterNum, int dim, int fitnessCount, double w_, double C1_, double C2_, double dt_, BestPathInfo* bestPathInfoList, 
	/*用于替代Particle Particle* particles_,*/double* fitness_GPU, double* position_GPU, double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU,
	curandState* globalState, double* randomNumList,double* range_interval_, double* upper_bound_, double* lower_bound_, double* all_best_position_, 
	/*ProblemParas problemParas*/Vector2* size, double* spaceLength, double workShopLength, double workShopWidth)
{
	
	//粒子的下标i需要自己计算
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	//先更新朝向，然后根据朝向调整粒子的范围
	for (int j = 2; j < dim; j += 3)
	{
		double last_position = position_GPU[i * dim + j];

		velocity_GPU[i * dim + j] = w_ * velocity_GPU[i * dim + j] +
			C1_ * GetDoubleRand() * (best_position_GPU[i * dim + j] - position_GPU[i * dim + j]) +
			C2_ * GetDoubleRand() * (all_best_position_[i * dim + j] - position_GPU[i * dim + j]);
		position_GPU[i * dim + j] += dt_ * velocity_GPU[i * dim + j];

		// 如果搜索区间有上下限限制
		if (upper_bound_ && lower_bound_)
		{
			if (position_GPU[i * dim + j] >= upper_bound_[j])//注意对于设备朝向，=也不行
			{
				double thre = createARandomNum(globalState, i);//直接生成一个随机数
				if (last_position >= upper_bound_[j] - 1)//注意upper_bound_[j]-1=3
				{
					position_GPU[i * dim + j] = createARandomNum(globalState, i) * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					position_GPU[i * dim + j] = upper_bound_[j] - (upper_bound_[j] - last_position) * createARandomNum(globalState, i);
				}
				else
				{
					position_GPU[i * dim + j] = upper_bound_[j] - 0.5;
				}
			}
			if (position_GPU[i * dim + j] < lower_bound_[j])
			{
				double thre = createARandomNum(globalState, i);
				if (last_position == lower_bound_[j])
				{
					position_GPU[i * dim + j] = createARandomNum(globalState, i) * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					position_GPU[i * dim + j] = lower_bound_[j] + (last_position - lower_bound_[j]) * createARandomNum(globalState, i);
				}
				else
				{
					position_GPU[i * dim + j] = lower_bound_[j];
				}
			}
		}
	}
	//根据朝向修改设备上下界范围
	for (int j = 2; j < dim; j += 3)
	{
		//double转int，转换为Direction，然后根据朝向重新计算设备尺寸和出入口
		//Rotate90或者Rotate270,修改上下限
		DeviceDirect curDirect = (DeviceDirect)(int)position_GPU[i * dim + j];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)//这一部分可能也要改（enum是C++语法）
		{
			//x和y
			lower_bound_[j - 2] = 0 + size[j / 3].y * 0.5 + spaceLength[j / 3];
			lower_bound_[j - 1] = 0 + size[j / 3].x * 0.5 + spaceLength[j / 3];

			upper_bound_[j - 2] = workShopLength - size[j / 3].y * 0.5 - spaceLength[j / 3];
			upper_bound_[j - 1] = workShopWidth - size[j / 3].x * 0.5 - spaceLength[j / 3];

		}
		else
		{
			//x和y
			lower_bound_[j - 2] = 0 + size[j / 3].x * 0.5 + spaceLength[j / 3];
			lower_bound_[j - 1] = 0 + size[j / 3].y * 0.5 + spaceLength[j / 3];

			upper_bound_[j - 2] = workShopLength - size[j / 3].x * 0.5 - spaceLength[j / 3];
			upper_bound_[j - 1] = workShopWidth - size[j / 3].y * 0.5 - spaceLength[j / 3];

		}
		range_interval_[j - 2] = upper_bound_[j - 2] - lower_bound_[j - 2];
		range_interval_[j - 1] = upper_bound_[j - 1] - lower_bound_[j - 1];
	}
	//cout << endl;
	for (int j = 0; j < dim; j++)
	{
		if (j % 3 != 2)
		{
			//保存上一次迭代结果的position和velocity
			double last_position = position_GPU[i * dim + j];

			velocity_GPU[i * dim + j] = w_ * velocity_GPU[i * dim + j] +
				C1_ * createARandomNum(globalState, i) * (best_position_GPU[i * dim + j] - position_GPU[i * dim + j]) +
				C2_ * createARandomNum(globalState, i) * (all_best_position_[i * dim + j] - position_GPU[i * dim + j]);
			position_GPU[i * dim + j] += dt_ * velocity_GPU[i * dim + j];

			// 如果搜索区间有上下限限制
			if (upper_bound_ && lower_bound_)
			{
				if (position_GPU[i * dim + j] > upper_bound_[j])
				{
					double thre = createARandomNum(globalState, i);
					if (last_position >= upper_bound_[j])
					{
						position_GPU[i * dim + j] = createARandomNum(globalState, i) * range_interval_[j] + lower_bound_[j];
					}
					else if (thre < 0.5)
					{
						position_GPU[i * dim + j] = upper_bound_[j] - abs(upper_bound_[j] - last_position) * createARandomNum(globalState, i);
					}
					else
					{
						position_GPU[i * dim + j] = upper_bound_[j];
					}
				}
				if (position_GPU[i * dim + j] < lower_bound_[j])
				{
					double thre = createARandomNum(globalState, i);
					if (last_position <= lower_bound_[j])
					{
						position_GPU[i * dim + j] = createARandomNum(globalState, i) * range_interval_[j] + lower_bound_[j];
					}
					else if (thre < 0.5)
					{
						position_GPU[i * dim + j] = lower_bound_[j] + abs(last_position - lower_bound_[j]) * createARandomNum(globalState, i);
					}
					else
					{
						position_GPU[i * dim + j] = lower_bound_[j];
					}
				}
			}
		}
		
	}
}

//更新Pbest的GPU函数
static __global__ void UpdatePbest_kernal(Particle* particles_, int dim_, int fitness_count, curandState* globalState)
{
	//i需要计算
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	//比较历史pbest和当前适应度，决定是否要更新
	if (ComparePbest(i, fitness_count, particles_[i].fitness_, particles_[i].best_fitness_, globalState));
	{
		for (int j = 0; j < fitness_count; j++)
		{
			particles_[i].best_fitness_[j] = particles_[i].fitness_[j];
		}
		for (int j = 0; j < dim_; j++)
		{
			particles_[i].best_position_[j] = particles_[i].position_[j];
		}
	}
}

//更新Pbest
void PSOOptimizer::UpdatePbest()
{
	//调用GPU函数
	UpdatePbest_kernal <<<blockSum, threadsPerBlock>>> (particles_GPU, dim_, fitness_count, globalState);
}
// 更新Gbest
// 这个是否需要改成GPU并行的
// 如果要改，涉及到数据转移，而且这个函数的代码也不多，不如先不改成GPU的
void PSOOptimizer::UpdateGbest()
{
	vector<Particle> tempArchiveL(this->archive_list);
	//GPU->CPU
	//cudaMemcpy(particles_CPU, particles_GPU, sizeof(Particle) * particle_num_, cudaMemcpyDeviceToHost);
	cudaMemcpy(position_CPU, position_GPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyDeviceToHost);
	cudaMemcpy(velocity_CPU, velocity_GPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyDeviceToHost);
	cudaMemcpy(best_position_CPU, best_position_GPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyDeviceToHost);
	cudaMemcpy(fitness_CPU, fitness_GPU, sizeof(double) * particle_num_ * fitness_count, cudaMemcpyDeviceToHost);
	cudaMemcpy(best_fitness_CPU, best_fitness_GPU, sizeof(double) * particle_num_ * fitness_count, cudaMemcpyDeviceToHost);

	cudaMemcpy(lower_bound_CPU, lower_bound_, sizeof(double) * dim_, cudaMemcpyDeviceToHost);
	cudaMemcpy(upper_bound_CPU, upper_bound_, sizeof(double) * dim_, cudaMemcpyDeviceToHost);
	GetGbest getG(tempArchiveL, this->meshDivCount, lower_bound_CPU, upper_bound_CPU, this->dim_, this->particle_num_);
	Particle* gbestList = getG.getGbest();

	//GPU
	UpdateGbest_GPU <<<blockSum, threadsPerBlock>>> (fitness_count, dim_, all_best_fitness_, all_best_position_, gbestList);
}

//更新Gbest的GPU函数
static __global__ void UpdateGbest_GPU(int fitness_count, int dim_, double* all_best_fitness_, double* all_best_position_, Particle* gbestList)
{
	//下标自己算
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	//更新
	for (int j = 0; j < fitness_count; j++)
	{
		all_best_fitness_[i * fitness_count + j] = gbestList[i].best_fitness_[j];
	}
	for (int k = 0; k < dim_; k++)
	{
		all_best_position_[i * dim_ + k] = gbestList[i].best_position_[k];
	}
}


// 比较两个粒子的适应度，判断是否完全支配，从而计算出pbest
static __device__ bool ComparePbest(int index, int fitness_count, double* fitness, double* pbestFitness, curandState* globalState)
{
	int numGreater = 0;
	int numLess = 0;
	for (int i = 0; i < fitness_count; i++)
	{
		if (fitness[i] < pbestFitness[i])
		{
			numGreater++;
		}
		if (fitness[i] > pbestFitness[i])
		{
			numLess++;
		}
	}
	//如果当前支配历史，更新
	if (numGreater > 0 && numLess == 0)
	{
		return true;
	}
	//如果历史支配当前粒子，不更新
	else if (numGreater == 0 and numLess > 0)
	{
		return false;
	}
	//如果互不支配，随机选择（适应度1的概率高点）
	else
	{
		double randomProb = createARandomNum(globalState, index);//产生随机小数
		if (fitness[0] < pbestFitness[0])
		{
			if (randomProb < 0.75)
			{
				return true;
			}
			else
			{
				return false;
			}
		} 
		else
		{
			if (randomProb > 0.5)
			{
				return true;
			} 
			else
			{
				return false;
			}
		}
	}
}

//初始化第i个粒子
//这里的problemParas可以直接用CPU的
void PSOOptimizer::InitialParticle(double* fitness_CPU, double* position_CPU, double* velocity_CPU, double* best_position_CPU, double* best_fitness_CPU, int i)
{
	#pragma region 初始化position/veloctiy值
	//先随机朝向，然后根据朝向调整粒子的范围
	for (int j = 2; j < dim_; j += 3)
	{
		position_CPU[i * dim_ + j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
		velocity_CPU[i * dim_ + j] = GetDoubleRand() * range_interval_[j] / 300;
	}
	//根据朝向修改设备上下界范围&设备坐标
	vector<Vector2> deviceSizeCopy;
	for (int i = 0; i < problemParas.DeviceSum; i++)
	{
		deviceSizeCopy.push_back(Vector2(problemParas.size[i].x, problemParas.size[i].y));
	}
	for (int j = 2; j < dim_; j += 3)
	{
		//double转int，转换为Direction，然后根据朝向重新计算设备尺寸和出入口
		//Rotate90或者Rotate270,修改上下限
		DeviceDirect curDirect = (DeviceDirect)(int)position_CPU[i * dim_ + j];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)
		{
			//x和y
			lower_bound_[j - 2] = 0 + problemParas.size[j / 3].y * 0.5 + problemParas.spaceLength[j / 3];
			lower_bound_[j - 1] = 0 + problemParas.size[j / 3].x * 0.5 + problemParas.spaceLength[j / 3];

			upper_bound_[j - 2] = problemParas.workShopLength - problemParas.size[j / 3].y * 0.5 - problemParas.spaceLength[j / 3];
			upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.size[j / 3].x * 0.5 - problemParas.spaceLength[j / 3];

			//size的x和y需要互换
			swap(deviceSizeCopy[j / 3].x, deviceSizeCopy[j / 3].y);
		}
		else
		{
			//x和y
			lower_bound_[j - 2] = 0 + problemParas.size[j / 3].x * 0.5 + problemParas.spaceLength[j / 3];
			lower_bound_[j - 1] = 0 + problemParas.size[j / 3].y * 0.5 + problemParas.spaceLength[j / 3];

			upper_bound_[j - 2] = problemParas.workShopLength - problemParas.size[j / 3].x * 0.5 - problemParas.spaceLength[j / 3];
			upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.size[j / 3].y * 0.5 - problemParas.spaceLength[j / 3];
		}
		range_interval_[j - 2] = upper_bound_[j - 2] - lower_bound_[j - 2];
		range_interval_[j - 1] = upper_bound_[j - 1] - lower_bound_[j - 1];
	}

	#pragma region 完全随机
	//for (int j = 0; j < dim_; j += 3) {
	//	particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
	//	particles_[i].position_[j + 1] = GetDoubleRand() * range_interval_[j + 1] + lower_bound_[j + 1];
	//	particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
	//	particles_[i].velocity_[j + 1] = GetDoubleRand() * range_interval_[j + 1] / 300;
	//}
	#pragma endregion

	#pragma region 考虑非重叠约束，这里分块产生随机点
	//(每隔1米产生一个随机点，只要找到一个随机点满足非重叠约束，就采用）朝向默认为0
	//新的随机：随机设备的摆放顺序
	vector<int> unmakeDeviceIndexVec;
	vector<int> madeDeviceIndexVec;
	for (int i = 0; i < problemParas.DeviceSum; i++)
	{
		unmakeDeviceIndexVec.push_back(i);
	}
	default_random_engine e;

	clock_t startTime, endTime;
	startTime = clock();//计时开始
	while (unmakeDeviceIndexVec.size() > 0)
	{
		endTime = clock();
		if ((static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC) > 1) {
			#pragma region 完全随机
			for (int j = 0; j < dim_; j += 3) {
				position_CPU[i * dim_ + j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
				position_CPU[i * dim_ + j + 1] = GetDoubleRand() * range_interval_[j + 1] + lower_bound_[j + 1];
				velocity_CPU[i * dim_ + j] = GetDoubleRand() * range_interval_[j] / 300;
				velocity_CPU[i * dim_ + j + 1] = GetDoubleRand() * range_interval_[j + 1] / 300;
			}
			#pragma endregion
			break;
		}
		//微秒级精度的随机数种子
		e.seed(GetRamdonSeed());
		uniform_int_distribution<unsigned> u(0, unmakeDeviceIndexVec.size() - 1);
		int randomVecIndex = u(e);
		int randomDeviceIndex = unmakeDeviceIndexVec[randomVecIndex];//得到设备的index
		int j = randomDeviceIndex * 3;

		double Xstart = lower_bound_[j];
		double Ystart = lower_bound_[j + 1];

		double tempPositionX = 0;
		double tempPositionY = 0;

		bool findParticle = false;
		while (Ystart <= upper_bound_[j + 1] - 1 && findParticle == false) {//X和Y要在范围内
			Xstart = lower_bound_[j];
			while (Xstart <= upper_bound_[j] - 1 && findParticle == false) {
				tempPositionX = GetDoubleRand() * 1.0 + Xstart;//得到Xstart到Xstart+1之间的一个随机数
				tempPositionY = GetDoubleRand() * 1.0 + Ystart;//得到Ystart到Ystart+1之间的一个随机数
				double halfX = deviceSizeCopy[j / 3].x * 0.5 + problemParas.spaceLength[j / 3];
				double halfY = deviceSizeCopy[j / 3].y * 0.5 + problemParas.spaceLength[j / 3];
				double tempLowX = tempPositionX - halfX;
				double tempUpX = tempPositionX + halfX;
				double tempLowY = tempPositionY - halfY;
				double tempUpY = tempPositionY + halfY;

				bool IsCross = false;
				//检查当前设备是否与其他重叠
				for (int k = 0; k < madeDeviceIndexVec.size(); k++)
				{
					int curDeviceIndex = madeDeviceIndexVec[k];
					int curDimIndex = curDeviceIndex * 3;

					double halfX1 = deviceSizeCopy[curDeviceIndex].x * 0.5 + problemParas.spaceLength[curDeviceIndex];
					double halfY1 = deviceSizeCopy[curDeviceIndex].y * 0.5 + problemParas.spaceLength[curDeviceIndex];

					double curLowX = position_CPU[i * dim_ + curDimIndex] - halfX1;
					double curUpX = position_CPU[i * dim_ + curDimIndex] + halfX1;
					double curLowY = position_CPU[i * dim_ + curDimIndex + 1] - halfY1;
					double curUpY = position_CPU[i * dim_ + curDimIndex + 1] + halfY1;
					//若发生重叠，退出
					if (IsRangeOverlap(tempLowX, tempUpX, curLowX, curUpX) && IsRangeOverlap(tempLowY, tempUpY, curLowY, curUpY)) {
						IsCross = true;
						break;
					}
				}
				//全部不重叠，给粒子赋值
				if (IsCross == false) {
					findParticle = true; 
					position_CPU[i * dim_ + j] = tempPositionX;
					position_CPU[i * dim_ + j + 1] = tempPositionY;
					velocity_CPU[i * dim_ + j] = GetDoubleRand() * range_interval_[j] / 300;
					velocity_CPU[i * dim_ + j + 1] = GetDoubleRand() * range_interval_[j + 1] / 300;

					//更新vec
					madeDeviceIndexVec.push_back(randomDeviceIndex);
					unmakeDeviceIndexVec.erase(unmakeDeviceIndexVec.begin() + randomVecIndex);

				}
				Xstart++;
				if (Xstart >= upper_bound_[j] - 1) {
					Ystart++;
				}
			}
		}
	}
	#pragma endregion
	
	#pragma endregion

}

