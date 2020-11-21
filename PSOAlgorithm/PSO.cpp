#pragma once
#include "Pareto.h"
#include "Archive.h"
#include "FitnessFunction.h"
#include <ctime>
// 构造函数(初始化各种算法的参数，给数组分配空间)
PSOOptimizer::PSOOptimizer(PSOPara* pso_para)
{
	particle_num_ = pso_para->particle_num_;
	max_iter_num_ = pso_para->max_iter_num_;
	dim_ = pso_para->dim_;
	fitness_count = pso_para->fitness_count_;
	curr_iter_ = 0;

	//dt_ = new double[dim_];
	//wstart_ = new double[dim_];
	//wend_ = new double[dim_];
	//C1_ = new double[dim_];
	//C2_ = new double[dim_];
	dt_ = pso_para->dt_;
	wstart_ = pso_para->wstart_;
	wend_ = pso_para->wend_;
	C1_ = pso_para->C1_;
	C2_ = pso_para->C2_;

	/*for (int i = 0; i < dim_; i++)
	{
		dt_[i] = pso_para->dt_[i];
		wstart_[i] = pso_para->wstart_[i];
		wend_[i] = pso_para->wend_[i];
		C1_[i] = pso_para->C1_[i];
		C2_[i] = pso_para->C2_[i];
	}*/

	if (pso_para->upper_bound_ && pso_para->lower_bound_)
	{
		upper_bound_ = new double[dim_];
		lower_bound_ = new double[dim_];
		range_interval_ = new double[dim_];

		for (int i = 0; i < dim_; i++)
		{
			upper_bound_[i] = pso_para->upper_bound_[i];
			lower_bound_[i] = pso_para->lower_bound_[i];
			range_interval_[i] = upper_bound_[i] - lower_bound_[i];
		}
	}

	particles_ = new Particle[particle_num_];
	//w_ = new double[dim_];

	all_best_position_ = new double[particle_num_ * dim_];

	all_best_fitness_ = new double[particle_num_ * fitness_count];

	meshDivCount = pso_para->mesh_div_count;
	problemParas = pso_para->problemParas;//布局问题的参数

	this->archiveMaxCount = pso_para->archive_max_count;

	this->bestPathInfoList = new BestPathInfo[fitness_count];//默认初始化
}

//用另一个pso对象初始化一个GPU上的pso对象
PSOOptimizer::PSOOptimizer(const PSOOptimizer& obj, int index)
{
	//分配内存
	particle_num_ = obj.particle_num_;
	max_iter_num_ = obj.max_iter_num_;
	dim_ = obj.dim_;
	fitness_count = obj.fitness_count;
	curr_iter_ = 0;

	//dt_ = new double[dim_];
	//wstart_ = new double[dim_];
	//wend_ = new double[dim_];
	//C1_ = new double[dim_];
	//C2_ = new double[dim_];
	dt_ = obj.dt_;
	wstart_ = obj.wstart_;
	wend_ = obj.wend_;
	C1_ = obj.C1_;
	C2_ = obj.C2_;

	//相当于重新分配内存了
	CUDA_SAFE_CALL(cudaMalloc((void**)& upper_bound_, sizeof(double) * dim_));
	CUDA_SAFE_CALL(cudaMalloc((void**)& lower_bound_, sizeof(double) * dim_));
	CUDA_SAFE_CALL(cudaMalloc((void**)& range_interval_, sizeof(double) * dim_));

	CUDA_SAFE_CALL(cudaMalloc((void**)& all_best_position_, sizeof(double) * particle_num_ * dim_));
	CUDA_SAFE_CALL(cudaMalloc((void**)& all_best_fitness_, sizeof(double) * particle_num_ * fitness_count));
	//只有并行的数据用GPU分配内存


	meshDivCount = obj.meshDivCount;
	this->archiveMaxCount = obj.archiveMaxCount;


	problemParas = obj.problemParas;//布局问题的参数,也需要在GPU中

	this->bestPathInfoList = new BestPathInfo[fitness_count];//默认初始化
}
PSOOptimizer::~PSOOptimizer()
{
	if (particles_) { delete[]particles_; }
	if (upper_bound_) { delete[]upper_bound_; }
	if (lower_bound_) { delete[]lower_bound_; }
	if (range_interval_) { delete[]range_interval_; }
	//if (dt_) { delete[]dt_; }
	//if (wstart_) { delete[]wstart_; }
	//if (wend_) { delete[]wend_; }
	//if (w_) { delete[]w_; }
	//if (C1_) { delete[]C1_; }
	//if (C2_) { delete[]C2_; }
	if (all_best_position_) { delete[]all_best_position_; }
}

// 初始化所有粒子（没有更新全局最佳）
void PSOOptimizer::InitialAllParticles()
{
	// 先给所有粒子分配内存
	for (int i = 0; i < particle_num_; ++i) {
		particles_[i].dim_ = dim_;
		particles_[i].fitnessCount = this->fitness_count;

		particles_[i].position_ = new double[dim_];
		particles_[i].velocity_ = new double[dim_];
		particles_[i].best_position_ = new double[dim_];
		particles_[i].fitness_ = new double[fitness_count];
		particles_[i].best_fitness_ = new double[fitness_count];
	}
	for (int i = 0; i < particle_num_; ++i)
	{
		cout << i << endl;
		InitialParticle(i);
	}
	cout << "初始化完成";
}

// 初始化Archive数组
void PSOOptimizer::InitialArchiveList()
{
	vector<Particle> particleList(this->particles_, this->particles_ + this->particle_num_);
	Pareto initPareto(particleList);
	this->archive_list = initPareto.GetPareto();
}

// 更新Archive数组
void PSOOptimizer::UpdateArchiveList() 
{
	//首先，计算当前粒子群的pareto边界，将边界粒子加入到存档archiving中
	vector<Particle> particleList(this->particles_, this->particles_ + particle_num_);
	Pareto pareto1(particleList);
	vector<Particle> curParetos = pareto1.GetPareto();
	//其次，在存档中根据支配关系进行第二轮筛选，将非边界粒子去除
	vector<Particle> newParetos;
	curParetos.insert(curParetos.end(), this->archive_list.begin(), this->archive_list.end());//合并cur和原Archive
	Pareto pareto2(curParetos);
	vector<Particle> curArchives = pareto2.GetPareto();
	//判断存档数量是否已超过存档阈值，如果是，清除掉一部分，拥挤度较高的那部分被清除的概率更大
	if (curArchives.size() > this->archiveMaxCount)
	{
		GetGbest clear(curArchives, this->meshDivCount, this->lower_bound_, this->upper_bound_, this->dim_, this->particle_num_);
		curArchives = clear.Clear(this->archiveMaxCount);
	}
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

void PSOOptimizer::GetFitness(Particle& particle)
{
	FitnessFunction(curr_iter_, max_iter_num_, bestPathInfoList, problemParas, particle);
}

void PSOOptimizer::UpdateAllParticles()
{
	GetInertialWeight();//计算当前代的惯性系数
	//更新当前代所有粒子
	for (int i = 0; i < particle_num_; i++)
	{
		UpdateParticle(i);
	}
	curr_iter_++;
}

//更新i索引的粒子&计算适应度值数组
void PSOOptimizer::UpdateParticle(int i)
{
	//先更新朝向，然后根据朝向调整粒子的范围
	for (int j = 2; j < dim_; j += 3)
	{
		double last_position = particles_[i].position_[j];

		particles_[i].velocity_[j] = w_ * particles_[i].velocity_[j] +
			C1_ * GetDoubleRand() * (particles_[i].best_position_[j] - particles_[i].position_[j]) +
			C2_ * GetDoubleRand() * (all_best_position_[i * dim_ + j] - particles_[i].position_[j]);
		particles_[i].position_[j] += dt_ * particles_[i].velocity_[j];


		// 如果搜索区间有上下限限制
		if (upper_bound_ && lower_bound_)
		{
			if (particles_[i].position_[j] >= upper_bound_[j])//注意对于设备朝向，=也不行
			{
				double thre = GetDoubleRand(99);
				if (last_position >= upper_bound_[j] - 1)//注意upper_bound_[j]-1=3
				{
					particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					particles_[i].position_[j] = upper_bound_[j] - (upper_bound_[j] - last_position) * GetDoubleRand();
				}
				else
				{
					particles_[i].position_[j] = upper_bound_[j] - 0.5;
				}
			}
			if (particles_[i].position_[j] < lower_bound_[j])
			{
				double thre = GetDoubleRand(99);
				if (last_position == lower_bound_[j])
				{
					particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					particles_[i].position_[j] = lower_bound_[j] + (last_position - lower_bound_[j]) * GetDoubleRand();
				}
				else
				{
					particles_[i].position_[j] = lower_bound_[j];
				}
			}
		}
	}
	//根据朝向修改设备上下界范围
	for (int j = 2; j < dim_; j += 3)
	{
		//double转int，转换为Direction，然后根据朝向重新计算设备尺寸和出入口
		//Rotate90或者Rotate270,修改上下限
		DeviceDirect curDirect = (DeviceDirect)(int)particles_[i].position_[j];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)
		{
			//x和y
			lower_bound_[j - 2] = 0 + problemParas.deviceParaList[j / 3].size.y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
			lower_bound_[j - 1] = 0 + problemParas.deviceParaList[j / 3].size.x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;

			upper_bound_[j - 2] = problemParas.workShopLength - problemParas.deviceParaList[j / 3].size.y * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;
			upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.deviceParaList[j / 3].size.x * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;

		}
		else
		{
			//x和y
			lower_bound_[j - 2] = 0 + problemParas.deviceParaList[j / 3].size.x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
			lower_bound_[j - 1] = 0 + problemParas.deviceParaList[j / 3].size.y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;

			upper_bound_[j - 2] = problemParas.workShopLength - problemParas.deviceParaList[j / 3].size.x * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;
			upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.deviceParaList[j / 3].size.y * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;

		}
		range_interval_[j - 2] = upper_bound_[j - 2] - lower_bound_[j - 2];
		range_interval_[j - 1] = upper_bound_[j - 1] - lower_bound_[j - 1];
	}
	//cout << endl;
	for (int j = 0; j < dim_; j++)
	{
		if (j % 3 != 2)
		{
			//保存上一次迭代结果的position和velocity
			double last_position = particles_[i].position_[j];

			particles_[i].velocity_[j] = w_ * particles_[i].velocity_[j] +
				C1_ * GetDoubleRand() * (particles_[i].best_position_[j] - particles_[i].position_[j]) +
				C2_ * GetDoubleRand() * (all_best_position_[i * dim_ + j] - particles_[i].position_[j]);
			particles_[i].position_[j] += dt_ * particles_[i].velocity_[j];

			// 如果搜索区间有上下限限制
			if (upper_bound_ && lower_bound_)
			{
				if (particles_[i].position_[j] > upper_bound_[j])
				{
					double thre = GetDoubleRand(99);
					if (last_position >= upper_bound_[j])
					{
						particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
					}
					else if (thre < 0.5)
					{
						particles_[i].position_[j] = upper_bound_[j] - abs(upper_bound_[j] - last_position) * GetDoubleRand();
					}
					else
					{
						particles_[i].position_[j] = upper_bound_[j];
					}
				}
				if (particles_[i].position_[j] < lower_bound_[j])
				{
					double thre = GetDoubleRand(99);
					if (last_position <= lower_bound_[j])
					{
						particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
					}
					else if (thre < 0.5)
					{
						particles_[i].position_[j] = lower_bound_[j] + abs(last_position - lower_bound_[j]) * GetDoubleRand();
					}
					else
					{
						particles_[i].position_[j] = lower_bound_[j];
					}
				}
			}
		}
		
	}

	//计算更新后粒子的适应度值数组
	GetFitness(particles_[i]);
}

//更新Pbest
void PSOOptimizer::UpdatePbest()
{
	for (int i = 0; i < this->particle_num_; i++)
	{
		//比较历史pbest和当前适应度，决定是否要更新
		if (ComparePbest(this->particles_[i].fitness_, this->particles_[i].best_fitness_))
		{
			for (int j = 0; j < fitness_count; j++)
			{
				this->particles_[i].best_fitness_[j] = this->particles_[i].fitness_[j];
			}
			for (int j = 0; j < dim_; j++)
			{
				this->particles_[i].best_position_[j] = this->particles_[i].position_[j];
			}
		}
	}
}

// 更新Gbest
void PSOOptimizer::UpdateGbest()
{
	vector<Particle> tempArchiveL(this->archive_list);
	GetGbest getG(tempArchiveL, this->meshDivCount, this->lower_bound_, this->upper_bound_, this->dim_, this->particle_num_);
	Particle* gbestList = getG.getGbest();
	for (int i = 0; i < particle_num_; i++)
	{
		for (int j = 0; j < fitness_count; j++)
		{
			this->all_best_fitness_[i * fitness_count + j] = gbestList[i].best_fitness_[j];
		}
		//cout << this->all_best_fitness_[i][0] << "," << this->all_best_fitness_[i][1] << endl; 
		for (int k = 0; k < dim_; k++)
		{
			this->all_best_position_[i * dim_ + k] = gbestList[i].best_position_[k];
		}
	}
	//cout << endl;
}

// 比较两个粒子的适应度，判断是否完全支配，从而计算出pbest
bool PSOOptimizer::ComparePbest(double* fitness, double* pbestFitness)
{
	int numGreater = 0;
	int numLess = 0;
	for (int i = 0; i < this->fitness_count; i++)
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
		double randomProb = rand() % 1000 / (double)1000;//产生随机小数
		if (fitness[0] < pbestFitness[0])
		{
			return randomProb < 0.75 ? true : false;
		} 
		else
		{
			return randomProb > 0.5 ? true : false;
		}
	}
}

//获取惯性系数
void PSOOptimizer::GetInertialWeight()
{
	double temp = curr_iter_ / (double)max_iter_num_;
	//temp *= temp;
	w_ = wstart_ - (wstart_ - wend_) * temp;
}

//初始化第i个粒子
void PSOOptimizer::InitialParticle(int i)
{
	#pragma region 初始化position/veloctiy值
	//先随机朝向，然后根据朝向调整粒子的范围
	for (int j = 2; j < dim_; j += 3)
	{
		particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
		particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
	}
	//根据朝向修改设备上下界范围&设备坐标
	vector<Vector2> deviceSizeCopy;
	for (int i = 0; i < problemParas.DeviceSum; i++)
	{
		deviceSizeCopy.push_back(Vector2(problemParas.deviceParaList[i].size.x, problemParas.deviceParaList[i].size.y));
	}
	for (int j = 2; j < dim_; j += 3)
	{
		//double转int，转换为Direction，然后根据朝向重新计算设备尺寸和出入口
		//Rotate90或者Rotate270,修改上下限
		DeviceDirect curDirect = (DeviceDirect)(int)particles_[i].position_[j];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)
		{
			//x和y
			lower_bound_[j - 2] = 0 + problemParas.deviceParaList[j / 3].size.y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
			lower_bound_[j - 1] = 0 + problemParas.deviceParaList[j / 3].size.x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;

			upper_bound_[j - 2] = problemParas.workShopLength - problemParas.deviceParaList[j / 3].size.y * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;
			upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.deviceParaList[j / 3].size.x * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;

			//size的x和y需要互换
			swap(deviceSizeCopy[j / 3].x, deviceSizeCopy[j / 3].y);
		}
		else
		{
			//x和y
			lower_bound_[j - 2] = 0 + problemParas.deviceParaList[j / 3].size.x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
			lower_bound_[j - 1] = 0 + problemParas.deviceParaList[j / 3].size.y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;

			upper_bound_[j - 2] = problemParas.workShopLength - problemParas.deviceParaList[j / 3].size.x * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;
			upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.deviceParaList[j / 3].size.y * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;

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
				particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
				particles_[i].position_[j + 1] = GetDoubleRand() * range_interval_[j + 1] + lower_bound_[j + 1];
				particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
				particles_[i].velocity_[j + 1] = GetDoubleRand() * range_interval_[j + 1] / 300;
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
				double halfX = deviceSizeCopy[j / 3].x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
				double halfY = deviceSizeCopy[j / 3].y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
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

					double halfX1 = deviceSizeCopy[curDeviceIndex].x * 0.5 + problemParas.deviceParaList[curDeviceIndex].spaceLength;
					double halfY1 = deviceSizeCopy[curDeviceIndex].y * 0.5 + problemParas.deviceParaList[curDeviceIndex].spaceLength;

					double curLowX = particles_[i].position_[curDimIndex] - halfX1;
					double curUpX = particles_[i].position_[curDimIndex] + halfX1;
					double curLowY = particles_[i].position_[curDimIndex + 1] - halfY1;
					double curUpY = particles_[i].position_[curDimIndex + 1] + halfY1;
					//若发生重叠，退出
					if (IsRangeOverlap(tempLowX, tempUpX, curLowX, curUpX) && IsRangeOverlap(tempLowY, tempUpY, curLowY, curUpY)) {
						IsCross = true;
						break;
					}
				}
				//全部不重叠，给粒子赋值
				if (IsCross == false) {
					findParticle = true;
					particles_[i].position_[j] = tempPositionX;
					particles_[i].position_[j + 1] = tempPositionY;
					particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
					particles_[i].velocity_[j + 1] = GetDoubleRand() * range_interval_[j + 1] / 300;

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
	//计算自身的适应度值
	GetFitness(particles_[i]); 
	// 初始化个体最优位置
	for (int j = 0; j < dim_; j++)
	{
		particles_[i].best_position_[j] = particles_[i].position_[j];
	}
	// 初始化粒子个体历史最佳
	for (int j = 0; j < fitness_count; j++) 
	{
		particles_[i].best_fitness_[j] = particles_[i].fitness_[j];
	}
}

