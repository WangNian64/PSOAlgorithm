#include "Tools.cu"
#include "APoint.h"
#include "DevicePara.h"
#include "PSO.h"


extern "C" void InitPointDirectArray(int* pointDirectArray)
{
	InitPointDirectArray_Kernal<<<1,1>>>(pointDirectArray);
}
__global__ void InitPointDirectArray_Kernal(int* pointDirectArray)
{
	pointDirectArray[0] = -1;
	pointDirectArray[1] = -1;
	pointDirectArray[2] = -1;
	pointDirectArray[3] = -1;
	pointDirectArray[4] = -1;

	pointDirectArray[5] = -1;
	pointDirectArray[6] = 1;
	pointDirectArray[7] = 2;
	pointDirectArray[8] = 3;
	pointDirectArray[9] = 4;

	pointDirectArray[10] = -1;
	pointDirectArray[11] = 5;
	pointDirectArray[12] = 6;
	pointDirectArray[13] = 7;
	pointDirectArray[14] = 8;

	pointDirectArray[15] = -1;
	pointDirectArray[16] = 9;
	pointDirectArray[17] = 10;
	pointDirectArray[18] = 11;
	pointDirectArray[19] = 12;

	pointDirectArray[20] = -1;
	pointDirectArray[21] = 13;
	pointDirectArray[22] = 14;
	pointDirectArray[23] = 15;
	pointDirectArray[24] = 16;
}



//初始化curBestPath_FitnessVal的值 1
extern "C" void InitCurBestPathFit(double* curBestPath_FitnessVal)
{
	InitCurBestPathFit_Kernal<<<1,1>>>(curBestPath_FitnessVal);
}
__global__ void InitCurBestPathFit_Kernal(double* curBestPath_FitnessVal)
{
	curBestPath_FitnessVal[0] = 1000000;
}




//更新迭代后的当前最优解信息 1
extern "C" void UpdateCurBestPathInfo(int fitnessCount, double* fitness_GPU, int* bestParticleIndex,
	int tempStrConveyorList_PointSum, int tempCurveConveyorList_PointSum,
	/*所有的输送线信息*/
	double* curBestFitnessVal, int inoutPSize, InoutPoint* inoutPoints, StraightConveyorInfo* strConveyorList,
	int* strConveyorListSum, Vector2Int* curveConveyorList, int* curveConveyorListSum,
	/*最佳的输送线信息*/
	double* curBestPath_FitnessVal, int curBestPath_InoutPSize, InoutPoint* curBestPath_InoutPoints, StraightConveyorInfo* curBestPath_StrConveyorList,
	int* curBestPath_StrConveyorListSum, Vector2Int* curBestPath_CurveConveyorList, int* curBestPath_CurveConveyorListSum)
{
	UpdateCurBestPathInfo_Kernal <<<1, 1>>> (fitnessCount, fitness_GPU, bestParticleIndex,
		tempStrConveyorList_PointSum, tempCurveConveyorList_PointSum,
		/*所有的输送线信息*/
		curBestFitnessVal, inoutPSize, inoutPoints, strConveyorList,
		strConveyorListSum, curveConveyorList, curveConveyorListSum,
		/*最佳的输送线信息*/
		curBestPath_FitnessVal, curBestPath_InoutPSize, curBestPath_InoutPoints, curBestPath_StrConveyorList,
		curBestPath_StrConveyorListSum, curBestPath_CurveConveyorList, curBestPath_CurveConveyorListSum);
}
//更新当前最佳输送线路信息(在FitnessFunc中调用),只需调用一次
__global__ void UpdateCurBestPathInfo_Kernal(int fitnessCount, double* fitness_GPU, int* bestParticleIndex,
	int tempStrConveyorList_PointSum, int tempCurveConveyorList_PointSum,
	/*所有的输送线信息*/
	double* curBestFitnessVal, int inoutPSize, InoutPoint* inoutPoints, StraightConveyorInfo* strConveyorList,
	int* strConveyorListSum, Vector2Int* curveConveyorList, int* curveConveyorListSum,
	/*最佳的输送线信息*/
	double* curBestPath_FitnessVal, int curBestPath_InoutPSize, InoutPoint* curBestPath_InoutPoints, StraightConveyorInfo* curBestPath_StrConveyorList,
	int* curBestPath_StrConveyorListSum, Vector2Int* curBestPath_CurveConveyorList, int* curBestPath_CurveConveyorListSum)
{
	//用curbest和所有的输送线最佳比较，更新curBest
	//这个应该只要更新一次
	//转移数据只能用复制的方法了
	int bestIndex = bestParticleIndex[0];//最佳粒子对应的下标
	if (curBestPath_FitnessVal[0] > fitness_GPU[bestIndex * fitnessCount + 0]) {
		curBestPath_FitnessVal[0] = fitness_GPU[bestIndex * fitnessCount + 0];//注意偏移值
		for (int i = 0; i < curBestPath_InoutPSize; i++)
		{
			curBestPath_InoutPoints[i] = inoutPoints[bestIndex * inoutPSize + i];
		}
		for (int i = 0; i < strConveyorListSum[bestIndex]; i++) {
			curBestPath_StrConveyorList[i] = strConveyorList[bestIndex * tempStrConveyorList_PointSum + i];
		}
		curBestPath_StrConveyorListSum[0] = strConveyorListSum[bestIndex];
		for (int i = 0; i < curveConveyorListSum[bestIndex]; i++) {
			curBestPath_CurveConveyorList[i] = curveConveyorList[bestIndex * tempCurveConveyorList_PointSum + i];
		}
		curBestPath_CurveConveyorListSum[0] = curveConveyorListSum[bestIndex];
	}
}







//更新粒子&计算适应度
extern "C" void UpdateParticle(int blockSum, int threadsPerBlock, int curIterNum, int maxIterNum, int dim, int fitnessCount, double w_, double C1_, double C2_, double dt_,
	/*用于替代Particle Particle* particles_,*/double* fitness_GPU, double* position_GPU, double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU,
	curandState* globalState, double* randomNumList, double* range_interval_, double* upper_bound_, double* lower_bound_, double* all_best_position_,
	/*ProblemParas problemParas*/Vector2* size, double* spaceLength, double workShopLength, double workShopWidth)
{
	UpdateParticle_Kernal << <blockSum, threadsPerBlock >> > (curIterNum, maxIterNum, dim, fitnessCount, w_, C1_, C2_, dt_,
		/*用于替代Particle Particle* particles_,*/fitness_GPU, position_GPU, velocity_GPU, best_position_GPU, best_fitness_GPU,
		globalState, randomNumList, range_interval_, upper_bound_, lower_bound_, all_best_position_,
		/*ProblemParas problemParas*/size, spaceLength, workShopLength, workShopWidth);
}
__global__ void UpdateParticle_Kernal(int curIterNum, int maxIterNum, int dim, int fitnessCount, double w_, double C1_, double C2_, double dt_,
	/*用于替代Particle Particle* particles_,*/double* fitness_GPU, double* position_GPU, double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU,
	curandState* globalState, double* randomNumList, double* range_interval_, double* upper_bound_, double* lower_bound_, double* all_best_position_,
	/*ProblemParas problemParas*/Vector2* size, double* spaceLength, double workShopLength, double workShopWidth)
{

	//粒子的下标i需要自己计算
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	//先更新朝向，然后根据朝向调整粒子的范围
	for (int j = 2; j < dim; j += 3)
	{
		double last_position = position_GPU[i * dim + j];

		velocity_GPU[i * dim + j] = w_ * velocity_GPU[i * dim + j] +
			C1_ * createARandomNum(globalState, i) * (best_position_GPU[i * dim + j] - position_GPU[i * dim + j]) +
			C2_ * createARandomNum(globalState, i) * (all_best_position_[i * dim + j] - position_GPU[i * dim + j]);
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
extern "C" UpdatePbest_GPU(int blockSum, int threadsPerBlock, int dim_, int fitness_count, double* fitness_GPU, double* position_GPU,
	double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU, curandState* globalState)
{
	UpdatePbest_Kernal<<<blockSum>>>
}
__global__ void UpdatePbest_Kernal(int dim_, int fitness_count, double* fitness_GPU, double* position_GPU,
	double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU, curandState* globalState)
{
	//i需要计算
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	//比较历史pbest和当前适应度，决定是否要更新
	if (ComparePbest(i, fitness_count, fitness_GPU + i * fitness_count, best_fitness_GPU + i * fitness_count, globalState));
	{
		for (int j = 0; j < fitness_count; j++)
		{
			best_fitness_GPU[i * fitness_count + j] = fitness_GPU[i * fitness_count + j];
			//particles_[i].best_fitness_[j] = particles_[i].fitness_[j];
		}
		for (int j = 0; j < dim_; j++)
		{
			best_position_GPU[i * dim_ + j] = position_GPU[i * dim_ + j];
			//particles_[i].best_position_[j] = particles_[i].position_[j];
		}
	}
}





//更新Gbest的GPU函数
__global__ void UpdateGbest_Kernal(int fitness_count, int dim_, double* all_best_fitness_, double* all_best_position_, Particle* gbestList)
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
__device__ bool ComparePbest(int index, int fitness_count, double* fitness, double* pbestFitness, curandState* globalState)
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
	else if (numGreater == 0 && numLess > 0)
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
