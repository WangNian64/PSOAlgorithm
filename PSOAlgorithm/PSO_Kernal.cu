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



//��ʼ��curBestPath_FitnessVal��ֵ 1
extern "C" void InitCurBestPathFit(double* curBestPath_FitnessVal)
{
	InitCurBestPathFit_Kernal<<<1,1>>>(curBestPath_FitnessVal);
}
__global__ void InitCurBestPathFit_Kernal(double* curBestPath_FitnessVal)
{
	curBestPath_FitnessVal[0] = 1000000;
}




//���µ�����ĵ�ǰ���Ž���Ϣ 1
extern "C" void UpdateCurBestPathInfo(int fitnessCount, double* fitness_GPU, int* bestParticleIndex,
	int tempStrConveyorList_PointSum, int tempCurveConveyorList_PointSum,
	/*���е���������Ϣ*/
	double* curBestFitnessVal, int inoutPSize, InoutPoint* inoutPoints, StraightConveyorInfo* strConveyorList,
	int* strConveyorListSum, Vector2Int* curveConveyorList, int* curveConveyorListSum,
	/*��ѵ���������Ϣ*/
	double* curBestPath_FitnessVal, int curBestPath_InoutPSize, InoutPoint* curBestPath_InoutPoints, StraightConveyorInfo* curBestPath_StrConveyorList,
	int* curBestPath_StrConveyorListSum, Vector2Int* curBestPath_CurveConveyorList, int* curBestPath_CurveConveyorListSum)
{
	UpdateCurBestPathInfo_Kernal <<<1, 1>>> (fitnessCount, fitness_GPU, bestParticleIndex,
		tempStrConveyorList_PointSum, tempCurveConveyorList_PointSum,
		/*���е���������Ϣ*/
		curBestFitnessVal, inoutPSize, inoutPoints, strConveyorList,
		strConveyorListSum, curveConveyorList, curveConveyorListSum,
		/*��ѵ���������Ϣ*/
		curBestPath_FitnessVal, curBestPath_InoutPSize, curBestPath_InoutPoints, curBestPath_StrConveyorList,
		curBestPath_StrConveyorListSum, curBestPath_CurveConveyorList, curBestPath_CurveConveyorListSum);
}
//���µ�ǰ���������·��Ϣ(��FitnessFunc�е���),ֻ�����һ��
__global__ void UpdateCurBestPathInfo_Kernal(int fitnessCount, double* fitness_GPU, int* bestParticleIndex,
	int tempStrConveyorList_PointSum, int tempCurveConveyorList_PointSum,
	/*���е���������Ϣ*/
	double* curBestFitnessVal, int inoutPSize, InoutPoint* inoutPoints, StraightConveyorInfo* strConveyorList,
	int* strConveyorListSum, Vector2Int* curveConveyorList, int* curveConveyorListSum,
	/*��ѵ���������Ϣ*/
	double* curBestPath_FitnessVal, int curBestPath_InoutPSize, InoutPoint* curBestPath_InoutPoints, StraightConveyorInfo* curBestPath_StrConveyorList,
	int* curBestPath_StrConveyorListSum, Vector2Int* curBestPath_CurveConveyorList, int* curBestPath_CurveConveyorListSum)
{
	//��curbest�����е���������ѱȽϣ�����curBest
	//���Ӧ��ֻҪ����һ��
	//ת������ֻ���ø��Ƶķ�����
	int bestIndex = bestParticleIndex[0];//������Ӷ�Ӧ���±�
	if (curBestPath_FitnessVal[0] > fitness_GPU[bestIndex * fitnessCount + 0]) {
		curBestPath_FitnessVal[0] = fitness_GPU[bestIndex * fitnessCount + 0];//ע��ƫ��ֵ
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







//��������&������Ӧ��
extern "C" void UpdateParticle(int blockSum, int threadsPerBlock, int curIterNum, int maxIterNum, int dim, int fitnessCount, double w_, double C1_, double C2_, double dt_,
	/*�������Particle Particle* particles_,*/double* fitness_GPU, double* position_GPU, double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU,
	curandState* globalState, double* randomNumList, double* range_interval_, double* upper_bound_, double* lower_bound_, double* all_best_position_,
	/*ProblemParas problemParas*/Vector2* size, double* spaceLength, double workShopLength, double workShopWidth)
{
	UpdateParticle_Kernal << <blockSum, threadsPerBlock >> > (curIterNum, maxIterNum, dim, fitnessCount, w_, C1_, C2_, dt_,
		/*�������Particle Particle* particles_,*/fitness_GPU, position_GPU, velocity_GPU, best_position_GPU, best_fitness_GPU,
		globalState, randomNumList, range_interval_, upper_bound_, lower_bound_, all_best_position_,
		/*ProblemParas problemParas*/size, spaceLength, workShopLength, workShopWidth);
}
__global__ void UpdateParticle_Kernal(int curIterNum, int maxIterNum, int dim, int fitnessCount, double w_, double C1_, double C2_, double dt_,
	/*�������Particle Particle* particles_,*/double* fitness_GPU, double* position_GPU, double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU,
	curandState* globalState, double* randomNumList, double* range_interval_, double* upper_bound_, double* lower_bound_, double* all_best_position_,
	/*ProblemParas problemParas*/Vector2* size, double* spaceLength, double workShopLength, double workShopWidth)
{

	//���ӵ��±�i��Ҫ�Լ�����
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	//�ȸ��³���Ȼ����ݳ���������ӵķ�Χ
	for (int j = 2; j < dim; j += 3)
	{
		double last_position = position_GPU[i * dim + j];

		velocity_GPU[i * dim + j] = w_ * velocity_GPU[i * dim + j] +
			C1_ * createARandomNum(globalState, i) * (best_position_GPU[i * dim + j] - position_GPU[i * dim + j]) +
			C2_ * createARandomNum(globalState, i) * (all_best_position_[i * dim + j] - position_GPU[i * dim + j]);
		position_GPU[i * dim + j] += dt_ * velocity_GPU[i * dim + j];

		// �����������������������
		if (upper_bound_ && lower_bound_)
		{
			if (position_GPU[i * dim + j] >= upper_bound_[j])//ע������豸����=Ҳ����
			{
				double thre = createARandomNum(globalState, i);//ֱ������һ�������
				if (last_position >= upper_bound_[j] - 1)//ע��upper_bound_[j]-1=3
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
	//���ݳ����޸��豸���½緶Χ
	for (int j = 2; j < dim; j += 3)
	{
		//doubleתint��ת��ΪDirection��Ȼ����ݳ������¼����豸�ߴ�ͳ����
		//Rotate90����Rotate270,�޸�������
		DeviceDirect curDirect = (DeviceDirect)(int)position_GPU[i * dim + j];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)//��һ���ֿ���ҲҪ�ģ�enum��C++�﷨��
		{
			//x��y
			lower_bound_[j - 2] = 0 + size[j / 3].y * 0.5 + spaceLength[j / 3];
			lower_bound_[j - 1] = 0 + size[j / 3].x * 0.5 + spaceLength[j / 3];

			upper_bound_[j - 2] = workShopLength - size[j / 3].y * 0.5 - spaceLength[j / 3];
			upper_bound_[j - 1] = workShopWidth - size[j / 3].x * 0.5 - spaceLength[j / 3];

		}
		else
		{
			//x��y
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
			//������һ�ε��������position��velocity
			double last_position = position_GPU[i * dim + j];

			velocity_GPU[i * dim + j] = w_ * velocity_GPU[i * dim + j] +
				C1_ * createARandomNum(globalState, i) * (best_position_GPU[i * dim + j] - position_GPU[i * dim + j]) +
				C2_ * createARandomNum(globalState, i) * (all_best_position_[i * dim + j] - position_GPU[i * dim + j]);
			position_GPU[i * dim + j] += dt_ * velocity_GPU[i * dim + j];

			// �����������������������
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





//����Pbest��GPU����
extern "C" UpdatePbest_GPU(int blockSum, int threadsPerBlock, int dim_, int fitness_count, double* fitness_GPU, double* position_GPU,
	double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU, curandState* globalState)
{
	UpdatePbest_Kernal<<<blockSum>>>
}
__global__ void UpdatePbest_Kernal(int dim_, int fitness_count, double* fitness_GPU, double* position_GPU,
	double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU, curandState* globalState)
{
	//i��Ҫ����
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	//�Ƚ���ʷpbest�͵�ǰ��Ӧ�ȣ������Ƿ�Ҫ����
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





//����Gbest��GPU����
__global__ void UpdateGbest_Kernal(int fitness_count, int dim_, double* all_best_fitness_, double* all_best_position_, Particle* gbestList)
{
	//�±��Լ���
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	//����
	for (int j = 0; j < fitness_count; j++)
	{
		all_best_fitness_[i * fitness_count + j] = gbestList[i].best_fitness_[j];
	}
	for (int k = 0; k < dim_; k++)
	{
		all_best_position_[i * dim_ + k] = gbestList[i].best_position_[k];
	}
}




// �Ƚ��������ӵ���Ӧ�ȣ��ж��Ƿ���ȫ֧�䣬�Ӷ������pbest
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
	//�����ǰ֧����ʷ������
	if (numGreater > 0 && numLess == 0)
	{
		return true;
	}
	//�����ʷ֧�䵱ǰ���ӣ�������
	else if (numGreater == 0 && numLess > 0)
	{
		return false;
	}
	//�������֧�䣬���ѡ����Ӧ��1�ĸ��ʸߵ㣩
	else
	{
		double randomProb = createARandomNum(globalState, index);//�������С��
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
