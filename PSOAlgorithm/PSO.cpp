#pragma once
#include "Pareto.h"
#include "Archive.h"
#include "FitnessFunction.h"
#include "Tools.h"
#include <ctime>
// ���캯��(��ʼ�������㷨�Ĳ��������������ռ�)
PSOOptimizer::PSOOptimizer(PSOPara* pso_para)
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



	//��CPU���ӷ���ռ�
	position_CPU = (double*)malloc(sizeof(double) * particle_num_ * dim_);
	velocity_CPU = (double*)malloc(sizeof(double) * particle_num_ * dim_);
	best_position_CPU = (double*)malloc(sizeof(double) * particle_num_ * dim_);

	fitness_CPU = (double*)malloc(sizeof(double) * particle_num_ * fitness_count);
	best_fitness_CPU = (double*)malloc(sizeof(double) * particle_num_ * fitness_count);

	//��GPU���ӷ���ռ�
	cudaMalloc((void**)& position_GPU, sizeof(double) * particle_num_ * dim_);
	cudaMalloc((void**)& velocity_GPU, sizeof(double) * particle_num_ * dim_);
	cudaMalloc((void**)& best_position_GPU, sizeof(double) * particle_num_ * dim_);

	cudaMalloc((void**)& fitness_GPU, sizeof(double) * particle_num_ * fitness_count);
	cudaMalloc((void**)& best_fitness_GPU, sizeof(double) * particle_num_ * fitness_count);



	//��ʼ�����������
	cudaMalloc(&devStates, particle_num_ * sizeof(curandState));
	initRandomGenerator <<< 1, particle_num_ >>> (devStates, unsigned(time(NULL)));//time��Ϊ���������

	//��������������GPU�ռ�
	cudaMalloc((void**)& randomNumList, sizeof(double) * particle_num_);


	//GPU�ڴ����
	cudaMalloc((void**)& upper_bound_, sizeof(double) * dim_);
	cudaMalloc((void**)& lower_bound_, sizeof(double) * dim_);
	cudaMalloc((void**)& range_interval_, sizeof(double) * dim_);
	//����CPU->GPU
	cudaMemcpy(upper_bound_, pso_para->upper_bound_, sizeof(double) * dim_, cudaMemcpyHostToDevice);
	cudaMemcpy(lower_bound_, pso_para->lower_bound_, sizeof(double) * dim_, cudaMemcpyHostToDevice);
	cudaMemcpy(range_interval_, pso_para->range_interval_, sizeof(double) * dim_, cudaMemcpyHostToDevice);


	//CPU�汾��bound
	lower_bound_CPU = new double[dim_];
	upper_bound_CPU = new double[dim_];


	//ԭ���Ƕ�ά�ģ���Ϊһά
	cudaMalloc((void**)& all_best_position_, sizeof(double) * particle_num_ * dim_);
	cudaMalloc((void**)& all_best_fitness_, sizeof(double) * particle_num_ * fitness_count);


	//ProblemParas
	//CPU->GPU
	//int deviceParaSize = accumDeviceParaSize;
	//int cargoTypeSize = accumCargoTypeSize;
	//int otherSize = sizeof(int) * 8 + sizeof(double) * 9 + sizeof(Vector2) * 2;
	//int totalSize = deviceParaSize + cargoTypeSize + otherSize;
	//cudaMalloc((void**)& problemParas, totalSize);
	//cudaMemcpy(problemParas, pso_para->problemParas, totalSize, cudaMemcpyHostToDevice);




	this->bestPathInfoList = new BestPathInfo[fitness_count];//Ĭ�ϳ�ʼ��(��Ҳ�Ǹ����⣬��Ϊlist�ڲ������ӽṹҲ��*����
}

PSOOptimizer::~PSOOptimizer()//������������Ҫ�޸�
{
	if (upper_bound_) { delete[]upper_bound_; }
	if (lower_bound_) { delete[]lower_bound_; }
	if (range_interval_) { delete[]range_interval_; }
	if (all_best_position_) { delete[]all_best_position_; }
}

// ��ʼ���������ӣ�û�и���ȫ����ѣ�
void PSOOptimizer::InitialAllParticles()
{
	for (int i = 0; i < particle_num_; ++i)
	{
		InitialParticle(fitness_CPU, position_CPU, velocity_CPU, best_position_CPU, best_fitness_CPU, i);
	}
	
	//CPU->GPU
	cudaMemcpy(position_GPU, position_CPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyHostToDevice);
	cudaMemcpy(velocity_GPU, velocity_CPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyHostToDevice);
	cudaMemcpy(best_position_GPU, best_position_CPU, sizeof(double) * particle_num_ * dim_, cudaMemcpyHostToDevice);

	cudaMemcpy(fitness_GPU, fitness_CPU, sizeof(double) * particle_num_ * fitness_count, cudaMemcpyHostToDevice);
	cudaMemcpy(best_fitness_GPU, best_fitness_CPU, sizeof(double) * particle_num_ * fitness_count, cudaMemcpyHostToDevice);
	//����Fitnessֵ
	for (int i = 0; i < particle_num_; ++i)
	{
		GetFitness(dim_, fitness_count, fitness_CPU, position_CPU, velocity_CPU, best_position_CPU, best_fitness_CPU, i);
		// ��ʼ����������λ��
		for (int j = 0; j < dim_; j++)
		{
			best_position_CPU[i * dim_ + j] = position_CPU[i * dim_ + j];
		}
		// ��ʼ�����Ӹ�����ʷ���
		for (int j = 0; j < fitness_count; j++)
		{
			best_fitness_CPU[i * fitness_count + j] = fitness_CPU[i * fitness_count + j];
		}
	}

}

// ��ʼ��Archive����
void PSOOptimizer::InitialArchiveList()
{
	vector<Particle> particleList(this->particles_GPU, this->particles_GPU + this->particle_num_);
	Pareto initPareto(particleList);
	this->archive_list = initPareto.GetPareto();
}

// ����Archive���� �������ÿ�����Ӷ����㣬Ҳ����˵��ֻ��Ҫһ���߳̾Ϳ���
// CPU
void PSOOptimizer::UpdateArchiveList() 
{
	//���Ƚ����Ӵ�GPU->CPU
	cudaMemcpy(particles_CPU, particles_GPU, particle_size * particle_num_, cudaMemcpyDeviceToHost);

	//���ȣ����㵱ǰ����Ⱥ��pareto�߽磬���߽����Ӽ��뵽�浵archiving��
	//������Ҫ���
	vector<Particle> particleList(this->particles_CPU, this->particles_CPU + particle_num_);
	Pareto pareto1(particleList);
	vector<Particle> curParetos = pareto1.GetPareto();
	//��Σ��ڴ浵�и���֧���ϵ���еڶ���ɸѡ�����Ǳ߽�����ȥ��
	vector<Particle> newParetos;
	curParetos.insert(curParetos.end(), this->archive_list.begin(), this->archive_list.end());//�ϲ�cur��ԭArchive
	Pareto pareto2(curParetos);
	vector<Particle> curArchives = pareto2.GetPareto();

	this->archive_list = curArchives;
}

// ��ʼ��ȫ������
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

//i�����ӵ��±�
void PSOOptimizer::GetFitness(int dim, int fitnessCount, double* fitness_GPU, double* position_GPU, double* velocity_GPU, double* best_position_GPU, double* best_fitness_GPU, int i)
{
	FitnessFunction(curr_iter_, max_iter_num_, bestPathInfoList, problemParas, 
		dim, fitnessCount, fitness_GPU, position_GPU, velocity_GPU, best_position_GPU, best_fitness_GPU, i);
}

//�ȸĵ�һ����
void PSOOptimizer::UpdateAllParticles()
{
	//���㵱ǰ���Ĺ���ϵ��
	double temp = curr_iter_ / (double)max_iter_num_;
	//temp *= temp;//ϵ���仯
	w_ = wstart_ - (wstart_ - wend_) * temp;
	//���µ�ǰ����������
	//����ֻ�ܴ����洫��ȥ
	UpdateParticle<<<blockSum, threadsPerBlock>>>(curr_iter_, max_iter_num_, dim_, w_, C1_, C2_, dt_,
			bestPathInfoList, particles_GPU, randomNumList, range_interval_, upper_bound_, lower_bound_, all_best_position_, problemParas);//����ǲ��е�
	curr_iter_++;
}

//��������&������Ӧ��
static __global__ void UpdateParticle(int curIterNum, int maxIterNum, int dim_, double w_, double C1_, double C2_, double dt_,
	BestPathInfo* bestPathInfoList, Particle* particles_, curandState* globalState, double* randomNumList, 
	double* range_interval_, double* upper_bound_, double* lower_bound_, double* all_best_position_, ProblemParas problemParas)
{
	//���ӵ��±�i��Ҫ�Լ�����
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	//�ȸ��³���Ȼ����ݳ���������ӵķ�Χ
	for (int j = 2; j < dim_; j += 3)
	{
		double last_position = particles_[i].position_[j];

		particles_[i].velocity_[j] = w_ * particles_[i].velocity_[j] +
			C1_ * GetDoubleRand() * (particles_[i].best_position_[j] - particles_[i].position_[j]) +
			C2_ * GetDoubleRand() * (all_best_position_[i * dim_ + j] - particles_[i].position_[j]);
		particles_[i].position_[j] += dt_ * particles_[i].velocity_[j];

		// �����������������������
		if (upper_bound_ && lower_bound_)
		{
			if (particles_[i].position_[j] >= upper_bound_[j])//ע������豸����=Ҳ����
			{
				double thre = createARandomNum(globalState, i);//ֱ������һ�������
				if (last_position >= upper_bound_[j] - 1)//ע��upper_bound_[j]-1=3
				{
					particles_[i].position_[j] = createARandomNum(globalState, i) * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					particles_[i].position_[j] = upper_bound_[j] - (upper_bound_[j] - last_position) * createARandomNum(globalState, i);
				}
				else
				{
					particles_[i].position_[j] = upper_bound_[j] - 0.5;
				}
			}
			if (particles_[i].position_[j] < lower_bound_[j])
			{
				double thre = createARandomNum(globalState, i);
				if (last_position == lower_bound_[j])
				{
					particles_[i].position_[j] = createARandomNum(globalState, i) * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					particles_[i].position_[j] = lower_bound_[j] + (last_position - lower_bound_[j]) * createARandomNum(globalState, i);
				}
				else
				{
					particles_[i].position_[j] = lower_bound_[j];
				}
			}
		}
	}
	//���ݳ����޸��豸���½緶Χ
	for (int j = 2; j < dim_; j += 3)
	{
		//doubleתint��ת��ΪDirection��Ȼ����ݳ������¼����豸�ߴ�ͳ����
		//Rotate90����Rotate270,�޸�������
		DeviceDirect curDirect = (DeviceDirect)(int)particles_[i].position_[j];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)//��һ���ֿ���ҲҪ�ģ�enum��C++�﷨��
		{
			//x��y
			lower_bound_[j - 2] = 0 + problemParas.deviceParaList[j / 3].size.y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
			lower_bound_[j - 1] = 0 + problemParas.deviceParaList[j / 3].size.x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;

			upper_bound_[j - 2] = problemParas.workShopLength - problemParas.deviceParaList[j / 3].size.y * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;
			upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.deviceParaList[j / 3].size.x * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;

		}
		else
		{
			//x��y
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
			//������һ�ε��������position��velocity
			double last_position = particles_[i].position_[j];

			particles_[i].velocity_[j] = w_ * particles_[i].velocity_[j] +
				C1_ * createARandomNum(globalState, i) * (particles_[i].best_position_[j] - particles_[i].position_[j]) +
				C2_ * createARandomNum(globalState, i) * (all_best_position_[i * dim_ + j] - particles_[i].position_[j]);
			particles_[i].position_[j] += dt_ * particles_[i].velocity_[j];

			// �����������������������
			if (upper_bound_ && lower_bound_)
			{
				if (particles_[i].position_[j] > upper_bound_[j])
				{
					double thre = createARandomNum(globalState, i);
					if (last_position >= upper_bound_[j])
					{
						particles_[i].position_[j] = createARandomNum(globalState, i) * range_interval_[j] + lower_bound_[j];
					}
					else if (thre < 0.5)
					{
						particles_[i].position_[j] = upper_bound_[j] - abs(upper_bound_[j] - last_position) * createARandomNum(globalState, i);
					}
					else
					{
						particles_[i].position_[j] = upper_bound_[j];
					}
				}
				if (particles_[i].position_[j] < lower_bound_[j])
				{
					double thre = createARandomNum(globalState, i);
					if (last_position <= lower_bound_[j])
					{
						particles_[i].position_[j] = createARandomNum(globalState, i) * range_interval_[j] + lower_bound_[j];
					}
					else if (thre < 0.5)
					{
						particles_[i].position_[j] = lower_bound_[j] + abs(last_position - lower_bound_[j]) * createARandomNum(globalState, i);
					}
					else
					{
						particles_[i].position_[j] = lower_bound_[j];
					}
				}
			}
		}
		
	}

	//������º����ӵ���Ӧ��ֵ����
	//ҲҪ�ĳ�GPU���е�
	FitnessFunction(curIterNum, maxIterNum, bestPathInfoList, problemParas, particles_[i]);
}

//����Pbest��GPU����
static __global__ void UpdatePbest_kernal(Particle* particles_, int dim_, int fitness_count, curandState* globalState)
{
	//i��Ҫ����
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	//�Ƚ���ʷpbest�͵�ǰ��Ӧ�ȣ������Ƿ�Ҫ����
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

//����Pbest
void PSOOptimizer::UpdatePbest()
{
	//����GPU����
	UpdatePbest_kernal <<<blockSum, threadsPerBlock>>> (particles_GPU, dim_, fitness_count, globalState);
}
// ����Gbest
// ����Ƿ���Ҫ�ĳ�GPU���е�
// ���Ҫ�ģ��漰������ת�ƣ�������������Ĵ���Ҳ���࣬�����Ȳ��ĳ�GPU��
void PSOOptimizer::UpdateGbest()
{
	vector<Particle> tempArchiveL(this->archive_list);
	//GPU->CPU
	cudaMemcpy(particles_CPU, particles_GPU, sizeof(Particle) * particle_num_, cudaMemcpyDeviceToHost);
	cudaMemcpy(lower_bound_CPU, lower_bound_, sizeof(double) * dim_, cudaMemcpyDeviceToHost);
	cudaMemcpy(upper_bound_CPU, upper_bound_, sizeof(double) * dim_, cudaMemcpyDeviceToHost);
	GetGbest getG(tempArchiveL, this->meshDivCount, lower_bound_CPU, upper_bound_CPU, this->dim_, this->particle_num_);
	Particle* gbestList = getG.getGbest();

	//GPU
	UpdateGbest_GPU <<<blockSum, threadsPerBlock>>> (fitness_count, dim_, all_best_fitness_, all_best_position_, gbestList);
}

//����Gbest��GPU����
static __global__ void UpdateGbest_GPU(int fitness_count, int dim_, double* all_best_fitness_, double* all_best_position_, Particle* gbestList)
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
	//�����ǰ֧����ʷ������
	if (numGreater > 0 && numLess == 0)
	{
		return true;
	}
	//�����ʷ֧�䵱ǰ���ӣ�������
	else if (numGreater == 0 and numLess > 0)
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

//��ʼ����i������
void PSOOptimizer::InitialParticle(double* fitness_CPU, double* position_CPU, double* velocity_CPU, double* best_position_CPU, double* best_fitness_CPU, int i)
{
	#pragma region ��ʼ��position/veloctiyֵ
	//���������Ȼ����ݳ���������ӵķ�Χ
	for (int j = 2; j < dim_; j += 3)
	{
		position_CPU[i * dim_ + j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
		velocity_CPU[i * dim_ + j] = GetDoubleRand() * range_interval_[j] / 300;
	}
	//���ݳ����޸��豸���½緶Χ&�豸����
	vector<Vector2> deviceSizeCopy;
	for (int i = 0; i < problemParas.DeviceSum; i++)
	{
		deviceSizeCopy.push_back(Vector2(problemParas.deviceParaList[i].size.x, problemParas.deviceParaList[i].size.y));
	}
	for (int j = 2; j < dim_; j += 3)
	{
		//doubleתint��ת��ΪDirection��Ȼ����ݳ������¼����豸�ߴ�ͳ����
		//Rotate90����Rotate270,�޸�������
		DeviceDirect curDirect = (DeviceDirect)(int)position_CPU[i * dim_ + j];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)
		{
			//x��y
			lower_bound_[j - 2] = 0 + problemParas.deviceParaList[j / 3].size.y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
			lower_bound_[j - 1] = 0 + problemParas.deviceParaList[j / 3].size.x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;

			upper_bound_[j - 2] = problemParas.workShopLength - problemParas.deviceParaList[j / 3].size.y * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;
			upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.deviceParaList[j / 3].size.x * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;

			//size��x��y��Ҫ����
			swap(deviceSizeCopy[j / 3].x, deviceSizeCopy[j / 3].y);
		}
		else
		{
			//x��y
			lower_bound_[j - 2] = 0 + problemParas.deviceParaList[j / 3].size.x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
			lower_bound_[j - 1] = 0 + problemParas.deviceParaList[j / 3].size.y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;

			upper_bound_[j - 2] = problemParas.workShopLength - problemParas.deviceParaList[j / 3].size.x * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;
			upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.deviceParaList[j / 3].size.y * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;

		}
		range_interval_[j - 2] = upper_bound_[j - 2] - lower_bound_[j - 2];
		range_interval_[j - 1] = upper_bound_[j - 1] - lower_bound_[j - 1];
	}

	#pragma region ��ȫ���
	//for (int j = 0; j < dim_; j += 3) {
	//	particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
	//	particles_[i].position_[j + 1] = GetDoubleRand() * range_interval_[j + 1] + lower_bound_[j + 1];
	//	particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
	//	particles_[i].velocity_[j + 1] = GetDoubleRand() * range_interval_[j + 1] / 300;
	//}
	#pragma endregion

	#pragma region ���Ƿ��ص�Լ��������ֿ���������
	//(ÿ��1�ײ���һ������㣬ֻҪ�ҵ�һ�������������ص�Լ�����Ͳ��ã�����Ĭ��Ϊ0
	//�µ����������豸�İڷ�˳��
	vector<int> unmakeDeviceIndexVec;
	vector<int> madeDeviceIndexVec;
	for (int i = 0; i < problemParas.DeviceSum; i++)
	{
		unmakeDeviceIndexVec.push_back(i);
	}
	default_random_engine e;

	clock_t startTime, endTime;
	startTime = clock();//��ʱ��ʼ
	while (unmakeDeviceIndexVec.size() > 0)
	{
		endTime = clock();
		if ((static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC) > 1) {
			#pragma region ��ȫ���
			for (int j = 0; j < dim_; j += 3) {
				position_CPU[i * dim_ + j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
				position_CPU[i * dim_ + j + 1] = GetDoubleRand() * range_interval_[j + 1] + lower_bound_[j + 1];
				velocity_CPU[i * dim_ + j] = GetDoubleRand() * range_interval_[j] / 300;
				velocity_CPU[i * dim_ + j + 1] = GetDoubleRand() * range_interval_[j + 1] / 300;
			}
			#pragma endregion
			break;
		}
		//΢�뼶���ȵ����������
		e.seed(GetRamdonSeed());
		uniform_int_distribution<unsigned> u(0, unmakeDeviceIndexVec.size() - 1);
		int randomVecIndex = u(e);
		int randomDeviceIndex = unmakeDeviceIndexVec[randomVecIndex];//�õ��豸��index
		int j = randomDeviceIndex * 3;

		double Xstart = lower_bound_[j];
		double Ystart = lower_bound_[j + 1];

		double tempPositionX = 0;
		double tempPositionY = 0;

		bool findParticle = false;
		while (Ystart <= upper_bound_[j + 1] - 1 && findParticle == false) {//X��YҪ�ڷ�Χ��
			Xstart = lower_bound_[j];
			while (Xstart <= upper_bound_[j] - 1 && findParticle == false) {
				tempPositionX = GetDoubleRand() * 1.0 + Xstart;//�õ�Xstart��Xstart+1֮���һ�������
				tempPositionY = GetDoubleRand() * 1.0 + Ystart;//�õ�Ystart��Ystart+1֮���һ�������
				double halfX = deviceSizeCopy[j / 3].x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
				double halfY = deviceSizeCopy[j / 3].y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
				double tempLowX = tempPositionX - halfX;
				double tempUpX = tempPositionX + halfX;
				double tempLowY = tempPositionY - halfY;
				double tempUpY = tempPositionY + halfY;

				bool IsCross = false;
				//��鵱ǰ�豸�Ƿ��������ص�
				for (int k = 0; k < madeDeviceIndexVec.size(); k++)
				{
					int curDeviceIndex = madeDeviceIndexVec[k];
					int curDimIndex = curDeviceIndex * 3;

					double halfX1 = deviceSizeCopy[curDeviceIndex].x * 0.5 + problemParas.deviceParaList[curDeviceIndex].spaceLength;
					double halfY1 = deviceSizeCopy[curDeviceIndex].y * 0.5 + problemParas.deviceParaList[curDeviceIndex].spaceLength;

					double curLowX = position_CPU[i * dim_ + curDimIndex] - halfX1;
					double curUpX = position_CPU[i * dim_ + curDimIndex] + halfX1;
					double curLowY = position_CPU[i * dim_ + curDimIndex + 1] - halfY1;
					double curUpY = position_CPU[i * dim_ + curDimIndex + 1] + halfY1;
					//�������ص����˳�
					if (IsRangeOverlap(tempLowX, tempUpX, curLowX, curUpX) && IsRangeOverlap(tempLowY, tempUpY, curLowY, curUpY)) {
						IsCross = true;
						break;
					}
				}
				//ȫ�����ص��������Ӹ�ֵ
				if (IsCross == false) {
					findParticle = true; 
					position_CPU[i * dim_ + j] = tempPositionX;
					position_CPU[i * dim_ + j + 1] = tempPositionY;
					velocity_CPU[i * dim_ + j] = GetDoubleRand() * range_interval_[j] / 300;
					velocity_CPU[i * dim_ + j + 1] = GetDoubleRand() * range_interval_[j + 1] / 300;

					//����vec
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

