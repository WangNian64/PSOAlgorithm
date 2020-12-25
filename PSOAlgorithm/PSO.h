#pragma once
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <random>
#include "ProblemParas.h"

#include <cuda_runtime.h>//����cuda��API
#include <curand.h>
#include <curand_kernel.h>
// ��Ӧ����Խ��Խ�û���ԽСԽ��
#define MINIMIZE_FITNESS
//#define MAXIMIZE_FITNESS

struct PSOPara
{
	int dim_;									// ����ά�ȣ�position��velocity��ά�ȣ�
	int fitness_count_;							// ��Ӧ����Ŀ
	int particle_num_;							// ���Ӹ���
	int max_iter_num_;							// ����������

	int mesh_div_count;

	double dt_;									// ʱ�䲽��
	double wstart_;								// ��ʼȨ��
	double wend_;								// ��ֹȨ��
	double C1_;									// ���ٶ�����1
	double C2_;									// ���ٶ�����2

	double* lower_bound_ = nullptr;				// position������Χ����
	double* upper_bound_ = nullptr;				// position������Χ����
	double* range_interval_ = nullptr;			// position�������䳤��

	int archive_max_count;						// pareto���Ž���������ֵ
	ProblemParas problemParas;					// �����Ӷ�Ӧ���豸���ֲ��� ȫ�ֲ���

	int blockSum;								// ÿ��Grid��block����Ŀ
	int threadsPerBlock;						// ÿ��Block��thread����Ŀ
	PSOPara() {}

	PSOPara(int dim)
	{
		dim_ = dim;

		lower_bound_ = new double[dim_];
		upper_bound_ = new double[dim_];
		range_interval_ = new double[dim_];

		ProblemParas problemParas();
	}

	// �����������ͷŶ��ڴ�
	~PSOPara()
	{
		if (lower_bound_) { delete[]lower_bound_; }
		if (upper_bound_) { delete[]upper_bound_; }
		if (range_interval_) { delete[]range_interval_; }
	}

	// ��������������ز���
	void SetProblemParas(ProblemParas paras) {
		problemParas = paras;
	}

	// ����ʱ�䲽��
	void SetDt(double stepLength) {
		dt_ = stepLength;
	}

	// ����wstart_
	void SetWstart(double startWeight) {
		wstart_ = startWeight;
	}

	// ����wend_
	void SetWend(double endWeight) {
		wend_ = endWeight;
	}

	// ����C1
	void SetC1(double c1) {
		C1_ = c1;
	}

	// ����C2
	void SetC2(double c2) {
		C2_ = c2;
	}

	// ����low_bound
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

	// ����upper_bound
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
//�洢�������ӵ���������Ϣ
struct BestPathInfo
{
	double curBestFitnessVal;//��ǰĿ�������ֵ�������ж��Ƿ����
	int inoutPSize;//�������Ŀ
	InoutPoint* inoutPoints;//����ڼ���
	StraightConveyorInfo* strConveyorList;//ֱ�����ͻ���Ϣ�б�
	int strConveyorListSum;
	Vector2Int* curveConveyorList;//ת�����ͻ���Ϣ�б�
	int curveConveyorListSum;
	BestPathInfo() {
		curBestFitnessVal = INT_MAX;
	}
	void Clear() {

	}
};
//���ӽṹ��
struct Particle
{
	int dim_;							// ����ά�ȣ�position��velocity��ά�ȣ�

	int fitnessCount;					//��Ӧ�ȵĸ���
	double* fitness_ = nullptr;			//��Ӧ������
	double* position_ = nullptr;		//����λ������
	double* velocity_ = nullptr;		//�����ٶ�����

	double* best_position_ = nullptr;	//���ӵĸ�������λ������
	double* best_fitness_ = nullptr;	//���ӵĸ���������Ӧ������

	//vector<PointLink> pointLinks; //����·������

	//map<Vector2Int, PointInfo> pathPointInfoMap;//·������Ϣmap
	//set<SegPath> segPathSet;
	//int pointLinkSum = 0;//·������Ŀ
	Particle() {}
	Particle(const Particle& particle)//�������캯��
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
	}
	~Particle()
	{

	}
};

class PSOOptimizer
{
public:
	int blockNum;							// block������Ŀ
	int threadsPerBlock;					// ÿ��block��thread��Ŀ

	int particle_num_;						// ���Ӹ���
	int max_iter_num_;						// ����������
	int curr_iter_;							// ��ǰ��������

	int dim_;								// ����ά�ȣ�position��velocity��ά�ȣ�
	int fitness_count;						// ��Ӧ����Ŀ
	int meshDivCount;						// ����ȷ����ӣ�Ĭ��Ϊ10��
	Particle* particles_;					// �������ӣ�GPU��
	Particle* particles_CPU;					// �������ӣ�CPU��
	int particle_size;						// һ�����ӵ��ڴ��С

	double* randomNumList;					// �������������
	curandState* devStates;					// cuda�����״̬����

	double* upper_bound_;					// position������Χ����
	double* lower_bound_;					// position������Χ����
	double* range_interval_;				// position�������䳤��

	double* lower_bound_CPU;				// position������Χ����
	double* upper_bound_CPU;				// position������Χ����

	double dt_;								// ʱ�䲽��
	double wstart_;							// ��ʼȨ��
	double wend_;							// ��ֹȨ��
	double w_;								// ��ǰ����Ȩ��
	double C1_;								// ���ٶ�����
	double C2_;								// ���ٶ�����

	double* all_best_fitness_;				// ȫ���������ӵ���Ӧ������ 100x2
	double* all_best_position_;				// ȫ���������ӵ�position 100x12

	ProblemParas problemParas;				// ����������� //////

	//MOPSO��ز���,����Ҫ����GPU
	vector<Particle> archive_list;			// ���pareto���ӽ������ CPU
	int archiveList_CurSize;				// ��ǰ��archiveList��С
	int archiveMaxCount;					// archive����������Ŀ
	BestPathInfo* bestPathInfoList;			// ����·����Ϣ   //////

public:
	// Ĭ�Ϲ��캯��
	PSOOptimizer() {}

	// ���캯��
	PSOOptimizer(PSOPara* pso_para);

	// ��������
	~PSOOptimizer();

	// ��ʼ���������Ӳ���
	void InitialAllParticles();

	// ��ʼ����i�����Ӳ���
	void InitialParticle(Particle* particles_CPU, int i);

	// ��ʼ��Archive����
	void InitialArchiveList();

	// ����Archive����
	void UpdateArchiveList();

	// ��ʼ��ȫ������
	void InitGbest();

	// ��������ӵ���Ӧ��ֵ
	void GetFitness(Particle& particle);

	// �����������Ӳ���
	void UpdateAllParticles();

	// ���µ�i������
	//void UpdateParticle(int i);

	// ����Pbest
	void UpdatePbest();

	// ����Gbest
	void UpdateGbest();

	// �Ƚ��������ӵ���Ӧ�ȣ��ж��Ƿ���ȫ֧�䣬�Ӷ������pbest
	bool ComparePbest(double* fitness, double* pbestFitness);


};