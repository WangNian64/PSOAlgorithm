#pragma once
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <random>
#include "ProblemParas.h"

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
	double* dt_ = nullptr;						// ʱ�䲽��
	double* wstart_ = nullptr;					// ��ʼȨ��
	double* wend_ = nullptr;					// ��ֹȨ��
	double* C1_ = nullptr;						// ���ٶ�����1
	double* C2_ = nullptr;						// ���ٶ�����2

	double* lower_bound_ = nullptr;				// position������Χ����
	double* upper_bound_ = nullptr;				// position������Χ����
	double* range_interval_ = nullptr;			// position�������䳤��

	int results_dim_ = 0;						// results��ά��

	int archive_max_count;						// pareto���Ž���������ֵ
	ProblemParas problemParas;					//�����Ӷ�Ӧ���豸���ֲ���

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

	// �����������ͷŶ��ڴ�
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

	// ��������������ز���
	void SetProblemParas(ProblemParas paras) {
		problemParas = paras;
	}

	// ����ʱ�䲽��
	void SetDt(double stepLength) {
		for (int i = 0; i < dim_; i++) {
			dt_[i] = stepLength;
		}
	}

	// ����wstart_
	void SetWstart(double startWeight) {
		for (int i = 0; i < dim_; i++) {
			wstart_[i] = startWeight;
		}
	}

	// ����wend_
	void SetWend(double endWeight) {
		for (int i = 0; i < dim_; i++) {
			wend_[i] = endWeight;
		}
	}

	// ����C1
	void SetC1(double c1) {
		for (int i = 0; i < dim_; i++) {
			C1_[i] = c1;
		}
	}

	// ����C2
	void SetC2(double c2) {
		for (int i = 0; i < dim_; i++) {
			C2_[i] = c2;
		}
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
	vector<InoutPoint> inoutPoints;//����ڼ���
	set<StraightConveyorInfo> strConveyorList;//ֱ�����ͻ���Ϣ�б�
	set<Vector2Int> curveConveyorList;//ת�����ͻ���Ϣ�б�
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
	int particle_num_;								// ���Ӹ���
	int max_iter_num_;								// ����������
	int curr_iter_;									// ��ǰ��������

	int dim_;										// ����ά�ȣ�position��velocity��ά�ȣ�
	int fitness_count;								// ��Ӧ����Ŀ
	int meshDivCount;								// ����ȷ����ӣ�Ĭ��Ϊ10��
	Particle* particles_ = nullptr;					// ��������

	double* upper_bound_ = nullptr;					// position������Χ����
	double* lower_bound_ = nullptr;					// position������Χ����
	double* range_interval_ = nullptr;				// position�������䳤��

	double* dt_ = nullptr;							// ʱ�䲽��
	double* wstart_ = nullptr;						// ��ʼȨ��
	double* wend_ = nullptr;						// ��ֹȨ��
	double* w_ = nullptr;							// ��ǰ����Ȩ��
	double* C1_ = nullptr;							// ���ٶ�����
	double* C2_ = nullptr;							// ���ٶ�����

	double** all_best_fitness_ = nullptr;			// ȫ���������ӵ���Ӧ������ 100x2
	double** all_best_position_ = nullptr;			// ȫ���������ӵ�position 100x12

	ProblemParas problemParas;						// �����������

	ComputeFitness fitness_fun_ = nullptr;			// ��Ӧ�Ⱥ���

	//MOPSO��ز���
	vector<Particle> archive_list;					// ���pareto���ӽ������
	int archiveMaxCount;							// archive����������Ŀ

public:
	// Ĭ�Ϲ��캯��
	PSOOptimizer() {}

	// ���캯��
	PSOOptimizer(PSOPara* pso_para, ComputeFitness fitness_fun);

	// ��������
	~PSOOptimizer();

	// ��ʼ���������Ӳ���
	void InitialAllParticles();
	// ��ʼ����i�����Ӳ���
	void InitialParticle(int i);

	// ��ʼ��Archive����
	void InitialArchiveList();
	// ����Archive����
	void UpdateArchiveList();

	// ��ʼ��ȫ������
	void InitGbest();

	// ��ȡ˫�����������Ĭ�Ͼ���Ϊ0.1��
	double GetDoubleRand(int N = 9);

	//(����һ��0 - N����N��int��
	int GetIntRand(int N);

	// ��������ӵ���Ӧ��ֵ
	void GetFitness(Particle& particle);

	// �����������Ӳ���
	void UpdateAllParticles();

	// ���µ�i������
	void UpdateParticle(int i);

	// ����Pbest
	void UpdatePbest();
	// ����Gbest
	void UpdateGbest();
	// �Ƚ��������ӵ���Ӧ�ȣ��ж��Ƿ���ȫ֧�䣬�Ӷ������pbest
	bool ComparePbest(double* fitness, double* pbestFitness);
	// ��ȡ��ǰ������Ȩ��
	void GetInertialWeight();

};