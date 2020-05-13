#pragma once
#include "PSO.h"
#include "Pareto.h"
#include "Tools.h"
#include "Archive.h"
#include <ctime>
// ���캯��(��ʼ�������㷨�Ĳ��������������ռ�)
PSOOptimizer::PSOOptimizer(PSOPara* pso_para, ComputeFitness fitness_fun)
{
	particle_num_ = pso_para->particle_num_;
	max_iter_num_ = pso_para->max_iter_num_;
	dim_ = pso_para->dim_;
	fitness_count = pso_para->fitness_count_;
	curr_iter_ = 0;

	dt_ = new double[dim_];
	wstart_ = new double[dim_];
	wend_ = new double[dim_];
	C1_ = new double[dim_];
	C2_ = new double[dim_];

	for (int i = 0; i < dim_; i++)
	{
		dt_[i] = pso_para->dt_[i];
		wstart_[i] = pso_para->wstart_[i];
		wend_[i] = pso_para->wend_[i];
		C1_[i] = pso_para->C1_[i];
		C2_[i] = pso_para->C2_[i];
	}

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
	w_ = new double[dim_];

	all_best_position_ = new double*[particle_num_];
	for (int j = 0; j < particle_num_; j++)
	{
		all_best_position_[j] = new double[dim_];
	}

	all_best_fitness_ = new double* [particle_num_];
	for (int j = 0; j < particle_num_; j++)
	{
		all_best_fitness_[j] = new double[fitness_count];
	}

	meshDivCount = pso_para->mesh_div_count;
	problemParas = pso_para->problemParas;//��������Ĳ���
	fitness_fun_ = fitness_fun;

	this->archiveMaxCount = pso_para->archive_max_count;
}

PSOOptimizer::~PSOOptimizer()
{
	if (particles_) { delete[]particles_; }
	if (upper_bound_) { delete[]upper_bound_; }
	if (lower_bound_) { delete[]lower_bound_; }
	if (range_interval_) { delete[]range_interval_; }
	if (dt_) { delete[]dt_; }
	if (wstart_) { delete[]wstart_; }
	if (wend_) { delete[]wend_; }
	if (w_) { delete[]w_; }
	if (C1_) { delete[]C1_; }
	if (C2_) { delete[]C2_; }
	if (all_best_position_) { delete[]all_best_position_; }
}

// ��ʼ���������ӣ�û�и���ȫ����ѣ�
void PSOOptimizer::InitialAllParticles()
{
	// ��ʼ���������ӣ�������ʼ��ÿ�����ӵĸ�������ֵ��
	for (int i = 0; i < particle_num_; i++)
	{
		InitialParticle(i);
	}
}

// ��ʼ��Archive����
void PSOOptimizer::InitialArchiveList()
{
	vector<Particle> particleList(this->particles_, this->particles_ + this->particle_num_);
	Pareto initPareto(particleList);
	this->archive_list = initPareto.GetPareto();
}

// ����Archive����
void PSOOptimizer::UpdateArchiveList() 
{
	//���ȣ����㵱ǰ����Ⱥ��pareto�߽磬���߽����Ӽ��뵽�浵archiving��
	vector<Particle> particleList(this->particles_, this->particles_ + particle_num_);
	Pareto pareto1(particleList);
	vector<Particle> curParetos = pareto1.GetPareto();
	//��Σ��ڴ浵�и���֧���ϵ���еڶ���ɸѡ�����Ǳ߽�����ȥ��
	vector<Particle> newParetos;
	curParetos.insert(curParetos.end(), this->archive_list.begin(), this->archive_list.end());//�ϲ�cur��ԭArchive
	Pareto pareto2(curParetos);
	vector<Particle> curArchives = pareto2.GetPareto();
	//�жϴ浵�����Ƿ��ѳ����浵��ֵ������ǣ������һ���֣�ӵ���Ƚϸߵ��ǲ��ֱ�����ĸ��ʸ���
	if (curArchives.size() > this->archiveMaxCount)
	{
		GetGbest clear(curArchives, this->meshDivCount, this->lower_bound_, this->upper_bound_, this->dim_, this->particle_num_);
		curArchives = clear.Clear(this->archiveMaxCount);
	}
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
			this->all_best_fitness_[i][j] = gbestList[i].best_fitness_[j];
		}
		for (int k = 0; k < dim_; k++)
		{
			this->all_best_position_[i][k] = gbestList[i].best_position_[k];
		}
	}
}

// ��ȡ˫���������(����һ��0-1֮���С����
double PSOOptimizer::GetDoubleRand(int N)
{
	double temp = rand() % (N + 1) / (double)(N + 1);
	return temp;
}

//
int PSOOptimizer::GetIntRand(int N)
{
	int temp = rand() % (N + 1);
	return temp;
}
void PSOOptimizer::GetFitness(Particle& particle)
{
	return fitness_fun_(curr_iter_, particle, problemParas, lower_bound_, upper_bound_);
}

void PSOOptimizer::UpdateAllParticles()
{
	GetInertialWeight();//���㵱ǰ���Ĺ���ϵ��
	//���µ�ǰ����������
	for (int i = 0; i < particle_num_; i++)
	{
		UpdateParticle(i);
	}
	curr_iter_++;
}

//����i����������&������Ӧ��ֵ����
void PSOOptimizer::UpdateParticle(int i)
{
	//�ȸ��³���Ȼ����ݳ���������ӵķ�Χ
	for (int j = 2; j < dim_; j += 3)
	{
		double last_position = particles_[i].position_[j];

		particles_[i].velocity_[j] = w_[j] * particles_[i].velocity_[j] +
			C1_[j] * GetDoubleRand() * (particles_[i].best_position_[j] - particles_[i].position_[j]) +
			C2_[j] * GetDoubleRand() * (all_best_position_[i][j] - particles_[i].position_[j]);
		particles_[i].position_[j] += dt_[j] * particles_[i].velocity_[j];

		// �����������������������
		if (upper_bound_ && lower_bound_)
		{
			if (particles_[i].position_[j] > upper_bound_[j])
			{
				double thre = GetDoubleRand(99);
				if (last_position == upper_bound_[j])
				{
					particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
				}
				else if (thre < 0.5)
				{
					particles_[i].position_[j] = upper_bound_[j] - (upper_bound_[j] - last_position) * GetDoubleRand();
				}
				else
				{
					particles_[i].position_[j] = upper_bound_[j];
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

	//���ݳ����޸��豸���½緶Χ
	for (int j = 2; j < dim_; j += 3)
	{
		//doubleתint��ת��ΪDirection��Ȼ����ݳ������¼����豸�ߴ�ͳ����
		//Rotate90����Rotate270,�޸�������
		DeviceDirect curDirect = (DeviceDirect)(int)particles_[i].position_[j];
		if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)
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


	for (int j = 0; j < dim_; j++)
	{
		if (j % 3 != 2)
		{
			//������һ�ε��������position��velocity
			//double last_velocity = particles_[i].velocity_[j];
			double last_position = particles_[i].position_[j];

			particles_[i].velocity_[j] = w_[j] * particles_[i].velocity_[j] +
				C1_[j] * GetDoubleRand() * (particles_[i].best_position_[j] - particles_[i].position_[j]) +
				C2_[j] * GetDoubleRand() * (all_best_position_[i][j] - particles_[i].position_[j]);
			particles_[i].position_[j] += dt_[j] * particles_[i].velocity_[j];

			// �����������������������
			if (upper_bound_ && lower_bound_)
			{
				if (particles_[i].position_[j] > upper_bound_[j])
				{
					double thre = GetDoubleRand(99);
					if (last_position == upper_bound_[j])
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
					if (last_position == lower_bound_[j])
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
					if (particles_[i].position_[j] < lower_bound_[j])
					{
						cout << endl;
					}
				}
			}
		}
		
	}

	//��ӡ���е��豸����
	//cout << "��" << i << "��" << endl;
	//for (int j = 0; j < dim_; j++)
	//{
	//	cout << lower_bound_[j] << "," << particles_[i].position_[j] << "," << upper_bound_[j] << endl;
	//}
	//������º����ӵ���Ӧ��ֵ����
	GetFitness(particles_[i]);
}


// ����Pbest
void PSOOptimizer::UpdatePbest()
{
	for (int i = 0; i < this->particle_num_; i++)
	{
		//�Ƚ���ʷpbest�͵�ǰ��Ӧ�ȣ������Ƿ�Ҫ����
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
	//cout << endl;

}

// ����Gbest
void PSOOptimizer::UpdateGbest()
{
	vector<Particle> tempArchiveL(this->archive_list);
	GetGbest getG(tempArchiveL, this->meshDivCount, this->lower_bound_, this->upper_bound_, this->dim_, this->particle_num_);
	Particle* gbestList = getG.getGbest();
	for (int i = 0; i < particle_num_; i++)
	{
		for (int j = 0; j < fitness_count; j++)
		{
			this->all_best_fitness_[i][j] = gbestList[i].best_fitness_[j];
		}
		//cout << this->all_best_fitness_[i][0] << "," << this->all_best_fitness_[i][1] << endl; 
		for (int k = 0; k < dim_; k++)
		{
			this->all_best_position_[i][k] = gbestList[i].best_position_[k];
		}
	}
	//cout << endl;
}

// �Ƚ��������ӵ���Ӧ�ȣ��ж��Ƿ���ȫ֧�䣬�Ӷ������pbest
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
		double randomProb = rand() % 1000 / (double)1000;//�������С��
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

//��ȡ����ϵ��
void PSOOptimizer::GetInertialWeight()
{
	double temp = curr_iter_ / (double)max_iter_num_;
	temp *= temp;
	for (int i = 0; i < dim_; i++)
	{
		w_[i] = wstart_[i] - (wstart_[i] - wend_[i]) * temp;
		//w_[i] = wstart_[i];
	}

}

//��ʼ����i������
void PSOOptimizer::InitialParticle(int i)
{
	particles_[i].dim_ = dim_;
	particles_[i].fitnessCount = this->fitness_count;
	// Ϊÿ�����Ӷ�̬�����ڴ�
	particles_[i].position_ = new double[dim_];
	particles_[i].velocity_ = new double[dim_];
	particles_[i].best_position_ = new double[dim_];
	particles_[i].fitness_ = new double[fitness_count];
	particles_[i].best_fitness_ = new double[fitness_count];

	#pragma region ��ʼ��position/veloctiyֵ
	////���������Ȼ����ݳ���������ӵķ�Χ
	//for (int j = 2; j < dim_; j += 3)
	//{
	//	particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
	//	particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
	//}
	////���ݳ����޸��豸���½緶Χ
	//for (int j = 2; j < dim_; j += 3)
	//{
	//	//doubleתint��ת��ΪDirection��Ȼ����ݳ������¼����豸�ߴ�ͳ����
	//	//Rotate90����Rotate270,�޸�������
	//	DeviceDirect curDirect = (DeviceDirect)(int)particles_[i].position_[j];
	//	if (curDirect == DeviceDirect::Rotate90 || curDirect == DeviceDirect::Rotate270)
	//	{
	//		//x��y
	//		lower_bound_[j - 2] = 0 + problemParas.deviceParaList[j / 3].size.y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
	//		lower_bound_[j - 1] = 0 + problemParas.deviceParaList[j / 3].size.x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;

	//		upper_bound_[j - 2] = problemParas.workShopLength - problemParas.deviceParaList[j / 3].size.y * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;
	//		upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.deviceParaList[j / 3].size.x * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;

	//	}
	//	else
	//	{
	//		//x��y
	//		lower_bound_[j - 2] = 0 + problemParas.deviceParaList[j / 3].size.x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
	//		lower_bound_[j - 1] = 0 + problemParas.deviceParaList[j / 3].size.y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;

	//		upper_bound_[j - 2] = problemParas.workShopLength - problemParas.deviceParaList[j / 3].size.x * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;
	//		upper_bound_[j - 1] = problemParas.workShopWidth - problemParas.deviceParaList[j / 3].size.y * 0.5 - problemParas.deviceParaList[j / 3].spaceLength;

	//	}

	//	range_interval_[j - 2] = upper_bound_[j - 2] - lower_bound_[j - 2];
	//	range_interval_[j - 1] = upper_bound_[j - 1] - lower_bound_[j - 1];
	//}
	//���Ƿ��ص�Լ��������ֿ���������(ÿ��1�ײ���һ������㣬ֻҪ�ҵ�һ�������������ص�Լ�����Ͳ��ã�����ȫ��Ĭ��Ϊ0
	for (int j = 0; j < dim_; j += 3)
	{
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
				double halfX = problemParas.deviceParaList[j / 3].size.x * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
				double halfY = problemParas.deviceParaList[j / 3].size.y * 0.5 + problemParas.deviceParaList[j / 3].spaceLength;
				double tempLowX = tempPositionX - halfX;
				double tempUpX = tempPositionX + halfX;
				double tempLowY = tempPositionY - halfY;
				double tempUpY = tempPositionY + halfY;

				bool IsCross = false;
				for (int k = 0; k < j; k += 3) {//�����λ���Ƿ��������豸�ص�
					//��������������Ӻ͵�ǰi����j֮ǰ�������豸����Ƚϣ������Ƿ����ص�
					double halfX1 = problemParas.deviceParaList[k / 3].size.x * 0.5 + problemParas.deviceParaList[k / 3].spaceLength;
					double halfY1 = problemParas.deviceParaList[k / 3].size.y * 0.5 + problemParas.deviceParaList[k / 3].spaceLength;

					double curLowX = particles_[i].position_[k] - halfX1;
					double curUpX = particles_[i].position_[k] + halfX1;
					double curLowY = particles_[i].position_[k + 1] - halfY1;
					double curUpY = particles_[i].position_[k + 1] + halfY1;
					//�������ص����˳�
					if (IsRangeOverlap(tempLowX, tempUpX, curLowX, curUpX) && IsRangeOverlap(tempLowY, tempUpY, curLowY, curUpY)) {
						IsCross = true;
						break;
					}
				}
				//ȫ�����ص��������Ӹ�ֵ
				if (IsCross == false) {
					findParticle = true;
					particles_[i].position_[j] = tempPositionX;
					particles_[i].position_[j + 1] = tempPositionY;
					particles_[i].position_[j + 2] = DeviceDirect::Default;
					particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
					particles_[i].velocity_[j + 1] = GetDoubleRand() * range_interval_[j + 1] / 300;
					particles_[i].velocity_[j + 2] = GetDoubleRand() * range_interval_[j + 2] / 300;
				}
				Xstart++;
				if (Xstart >= upper_bound_[j] - 1) {
					Ystart++;
				}
			}
		}
	}
	//for (int j = 0; j < dim_; j += 2)
	//{
	//	double Xstart = lower_bound_[j];
	//	double Ystart = lower_bound_[j + 1];

	//	double tempPositionX = 0;
	//	double tempPositionY = 0;

	//	bool findParticle = false;
	//	while (Ystart <= upper_bound_[j + 1] - 1 && findParticle == false) {//X��YҪ�ڷ�Χ��
	//		Xstart = lower_bound_[j];
	//		while (Xstart <= upper_bound_[j] - 1 && findParticle == false) {
	//			tempPositionX = GetDoubleRand() * 1.0 + Xstart;//�õ�Xstart��Xstart+1֮���һ�������
	//			tempPositionY = GetDoubleRand() * 1.0 + Ystart;//�õ�Ystart��Ystart+1֮���һ�������

	//			double tempLowX = tempPositionX - 0.5 * problemParas.DeviceSizeArray[j / 2].x;
	//			double tempUpX = tempPositionX + 0.5 * problemParas.DeviceSizeArray[j / 2].x;
	//			double tempLowY = tempPositionY - 0.5 * problemParas.DeviceSizeArray[j / 2].y;
	//			double tempUpY = tempPositionY + 0.5 * problemParas.DeviceSizeArray[j / 2].y;

	//			bool IsCross = false;
	//			for (int k = 0; k < j; k += 2) {//�����λ���Ƿ��������豸�ص�
	//				//��������������Ӻ͵�ǰi����j֮ǰ�������豸����Ƚϣ������Ƿ����ص�
	//				double curLowX = particles_[i].position_[k] - 0.5 * problemParas.DeviceSizeArray[k / 2].x;
	//				double curUpX = particles_[i].position_[k] + 0.5 * problemParas.DeviceSizeArray[k / 2].x;
	//				double curLowY = particles_[i].position_[k + 1] - 0.5 * problemParas.DeviceSizeArray[k / 2].y;
	//				double curUpY = particles_[i].position_[k + 1] + 0.5 * problemParas.DeviceSizeArray[k / 2].y;
	//				//�������ص����˳�
	//				if (IsRangeOverlap(tempLowX, tempUpX, curLowX, curUpX) && IsRangeOverlap(tempLowY, tempUpY, curLowY, curUpY)) {
	//					IsCross = true;
	//					break;
	//				}
	//			}
	//			//ȫ�����ص��������Ӹ�ֵ
	//			if (IsCross == false) {
	//				findParticle = true;
	//				particles_[i].position_[j] = tempPositionX;
	//				particles_[i].position_[j + 1] = tempPositionY;
	//				particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
	//				particles_[i].velocity_[j + 1] = GetDoubleRand() * range_interval_[j + 1] / 300;
	//			}
	//			Xstart++;
	//			if (Xstart >= upper_bound_[j] - 1) {
	//				Ystart++;
	//			}
	//		}
	//	}
	//	//particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
	//	//particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;
	//}
	#pragma endregion

	//Ȼ�����λ�ã����ܲ��ǿ��н⣩
	//for (int j = 0; j < dim_; j++) {
	//	if (j % 3 != 2)
	//	{
	//		particles_[i].position_[j] = GetDoubleRand() * range_interval_[j] + lower_bound_[j];
	//		particles_[i].velocity_[j] = GetDoubleRand() * range_interval_[j] / 300;

	//	}
	//}
	//�����������Ӧ��ֵ
	GetFitness(particles_[i]); 
	// ��ʼ����������λ��
	for (int j = 0; j < dim_; j++)
	{
		particles_[i].best_position_[j] = particles_[i].position_[j];
	}
	// ��ʼ�����Ӹ�����ʷ���
	for (int j = 0; j < fitness_count; j++) 
	{
		particles_[i].best_fitness_[j] = particles_[i].fitness_[j];
	}
}

