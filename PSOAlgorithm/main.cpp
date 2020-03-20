#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <ctime>
#include "PSO.h"
#include "FitnessFunction.h"
//��������txt
//void SaveLayoutResults(PSOOptimizer psooptimizer, double* result) {
//	ofstream OutFile;
//	//��ÿһ�ε���Ӧ��ֵ#include <time.h>
//
//
//
//	OutFile.open("../../IterateResult.txt");
//	for (int i = 0; i < psooptimizer.max_iter_num_; i++)
//	{
//		string line = to_string(1.0 / result[i]) + "\n";
//		OutFile << line;
//	}
//	OutFile.close();
//	//�����Ĳ��ֽ��
//	OutFile.open("../../LayoutResult.txt");
//	for (int i = 0; i < psooptimizer.dim_; i += 2) {
//		string line = to_string(psooptimizer.all_best_position_[i]) + "," + to_string(psooptimizer.all_best_position_[i + 1]) + "\n";
//		OutFile << line;
//	}
//	OutFile.close();
//}
int main()
{
	#pragma region ����PSO����
	int deviceNum = 6;	
	int cargoTypeNum = 3;
	ProblemParas proParas(deviceNum, cargoTypeNum);//��ʼ�������豸��ز���

	int dim = deviceNum * 3;						// ��ά��=�豸��*3(x,y,����)
	PSOPara psopara(dim);							// dim�Ǳ���ά��
	psopara.mesh_div_count = 2;						// ���񻮷���Ŀ
	psopara.problemParas = proParas;				// ��������Ĳ���
	psopara.particle_num_ = 100;					// ���Ӹ���
	psopara.max_iter_num_ = 400;					// ����������
	psopara.fitness_count_ = 2;						// ��Ӧ����Ŀ
	psopara.archive_max_count = 200;				// archive����������Ŀ
	psopara.SetDt(1.0);								// ʱ�䲽��
	psopara.SetWstart(0.9);							// ��ʼȨ��
	psopara.SetWend(0.4);							// ����Ȩ��
	psopara.SetC1(1.49445);							// ���ٶ�����1
	psopara.SetC2(1.49445);							// ���ٶ�����2
	psopara.SetLowBound(0, 0, DeviceDirect::Default);				// position��������Χ����
	psopara.SetUpBound(proParas.workShopLength, proParas.workShopWidth, DeviceDirect::Rotate270);// position��������Χ����
	//
	#pragma endregion

	#pragma region ����PSO�㷨����������
	
	clock_t startTime, endTime;//��¼����ʱ��
	startTime = clock();//��ʱ��ʼ
	#pragma region ��ʼ��

	PSOOptimizer psooptimizer(&psopara, FitnessFunction);//���캯��

	std::srand((unsigned int)time(0));


	psooptimizer.InitialAllParticles();//��ʼ����������
	psooptimizer.InitialArchiveList();//��ʼ��Archive�浵
	psooptimizer.InitGbest();//��ʼ��ȫ������ //����ط�Ҫע��min��max

	#pragma endregion

	#pragma region ������������&��ÿһ�ε���Ӧ��ֵ
	ofstream OutFile;
	OutFile.open("../../archiveList.txt");
	for (int i = 0; i < psooptimizer.max_iter_num_; i++)
	{
		cout << i << endl;
		psooptimizer.UpdateAllParticles();//�����������ӵ�λ�ú��ٶ�
		psooptimizer.UpdatePbest();//����pbest
		psooptimizer.UpdateArchiveList();//�����ⲿ�浵����
		psooptimizer.UpdateGbest();//����gbest

		//�洢ÿ�ε�����Archive����
		OutFile << to_string(i) + "\n";
		for (auto it = psooptimizer.archive_list.begin(); it != psooptimizer.archive_list.end(); it++)
		{
			string line = to_string(it->fitness_[0]) + ", " + to_string(it->fitness_[1]) + "\n";
			OutFile << line;
		}
		OutFile << "\n";
	}
	OutFile.close();
	#pragma endregion
	endTime = clock();
	cout << "����" << psopara.max_iter_num_ << "�ε�������ʱ:" << static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	#pragma region �����豸�ߴ�&���ղ��ֽ��&���ߵ������
	OutFile.open("../../FinalResult.txt");


	#pragma region ��¼���ղ��ֽ��
	int resultIndex = 0;
	int minConveyValue = INTMAX_MAX;
	int minAreaVaule = INTMAX_MAX;
	//����ѡ����Ч����ߵ�
	for (int i = 0; i < psooptimizer.archive_list.size(); i++)
	{
		if (psooptimizer.archive_list[i].fitness_[0] < minConveyValue)
		{
			minConveyValue = psooptimizer.archive_list[i].fitness_[0];
			resultIndex = i;
		}
	}
	//����ѡ�����С��
	//for (int i = 0; i < psooptimizer.archive_list.size(); i++)
	//{
	//	if (psooptimizer.archive_list[i].fitness_[1] < minConveyValue)
	//	{
	//		minAreaVaule = psooptimizer.archive_list[i].fitness_[1];
	//		resultIndex = i;
	//	}
	//}

	for (int i = 0; i < dim; i += 3)
	{
		string line = to_string(psooptimizer.archive_list[resultIndex].position_[i]) +
			"," + to_string(psooptimizer.archive_list[resultIndex].position_[i + 1]) + "\n";
		OutFile << line;
	}
	#pragma endregion


	#pragma region ��¼�豸�ߴ�
	for (int i = 2; i < dim; i += 3)
	{
		DeviceDirect direct = (DeviceDirect)(int)psooptimizer.archive_list[resultIndex].position_[i];
		string line = "";
		if (direct == DeviceDirect::Rotate90 || direct == DeviceDirect::Rotate270)
		{
			line = to_string(psooptimizer.problemParas.deviceParaList[i / 3].size.y) + "," +
				to_string(psooptimizer.problemParas.deviceParaList[i / 3].size.x);
		} else {
			line = to_string(psooptimizer.problemParas.deviceParaList[i / 3].size.x) + "," +
				to_string(psooptimizer.problemParas.deviceParaList[i / 3].size.y);
		}
		OutFile << line + "\n";
	}
	#pragma endregion
	#pragma region ��¼�����·��
	vector<PointLink> p = psooptimizer.archive_list[resultIndex].pointLinks;
	for (int i = 0; i < p.size(); i++)
	{
		cout << psooptimizer.archive_list[resultIndex].pointLinks[i].device1Index << endl;
	}
	for (int i = 0; i < p.size(); i++)
	{
		string s1, s2;
		DevicePara device1, device2;
		s1 = to_string(p[i].device1Index) + " " + to_string(p[i].device2Index);
		//����s2
		for (int j = 0; j < p[i].points.size(); j++)
		{
			s2 += to_string(p[i].points[j].x) + "," + to_string(p[i].points[j].y);
			if (j != p[i].points.size() - 1)
			{
				s2 += "|";
			}
		}
		string line = s1 + " " + s2 + "\n";
		OutFile << line;
	}
	#pragma endregion

	OutFile.close();
	#pragma endregion
	
	//���Ľ���浽txt��(���ֽ����ÿһ�ε�������Ӧ��ֵ��
	//SaveLayoutResults(psooptimizer, result);

	#pragma endregion

	system("pause");
}
