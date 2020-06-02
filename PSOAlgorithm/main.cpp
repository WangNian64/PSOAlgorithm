#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <ctime>
#include <iomanip>
#include "PSO.h"
#include "FitnessFunction.h"
int main()
{
	#pragma region ����PSO����
	int deviceNum = 6;	
	int cargoTypeNum = 4;
	ProblemParas proParas(deviceNum, cargoTypeNum);		//��ʼ�������豸��ز���

	int dim = deviceNum * 3;							// ��ά��=�豸��*3(x,y,����)
	PSOPara psopara(dim);								// dim�Ǳ���ά��
	psopara.mesh_div_count = 4;							// ���񻮷���Ŀ
	psopara.problemParas = proParas;					// ��������Ĳ���
	psopara.particle_num_ = 100;						// ���Ӹ���
	psopara.max_iter_num_ = 200;						// ����������
	psopara.fitness_count_ = 2;							// ��Ӧ����Ŀ
	psopara.archive_max_count = 200;					// archive����������Ŀ
	psopara.SetDt(1.0);									// ʱ�䲽��
	psopara.SetWstart(0.9);								// ��ʼȨ��
	psopara.SetWend(0.4);								// ����Ȩ��
	psopara.SetC1(1.49445);								// ���ٶ�����1
	psopara.SetC2(1.49445);								// ���ٶ�����2
	psopara.SetLowBound(0, 0, DeviceDirect::Default);	// position��������Χ����
	psopara.SetUpBound(proParas.workShopLength, proParas.workShopWidth, DeviceDirect::Rotate270);// position��������Χ����
	#pragma endregion

	#pragma region ����PSO�㷨����������
	
	clock_t startTime, endTime;//��¼����ʱ��
	startTime = clock();//��ʱ��ʼ
	#pragma region ��ʼ��

	PSOOptimizer psooptimizer(&psopara, FitnessFunction);//���캯��

	std::srand((unsigned int)time(0));

	psooptimizer.InitialAllParticles();//��ʼ����������
	psooptimizer.InitialArchiveList();//��ʼ��Archive�浵
	psooptimizer.InitGbest();//��ʼ��ȫ������

	#pragma endregion

	#pragma region ������������&��ÿһ�ε���Ӧ��ֵ
	//Ŀ��1��ֵ����archiveList1�У�Ŀ��2��ֵ����archiveList2��
	ofstream OutFile;
	ofstream OutFile1;
	OutFile.open("../../archiveList1.txt");
	OutFile1.open("../../archiveList2.txt");
	for (int i = 0; i < psooptimizer.max_iter_num_; i++)
	{
		cout << (i + 1) << endl;
		psooptimizer.UpdateAllParticles();//�����������ӵ�λ�ú��ٶ�
		psooptimizer.UpdatePbest();//����pbest
		psooptimizer.UpdateArchiveList();//�����ⲿ�浵����
		psooptimizer.UpdateGbest();//����gbest

		//�洢ÿ�ε�����Archive����
		//OutFile << to_string(i) + "\n";
		cout << psooptimizer.archive_list.size() << endl;
		for (auto it = psooptimizer.archive_list.begin(); it != psooptimizer.archive_list.end(); it++)
		{
			//cout << it->fitness_[0] << ",";
			string line = to_string(it->fitness_[0]) + "\n";
			OutFile << line;
		}
		//cout << endl;
		OutFile << "\n";

		for (auto it = psooptimizer.archive_list.begin(); it != psooptimizer.archive_list.end(); it++)
		{
			string line = to_string(it->fitness_[1]) + "\n";
			OutFile1 << line;
		}
		OutFile1 << "\n";
	}
	OutFile.close();
	OutFile1.close();
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
		OutFile /*<< fixed << setprecision(1)*/ << psooptimizer.archive_list[resultIndex].position_[i];
		OutFile << ",";
		OutFile /*<< fixed << setprecision(1)*/ << psooptimizer.archive_list[resultIndex].position_[i + 1];
		//string line = to_string(psooptimizer.archive_list[resultIndex].position_[i]) +
		//	"," + to_string(psooptimizer.archive_list[resultIndex].position_[i + 1]) + "\n";
		//OutFile << line;
		OutFile << "\n";
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

	#pragma region ��¼��������꣨��ת֮��ģ������豸���꣩
	vector<InoutPoint> ioPoints = psooptimizer.archive_list[resultIndex].inoutPoints;
	OutFile << to_string(ioPoints.size()) + "\n";//�������Ŀ
	for (int i = 0; i < ioPoints.size(); i++)
	{
		//string line = "";
		if (ioPoints[i].pointDirect == PointDirect::Up || ioPoints[i].pointDirect == PointDirect::Down)
		{
			//line += "Vertical ";
			OutFile << "Vertical ";
		}
		else
		{
			//line += "Horizon ";
			OutFile << "Horizon ";
		}
		//line += to_string(ioPoints[i].pointAxis.x) + " " + to_string(ioPoints[i].pointAxis.y) + " \n";
		//OutFile << line;
		OutFile /*<< fixed << setprecision(1)*/ << ioPoints[i].pointAxis.x;
		OutFile << " ";
		OutFile /*<< fixed << setprecision(1)*/ << ioPoints[i].pointAxis.y;
		OutFile << "\n";
	}
	#pragma endregion

	#pragma region ��¼�����·��
	//�ȴ�ÿ�ֻ����·������
	string line = "";
	for (int i = 0; i < proParas.CargoTypeNum; i++)
	{
		line += to_string(proParas.cargoTypeList[i].deviceSum - 1);
		if (i != proParas.CargoTypeNum - 1)
		{
			line += " ";
		}
	}
	OutFile << line << "\n";

	vector<PointLink> p = psooptimizer.archive_list[resultIndex].pointLinks;
	for (int i = 0; i < p.size(); i++)
	{
		string s1, s2;
		DevicePara device1, device2;

		//s1 = to_string(p[i].device1Index) + " " + to_string(p[i].device2Index);
		OutFile << to_string(p[i].device1Index) + " " + to_string(p[i].device2Index) + " ";
		//����s2
		for (int j = 0; j < p[i].points.size(); j++)
		{
			OutFile /*<< fixed << setprecision(1)*/ << p[i].points[j].x;
			OutFile << ",";
			OutFile /*<< fixed << setprecision(1)*/ << p[i].points[j].y;
			//s2 += to_string(p[i].points[j].x) + "," + to_string(p[i].points[j].y);
			if (j != p[i].points.size() - 1)
			{
				//s2 += "|";
				OutFile << "|";
			}
		}
		OutFile << "\n";
		//string line = s1 + " " + s2 + "\n";
		//OutFile << line;
	}
	#pragma endregion

	#pragma region ��¼ֱ�����ͻ���ת�����ͻ�����
	set<StraightConveyorInfo> strInfoList = psooptimizer.archive_list[resultIndex].strConveyorList;
	set<Vector2Int> curveInfoList = psooptimizer.archive_list[resultIndex].curveConveyorList;
	OutFile << strInfoList.size() << "\n";
	for (StraightConveyorInfo sci : strInfoList) 
	{
		OutFile << to_string(sci.startPos.x) << "," << to_string(sci.startPos.y)
			<< ";" << to_string(sci.endPos.x) << "," << to_string(sci.endPos.y) 
			<< ";" << to_string(sci.startHnum) << ";" << to_string(sci.startVnum) 
			<< ";" << to_string(sci.endHnum) << ";" << to_string(sci.endVnum)
			<< "\n";
	}
	OutFile << curveInfoList.size() << "\n";
	for (Vector2Int v : curveInfoList) 
	{
		OutFile << to_string(v.x) << "," << to_string(v.y) << "\n";
	}


	//set<SegPath> segPathSet = psooptimizer.archive_list[resultIndex].segPathSet;
	//OutFile << segPathSet.size() << "\n";
	//for (SegPath sp : segPathSet)
	//{
	//	OutFile << to_string(sp.p1.x) << "," << to_string(sp.p1.y)
	//			<< ";" << to_string(sp.p2.x) << "," << to_string(sp.p2.y) << "\n";
	//}

	//map<Vector2, PointInfo> pathPointInfoMap = psooptimizer.archive_list[resultIndex].pathPointInfoMap;
	//OutFile << pathPointInfoMap.size() << "\n";
	//for (auto it = pathPointInfoMap.begin(); it != pathPointInfoMap.end(); it++) 
	//{
	//	OutFile << to_string(it->first.x) << "," << to_string(it->first.y)
	//		<< ";" << to_string(it->second.horiDirNum) << ";" << to_string(it->second.vertDirNum) << "\n";
	//}
	#pragma endregion

	OutFile.close();
	#pragma endregion
	

	#pragma endregion

	system("pause");
}
