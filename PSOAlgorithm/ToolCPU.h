#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <Windows.h>
using namespace std;
//�ַ����ָ��
static vector<string> split(const string& str, const string& pattern)
{
	vector<string> res;
	if ("" == str)
		return res;

	string strs = str + pattern;

	size_t pos = strs.find(pattern);
	size_t size = strs.size();

	while (pos != string::npos)
	{
		string x = strs.substr(0, pos);
		res.push_back(x);//stoi(x)ת����
		strs = strs.substr(pos + 1, size);
		pos = strs.find(pattern);
	}
	return res;
}

// ��ȡ˫���������(����һ��0-1֮���С��,������1��
double GetDoubleRand(int N = 99)
{
	double temp = rand() % (N + 1) / (double)(N + 1);
	return temp;
}

//����һ����������ӣ�΢�뼶��
static unsigned GetRamdonSeed()
{
	LARGE_INTEGER nFrequency;
	unsigned int res;
	if (QueryPerformanceFrequency(&nFrequency))
	{
		LARGE_INTEGER nStartCounter;

		QueryPerformanceCounter(&nStartCounter);
		res = (unsigned)nStartCounter.LowPart;
	}
	return res;
}
//�Զ���unique�����������Ѿ��ź�������飬ȥ�����е��ظ����֣�����������Ĵ�С)
//static int PointInfo_Unique(PointInfo* objArray, int start, int end)
//{
//	int l = 0;
//	int r = 1;
//	while (r <= end)
//	{
//		if (!objArray[r].AEqualB(objArray[l]))//���߲����
//		{
//			l++;
//			objArray[l] = objArray[r];
//		}
//		r++;
//	}
//	return l + 1;
//}
