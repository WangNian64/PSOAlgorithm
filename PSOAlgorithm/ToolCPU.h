#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <Windows.h>
using namespace std;
//字符串分割函数
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
		res.push_back(x);//stoi(x)转整型
		strs = strs.substr(pos + 1, size);
		pos = strs.find(pattern);
	}
	return res;
}

// 获取双精度随机数(返回一个0-1之间的小数,不包含1）
double GetDoubleRand(int N = 99)
{
	double temp = rand() % (N + 1) / (double)(N + 1);
	return temp;
}

//返回一个随机数种子（微秒级）
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
//自定义unique函数（对于已经排好序的数组，去除其中的重复部分，返回新数组的大小)
//static int PointInfo_Unique(PointInfo* objArray, int start, int end)
//{
//	int l = 0;
//	int r = 1;
//	while (r <= end)
//	{
//		if (!objArray[r].AEqualB(objArray[l]))//两者不相等
//		{
//			l++;
//			objArray[l] = objArray[r];
//		}
//		r++;
//	}
//	return l + 1;
//}
