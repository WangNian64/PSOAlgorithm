#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <Windows.h>
#define PI 3.14159265358979
#define MAX_FITNESS 10000000.0
using namespace std;
//判断两个区间是否重叠
static bool IsRangeOverlap(double low1, double upper1, double low2, double upper2) {
	if (max(low1, low2) <= min(upper1, upper2)) {
		return true;
	}
	else
	{
		return false;
	}
}
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
static double GetDoubleRand(int N = 99)
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

//快速排序（DeviceIDSize版本）
static void DeviceIDSize_Sort(DeviceIDSize* sizeArray, int start, int end)
{
	if (start < end) {
		int pivot = Double_Partition(sizeArray, start, end);
		DeviceIDSize_Sort(sizeArray, start, pivot - 1);
		DeviceIDSize_Sort(sizeArray, pivot + 1, end);
	}
}
int Double_Partition(DeviceIDSize* sizeArray, int start, int end)
{
	DeviceIDSize temp = sizeArray[start];
	double tempSize = temp.size.x * temp.size.y;
	int i = start;
	int j = end;
	while (i < j) {
		while (i < j && sizeArray[j].size.x * sizeArray[j].size.y >= tempSize)
			--j;
		sizeArray[i] = sizeArray[j];
		while (i < j && sizeArray[j].size.x * sizeArray[j].size.y <= tempSize)
			++i;
		sizeArray[j] = sizeArray[i];
	}
	sizeArray[i] = temp;//插入到正确位置
	return i;
}

//快速排序（double版本）
static void Double_Sort(double* numArray, int start, int end)
{
	if (start < end) {
		int pivot = Double_Partition(numArray, start, end);
		Double_Sort(numArray, start, pivot - 1);
		Double_Sort(numArray, pivot + 1, end);
	}
}
int Double_Partition(double* numArray, int start, int end)
{

	double temp = numArray[start];
	int i = start;
	int j = end;
	while (i < j) {
		while (i < j && numArray[j] >= temp)
			--j;
		numArray[i] = numArray[j];
		while (i < j && numArray[i] <= temp)
			++i;
		numArray[j] = numArray[i];
	}
	numArray[i] = temp;//插入到正确位置
	return i;
}        

//自定义unique函数（对于已经排好序的数组，去除其中的重复部分，返回新数组的大小)
static int Double_Unique(double* numArray, int start, int end)
{
	int l = 0; 
	int r = 1;
	while (r <= end)
	{
		if (numArray[r] != numArray[l]) 
		{
			l++;
			numArray[l] = numArray[r];
		}
		r++;
	}
	return l + 1;
}