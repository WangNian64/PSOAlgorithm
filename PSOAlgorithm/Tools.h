#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <Windows.h>
#include "DevicePara.h"

#include <cuda.h>  
#include <curand.h>  
#include <curand_kernel.h>  

#define PI 3.14159265358979
#define MAX_FITNESS 10000000.0
using namespace std;
//所有函数都加上定义

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
	DeviceIDSize& temp = sizeArray[start];
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
static int Double_Partition(double* numArray, int start, int end)
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
static void Double_Sort(double* numArray, int start, int end)
{
	if (start < end) {
		int pivot = Double_Partition(numArray, start, end);
		Double_Sort(numArray, start, pivot - 1);
		Double_Sort(numArray, pivot + 1, end);
	}
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

//自定义四舍五入算法
static int MyRound(double num)
{
	return (num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5);
}

//快速排序（SegPath版本）
int SegPath_Partition(SegPath* objArray, int start, int end)
{
	SegPath& temp = objArray[start];
	int i = start;
	int j = end;
	while (i < j) {
		while (i < j && objArray[j].ABigEqualB(temp))
			--j;
		objArray[i] = objArray[j];
		while (i < j && objArray[i].ASmallEqualB(temp))
			++i;
		objArray[j] = objArray[i];
	}
	objArray[i] = temp;//插入到正确位置
	return i;
}
static void SegPath_Sort(SegPath* objArray, int start, int end)
{
	if (start < end) {
		int pivot = SegPath_Partition(objArray, start, end);
		SegPath_Sort(objArray, start, pivot - 1);
		SegPath_Sort(objArray, pivot + 1, end);
	}
}


//自定义unique函数（对于已经排好序的数组，去除其中的重复部分，返回新数组的大小)
static int SegPath_Unique(SegPath* objArray, int start, int end)
{
	int l = 0;
	int r = 1;
	while (r <= end)
	{
		if (!objArray[r].AEqualB(objArray[l]))//两者不相等
		{
			l++;
			objArray[l] = objArray[r];
		}
		r++;
	}
	return l + 1;
}



//快速排序（PointInfo版本）
static int PointInfo_Partition(PointInfo* objArray, int start, int end)
{
	PointInfo& temp = objArray[start];
	int i = start;
	int j = end;
	while (i < j) {
		while (i < j && objArray[j].ABigEqualB(temp))
			--j;
		objArray[i] = objArray[j];
		while (i < j && objArray[i].ASmallEqualB(temp))
			++i;
		objArray[j] = objArray[i];
	}
	objArray[i] = temp;//插入到正确位置
	return i;
}
static void PointInfo_Sort(PointInfo* objArray, int start, int end)
{
	if (start < end) {
		int pivot = PointInfo_Partition(objArray, start, end);
		PointInfo_Sort(objArray, start, pivot - 1);
		PointInfo_Sort(objArray, pivot + 1, end);
	}
}


//自定义unique函数（对于已经排好序的数组，去除其中的重复部分，返回新数组的大小)
static int PointInfo_Unique(PointInfo* objArray, int start, int end)
{
	int l = 0;
	int r = 1;
	while (r <= end)
	{
		if (!objArray[r].AEqualB(objArray[l]))//两者不相等
		{
			l++;
			objArray[l] = objArray[r];
		}
		r++;
	}
	return l + 1;
}
//计算每个点水平和垂直方向的连线数目&去重
static int PointInfo_CalcuAndUnique(PointInfo* objArray, int start, int end)
{
	int l = 0;
	int r = 1;
	while (r <= end)
	{
		if (!objArray[r].AEqualB(objArray[l]))//两者不相等
		{
			l++;
			objArray[l] = objArray[r];
		}
		else //相等的话，需要累加vertNum和horiNum
		{
			objArray[l].horiDirNum += objArray[r].horiDirNum;
			objArray[l].vertDirNum += objArray[r].vertDirNum;
		}
		r++;//注意unique操作
	}
	return l + 1;
}

//二分查找
static PointInfo FindPointInfo(PointInfo* pointInfoList, int start, int end, Vector2Int point)
{
	int left = start;
	int right = end;
	PointInfo res;
	while (start <= end)
	{
		int mid = (start + end) >> 1;
		if (pointInfoList[mid].pointAxis == point)
		{
			res = pointInfoList[mid];
			break;
		}
		else if (pointInfoList[mid].pointAxis < point)
		{
			mid = left + 1;
		}
		else
		{
			mid = right - 1;
		}
	}
	return res;
}

//归并排序(APoint版本)
//注意排序规则，是按照APoint的f值进行比较
void Merge(APoint** objArray, int start, int middle, int end, APoint** tempArray)
{
	int i = start;
	int j = middle + 1;
	int index = 0;
	while (i <= middle && j <= end)
	{
		if (objArray[i]->f <= objArray[j]->f) {//排序规则
			tempArray[index++] = objArray[i++];
		}
		else {
			tempArray[index++] = objArray[j++];
		}
	}
	while (i <= middle) {
		tempArray[index++] = objArray[i++];
	}
	while (j <= end) {
		tempArray[index++] = objArray[j++];
	}
	for (int i = 0; i < index; i++) {
		objArray[start + i] = tempArray[i];
	}
}

//使用归并排序实现稳定的sort(APoint)
static void StableSort_APoint(APoint** objArray, int start, int end, APoint** tempArray)
{
	if (start < end)
	{
		int middle = (start + end) >> 1;
		StableSort_APoint(objArray, start, middle, tempArray);
		StableSort_APoint(objArray, middle + 1, end, tempArray);
		Merge(objArray, start, middle, end, tempArray);
	}
}

//生成随机数相关代码 GPU
//初始化随机数生成器
__global__ void initRandomGenerator(curandState* state, unsigned long seed)
{
	int id = threadIdx.x;
	curand_init(seed, id, 0, &state[id]);
}
//生成一个随机数
__device__ float createARandomNum(curandState* globalState, int ind)
{
	curandState localState = globalState[ind];
	float RANDOM = curand_uniform(&localState);
	globalState[ind] = localState;
	return RANDOM;
}
//生成随机数的核函数
//__global__ void generateRandomNum(float* N, curandState* globalState)
//{
//	int i = blockIdx.x * blockDim.x + threadIdx.x;//当前序号
//	float k = generate(globalState, i);
//	N[i] = k;
//}
