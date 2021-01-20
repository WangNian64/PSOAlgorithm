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
//���к��������϶���
//����һ�������
__device__ float createARandomNum(curandState* globalState, int index);
static __device__ double getAbs(double num)
{
	return num > 0 ? num : -num;
}

static __device__ double getMax(double a, double b)
{
	return a > b ? a : b;
}

static __device__ double getMin(double a, double b)
{
	return a < b ? a : b;
}
//�ж����������Ƿ��ص�
static __device__ bool IsRangeOverlap(double low1, double upper1, double low2, double upper2) {
	if (getMax(low1, low2) <= getMin(upper1, upper2)) {
		return true;
	}
	else
	{
		return false;
	}
}
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
static double GetDoubleRand(int N = 99)
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
//��������DeviceIDSize�汾��
static __device__ void DeviceIDSize_Sort(DeviceIDSize* sizeArray, int start, int end)
{
	if (start < end) {
		int pivot = DeviceIDSize_Partition(sizeArray, start, end);
		DeviceIDSize_Sort(sizeArray, start, pivot - 1);
		DeviceIDSize_Sort(sizeArray, pivot + 1, end);
	}
}
__device__ int DeviceIDSize_Partition(DeviceIDSize* sizeArray, int start, int end)
{
	DeviceIDSize& temp = sizeArray[start];//���ÿ���ʹ���𣿣���
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
	sizeArray[i] = temp;//���뵽��ȷλ��
	return i;
}
//��������double�汾��
static __device__ int Double_Partition(double* numArray, int start, int end)
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
	numArray[i] = temp;//���뵽��ȷλ��
	return i;
}
static __device__ void Double_Sort(double* numArray, int start, int end)
{
	if (start < end) {
		int pivot = Double_Partition(numArray, start, end);
		Double_Sort(numArray, start, pivot - 1);
		Double_Sort(numArray, pivot + 1, end);
	}
}


//�Զ���unique�����������Ѿ��ź�������飬ȥ�����е��ظ����֣�����������Ĵ�С)
static __device__ int Double_Unique(double* numArray, int start, int end)
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

//�Զ������������㷨
static int MyRound(double num)
{
	return (num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5);
}

//��������SegPath�汾��
__device__ int SegPath_Partition(SegPath* objArray, int start, int end)
{
	SegPath& temp = objArray[start];
	int i = start;
	int j = end;
	while (i < j) {
		while (i < j && objArray[j].ABigEqualB(temp, -1))
			--j;
		objArray[i] = objArray[j];
		while (i < j && objArray[i].ASmallEqualB(temp, -1))
			++i;
		objArray[j] = objArray[i];
	}
	objArray[i] = temp;//���뵽��ȷλ��
	return i;
}
static __device__ void SegPath_Sort(SegPath* objArray, int start, int end)
{
	if (start < end) {
		int pivot = SegPath_Partition(objArray, start, end);
		SegPath_Sort(objArray, start, pivot - 1);
		SegPath_Sort(objArray, pivot + 1, end);
	}
}


//�Զ���unique�����������Ѿ��ź�������飬ȥ�����е��ظ����֣�����������Ĵ�С)
static __device__ int SegPath_Unique(SegPath* objArray, int start, int end)
{
	int l = 0;
	int r = 1;
	while (r <= end)
	{
		if (!objArray[r].AEqualB(objArray[l], -1))//���߲����
		{
			l++;
			objArray[l] = objArray[r];
		}
		r++;
	}
	return l + 1;
}



//��������PointInfo�汾��
static __device__ int PointInfo_Partition(PointInfo* objArray, int start, int end)
{
	PointInfo& temp = objArray[start];
	int i = start;
	int j = end;
	while (i < j) {
		while (i < j && objArray[j].ABigEqualB(temp, -1))
			--j;
		objArray[i] = objArray[j];
		while (i < j && objArray[i].ASmallEqualB(temp, -1))
			++i;
		objArray[j] = objArray[i];
	}
	objArray[i] = temp;//���뵽��ȷλ��
	return i;
}
static __device__ void PointInfo_Sort(PointInfo* objArray, int start, int end)
{
	if (start < end) {
		int pivot = PointInfo_Partition(objArray, start, end);
		PointInfo_Sort(objArray, start, pivot - 1);
		PointInfo_Sort(objArray, pivot + 1, end);
	}
}


//�Զ���unique�����������Ѿ��ź�������飬ȥ�����е��ظ����֣�����������Ĵ�С)
static int PointInfo_Unique(PointInfo* objArray, int start, int end)
{
	int l = 0;
	int r = 1;
	while (r <= end)
	{
		if (!objArray[r].AEqualB(objArray[l]))//���߲����
		{
			l++;
			objArray[l] = objArray[r];
		}
		r++;
	}
	return l + 1;
}
//����ÿ����ˮƽ�ʹ�ֱ�����������Ŀ&ȥ��
static int PointInfo_CalcuAndUnique(PointInfo* objArray, int start, int end)
{
	int l = 0;
	int r = 1;
	while (r <= end)
	{
		if (!objArray[r].AEqualB(objArray[l]))//���߲����
		{
			l++;
			objArray[l] = objArray[r];
		}
		else //��ȵĻ�����Ҫ�ۼ�vertNum��horiNum
		{
			objArray[l].horiDirNum += objArray[r].horiDirNum;
			objArray[l].vertDirNum += objArray[r].vertDirNum;
		}
		r++;//ע��unique����
	}
	return l + 1;
}

//���ֲ���
static __device__ PointInfo FindPointInfo(PointInfo* pointInfoList, int start, int end, Vector2Int point)
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

//�鲢����(APoint�汾)
//ע����������ǰ���APoint��fֵ���бȽ�
void Merge(APoint** objArray, int start, int middle, int end, APoint** tempArray)
{
	int i = start;
	int j = middle + 1;
	int index = 0;
	while (i <= middle && j <= end)
	{
		if (objArray[i]->f <= objArray[j]->f) {//�������
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

//ʹ�ù鲢����ʵ���ȶ���sort(APoint)
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

//�����������ش��� GPU
//��ʼ�������������
__global__ void initRandomGenerator(curandState* state, unsigned long seed)
{
	int id = threadIdx.x;
	curand_init(seed, id, 0, &state[id]);
}
//����һ�������
__device__ float createARandomNum(curandState* globalState, int index)
{
	curandState localState = globalState[index];
	float RANDOM = curand_uniform(&localState);
	globalState[index] = localState;
	return RANDOM;
}
//����������ĺ˺���
//__global__ void generateRandomNum(float* N, curandState* globalState)
//{
//	int i = blockIdx.x * blockDim.x + threadIdx.x;//��ǰ���
//	float k = generate(globalState, i);
//	N[i] = k;
//}
