#pragma once
#include "PSO.h"
#include "Tools.h"
#include <math.h>
#include <algorithm>
#include <cmath>
#define PI 3.1415926
#define DOUBLE_MAX 1.7976931348623158e+308
#define DOUBLE_MIN 2.2250738585072014e-308
#define MAX_FITNESS 100000.0
void FitnessFunction(Particle & particle, ProblemParas proParas);
double CalcuTotalArea(Particle& particle, ProblemParas proParas);
//默认的适应度计算函数，可以替换
void FitnessFunction(Particle & particle, ProblemParas proParas)
{
	//double* fitness = new double[2];
	//for (int i = 0; i < 2; i++) {
	//	fitness[i] = 0;
	//}
	double deviceDist = 0;
	particle.fitness_[0] = particle.fitness_[1] = 0;
	for (int i = 0; i < particle.dim_; i+=2) {
		for (int j = 0; j < particle.dim_; j+=2) {
			deviceDist = abs(particle.position_[i] - particle.position_[j])
				+ abs(particle.position_[i + 1] - particle.position_[j + 1]);
			//if (deviceDist < proParas.MinDistArray[i / 2][j / 2])
			//{
			//	fitness[0] = fitness[1] = MAX_FITNESS;
			//	break;
			//}
			//添加成本作为适应度值（考虑最小距离）
			particle.fitness_[0] += deviceDist * proParas.CostParaArray[i / 2][j / 2].UnitCost * proParas.CostParaArray[i / 2][j / 2].MatFlow;
		}
	}
	//如果重叠，加入惩罚
	for (int i = 0; i < particle.dim_; i += 2) {
		double firstLowX = particle.position_[i] - 0.5 * proParas.DeviceSizeArray[i / 2].x;
		double firstUpX = particle.position_[i] + 0.5 * proParas.DeviceSizeArray[i / 2].x;
		double firstLowY = particle.position_[i + 1] - 0.5 * proParas.DeviceSizeArray[i / 2].y;
		double firstUpY = particle.position_[i + 1] + 0.5 * proParas.DeviceSizeArray[i / 2].y;
		for (int j = i + 2; j < particle.dim_; j += 2) {
			double secondLowX = particle.position_[j] - 0.5 * proParas.DeviceSizeArray[j / 2].x;
			double secondUpX = particle.position_[j] + 0.5 * proParas.DeviceSizeArray[j / 2].x;
			double secondLowY = particle.position_[j + 1] - 0.5 * proParas.DeviceSizeArray[j / 2].y;
			double secondUpY = particle.position_[j + 1] + 0.5 * proParas.DeviceSizeArray[j / 2].y;
			if (IsRangeOverlap(firstLowX, firstUpX, secondLowX, secondUpX) && IsRangeOverlap(firstLowY, firstUpY, secondLowY, secondUpY)) {
				particle.fitness_[0] = particle.fitness_[1] = MAX_FITNESS;
				return;
			}
		}
	}
	//面积也作为适应度值
	particle.fitness_[1] = CalcuTotalArea(particle, proParas);
	return;
}
//计算占地面积
double CalcuTotalArea(Particle& particle, ProblemParas proParas) {
	double area = 0;
	double min_X, min_Y, max_X, max_Y;
	int min_X_index, min_Y_index, max_X_index, max_Y_index;
	min_X = min_Y = DOUBLE_MAX;
	max_X = max_Y = DOUBLE_MIN;
	min_X_index = min_Y_index = max_X_index = max_Y_index = 0;
	for (int i = 0; i < particle.dim_; i+=2) {
		if (particle.position_[i] - proParas.DeviceSizeArray[i / 2].x * 0.5 < min_X) {
			min_X = particle.position_[i] - proParas.DeviceSizeArray[i / 2].x * 0.5;
			min_X_index = i / 2;
		}
		if (particle.position_[i + 1] - proParas.DeviceSizeArray[i / 2].y * 0.5 < min_Y) {
			min_Y = particle.position_[i + 1] - proParas.DeviceSizeArray[i / 2].y * 0.5;
			min_Y_index = i / 2; 
		}
		if (particle.position_[i] + proParas.DeviceSizeArray[i / 2].x * 0.5 > max_X) {
			max_X = particle.position_[i] + proParas.DeviceSizeArray[i / 2].x * 0.5;
			max_X_index = i / 2;
		}
		if (particle.position_[i + 1] + proParas.DeviceSizeArray[i / 2].y * 0.5 > max_Y) {
			max_Y = particle.position_[i + 1] + proParas.DeviceSizeArray[i / 2].y * 0.5;
			max_Y_index = i / 2;
		}
	}
	//计算总面积
	area = (max_X - min_X) * (max_Y - min_Y);
	return area;
}
