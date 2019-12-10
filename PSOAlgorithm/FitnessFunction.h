#pragma once
#include "PSO.h"
#include <math.h>
#include <cmath>
#define PI 3.1415926
#define DOUBLE_MAX 1.7976931348623158e+308
#define DOUBLE_MIN 2.2250738585072014e-308
double FitnessFunction(Particle & particle, ProblemParas proParas);
double CalcuTotalArea(Particle& particle, ProblemParas proParas);
//默认的适应度计算函数，可以替换
double FitnessFunction(Particle & particle, ProblemParas proParas)
{
	double fitness = 0.0;
	double deviceDist;

	for (int i = 0; i < particle.dim_; i+=2) {
		for (int j = 0; j < particle.dim_; j+=2) {
			deviceDist = abs(particle.position_[i] - particle.position_[i + 1])
				+ abs(particle.position_[j] - particle.position_[j + 1]);
			//添加成本作为适应度值
			fitness += deviceDist * proParas.CostParaArray[i / 2][j / 2].UnitCost * proParas.CostParaArray[i / 2][j / 2].MatFlow;
			//面积也作为适应度值
			fitness += 10 * CalcuTotalArea(particle, proParas);
		}
	}
	return 1.0 / fitness;//越小越好
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
		if (particle.position_[i] < min_X) {
			min_X = particle.position_[i];
			min_X_index = i / 2;
		}
		if (particle.position_[i + 1] < min_Y) {
			min_Y = particle.position_[i + 1];
			min_Y_index = i / 2; 
		}
		if (particle.position_[i] > max_X) {
			max_X = particle.position_[i];
			max_X_index = i / 2;
		}
		if (particle.position_[i + 1] > max_Y) {
			max_Y = particle.position_[i + 1];                                                                                                     
			max_Y_index = i / 2;
		}
	}
	//计算总面积+
	area = abs((max_X + 0.5 * proParas.DeviceSizeArray[max_X_index].x) - (min_X - 0.5 * proParas.DeviceSizeArray[min_X_index].x))
		* abs((max_Y + 0.5 * proParas.DeviceSizeArray[max_Y_index].y) - (min_Y - 0.5 * proParas.DeviceSizeArray[min_Y_index].y));
	return area;
}