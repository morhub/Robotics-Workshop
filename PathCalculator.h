/*
 * PathCalculator.h
 *
 *  Created on: May 9, 2018
 *      Author: Dana
 */

#ifndef PATHCALCULATOR_H_
#define PATHCALCULATOR_H_
#include <cstdio>


class PathCalculator
{
private:
	int rows;
	int cols;
	float** map;
	int* coordinates = NULL;
	int radius;
	int velocity;
public:
	PathCalculator (int rows, int cols, float** map, int* coordinates, int radius, int velocity);
};




#endif /* PATHCALCULATOR_H_ */
