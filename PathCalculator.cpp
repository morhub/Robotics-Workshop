/*
 * PathCalculator.cpp
 *
 *  Created on: May 9, 2018
 *      Author: Dana
 */

#include <cstdio>
#include "PathCalculator.h"

PathCalculator::PathCalculator(int rows, int cols, float** map, int* coordinates, int radius, int velocity)
{
	this->rows = rows;
	this->cols = cols;
	this->map = map;
	this->coordinates = coordinates;
	this->radius = radius;
	this->velocity = velocity;
}





