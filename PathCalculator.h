/*
 * PathCalculator.h
 *
 *  Created on: May 9, 2018
 *      Author: Dana
 */

#ifndef PATHCALCULATOR_H_
#define PATHCALCULATOR_H_

#include <cstdio>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

extern float** global_map;


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
	void PlanRoute();

};

#endif /* PATHCALCULATOR_H_ */
