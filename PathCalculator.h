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
	class myMotionValidator : public ob::MotionValidator
	{
	public:
		myMotionValidator(const ob::SpaceInformationPtr &si);
		bool checkMotion(const ob::State *s1, const ob::State *s2) const;
		bool checkMotion(const ob::State *s1, const ob::State *s2,  std::pair<ob::State *, double> &lastValid) const;
	};
	static ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation *si);
	static bool isStateValid(const ob::State *state);

public:
	PathCalculator (int rows, int cols, float** map, int* coordinates, int radius, int velocity);
	void PlanRoute();

};

#endif /* PATHCALCULATOR_H_ */
