/*
 * PathCalculator.cpp
 *
 *  Created on: May 9, 2018
 *      Author: Dana
 */

#include <cstdio>
#include "PathCalculator.h"
#include "Defs.h"

float** global_map;

PathCalculator::PathCalculator(int rows, int cols, float** map, int* coordinates, int radius, int velocity)
{
	this->rows = rows;
	this->cols = cols;
	this->map = map;
	global_map = map;
	this->coordinates = coordinates;
	this->radius = radius;
	this->velocity = velocity;
}

PathCalculator::myMotionValidator::myMotionValidator(const ob::SpaceInformationPtr &si) : ob::MotionValidator(si) {

}


bool PathCalculator::myMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{
	const ob::RealVectorStateSpace::StateType& pos1 =
			*s1->as<ob::RealVectorStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType& pos2 =
			*s2->as<ob::RealVectorStateSpace::StateType>();

	double x = pos1[0] - pos2[0];
	double y = pos1[1] - pos2[1];
	double z = pos1[2] - pos2[2];
	double dist;

	dist = x*x + y*y + z*z;
	dist = sqrt(dist);
	return (dist < MAX_DISTANCE_FOR_STEP);
}
bool PathCalculator::myMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2,  std::pair<ob::State *, double> &lastValid) const
{
	const ob::RealVectorStateSpace::StateType& pos1 =
			*s1->as<ob::RealVectorStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType& pos2 =
			*s2->as<ob::RealVectorStateSpace::StateType>();

	double x = pos1[0] - pos2[0];
	double y = pos1[1] - pos2[1];
	double z = pos1[2] - pos2[2];
	double dist;

	dist = x*x + y*y + z*z;
	dist = sqrt(dist);
	return (dist < MAX_DISTANCE_FOR_STEP);
}


// This is a problem-specific sampler that automatically generates valid
// states; it doesn't need to call SpaceInformation::isValid. This is an
// example of constrained sampling. If you can explicitly describe the set valid
// states and can draw samples from it, then this is typically much more
// efficient than generating random samples from the entire state space and
// checking for validity.
class MyValidStateSampler : public ob::ValidStateSampler
{
public:
    MyValidStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si)
    {
        name_ = "my sampler";
    }
    // Generate a sample in the valid part of the R^3 state space
    // Valid states satisfy the following constraints: z > global_map[x,y]
    bool sample(ob::State *state) override
    {
        double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
        double x = rng_.uniformReal(0,99.99);
        double y = rng_.uniformReal(0,99.99);
        int x_int = (int)x;
        int y_int = (int)y;
        if (global_map[x_int][y_int] > MAX_HEIGHT)
        	return false;
        double z = rng_.uniformReal((double)global_map[x_int][y_int],MAX_HEIGHT);

        val[0] = x;
        val[1] = y;
        val[2] = z;
        if (si_->isValid(state)) {
        	return true;
        }


        return false;
    }
    // We don't need this in the example below.
    bool sampleNear(ob::State* /*state*/, const ob::State* /*near*/, const double /*distance*/) override
    {
        throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
        return false;
    }
protected:
    ompl::RNG rng_;
};


// return an instance of my sampler
ob::ValidStateSamplerPtr PathCalculator::allocMyValidStateSampler(const ob::SpaceInformation *si)
{
    return std::make_shared<MyValidStateSampler>(si);
}


// this function is needed, even when we can write a sampler like the one
// above, because we need to check path segments for validity
bool PathCalculator::isStateValid(const ob::State *state)
{
    const ob::RealVectorStateSpace::StateType& pos = *state->as<ob::RealVectorStateSpace::StateType>();
    if ((pos[0]<MAP_SIZE && pos[0]>0) && (pos[1]<MAP_SIZE && pos[1]>0))
	    return pos[2] < 20;
    else
	    return false;
}

void PathCalculator::PlanRoute()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));
    // auto space(std::make_shared<ob::SE3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0);
    bounds.setHigh(MAP_SIZE);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    si->setStateValidityCheckingResolution(0.03);
    si->setValidStateSamplerAllocator(allocMyValidStateSampler);
    si->setMotionValidator(std::make_shared<myMotionValidator>(si));

    // create the start state at [1 1 1]
    ob::ScopedState<ob::SE3StateSpace> start(space);
    start[0] = 1;
    start[1] = 1;
    start[2] = 1;
    // start->rotation().setIdentity();

    // create the goal state at [99 99 1]
    ob::ScopedState<ob::SE3StateSpace> goal(space);
    goal[0] = 99;
    goal[1] = 99;
    goal[2] = 1;
    // goal->rotation().setIdentity();

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);
    // create a planner for the defined space
    auto planner(std::make_shared<og::PRM>(si));
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    // si->printSettings(std::cout);
    // print the problem settings
    // pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}
