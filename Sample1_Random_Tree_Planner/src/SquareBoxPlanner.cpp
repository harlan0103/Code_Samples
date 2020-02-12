#include <iostream>
#include <fstream>
#include <cmath>
#include <functional>
#include <memory>

// Including SimpleSetup will pull in MOST of what you need to plan
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "CollisionChecking.h"
#include "RTP.h"

using namespace std::placeholders;

// This is our state validity checker for checking if our square robot is in collision
bool isValidStateSquare(const ompl::base::State *state, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // Cast the state to a compound state
    auto cstate = state->as<ompl::base::CompoundState>();

    // Extract the real vector component (x, y)
    auto r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x = r2state->values[0];
    double y = r2state->values[1];

    // Extract theta (SO(2))
    auto so2State = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
    double theta = so2State->value;

    return isValidSquare(x, y, theta, sideLength, obstacles);
}

void planWithSimpleSetupSE2(const std::vector<Rectangle> &obstacles)
{
    // Create the state (configuration) space for your system
    ompl::base::StateSpacePtr se2;

    // Create the R^2 component of the space
    auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2);  // x and y have a minimum of -2
    bounds.setHigh(2);  // x and y have a maximum of 2

    // Set the bounds on R^2
    r2->setBounds(bounds);

    // Create the SO(2) component of the state space
    auto so2 = std::make_shared<ompl::base::SO2StateSpace>();

    // Create the compound state space (R^2 X SO(2) = SE(2))
    se2 = r2 + so2;

    // Create the SimpleSetup container for the motion planning problem.
    ompl::geometric::SimpleSetup ss(se2);

    // Setup the StateValidityChecker
    ss.setStateValidityChecker(std::bind(isValidStateSquare, _1, 0.3, obstacles));

    // Specify the start and goal states
    ompl::base::ScopedState<> start(se2);
    start[0] = -1.3;
    start[1] = -1.3;
    start[2] = 0.0;

    ompl::base::ScopedState<> goal(se2);
    goal[0] = 1.2;
    goal[1] = 1.2;
    goal[2] = 0.0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Specify a planning algorithm to use
    auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
         ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "SE2" << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void planBox(const std::vector<Rectangle> & obstacles)
{
    // TODO: Use your implementation of RTP to plan for a rotating square robot.
    planWithSimpleSetupSE2(obstacles);
}

void makeEnvironment(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your first environment.
    Rectangle obs1;
    obs1.x = -0.5;
    obs1.y = -1.8;
    obs1.width = 1.0;
    obs1.height = 0.1;
    obstacles.push_back(obs1);

    Rectangle obs2;
    obs2.x = -0.5;
    obs2.y = -1.2;
    obs2.width = 1.0;
    obs2.height = 0.1;
    obstacles.push_back(obs2);

    Rectangle obs3;
    obs3.x = -0.5;
    obs3.y = -0.6;
    obs3.width = 1.0;
    obs3.height = 0.1;
    obstacles.push_back(obs3);

    Rectangle obs4;
    obs4.x = -0.5;
    obs4.y = 0.0;
    obs4.width = 1.0;
    obs4.height = 0.1;
    obstacles.push_back(obs4);

    Rectangle obs5;
    obs5.x = -0.5;
    obs5.y = 0.6;
    obs5.width = 1.0;
    obs5.height = 0.1;
    obstacles.push_back(obs5);

    Rectangle obs6;
    obs6.x = -0.5;
    obs6.y = 1.2;
    obs6.width = 1.0;
    obs6.height = 0.1;
    obstacles.push_back(obs6);

    Rectangle obs7;
    obs7.x = -1.3;
    obs7.y = 0.0;
    obs7.width = 0.1;
    obs7.height = 2.0;
    obstacles.push_back(obs7);

    Rectangle obs8;
    obs8.x = 1.1;
    obs8.y = -2.0;
    obs8.width = 0.1;
    obs8.height = 2.3;
    obstacles.push_back(obs8);

    /** print obstacles data into txt file **/
    std::ofstream fout("obstacles.txt");
    for (auto obs : obstacles) {
        fout << to_string(obs.x) + " " + to_string(obs.y) + " " + to_string(obs.width) + " " + to_string(obs.height) << endl;
    }
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 1);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) Environment 1" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 1);

    switch (choice)
    {
        case 1:
            makeEnvironment(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
