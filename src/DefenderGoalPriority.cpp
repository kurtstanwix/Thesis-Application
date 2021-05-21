#include "DefenderGoalPriority.h"

#include <list>
#include <string>
#include <utility>
#include <vector>

#include "SFML/Graphics.hpp"

#include "NetworkSimulator.h"


void DefenderGoalPriority::reset()
{

}

// Chooses an action based on the closest action of the path to the goal and attempts to take it
void DefenderGoalPriority::TakeAction()
{
    SimAction &action = NetworkSimulator::getInstance().m_currPath.back();

    SimulationUnit::TakeAction(action);
}
