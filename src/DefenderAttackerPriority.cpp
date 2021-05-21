#include "DefenderAttackerPriority.h"

#include <list>
#include <string>
#include <utility>
#include <vector>

#include "SFML/Graphics.hpp"

#include "NetworkSimulator.h"


void DefenderAttackerPriority::reset()
{

}

// Chooses an action based on the closest action of the path to the attacker and attempts to take it
void DefenderAttackerPriority::TakeAction()
{
    SimAction &action = NetworkSimulator::getInstance().m_currPath.front();

    SimulationUnit::TakeAction(action);
}
