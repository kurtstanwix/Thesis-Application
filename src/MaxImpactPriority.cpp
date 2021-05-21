#include "MaxImpactPriority.h"

#include <list>
#include <string>
#include <utility>
#include <vector>

#include "SFML/Graphics.hpp"

#include "NetworkSimulator.h"


void MaxImpactPriority::reset()
{

}

// Chooses an action based on the maximum impact value of the path and attempts to take it
void MaxImpactPriority::TakeAction()
{
    // Find the highest cvss impact score
    std::list<SimAction>::iterator sa = NetworkSimulator::getInstance().m_currPath.begin();
    for (std::list<SimAction>::iterator sai = NetworkSimulator::getInstance().m_currPath.begin();
            sai != NetworkSimulator::getInstance().m_currPath.end(); ++sai) {
        TopologyWrapper::Vulnerability &currVul = NetworkSimulator::getInstance().m_top->getVulnerabilities().at(sa->first);
        TopologyWrapper::Vulnerability &newVul = NetworkSimulator::getInstance().m_top->getVulnerabilities().at(sai->first);
        if (newVul.impact > currVul.impact) {
            sa = sai;
        }
    }
    SimAction &action = *sa;

    SimulationUnit::TakeAction(action);
}
