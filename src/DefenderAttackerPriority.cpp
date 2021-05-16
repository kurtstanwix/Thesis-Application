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

void DefenderAttackerPriority::TakeAction()
{
    SimAction &action = NetworkSimulator::getInstance().m_currPath.front();
    float defendChance = 0.5;
    if ((float)rand() / RAND_MAX < defendChance) {
        TopologyWrapper::Link &link = NetworkSimulator::getInstance().m_top->getLinks().at(action.second);
        for (std::vector<TopologyWrapper::Link::VulInstance>::iterator it = link.m_vulnerabilities.begin();
                it != link.m_vulnerabilities.end(); ++it) {
            if (it->m_vul.cveID == action.first) {
                it->m_state = patched;
                it->m_highlighted = true;
                
                //NetworkSimulator::getInstance().m_currPath.clear();

                break;
            }
        }
        NetworkSimulator::getInstance().m_top->updateLinkInfo(link);
        NetworkSimulator::getInstance().m_defenderActions.emplace_back(action, success);
        PLOGI << "Defender: Patching " << action.first << " on link " << action.second.first << "->" << action.second.second << ": SUCCESS";
    } else {

        NetworkSimulator::getInstance().m_defenderActions.emplace_back(action, fail);
        PLOGI << "Defender: Patching " << action.first << " on link " << action.second.first << "->" << action.second.second << ": FAIL";
    }
}
