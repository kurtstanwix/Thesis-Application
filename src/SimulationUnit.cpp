#include "SimulationUnit.h"

#include "NetworkSimulator.h"

// Attempt to take the given action based on the probility of it succeeding
void SimulationUnit::TakeAction(const SimAction &action)
{
    TopologyWrapper::Vulnerability &vul = NetworkSimulator::getInstance().m_top->getVulnerabilities().at(action.first);
    float defendChance = vul.exploitability / MAX_EXPLOITABILITY_SCORE;
    // Scale probability to the range [0.4,0.8]
    defendChance -= 0.5f;
    defendChance /= 2.5f;
    defendChance += 0.6f;
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