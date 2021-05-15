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
    // Remove highlighting of previous defender moves
    for (std::list<std::pair<SimAction, ActionResult>>::iterator it = 
            NetworkSimulator::getInstance().m_defenderActions.begin();
            it != NetworkSimulator::getInstance().m_defenderActions.end(); ++it) {
        if (it->second == fail) {
            continue;
        }
        TopologyWrapper::Link &link = NetworkSimulator::getInstance().m_top->getLinks().at(it->first.second);
        if (NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->getLinkColor(
                link.m_endIds.first, link.m_endIds.second) == sf::Color::Green) {
            NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setLinkColor(
                    link.m_endIds.first, link.m_endIds.second, sf::Color::Blue);
        }
        for (std::vector<TopologyWrapper::Link::VulInstance>::iterator itt = link.m_vulnerabilities.begin();
                itt != link.m_vulnerabilities.end(); ++itt) {
            if (itt->m_vul.cveID == it->first.first) {
                itt->m_highlighted = false; // Only want the current move highlighted
                break;
            }
        }
        NetworkSimulator::getInstance().m_top->updateLinkInfo(link);
    }
    SimAction &action = NetworkSimulator::getInstance().m_currPath.front();
    float defendChance = 0.5;
    if ((float)rand() / RAND_MAX < defendChance) {
        TopologyWrapper::Link &link = NetworkSimulator::getInstance().m_top->getLinks().at(action.second);
        NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setLinkColor(
                link.m_endIds.first, link.m_endIds.second, sf::Color::Green);
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
