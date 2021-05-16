#ifndef _NETWORKSIMULATOR_H
#define _NETWORKSIMULATOR_H

#include "SimulationUnit.h"

#include <list>
#include <string>

#include "MDPGUIConfig.h"
#include "TopologyWrapper.h"

//#include "PathNetwork.h"

enum sim_turn {
    attacker,
    defender
};

enum ActionResult {
    fail,
    success
};

typedef std::pair<std::string, std::pair<int, int>> SimAction;

class NetworkSimulator
{
protected:
    NetworkSimulator() {
        m_step = 0;
    }
public:
    int m_startNode;
    int m_goalNode;
    int m_currNode;
    int m_step;
    bool m_finished;
    sim_turn m_toMove;
    MDPGUIConfig m_config;

    SimulationUnit *m_attacker;
    SimulationUnit *m_defender;

    std::list<SimAction> m_currPath;
    
    std::list<std::pair<SimAction, ActionResult>> m_attackerActions;
    std::list<std::pair<SimAction, ActionResult>> m_defenderActions;


    TopologyWrapper *m_top;



    static NetworkSimulator& getInstance()
    {
        static NetworkSimulator instance;
        return instance;
    }

    /**
     * Cannot clone a singleton
     */
    NetworkSimulator(NetworkSimulator &other) = delete;
    void operator=(const NetworkSimulator &rhs) = delete;
    
    void reset() {
        m_step = 0;
        m_currNode = m_startNode;
        m_finished = false;
    }

    void nextAction()
    {
        if (!m_finished) {
            if (m_toMove == attacker) {
                m_attacker->reset();

                for (std::list<SimAction>::iterator it = m_currPath.begin(); it != m_currPath.end(); ++it) {
                    if (m_top->m_netWindow.m_nettop->getLinkColor(
                            it->second.first, it->second.second) == m_config.linkNewPathColor) {
                        m_top->m_netWindow.m_nettop->setLinkColor(
                                it->second.first, it->second.second, m_config.linkColor);
                    }
                }

                m_attacker->TakeAction();

                // Set the already travelled path colour
                for (std::list<std::pair<SimAction, ActionResult>>::iterator it =
                        m_attackerActions.begin(); it != m_attackerActions.end(); ++it) {
                    if (it->second == fail) {
                        continue;
                    }
                    TopologyWrapper::Link &link = m_top->getLinks().at(it->first.second);
                    m_top->m_netWindow.m_nettop->setLinkColor(link.m_endIds.first,
                            link.m_endIds.second, m_config.linkOldPathColor);
                    for (std::vector<TopologyWrapper::Link::VulInstance>::iterator itt = link.m_vulnerabilities.begin();
                            itt != link.m_vulnerabilities.end(); ++itt) {
                        if (itt->m_vul.cveID == it->first.first) {
                            itt->m_highlighted = false; // Only want the current move highlighted
                        }
                    }
                    m_top->updateLinkInfo(link);
                }

                std::pair<SimAction, ActionResult> &action = m_attackerActions.back();
                if (action.second == success) {
                    TopologyWrapper::Link &link = m_top->getLinks().at(action.first.second);
                    m_top->m_netWindow.m_nettop->setLinkColor(link.m_endIds.first,
                            link.m_endIds.second, m_config.attackerMoveColor);
                    for (std::vector<TopologyWrapper::Link::VulInstance>::iterator it = link.m_vulnerabilities.begin();
                            it != link.m_vulnerabilities.end(); ++it) {
                        if (it->m_vul.cveID == action.first.first) {
                            it->m_highlighted = true;
                            it->m_state = vulnerability_state::compromised;
                        }
                    }
                    m_top->updateLinkInfo(link);
                    m_top->m_netWindow.m_nettop->setNodeColor(
                            action.first.second.first, m_config.linkOldPathColor);
                    m_top->m_netWindow.m_nettop->setNodeColor(
                            action.first.second.second, m_config.attackerMoveColor);
                }

                /*
                PLOGI << "Transitions:";
                PLOGI << ((PathNetwork*)m_attacker)->GetTransitionModelDiscretePtr()->SoftPrint();
                
                PLOGI << "Observations:";
                PLOGI << ((PathNetwork*)m_attacker)->GetObservationModelDiscretePtr()->SoftPrint();

                PLOGI << "Rewards:";
                PLOGI << ((PathNetwork*)m_attacker)->GetRewardModelPtr()->SoftPrint();
                */
                m_toMove = defender;
                for (std::list<SimAction>::iterator it =  m_currPath.begin(); it != m_currPath.end(); ++it) {
                    m_top->m_netWindow.m_nettop->setLinkColor(
                            it->second.first, it->second.second, m_config.linkNewPathColor);
                }
            } else {
                // Remove highlighting of previous defender moves
                for (std::list<std::pair<SimAction, ActionResult>>::iterator it = 
                        m_defenderActions.begin(); it != m_defenderActions.end(); ++it) {
                    if (it->second == fail) {
                        continue;
                    }
                    TopologyWrapper::Link &link = m_top->getLinks().at(it->first.second);
                    if (m_top->m_netWindow.m_nettop->getLinkColor(
                            link.m_endIds.first, link.m_endIds.second) == m_config.defenderMoveColor) {
                        m_top->m_netWindow.m_nettop->setLinkColor(
                                link.m_endIds.first, link.m_endIds.second, m_config.linkColor);
                    }
                    for (std::vector<TopologyWrapper::Link::VulInstance>::iterator itt = link.m_vulnerabilities.begin();
                            itt != link.m_vulnerabilities.end(); ++itt) {
                        if (itt->m_vul.cveID == it->first.first) {
                            itt->m_highlighted = false; // Only want the current move highlighted
                            break;
                        }
                    }
                    m_top->updateLinkInfo(link);
                }
                m_defender->TakeAction();
                std::pair<SimAction, ActionResult> &action = m_defenderActions.back();
                if (action.second == success) {
                    m_top->m_netWindow.m_nettop->setLinkColor(
                            action.first.second.first, action.first.second.second, m_config.defenderMoveColor);
                }
                m_toMove = attacker;
            }

            if (m_finished || m_currNode == m_goalNode) {
                m_finished = true;
                std::cout << "Attacker actions:"  << std::endl;
                for (std::list<std::pair<SimAction, ActionResult>>::iterator it = m_attackerActions.begin();
                        it != m_attackerActions.end(); ++it) {
                    std::cout << " Exploit " << it->first.first << " on link " << it->first.second.first << "->" << it->first.second.second << ": "
                            << (it->second == fail ? "FAILED" : "SUCCEEDED") << std::endl;
                }
                std::cout << "Defender actions:" << std::endl;
                for (std::list<std::pair<SimAction, ActionResult>>::iterator it = m_defenderActions.begin();
                        it != m_defenderActions.end(); ++it) {
                    std::cout << " Patch " << it->first.first << " on link " << it->first.second.first << "->" << it->first.second.second << ": "
                            << (it->second == fail ? "FAILED" : "SUCCEEDED") << std::endl;
                }
            }
        }
    }

    void runSimulation()
    {
        while (!m_finished) {
            nextAction();
        }
    }
};

#endif