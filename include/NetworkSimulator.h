#ifndef _NETWORKSIMULATOR_H
#define _NETWORKSIMULATOR_H

#include "SimulationUnit.h"

#include <list>
#include <string>

#include "TopologyWrapper.h"

#include "PathNetwork.h"

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
                m_attacker->TakeAction();

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
                            it->second.first, it->second.second, sf::Color::Yellow);
                }
            } else {
                m_defender->TakeAction();
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