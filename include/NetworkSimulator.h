#ifndef _NETWORKSIMULATOR_H
#define _NETWORKSIMULATOR_H

#include <list>
#include <string>

enum sim_turn {
    attacker,
    defender
};

enum ActionResult {
    fail,
    success
};

enum SimResult {
    undefined,
    attackerWin,
    defenderWin
};

typedef std::pair<std::string, std::pair<int, int>> SimAction;
typedef std::list<std::pair<SimAction, ActionResult>> SimActionHistory;

#include "SimulationUnit.h"


#include "MDPGUIConfig.h"
#include "TopologyWrapper.h"

// A singleton which manages the state of a simulation
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

    int m_numSimulations;
    int m_currSimulation;

    SimResult m_simResult;

    SimulationUnit *m_attacker;
    SimulationUnit *m_defender;

    std::list<SimAction> m_currPath;
    
    SimActionHistory m_attackerActions;
    SimActionHistory m_defenderActions;

    std::list<std::pair<std::pair<SimActionHistory, SimActionHistory>, SimResult>> m_history;


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
    
    // Reset the simulation
    void reset() {
        m_top->reset(m_config);
        m_step = 0;
        m_currSimulation++;
        m_currNode = m_startNode;
        m_finished = false;
        m_toMove = attacker;
        m_currPath.clear();
        saveHistory();
        m_simResult = undefined;
    }

    void nextAction();

    void printActions(const SimActionHistory &attackerActions,
            const SimActionHistory &defenderActions, const SimResult &result);

    void runSimulation();

    void runAllSimulations();

    void saveHistory();

    void printCurrentHistory();
};

#endif