#include <limits.h>

#include <iostream>

#include <chrono>
#include <ctime>
#include <stdlib.h>

#include "DecPOMDPDiscrete.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "NullPlanner.h"
#include "Timing.h"
#include "BruteForceSearchPlanner.h"

#include "MDPValueIteration.h"
#include "MDPPolicyIteration.h"
#include "QTable.h"
#include "AgentMDP.h"
#include "JointPolicyDiscrete.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"

#include "plog/Log.h"
#include "plog/Init.h"
#include "plog/Formatters/TxtFormatter.h"
#include "plog/Appenders/ConsoleAppender.h"

#include "SFML/Graphics.hpp"

#include "LayeredRenderable.h"
#include "NetworkWindow.h"


#include "Config.h"
#include "DefenderAttackerPriority.h"
#include "DefenderGoalPriority.h"
#include "Interface.h"
#include "MaxExploitabilityPriority.h"
#include "MaxImpactPriority.h"
#include "NetworkSimulator.h"
#include "PathNetwork.h"

#include "Utility.h"

#include "MDPGUIConfig.h"

#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

using namespace std;

// Callback for button to exit the application
void exitAction(InterfaceButton &caller)
{
    exit(0);
}

// Callback for button to save the current state
// WARNING currently out of date and can corrupt state
void saveAction(InterfaceButton &caller)
{
    PLOGI << "Saving";
    caller.m_parent.m_nettop.save("../test2");
}

// Callback for button to take a single step in the current simulation
void takeStep(InterfaceButton &caller)
{
    NetworkSimulator::getInstance().nextAction();
}

// Callback for button to run the current simulation to the end
void runSimulation(InterfaceButton &caller)
{
    NetworkSimulator::getInstance().runSimulation();
}

// Callback for button to run all remaining simulations
void runAllSimulations(InterfaceButton &caller)
{
    NetworkSimulator::getInstance().runAllSimulations();
}

// Callback for button to print all current simulation results
void printSimulations(InterfaceButton &caller)
{
    NetworkSimulator::getInstance().printCurrentHistory();
}

typedef pair<size_t, size_t> sizePair;

#if _TEST_ == 0
int main(int argc, char **argv)
{
    static plog::ConsoleAppender<plog::TxtFormatter> debugConsole;
    plog::init(plog::verbose, &debugConsole);
    
    
    MDPGUIConfig config = MDPGUIConfig::LoadConfig(RESOURCE("SimulationConfig.json"));
    NetworkSimulator::getInstance().m_config = config;

    plog::get()->setMaxSeverity(config.logLevel);
    
    using namespace std::chrono;
    
    int squareWidth = 100;
    int width = config.width;
    int height = config.height;

    srand(time(0));
    
    int nrAgents = 1;
    
    sf::RenderWindow window(sf::VideoMode(width, height), "MDP Policy");
    if (!config.GUI) {
        // In headless mode, simply hide the window
        window.setVisible(false);
    }
    sf::Vector2f windowSize(window.getSize());
    

    // Setup the simulation parameters
    TopologyWrapper mdpWindow(windowSize, config, std::vector<std::string>({"patch"}));

    NetworkSimulator::getInstance().m_currSimulation = 0;
    NetworkSimulator::getInstance().m_numSimulations = config.numSimulations;

    NetworkSimulator::getInstance().m_startNode = config.startNodeId;
    NetworkSimulator::getInstance().m_goalNode = config.goalNodeId;
    NetworkSimulator::getInstance().m_top = &mdpWindow;

    NetworkSimulator::getInstance().reset();

    NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setNodeColor(config.startNodeId, config.attackerMoveColor);
    NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setNodeColor(config.goalNodeId, config.goalColor);

    PathNetwork attacker = PathNetwork(config.startNodeId, config.goalNodeId, windowSize);
    NetworkSimulator::getInstance().m_attacker = &attacker;

    // Determine the policy the defender will take
    SimulationUnit *defender;
    switch (config.defenderPolicy) {
        case DefenderPolicy::attack:
            defender = new DefenderAttackerPriority();
            break;
        case DefenderPolicy::goal:
            defender = new DefenderGoalPriority();
            break;
        case DefenderPolicy::exploitability:
            defender = new MaxExploitabilityPriority();
            break;
        case DefenderPolicy::impact:
        default:
            defender = new MaxImpactPriority();
    }
    NetworkSimulator::getInstance().m_defender = defender;

    // Register the buttons as seen on the interface
    mdpWindow.m_netWindow.registerButton(
            "Exit", &exitAction, sf::Color(200, 30, 30),
            {windowSize.x - windowSize.x / 20.0f, windowSize.y / 20.0f}, windowSize / 20.0f);

    mdpWindow.m_netWindow.registerButton(
            "Step", &takeStep, sf::Color(200, 150, 50), windowSize - windowSize / 20.0f,
            windowSize / 20.0f);

    mdpWindow.m_netWindow.registerButton(
            "Run", &runSimulation, sf::Color(150, 0, 150), { windowSize.x / 20.0f, windowSize.y - windowSize.y / 20.0f },
            windowSize / 20.0f);
    
    mdpWindow.m_netWindow.registerButton(
            "Run All", &runAllSimulations, sf::Color(150, 0, 150), { 5.0f * windowSize.x / 40.0f + 10.0f, windowSize.y - windowSize.y / 20.0f },
            { windowSize.x / 10.0f, windowSize.y / 20.0f });
    
    mdpWindow.m_netWindow.registerButton(
            "Print", &printSimulations, sf::Color(30, 200, 30),
            { 3.0f * windowSize.x / 40.0f, windowSize.y / 20.0f },
            { windowSize.x / 10.0f, windowSize.y / 20.0f });

    // Show the created model
    vector<size_t> nrActions = attacker.GetNrActions();
    PLOGI << "Size " << nrActions.size();
    for (Index agentI = 0; agentI < nrAgents; agentI++) {
        for (Index actionI = 0; actionI < nrActions.at(agentI); actionI++) {
            const Action* a = attacker.GetAction(agentI, actionI);
            PLOGI << "Agent " << agentI << ": Action " << actionI << ": " <<
                    a->GetName() << ", " << a->GetDescription();
        }
    }
    PLOGI << "Transitions:";
    PLOGI << attacker.GetTransitionModelDiscretePtr()->SoftPrint();
    
    PLOGI << "Observations:";
    PLOGI << attacker.GetObservationModelDiscretePtr()->SoftPrint();

    PLOGI << "Rewards:";
    PLOGI << attacker.GetRewardModelPtr()->SoftPrint();
    PLOGI << "Reward type: " << attacker.GetRewardType();

    static const microseconds step(1000000 / 60); // Limit for system updates
    system_clock::time_point oldTime = system_clock::now();

    int ticks = 0;
    int nodeId = 5;
    if (config.GUI) {
        while (window.isOpen())
        {
            sf::Vector2f windowSize(window.getSize());
            // Limit updating the system to 60 times a second
            if (duration_cast<microseconds>(system_clock::now() - oldTime).count() >= step.count()) {
                ticks++;
                // Update logic
                sf::Event event;
                bool clickedOn = false;
                while (window.pollEvent(event)) {
                    if (event.type == sf::Event::Closed)
                        window.close();
                    else
                        mdpWindow.update(&event, windowSize, clickedOn);
                }
                mdpWindow.update(nullptr, windowSize, clickedOn);
                
                oldTime = system_clock::now();
            }
            
            // Render as fast as we can
            window.clear(config.backgroundColor);
            mdpWindow.render(window, windowSize);
            window.display();
        }
    } else {
        // If headless we simply run all simulations and prints the results
        NetworkSimulator::getInstance().runAllSimulations();
        NetworkSimulator::getInstance().printCurrentHistory();
    }
}

#endif