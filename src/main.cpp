/*#include <stdio.h>

int main(int argc, char **argv)
{
	printf("hello world\n");
	return 0;
}
*/

/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

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
#include "MDPNetwork.h"
#include "NetworkSimulator.h"
#include "PathNetwork.h"

#include "Utility.h"

#include "MDPGUIConfig.h"















#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

using namespace std;


void exitAction(InterfaceButton &caller)
{
    exit(0);
}

void saveAction(InterfaceButton &caller)
{
    PLOGI << "Saving";
    caller.m_parent.m_nettop.save("../test2");
}

void takeStep(InterfaceButton &caller)
{
    NetworkSimulator::getInstance().nextAction();
}

void runSimulation(InterfaceButton &caller)
{
    NetworkSimulator::getInstance().runSimulation();
}

//int main(int argc, char **argv);

// Program documentation
/*
static char doc[] =
"example_MMDP_SolveAndSimulate - loads an (MMDP) problem, runs value iteration, and simulates the resulting joint policy using AgentMDP class. \
\vFor more information please consult the MADP documentation. \
\
+-------------------------------------------------------------------------+ \
|This example version is limited in functionality: e.g., it does not write| \
|results. For the full version, see ../solvers/MMDP_VI.                   | \
+-------------------------------------------------------------------------+ \
";
*/

typedef pair<size_t, size_t> sizePair;

//string actions[] = {"UP", "DOWN", "LEFT", "RIGHT"};

#if _TEST_ == 0
int main(int argc, char **argv)
{
    
    char result[ PATH_MAX ];
    ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
    cout << std::string( result, (count > 0) ? count : 0 ) << endl;
    
    static plog::ConsoleAppender<plog::TxtFormatter> debugConsole;
    plog::init(plog::info, &debugConsole);
    
    
    MDPGUIConfig config = MDPGUIConfig::LoadConfig(RESOURCE("SimulationConfig.json"));
    
    using namespace std::chrono;
    
    int squareWidth = 100;
    int width = config.width;
    int height = config.height;
    
    
    /*
    
    // Reformat the save file
    std::ifstream i("../gridNetwork.json");
    json j;
    i >> j;
    j["systemVulnerabilities"] =
            {
                {
                    {"cveID", "test"},
                    {"cvss", 2},
                    {"cweName", "overflow"}
                }
            };
    
    json &nodes = j["nodes"];// = j["systemVulnerabilities"];
    for (json::iterator n = nodes.begin(); n != nodes.end(); n++) {
        //PLOGI << " ID: " << (*it)["cveID"];
        json &links = (*n)["links"];
        for (json::iterator l = links.begin(); l != links.end(); l++) {
            json vul =
                    {
                        {
                            {"cveID", "test"},
                            {"state", 0}
                        }
                    };
            *l = 
                    {
                        {"otherNodeID", *l},
                        {"vulnerabilities", vul}
                    };
        }
    }
    
    std::ofstream o("../fixedGridNetwork.json");
    o << j.dump(4);
    exit(0);
    
    */
    /*
    // Reformat the MDPExample save file
    std::ifstream i("../MDPExample.json");
    json j;
    i >> j;
    j["systemVulnerabilities"] =
            {
                {
                    {"cveID", "up"},
                    {"cvss", 10},
                    {"cweName", "Move Up"}
                },
                {
                    {"cveID", "down"},
                    {"cvss", 10},
                    {"cweName", "Move Down"}
                },
                {
                    {"cveID", "left"},
                    {"cvss", 10},
                    {"cweName", "Move Left"}
                },
                {
                    {"cveID", "right"},
                    {"cvss", 10},
                    {"cweName", "Move Right"}
                }
            };
    
    int width = 4;
    int height = 3;
    json &nodes = j["nodes"];// = j["systemVulnerabilities"];
    for (json::iterator n = nodes.begin(); n != nodes.end(); n++) {
        //PLOGI << " ID: " << (*it)["cveID"];
        //json &links = (*n)["links"];
        json links;
        int id = (*n)["id"];
        if (id >= width) {
            // Below top row
            links.push_back(
                {
                    {"otherNodeID", id - width},
                    {"vulnerabilities",
                        {
                            {
                                {"cveID", "up"},
                                {"state", 0}
                            }
                        }
                    }
                });
        }
        if (id / width < height - 1) {
            // Above bottom row
            links.push_back(
                {
                    {"otherNodeID", id + width},
                    {"vulnerabilities",
                        {
                            {
                                {"cveID", "down"},
                                {"state", 0}
                            }
                        }
                    }
                });
        }
        if (id % width > 0) {
            // To the right of left most column
            links.push_back(
                {
                    {"otherNodeID", id - 1},
                    {"vulnerabilities",
                        {
                            {
                                {"cveID", "left"},
                                {"state", 0}
                            }
                        }
                    }
                });
        }
        if (id % width < width - 1) {
            // To the right of left most column
            links.push_back(
                {
                    {"otherNodeID", id + 1},
                    {"vulnerabilities",
                        {
                            {
                                {"cveID", "right"},
                                {"state", 0}
                            }
                        }
                    }
                });
        }
        (*n)["links"] = links;
    }
    
    std::ofstream o("../MDPExample1.json");
    o << j.dump(4);
    exit(0);
    */
    
    /*
    srand(time(0));
    std::ifstream i("../testNetwork.json");
    json j;
    i >> j;
    json sysVul = j["systemVulnerabilities"];
    
    json &nodes = j["nodes"];// = j["systemVulnerabilities"];
    for (json::iterator n = nodes.begin(); n != nodes.end(); n++) {
        //PLOGI << " ID: " << (*it)["cveID"];
        json &links = (*n)["links"];
        for (json::iterator l = links.begin(); l != links.end(); l++) {
            json vul;
            int numVul = rand() % sysVul.size() + 1;
            std::set<int> indices;
            while (indices.size() != numVul) {
                int i = rand() % sysVul.size();
                if (indices.insert(i).second == true) {
                    json vulState = 
                            {
                                {"cveID", sysVul[i]["cveID"]},
                                {"state", rand() % 3}
                            };
                    vul.emplace_back(vulState);
                }
            }
            (*l)["vulnerabilities"] = vul;;
        }
    }
    
    std::ofstream o("../testNetwork1.json");
    o << j.dump(4);
    exit(0);
    
    */
    
    
    
    
    ////////////////////////////////
    /*
    using json = nlohmann::json;
    ifstream i("../gridNetwork.json");
    json j;
    i >> j;
    
    j["systemVulnerabilities"] =
            {
                {
                    {"cveID", "test"},
                    {"cweName", "overflow"},
                    {"cvss", 2}
                }
            };
    
    json &nodes = j["nodes"];
    map<int, set<int>> nodeLinks;
    for (json::iterator it = nodes.begin(); it != nodes.end(); it++) {
        json &links = (*it)["links"];
        for (json::iterator itt = links.begin(); itt != links.end(); itt++) {
            json o;
            o["otherNodeID"] = *itt;
            json vul =
                    {
                        {"cveID", "test"},
                        {"state", "vulnerable"}
                    };
            o["vulnerabilities"].emplace_back(vul);
            *itt = o;
        }
        //(*it)["id"] << std::endl;
    }
    
    std::ofstream out("../fixedGridNetwork.json");
    out << j.dump(4);
    out.close();
    */
    /*
    for (std::map<int, std::set<int>>::iterator it = nodeLinks.begin();
            it != nodeLinks.end(); it++) {
        std::stringstream ss;
        ss << "id: " << it->first << ", links: [";
        for (std::set<int>::iterator iit = it->second.begin();
                iit != it->second.end(); iit++) {
            ss << *iit << ", ";
        }
        PLOGD << ss.str() << "]";
        //(*it)["id"] << std::endl;
    }
    
    PLOGD << "SIZE: " << nodeLinks.size();
    
    NetworkTopology *test = new NetworkTopology(nodeLinks, nodeWidth,
            windowSize, layout);
    //NetworkTopology *test = new NetworkTopology(numNodes, nodeWidth, windowSize);
    for (std::list<std::reference_wrapper<Link>>::iterator it = test->m_links.begin();
            it != test->m_links.end(); it++) {
        PLOGD << "Link: " << it->get();
    }
    return test;
    /////////////////////////////
    
    json toSave;
    for (std::set<std::reference_wrapper<Node>>::iterator it = m_nodes.begin();
            it != m_nodes.end(); it++) {
        Node &node = it->get();
        json nodeObj = json::object();
        nodeObj["id"] = node.id;
        
        json nodeLinks = json::array();
        for (std::list<std::reference_wrapper<Link>>::iterator iit = node.links.begin();
                iit != node.links.end(); iit++) {
            const Node &otherNode = iit->get().getOtherEnd(node);
            if (&otherNode != &Node::null_node) {
                nodeLinks.push_back(otherNode.id);
            }
        }
        nodeObj["links"] = nodeLinks;
        
        toSave.push_back(nodeObj);
    }
    std::ofstream out(fileName);
    out << toSave.dump(4);
    out.close();
    
    */
    int nrAgents = 1;
    
    sf::RenderWindow window(sf::VideoMode(width, height), "MDP Policy");
    sf::Vector2f windowSize(window.getSize());
    

    TopologyWrapper mdpWindow(windowSize, config.networkFile, std::vector<std::string>({"patch"}));

    int startNodeId = 9;
    int goalNodeId = 3;
    NetworkSimulator::getInstance().m_startNode = startNodeId;
    NetworkSimulator::getInstance().m_goalNode = goalNodeId;
    NetworkSimulator::getInstance().m_top = &mdpWindow;

    PathNetwork attacker = PathNetwork(startNodeId, goalNodeId, windowSize);
    NetworkSimulator::getInstance().m_attacker = &attacker;

    SimulationUnit *defender;
    switch (config.defenderPolicy) {
        case DefenderPolicy::attack:
            defender = new DefenderAttackerPriority();
            break;
        case DefenderPolicy::goal:
        default:
            defender = new DefenderGoalPriority();
    }
    NetworkSimulator::getInstance().m_defender = defender;

    NetworkSimulator::getInstance().reset();



    mdpWindow.m_netWindow.registerButton(
            "Step", windowSize, &takeStep, sf::Color(200, 150, 50), windowSize - windowSize / 20.0f);

    mdpWindow.m_netWindow.registerButton(
            "Run", windowSize, &runSimulation, sf::Color(150, 0, 150), { windowSize.x / 20.0f, windowSize.y - windowSize.y / 20.0f });

    
    
    
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
    /*
    //cout << p->GetRewardModelPtr()->SoftPrint() << endl;
    for(Index sI = 0; sI < path.GetNrStates(); sI++){
        for(Index jaI = 0; jaI < path.GetNrJointActions(); jaI++){
            PLOGI << "sI " << sI << " jaI " << jaI << " r " << path.GetReward(sI, jaI);
        }
    }
    */
    
    
    
    /*
    for (Index i = 0; i < nrS; i++) {
        sizePair pos = p->GetPosOfIndex(i);
        bool isBadPos = false;
        for(vector<sizePair>::const_iterator it = badPos.begin();
                it < badPos.end(); it++) {
            if (pos == *it) {
                isBadPos = true;
                break;
            }
        }
        bool isObs = false;
        for(vector<sizePair>::const_iterator it = obsPos.begin();
                it < obsPos.end(); it++) {
            if (pos == *it) {
                isObs = true;
                break;
            }
        }
        sf::RectangleShape shape(sf::Vector2f(squareWidth, squareWidth));
        
        if (isBadPos) {
            shape.setFillColor(sf::Color::Red);
        } else if (isObs) {
            shape.setFillColor(sf::Color::Black);
        } else if (pos == goalPos) {
            shape.setFillColor(sf::Color::Green);
        } else {
            shape.setFillColor(sf::Color::White);
        }
        shape.setOutlineColor(sf::Color::Black);
        shape.setOutlineThickness(-3);
        
        shape.setPosition(squareWidth * pos.first, squareWidth * pos.second);
        
        
        sf::RectangleShape arrowLine(sf::Vector2f(0.7f * squareWidth, 6.0f));
        arrowLine.setFillColor(sf::Color::Blue);
        arrowLine.setOrigin(arrowLine.getSize().x / 2,
                arrowLine.getSize().y / 2);
        
        float r = 1.8f * arrowLine.getSize().y;
        sf::CircleShape arrowHead(r, 3);
        arrowHead.setFillColor(sf::Color::Blue);
        arrowHead.setOrigin(r, r);
        
        arrowLine.setPosition(squareWidth * (pos.first + 0.5f),
                squareWidth * (pos.second + 0.5f));
        switch(policy[i]) {
            case 0: // Up
                arrowLine.rotate(90.0f);
                arrowHead.setPosition(
                        arrowLine.getPosition().x,
                        arrowLine.getPosition().y - arrowLine.getSize().x / 2);
                break;
            case 1: // Down
                arrowLine.rotate(90.0f);
                arrowHead.setPosition(
                        arrowLine.getPosition().x,
                        arrowLine.getPosition().y + arrowLine.getSize().x / 2);
                arrowHead.rotate(180.0f);
                break;
            case 2: // Left
                arrowHead.setPosition(
                        arrowLine.getPosition().x - arrowLine.getSize().x / 2,
                        arrowLine.getPosition().y);
                arrowHead.rotate(270.0f);
                break;
            case 3: // Right
                arrowHead.setPosition(
                        arrowLine.getPosition().x + arrowLine.getSize().x / 2,
                        arrowLine.getPosition().y);
                arrowHead.rotate(90.0f);
                break;
        }
        
        rectShapes.push_back(shape);
        rectShapes.push_back(arrowLine);
        circShapes.push_back(arrowHead);
    }
    */
    
    //std::pair<std::vector<int>, JointPolicyDiscrete> mpolicy = path.GetPolicy();
    /*
    for (std::vector<std::pair<std::string, std::pair<int, int>>>::iterator it = attackerPath.begin();
            it != attackerPath.end(); ++it) {
        mdpWindow.m_netWindow.m_nettop->setLinkColor(it->second.first, it->second.second, sf::Color::Yellow);
    }
    path.ShowPolicy();*/
    
    srand(time(0));
    /*
    std::list<std::pair<std::string, std::pair<int, int>>> attackerPath = path.GetPath();

    int currNode = startNodeId;
    int simStep = 0;
    while (currNode != goalNodeId) {
        currNode = path.TakeAction(currNode, simStep, attackerPath);
    }*/
  


    
    //NetworkWindow networkWindow(20, squareWidth, sf::Vector2f(window.getSize()));
    //TopologyWrapper mdpWindow(sf::Vector2f(window.getSize()), "../MDPExample1.json");
    
    //window.setPosition(sf::Vector2i(50, 50));
    
    static const microseconds step(1000000 / 60);
    system_clock::time_point oldTime = system_clock::now();

    int ticks = 0;
    
    int nodeId = 5;
    //networkWindow.setNodeActive(nodeId, true);
    while (window.isOpen())
    {
        sf::Vector2f windowSize(window.getSize());
        if (duration_cast<microseconds>(system_clock::now() - oldTime).count() >= step.count()) {
            ticks++;
            //std::cout << "Ticks: " << ticks << std::endl;
            //std::cout << "tick" << std::endl;
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
            /*if (ticks % 120 == 0) {
                networkWindow.setNodeActive(nodeId, false);
                if (++nodeId == 15)
                    nodeId = 5;
                networkWindow.setNodeActive(nodeId, true);
            }*/
            //window.clear();
            //networkWindow.render(window, windowSize);
            //window.display();
            
            oldTime = system_clock::now();
        }
        
        window.clear();
        mdpWindow.render(window, windowSize);
        window.display();
    }
    
    /*
    vector<sizePair> startPos = vector<sizePair>();
    //startPos.push_back(make_pair(0, 0));
    startPos.push_back(make_pair(1, 2));
    size_t nrAgents = startPos.size();
    
    sizePair gridSize = make_pair(4,3);
    
    vector<sizePair> obsPos = vector<sizePair>();
    obsPos.push_back(make_pair(1, 1));
    
    sizePair goalPos = make_pair(3,0);
    double goalReward = 100;
    
    vector<sizePair> badPos = vector<sizePair>();
    badPos.push_back(make_pair(3, 1));
    // Goes around
    //double badPosReward = -200;
    // Goes through
    double badPosReward = -100;
    
    /
    vector<sizePair> startPos = vector<sizePair>();
    //startPos.push_back(make_pair(0, 0));
    startPos.push_back(make_pair(1, 8));
    size_t nrAgents = startPos.size();
    
    sizePair gridSize = make_pair(10,10);
    
    vector<sizePair> obsPos = vector<sizePair>();
    obsPos.push_back(make_pair(1, 4));
    obsPos.push_back(make_pair(2, 7));
    obsPos.push_back(make_pair(4, 3));
    obsPos.push_back(make_pair(7, 6));
    
    sizePair goalPos = make_pair(6,2);
    double goalReward = 100;
    
    vector<sizePair> badPos = vector<sizePair>();
    badPos.push_back(make_pair(1, 2));
    badPos.push_back(make_pair(3, 1));
    badPos.push_back(make_pair(3, 4));
    badPos.push_back(make_pair(4, 6));
    badPos.push_back(make_pair(5, 2));
    badPos.push_back(make_pair(6, 0));
    badPos.push_back(make_pair(6, 4));
    badPos.push_back(make_pair(6, 8));
    badPos.push_back(make_pair(7, 5));
    badPos.push_back(make_pair(8, 2));
    badPos.push_back(make_pair(9, 8));
    
    double badPosReward = -100;
    *
    //DecPOMDPDiscreteInterface* p = new ProblemTestGrid(startPos, gridSize,
    //        obsPos, goalPos, goalReward, badPos, badPosReward);
    ProblemTestGrid* p = new ProblemTestGrid(startPos, gridSize,
            obsPos, goalPos, goalReward, badPos, badPosReward);
    
    
    vector<size_t> nrActions = p->GetNrActions();
    cout << "Size " << nrActions << "\n";
    for (Index agentI = 0; agentI < nrAgents; agentI++) {
        for (Index actionI = 0; actionI < nrActions.at(agentI); actionI++) {
            const Action* a = p->GetAction(agentI, actionI);
            cout << "Agent " << agentI << ": Action " << actionI << ": " <<
                    a->GetName() << ", " << a->GetDescription() << "\n";
        }
    }
    cout << "Transitions:" << endl;
    cout << p->GetTransitionModelDiscretePtr()->SoftPrint() << endl;
    
    //cout << "Observations:" << endl;
    printf("Observations\n");
    //cout << p->GetObservationModelDiscretePtr()->SoftPrint() << endl;
    
    //cout << "Rewards:" << endl;
    //cout << p->GetRewardModelPtr()->SoftPrint() << endl;
    for(Index sI = 0; sI < p->GetNrStates(); sI++){
        for(Index jaI = 0; jaI < p->GetNrJointActions(); jaI++){
            cout << "sI " << sI << " jaI " << jaI << " r " << p->GetReward(sI, jaI) << endl;
        }
    }
    
    int horizon = MAXHORIZON;
    printf("Making NullPlanner\n");
    PlanningUnitDecPOMDPDiscrete *np = new NullPlanner(horizon, p);
    printf("Making PolicyIteration\n");
    MDPPolicyIteration *mdpSolver = new MDPPolicyIteration(*np);
    
    Timing Time;
    //start planning
    Time.Start("Plan");
    mdpSolver->Plan(); // calls PlanSlow() on MDPPolicyIteration and MDPPolicyIterationGPU objects

    Time.Stop("Plan");
    cout << "...done."<<endl;
    
    int squareWidth = 100;
    int width = gridSize.first;
    int height = gridSize.second;
    
    sf::RenderWindow window(sf::VideoMode(squareWidth * width,
            squareWidth * height), "MDP Policy");
    
    vector<sf::RectangleShape> rectShapes = vector<sf::RectangleShape>();
    vector<sf::CircleShape> circShapes = vector<sf::CircleShape>();
    vector<int> policy = mdpSolver->GetPolicy();
    size_t nrS = p->GetNrStates();
    for (Index i = 0; i < nrS; i++) {
        sizePair pos = p->GetPosOfIndex(i);
        bool isBadPos = false;
        for(vector<sizePair>::const_iterator it = badPos.begin();
                it < badPos.end(); it++) {
            if (pos == *it) {
                isBadPos = true;
                break;
            }
        }
        bool isObs = false;
        for(vector<sizePair>::const_iterator it = obsPos.begin();
                it < obsPos.end(); it++) {
            if (pos == *it) {
                isObs = true;
                break;
            }
        }
        sf::RectangleShape shape(sf::Vector2f(squareWidth, squareWidth));
        
        if (isBadPos) {
            shape.setFillColor(sf::Color::Red);
        } else if (isObs) {
            shape.setFillColor(sf::Color::Black);
        } else if (pos == goalPos) {
            shape.setFillColor(sf::Color::Green);
        } else {
            shape.setFillColor(sf::Color::White);
        }
        shape.setOutlineColor(sf::Color::Black);
        shape.setOutlineThickness(-3);
        
        shape.setPosition(squareWidth * pos.first, squareWidth * pos.second);
        
        
        sf::RectangleShape arrowLine(sf::Vector2f(0.7f * squareWidth, 6.0f));
        arrowLine.setFillColor(sf::Color::Blue);
        arrowLine.setOrigin(arrowLine.getSize().x / 2,
                arrowLine.getSize().y / 2);
        
        float r = 1.8f * arrowLine.getSize().y;
        sf::CircleShape arrowHead(r, 3);
        arrowHead.setFillColor(sf::Color::Blue);
        arrowHead.setOrigin(r, r);
        
        arrowLine.setPosition(squareWidth * (pos.first + 0.5f),
                squareWidth * (pos.second + 0.5f));
        switch(policy[i]) {
            case 0: // Up
                arrowLine.rotate(90.0f);
                arrowHead.setPosition(
                        arrowLine.getPosition().x,
                        arrowLine.getPosition().y - arrowLine.getSize().x / 2);
                break;
            case 1: // Down
                arrowLine.rotate(90.0f);
                arrowHead.setPosition(
                        arrowLine.getPosition().x,
                        arrowLine.getPosition().y + arrowLine.getSize().x / 2);
                arrowHead.rotate(180.0f);
                break;
            case 2: // Left
                arrowHead.setPosition(
                        arrowLine.getPosition().x - arrowLine.getSize().x / 2,
                        arrowLine.getPosition().y);
                arrowHead.rotate(270.0f);
                break;
            case 3: // Right
                arrowHead.setPosition(
                        arrowLine.getPosition().x + arrowLine.getSize().x / 2,
                        arrowLine.getPosition().y);
                arrowHead.rotate(90.0f);
                break;
        }
        
        rectShapes.push_back(shape);
        rectShapes.push_back(arrowLine);
        circShapes.push_back(arrowHead);
    }
    
    sf::Shape Line = sf::Shape::Line(x1,y1,x2,y2,1,sf::Color::White);
	sf::Shape Triangle;
	float angle = 2 * PI/3;
	float angle_rect = PI/2;
	float d = 50;
	
	for (int i = 0; i < 3; i++) {
		Triangle.AddPoint(x2+d*sin(angle*i), y2+d*cos(angle*i), sf::Color::White);
	}
	App.Draw(Line);
	sf::Vector2f av =Triangle.GetPosition();
	App.Draw(Triangle);
    

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);
        
        for(vector<sf::RectangleShape>::const_iterator it = rectShapes.begin();
                it < rectShapes.end(); it++) {
            window.draw(*it);
        }
        for(vector<sf::CircleShape>::const_iterator it = circShapes.begin();
                it < circShapes.end(); it++) {
            window.draw(*it);
        }
        window.display();
    }*/
}

#endif