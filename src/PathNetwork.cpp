#include "PathNetwork.h"

#include <sstream>
#include <stdexcept>

#include "BruteForceSearchPlanner.h"
#include "DICEPSPlanner.h"
#include "JESPExhaustivePlanner.h"
#include "JESPDynamicProgrammingPlanner.h"

#include "Interface.h"
#include "NetworkSimulator.h"



PathNetwork::PathNetwork(int startNodeId, int goalNodeId, const sf::Vector2f &windowSize)
    :
        m_nrAgents(1),
        m_startNode(startNodeId),
        m_goalNode(goalNodeId),
        m_bfs(nullptr),
        m_mdpSolver(nullptr),
        m_johI(INITIAL_JOHI)
{
    SetNrAgents(m_nrAgents);
    
    // Number of states (one per node position in this case)
    PLOGI << "Number of states: " << NetworkSimulator::getInstance().m_top->getNodes().size();
    
    Index stateI = 0;
    for (std::map<int, TopologyWrapper::Node>::iterator it =
            NetworkSimulator::getInstance().m_top->getNodes().begin();
            it != NetworkSimulator::getInstance().m_top->getNodes().end(); ++it) {
        std::stringstream ss;
        ss << "S_node" << it->second.id;
        m_nodeStateMapping[it->second.id] = stateI;
        m_stateNodeMapping[stateI++] = it->second.id;
        this->AddState(ss.str());
    }

    // add actions:
    ConstructActions();
    // add joint actions
    ConstructJointActions();
    SetActionsInitialized(true);
    // add observations
    ConstructObservations();
    ConstructJointObservations();
    SetObservationsInitialized(true);
    
    reset();
    
}

// Actions for this model are exploit the vulnerabilities on each link
void PathNetwork::ConstructActions()
{
    int jointActionI = 0;
    Index agentIndex = 0;
    int actionI = 0;
    for (std::map<std::pair<int, int>, TopologyWrapper::Link>::iterator it =
            NetworkSimulator::getInstance().m_top->getLinks().begin();
            it != NetworkSimulator::getInstance().m_top->getLinks().end(); ++it) {
        for (std::vector<TopologyWrapper::Link::VulInstance>::iterator itt = it->second.m_vulnerabilities.begin();
                itt != it->second.m_vulnerabilities.end(); ++itt) {
            std::stringstream ss;
            ss << "SimAction " << actionI++ << " of agent " << agentIndex 
                << ": exploit vulnerability " << itt->m_vul.cweName << " on link ("
                << it->first.first << ", " << it->first.second << ")";
            std::string descr = ss.str();
            m_vulActionMapping[std::make_pair(itt->m_vul.cveID, it->first)] = jointActionI;
            m_actionVulMapping[jointActionI++] = std::make_pair(itt->m_vul.cveID, it->first);
            
            std::stringstream ssName;
            ssName << itt->m_vul.cveID << ", " << itt->m_vul.cweName << " (" << it->first.first << ", " << it->first.second << ")";
            AddAction(agentIndex,ssName.str(),descr);
        }
    }
    std::stringstream ss;
    ss << "SimAction " << actionI++ << " of agent " << agentIndex
            << ": do nothing";
    m_vulActionMapping[std::make_pair("do nothing", std::make_pair(-1, -1))] = jointActionI;
    m_actionVulMapping[jointActionI++] = std::make_pair("do nothing", std::make_pair(-1, -1));
    AddAction(agentIndex,ss.str(),ss.str());
}

// Solve the MDP using the given model
void PathNetwork::PlanMDP()
{
    if (m_mdpSolver != nullptr) {
        delete m_mdpSolver;
        m_mdpSolver = nullptr;
    }
    if (m_bfs != nullptr) {
        delete m_bfs;
        m_bfs = nullptr;
    }
    int horizon = MAXHORIZON;
    PLOGI << "Making NullPlanner";
    m_bfs = new BruteForceSearchPlanner(horizon, this);
    PLOGI << "Making PolicyIteration";
    m_mdpSolver = new MDPPolicyIteration(*m_bfs);
    
    Timing Time;
    //start planning
    Time.Start("Plan");
    PLOGI << "nrJointActions: " << m_bfs->GetNrJointActions();
    PLOGI << "nrStates: " << m_bfs->GetNrStates();
    PLOGI << "nrObservations: " << m_bfs->GetNrJointObservations();
    PLOGI << "prob state 0-0-1: " << m_bfs->GetTransitionProbability(0, 0, 1);
    PLOGI << "prob state 0-0-2: " << m_bfs->GetTransitionProbability(0, 0, 2);
    try{
        m_mdpSolver->Plan(); // calls PlanSlow() on MDPPolicyIteration and MDPPolicyIterationGPU objects
    } catch (const E &e) {
        PLOGI << "ERROR: " << e.SoftPrint();
    }


    Time.Stop("Plan");
    PLOGI << "...done.";
}


void PathNetwork::setStartNode(int nodeId)
{
    m_startNode = nodeId;
}

// Gets the policy from a solved MDP
std::vector<int> PathNetwork::GetPolicy()
{
    if (m_bfs == nullptr) {
        PlanMDP();
    }
    return m_mdpSolver->GetPolicy();
}

// Helper method to show the policy a plan made
void PathNetwork::ShowPolicy()
{
    if (m_bfs == nullptr) {
        PlanMDP();
    }
    std::vector<int> mpolicy = GetPolicy();

    PLOGI << "POLICY:";
    for (std::vector<int>::iterator it = mpolicy.begin(); it != mpolicy.end();
            ++it) {
        std::pair<int, int> link = m_actionVulMapping[*it].second;
        std::string cveID = m_actionVulMapping[*it].first;
        if (cveID == "do nothing") {
            PLOGI << "do nothing";
        } else {
            TopologyWrapper::Vulnerability &vul = NetworkSimulator::getInstance().m_top->getVulnerabilities()[cveID];
            TopologyWrapper::Node &node = NetworkSimulator::getInstance().m_top->getNodes()[link.first];
            PLOGI << "Node " << node.id << " exploit " << vul.cveID << " to " << link.second;
        }
    }
}

std::list<std::pair<std::string, std::pair<int, int>>> PathNetwork::GetPath()
{
    if (m_bfs == nullptr) {
        PlanMDP();
    }

    std::vector<int> mpolicy = GetPolicy();
    
    std::list<std::pair<std::string, std::pair<int, int>>> resultVal;
    int currNode = m_startNode;
    while (currNode != m_goalNode) {
        Index state = m_nodeStateMapping[currNode];
        Index action = mpolicy[state];
        std::pair<int, int> link = m_actionVulMapping[action].second;
        TopologyWrapper::Vulnerability &vul =
                NetworkSimulator::getInstance().m_top->getVulnerabilities()[m_actionVulMapping[action].first];
        
        resultVal.push_back(make_pair(vul.cveID, link));
        currNode = link.second;
        if (resultVal.size() > GetNrStates() || m_actionVulMapping[action].first == "do nothing") {
            resultVal.clear();
            break;
        }
    }

    PLOGI << "Path size: " << resultVal.size();
    std::stringstream ss;
    for (std::list<std::pair<std::string, std::pair<int, int>>>::iterator it = resultVal.begin();
            it != resultVal.end(); ++it) {
        ss << it->second.first << "->" << it->second.second;
    }
    PLOGI << ss.str();

    return resultVal;
}

// Reset the MDP model
void PathNetwork::reset()
{
    m_startNode = NetworkSimulator::getInstance().m_currNode;
    // Initial State Distribution
    std::vector<double> isd(NetworkSimulator::getInstance().m_top->getNodes().size(), 0.0);
    isd.at(m_startNode) = 1.0;
    StateDistributionVector *isdv = new StateDistributionVector(isd);
    this->SetISD(isdv);
    
    SetStatesInitialized(true);
    SetDiscount(0.9);
    
    // add the transition model
    CreateNewTransitionModel();
    FillTransitionModel();
    // add observation model
    CreateNewObservationModel();
    FillObservationModel();
    try{
        MultiAgentDecisionProcessDiscrete::SetInitialized(true);
    }catch(const E &e) {
        PLOGI << "ERROR " << e.SoftPrint();
    }
    // add rewards
    CreateNewRewardModel();
    FillRewardModel();
    
    try{
        DecPOMDPDiscrete::SetInitialized(true);
    }catch(const E &e) {
        PLOGI << "ERROR " << e.SoftPrint();
    }

    if (m_bfs != nullptr) {
        delete m_bfs;
        m_bfs = nullptr;
    }
}

// Solve the MDP model and take an action based on it
void PathNetwork::TakeAction()
{
    if (m_bfs == nullptr) {
        // No plan made, must do this first
        PlanMDP();
        NetworkSimulator::getInstance().m_currPath = GetPath();
        if (NetworkSimulator::getInstance().m_currPath.empty()) {
            // No path to goal, end of sim
            NetworkSimulator::getInstance().m_finished = true;
            return;
        }
    }
    std::list<SimAction> &path = NetworkSimulator::getInstance().m_currPath;

    Index jaI,sI,joI;
    double r,sumR=0;
    if (NetworkSimulator::getInstance().m_step == 0) {
        sI = SampleInitialState(); // Deterministic in this case
    } else {
        sI = m_nodeStateMapping[NetworkSimulator::getInstance().m_currNode];
    }

    if (path.front().second.first != NetworkSimulator::getInstance().m_currNode) {
        PLOGF << "Discrepency in current node: " << NetworkSimulator::getInstance().m_currNode << " and path: " << path.front().second.first;
        exit(1);
    }
    
    jaI = m_vulActionMapping[path.front()];

    // Calculate the next state and the given observation
    PLOGI << "Before state: " << sI;
    sI = SampleSuccessorState(sI, jaI);
    PLOGI << "After state: " << sI;
    joI = SampleJointObservation(jaI, sI);

    SimAction action = path.front();
    if (sI == path.front().second.second) {
        // Made a move
        NetworkSimulator::getInstance().m_attackerActions.emplace_back(action, success);
        PLOGI << "Attacker: Exploiting " << action.first << " on link " << action.second.first << "->" << action.second.second << ": SUCCESS";
        path.pop_front();
    } else {
        NetworkSimulator::getInstance().m_attackerActions.emplace_back(action, fail);
        PLOGI << "Defender: Exploiting " << action.first << " on link " << action.second.first << "->" << action.second.second << ": FAIL";
    }

    NetworkSimulator::getInstance().m_step++;

    NetworkSimulator::getInstance().m_currNode = m_stateNodeMapping[sI];
}

// Set the transition probabilities for each action in each state
void PathNetwork::FillTransitionModel()
{
    for (Index ja = 0; ja < GetNrJointActions(); ja++) {
        std::pair<int, int> link = m_actionVulMapping[ja].second;
        std::string cveID = m_actionVulMapping[ja].first;
        TopologyWrapper::Vulnerability *vul = nullptr;
        if (cveID != "do nothing") {
            vul = &NetworkSimulator::getInstance().m_top->getVulnerabilities()[cveID];
        }
        
        for (Index s1 = 0; s1 < GetNrStates(); s1++) {
            if (cveID == "do nothing") {
                // Guaranteed to end up in the same state
                SetTransitionProbability(s1, ja, s1, 1.0);
                for (Index s2 = 0; s2 < GetNrStates(); s2++) {
                    if (s1 == s2) continue;
                    SetTransitionProbability(s1, ja, s2, 0.0);
                }
                continue;
            }
            
            TopologyWrapper::Node &node = NetworkSimulator::getInstance().m_top->getNodes()[m_stateNodeMapping[s1]];
            if (node.id != link.first || node.id == m_goalNode) {
                // Can't take an action if the link isn't from this node.
                // As each action in each state needs a summed transition probability of 1, we make these
                // impossible actions remain in the same state and have the reward highly penalising.
                // Or we're in the goal state so make this a sink state effectively ending
                // the simulation once it reaches here
                SetTransitionProbability(s1, ja, s1, 1.0);
                for (Index s2 = 0; s2 < GetNrStates(); s2++) {
                    if (s1 == s2) continue;
                    SetTransitionProbability(s1, ja, s2, 0.0);
                }
                continue;
            }
            
            for (Index s2 = 0; s2 < GetNrStates(); s2++) {
                TopologyWrapper::Node &otherNode = NetworkSimulator::getInstance().m_top->getNodes()[m_stateNodeMapping[s2]];
                if (otherNode.id != link.second && otherNode.id != node.id) {
                    // Can't take an action from one node to another if the action is not moving over a link between the two
                    SetTransitionProbability(s1, ja, s2, 0.0);
                    continue;
                }
                TopologyWrapper::Link &actualLink = NetworkSimulator::getInstance().m_top->getLinks()[link];
                TopologyWrapper::Link::VulInstance *vulInst = nullptr;
                for (std::vector<TopologyWrapper::Link::VulInstance>::iterator it = actualLink.m_vulnerabilities.begin();
                        it != actualLink.m_vulnerabilities.end(); ++it) {
                    if (it->m_vul.cveID == vul->cveID) {
                        // Matched the vulnerability on the link to the action
                        vulInst = &(*it);
                    }
                }
                if (vulInst == nullptr) {
                    // Should never happen if actions were set up correctly
                    SetTransitionProbability(s1, ja, s2, 0.0);
                    continue;
                }
                if (vulInst->m_state == patched) {
                    // Can't take this action, so it ends up in same state and reward is highly penalised
                    SetTransitionProbability(s1, ja, s1, 1.0);
                } else if (otherNode.id != node.id) {
                    double p;
                    if (vulInst->m_vul.exploitability == -1.0) {
                        p = 0.5;
                    } else {
                        p = std::min(vulInst->m_vul.exploitability / MAX_EXPLOITABILITY_SCORE, 1.0);
                        // Scale probability to the range [0,0.85]
                        p -= 0.5f;
                        p /= 1.176471f;
                        p += 0.425f;
                    }
                    SetTransitionProbability(s1, ja, s2, p);
                    SetTransitionProbability(s1, ja, s1, 1.0 - p);
                }
            }
        }
    }
}

// Set up all the potential observations an agent can recieve at each state
void PathNetwork::ConstructObservations()
{
    Index agentIndex=0;
    // Observations are just which position the agent is at (individual)
    size_t nrObservationsThisAgent = GetNrStates();
    for(Index obsI=0; obsI < nrObservationsThisAgent; obsI++)
    {
        std::stringstream ss;;
        ss << "state" << obsI;
        std::string name = ss.str();
        ss.str("");
        ss << "Observation " << obsI << " of agent " << agentIndex << ": " << name;
        std::string descr = ss.str();
        AddObservation(agentIndex,name,descr);
    }
}

// Add the observation probabilities for an agent in each state if it takes a set action
void PathNetwork::FillObservationModel()
{
    for (Index o1 = 0; o1 < GetNrStates(); o1++) {
        for(Index ja = 0; ja < GetNrJointActions(); ja++) {
            for(Index s1 = 0; s1 < GetNrStates(); s1++) {
                double prob = GetTransitionProbability(s1, ja, o1);
                SetObservationProbability(ja, s1, o1, prob);
            }
        }
    }
}

// Set up the reward an agent receives for taking an action in a state and ending up in another state
void PathNetwork::FillRewardModel()
{
    for (Index ja = 0; ja < GetNrJointActions(); ja++) {
        std::pair<int, int> link = m_actionVulMapping[ja].second;
        std::string cveID = m_actionVulMapping[ja].first;
        TopologyWrapper::Link &actionLink = NetworkSimulator::getInstance().m_top->getLinks()[link];
        TopologyWrapper::Link::VulInstance *vul = nullptr;
        if (cveID != "do nothing") {
            // Find if the vulnerability exists on the link, if not, it will 
            for (std::vector<TopologyWrapper::Link::VulInstance>::iterator vi = actionLink.m_vulnerabilities.begin();
                    vi != actionLink.m_vulnerabilities.end(); ++vi) {
                if (vi->m_vul.cveID == cveID) {
                    vul = &(*vi);
                    break;
                }
            }
        }
        for (Index s1 = 0; s1 < GetNrStates(); ++s1) {
            TopologyWrapper::Node &node1 = NetworkSimulator::getInstance().m_top->getNodes()[m_stateNodeMapping[s1]];
            if (cveID == "do nothing") {
                if (node1.id == m_goalNode) {
                    SetReward(s1, ja, 0.0);
                } else {
                    SetReward(s1, ja, -100.0);
                }
                continue;
            }
            if (link.first != node1.id || vul->m_state == patched) {
                // Highly penalise actions that can't be taken, eg. moving over a link that doesn't
                // originate from the current node or moving over a patched vulnerability
                SetReward(s1, ja, -10000.0);
                continue;
            }
            for (Index s2 = 0; s2 < GetNrStates(); ++s2) {
                TopologyWrapper::Node &node2 = NetworkSimulator::getInstance().m_top->getNodes()[m_stateNodeMapping[s2]];
                double p;
                if (vul->m_vul.impact == -1.0) {
                    // Impact not given, default to 50%
                    p = 0.5;
                } else {
                    p = vul->m_vul.impact / MAX_IMPACT_SCORE;
                }
                // Reward the goal node more or punish non goal node moves less proportionally
                // based on how much impact the exploit has. 
                if (node2.id == m_goalNode && node1.id != m_goalNode) {
                    SetReward(s1, ja, s2, 1000.0 * p);
                } else {
                    SetReward(s1, ja, s2, -100.0 * (1.0 - p));
                }
            }
        }
    }
}


