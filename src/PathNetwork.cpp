#include "PathNetwork.h"

#include <sstream>
#include <stdexcept>

//#include "NullPlanner.h"
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
        m_sim(nullptr),
        m_johI(INITIAL_JOHI)
{
    SetNrAgents(m_nrAgents);

    NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setNodeColor(m_startNode, sf::Color::Red);
    NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setNodeColor(m_goalNode, sf::Color::Green);
    
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

/* Actions for this example are simply the "vulnerabilities" of this system
 * moving up, down, left, right */
void PathNetwork::ConstructActions()
{
    int jointActionI = 0;
    Index agentIndex = 0;
    //for (Index agentIndex = 0; agentIndex < m_nrAgents; agentIndex++) {
//         _m_nrActions.push_back(nrActionsThisAgent);
//         _m_actionVecs.push_back( vector<ActionDiscrete>() );
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

        //             ActionDiscrete ad_temp =   ActionDiscrete(actionI,name,descr);
        //             _m_actionVecs[agentIndex].push_back( ad_temp );
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
    //}
}

void PathNetwork::CreateSimulator()
{
    if (m_bfs == nullptr) {
        PlanMDP();
    }
    int nrRuns = 1; //500;
    int seed = rand();
    m_sim = new SimulationDecPOMDPDiscrete(*m_bfs, nrRuns, seed);
}

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
        //m_bfs->Plan();
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

std::vector<int> PathNetwork::GetPolicy()
{
    if (m_bfs == nullptr) {
        PlanMDP();
    }
    return m_mdpSolver->GetPolicy();
}

boost::shared_ptr<JointPolicyDiscrete> PathNetwork::GetJointPolicy()
{
    if (m_bfs == nullptr) {
        PlanMDP();
    }
    return ((BruteForceSearchPlanner*)m_bfs)->GetJointPolicyDiscrete();
}

void PathNetwork::ShowPolicy()
{
    if (m_bfs == nullptr) {
        PlanMDP();
    }
    std::vector<int> mpolicy = GetPolicy();
    
    //size_t nrS = GetNrStates();
    
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
            //m_top->m_netWindow.m_nettop->setLinkColor(link.first, link.second, sf::Color::Yellow);
            /*bool foundVul = false;
            for (std::set<int>::iterator l = node.m_links.begin();
                    l != node.m_links.end(); ++l) {
                Link &link = m_links[std::make_pair(node.id, *l)];
                for (std::vector<Link::VulInstance>::iterator v = link.m_vulnerabilities.begin();
                        v != link.m_vulnerabilities.end(); ++v) {
                    if (v->m_vul.cveID == vul.cveID) {
                        foundVul = true;
                        break;
                    }
                }
                if (foundVul) {
                    m_netWindow.m_nettop->setLinkColor(node.id, *l, sf::Color::Yellow);
                    break;
                }
            }*/
            //PLOGI << "Node " << i;// << " exists: " << (m_nodes.find(i) != m_nodes.end());
            PLOGI << "Node " << node.id << " exploit " << vul.cveID << " to " << link.second;
        }
    }
}

std::list<std::pair<std::string, std::pair<int, int>>> PathNetwork::GetPath()
{
    if (m_bfs == nullptr) {
        PlanMDP();
    }
    //boost::shared_ptr<JointPolicyDiscrete> pol = ((BruteForceSearchPlanner*)m_bfs)->GetJointPolicyDiscrete();

    //PLOGI << "Prob of taking action 0 in state 0: " << pol->GetJointActionProb((Index)0, 0);
    //PLOGI << "Prob of taking action 5 in state 0: " << pol->GetJointActionProb((Index)0, 5);

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
        if (resultVal.size() > GetNrStates()) {
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

void PathNetwork::TakeAction()
{
    if (m_bfs == nullptr) {
        PlanMDP();
        for (std::list<std::pair<std::string, std::pair<int, int>>>::iterator it = 
                NetworkSimulator::getInstance().m_currPath.begin();
                it != NetworkSimulator::getInstance().m_currPath.end(); ++it) {
            TopologyWrapper::Link &link = NetworkSimulator::getInstance().m_top->getLinks().at(it->second);
            if (NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->getLinkColor(
                    link.m_endIds.first, link.m_endIds.second) == sf::Color::Yellow) {
                NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setLinkColor(
                        it->second.first, it->second.second, sf::Color::Blue);
            }
        }
        NetworkSimulator::getInstance().m_currPath = GetPath();
        if (NetworkSimulator::getInstance().m_currPath.empty()) {
            NetworkSimulator::getInstance().m_finished = true;
            return;
        }
    }
    if (m_sim == nullptr) {
        CreateSimulator();
    }
    std::list<std::pair<std::string, std::pair<int, int>>> &path = NetworkSimulator::getInstance().m_currPath;

    Index jaI,sI,joI;
    double r,sumR=0;
    if (NetworkSimulator::getInstance().m_step == 0) {
        sI = SampleInitialState();
        //m_johI = 1;
    } else {
        sI = m_nodeStateMapping[NetworkSimulator::getInstance().m_currNode];
    }

    if (path.front().second.first != NetworkSimulator::getInstance().m_currNode) {
        PLOGI << "Discrepency in current node: " << NetworkSimulator::getInstance().m_currNode << " and path: " << path.front().second.first;
        exit(1);
    }
    

    /*if(GetVerbose())
    cout << "Simulation::RunSimulation " << endl
            << "Simulation::RunSimulation set initial state to " 
            << sI << endl;*/
    //PLOGI << "Prob of taking action 0 in state 0: " << jpolicy->GetActionProb(0, 0, 0);
    //PLOGI << "Prob of taking action 5 in state 0: " << jpolicy->GetActionProb(0, 0, 5);
    //PLOGI << "Prob of taking action 0 in state 9: " << jpolicy->GetIndividualPolicyDiscrete(0)->GetActionProb(9, 0);
    //PLOGI << "Prob of taking action 0 in state 9: " << jpolicy->GetJointActionProb((Index)9, 0);
    //PLOGI << "JOINT POLICY:" << std::endl << jpolicy->SoftPrint();
    //PLOGI << "Num Domain elements: " << jpolicy->GetNrDomainElements(0);

    //jaI = jpolicy->SampleJointAction(sI);
    jaI = m_vulActionMapping[path.front()];


    PLOGI << "Before state: " << sI;
    sI = SampleSuccessorState(sI, jaI);
    PLOGI << "After state: " << sI;
    joI = SampleJointObservation(jaI, sI);

    // Set the already travelled path colour
    for (std::list<std::pair<SimAction, ActionResult>>::iterator it =
            NetworkSimulator::getInstance().m_attackerActions.begin();
            it != NetworkSimulator::getInstance().m_attackerActions.end(); ++it) {
        if (it->second == fail) {
            continue;
        }
        TopologyWrapper::Link &link = NetworkSimulator::getInstance().m_top->getLinks().at(it->first.second);
        NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setLinkColor(link.m_endIds.first,
                link.m_endIds.second, sf::Color(125, 0, 0));
        for (std::vector<TopologyWrapper::Link::VulInstance>::iterator itt = link.m_vulnerabilities.begin();
                itt != link.m_vulnerabilities.end(); ++itt) {
            if (itt->m_vul.cveID == it->first.first) {
                itt->m_highlighted = false; // Only want the current move highlighted
            }
        }
        NetworkSimulator::getInstance().m_top->updateLinkInfo(link);
    }

    SimAction action = path.front();
    if (sI == path.front().second.second) {
        // Made a move
        NetworkSimulator::getInstance().m_attackerActions.emplace_back(action, success);
        try {
            TopologyWrapper::Link &link = NetworkSimulator::getInstance().m_top->getLinks().at(action.second);
            NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setLinkColor(link.m_endIds.first,
                    link.m_endIds.second, sf::Color::Red);
            for (std::vector<TopologyWrapper::Link::VulInstance>::iterator it = link.m_vulnerabilities.begin();
                    it != link.m_vulnerabilities.end(); ++it) {
                if (it->m_vul.cveID == path.front().first) {
                    it->m_highlighted = true;
                    it->m_state = vulnerability_state::compromised;
                }
            }
            NetworkSimulator::getInstance().m_top->updateLinkInfo(link);
            NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setNodeColor(
                    action.second.first, sf::Color(150, 0, 0));
            NetworkSimulator::getInstance().m_top->m_netWindow.m_nettop->setNodeColor(
                    action.second.second, sf::Color::Red);
            PLOGI << "Attacker: Exploiting " << action.first << " on link " << action.second.first << "->" << action.second.second << ": SUCCESS";
            path.pop_front();
        } catch (const std::out_of_range &oor) {} // No element was found which should never happen
    } else {
        NetworkSimulator::getInstance().m_attackerActions.emplace_back(action, fail);
        PLOGI << "Defender: Exploiting " << action.first << " on link " << action.second.first << "->" << action.second.second << ": FAIL";
    }

    //m_sim->Step(jaI, step, sI, joI, r, sumR, 0);
    
    /* action taken at ts=0,...,hor-1 - therefore only observation
        * histories at ts=0,...,hor-2 have successors.*/
    NetworkSimulator::getInstance().m_step++;

    NetworkSimulator::getInstance().m_currNode = m_stateNodeMapping[sI];
}

std::string PathNetwork::SoftPrintBriefDescription(
        size_t nrAgents, size_t nrActions, size_t width, size_t height)
{
    std::stringstream ss;
    ss << "TestGrid_" << nrAgents << 
        "_" << nrActions <<
        "_" << width <<
        "x" << height;
    return ss.str();
}

std::string PathNetwork::SoftPrintDescription(
        size_t nrAgents, size_t nrActions, size_t width, size_t height)
{
    std::stringstream ss;
    ss << "The fully observable test grid problem with" << nrAgents << 
        " Agents, " << nrActions << " actions, " << width << " width and " 
        << height << " height";
    return ss.str();
}

void PathNetwork::FillTransitionModel()
{
    // CURRENTLY ONLY SUPPORTS ONE AGENT
    // NEED TO FLESH OUT THE PROBLEM TO MODEL MORE
    // add transitions:
    for (Index ja = 0; ja < GetNrJointActions(); ja++) {
        std::pair<int, int> link = m_actionVulMapping[ja].second;
        std::string cveID = m_actionVulMapping[ja].first;
        TopologyWrapper::Vulnerability *vul = nullptr;
        if (cveID != "do nothing") {
            vul = &NetworkSimulator::getInstance().m_top->getVulnerabilities()[cveID];
        }
        
        //const SimAction* act = GetAction(0, ja);
        for (Index s1 = 0; s1 < GetNrStates(); s1++) {
/*#if DEBUG_CTM
            cout << "Transitions from s1="<<s1<<endl;
#endif*/
            //vector< Index > s1_vec = GetStateVector(s1);
/* skip check for matlab generation compatibility            
            size_t nrSPs1 = NumberOfContainedStartPositions(s1_vec);
            if(nrSPs1 > 0 && nrSPs1 != GetNrAgents())
            {
                //illegal state, the prob. is zero except for transition
                //to self:
                SetTransitionProbability(s1, ja, s1, 1.0);
                //else 0, but we do not need to add that
                continue;
            }
*/
            if (cveID == "do nothing") {
                SetTransitionProbability(s1, ja, s1, 1.0);
                for (Index s2 = 0; s2 < GetNrStates(); s2++) {
                    if (s1 == s2) continue;
                    SetTransitionProbability(s1, ja, s2, 0.0);
                }
                continue;
            }

            //State* fromState = GetState(s1);
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
                    }
                    SetTransitionProbability(s1, ja, s2, p);
                    SetTransitionProbability(s1, ja, s1, 1.0 - p);
                }
            }

            /*
            bool found = false;
            for (std::set<int>::iterator l = node.m_links.begin();
                    l != node.m_links.end(); ++l) {
                Link &link = m_links[std::make_pair(node.id, *l)];
                for (std::vector<Link::VulInstance>::iterator v = link.m_vulnerabilities.begin();
                        v != link.m_vulnerabilities.end(); ++v) {
                    if (v->m_vul.cveID == vul.cveID) {
                        // Link contains the current action, so it's allowed
                        if (m_nodes[*l].id != 5) { // Only allow if legal state
                            SetTransitionProbability(s1, ja, m_nodeStateMapping[*l], 1.0);
                            found = true;
                        }
                        break;
                    }
                }
                if (found)
                    break;
            }
            if (!found) {
                // No links contained the action, not allowed
                SetTransitionProbability(s1, ja, s1, 1.0);
            }
            */
            /*
            if (node.m_links.find(vul))
            
            
            
            sizePair fromPos = GetPosOfIndex(s1);
            string actName = act->GetName();
            if (actName == actions[0]) {
                // Trying to move up
                if (fromPos.second == 0) {
                    // At the top, can't go up
                    SetTransitionProbability(s1, ja, s1, 1.0);
                    continue;
                }
            } else if (actName == actions[1]) {
                // Trying to move down
                if (fromPos.second == _m_height - 1) {
                    // At the bottom, can't go down
                    SetTransitionProbability(s1, ja, s1, 1.0);
                    continue;
                }
            } else if (actName == actions[2]) {
                // Trying to move left
                if (fromPos.first == 0) {
                    // At the leftmost edge, can't go left
                    SetTransitionProbability(s1, ja, s1, 1.0);
                    continue;
                }
            } else if (actName == actions[3]) {
                // Trying to move right
                if (fromPos.first == _m_width - 1) {
                    // At the rightmost edge, can't go right
                    SetTransitionProbability(s1, ja, s1, 1.0);
                    continue;
                }
            }
            //Since movements are deterministic, we can simply fill out
            //the position elements of the state factor
            //therefore we only need to loop over joint-firelevel vectors 
            //here
            for(Index s2=0; s2<GetNrStates();s2++) 
            {
                //vector< Index > s2_vec = IndexTools::JointToIndividualIndices
                //    (s2, _m_nrFLs_vec);
                //no longer necessary since we fill in the position components
                //if(NumberOfContainedStartPositions(s2_vec) > 0)
                    //continue;
                /*
                vector< Index > s1_vec_stripped(&s1_vec[0], 
                        &s1_vec[_m_nrHouses] );
                vector< Index > ja_vec = JointToIndividualActionIndices(ja);
                double p = ComputeTransitionProb(s1_vec_stripped, 
                        ja_vec, s2_vec);
#if DEBUG_CTM
                cout << "Trans from s="
                    << SoftPrintVector(s1_vec) 
                    << ", a=" 
                    << SoftPrintVector(ja_vec) 
                    << " to s'="
                    << SoftPrintVector(s2_vec) 
                    << " Prob=" << p << endl;
#endif
                *
                //State* toState = GetState(s2);
                sizePair toPos = GetPosOfIndex(s2);
                
                bool obs = false;
                for(vector<sizePair>::const_iterator it = _m_obsPos.begin();
                        it < _m_obsPos.end(); it++) {
                    if (toPos == *it) {
                        // Can't move into an obstacle
                        obs = true;
                        break;
                    }
                }
                if (actName == actions[0]) {
                    // Trying to move up
                    if (toPos.first == fromPos.first &&
                            toPos.second == fromPos.second - 1) {
                        if (obs) {
                            SetTransitionProbability(s1, ja, s1, 1.0);
                        } else {
                            SetTransitionProbability(s1, ja, s2, 1.0);
                        }
                    }
                } else if (actName == actions[1]) {
                    // Trying to move down
                    if (toPos.first == fromPos.first &&
                            toPos.second == fromPos.second + 1) {
                        if (obs) {
                            SetTransitionProbability(s1, ja, s1, 1.0);
                        } else {
                            SetTransitionProbability(s1, ja, s2, 1.0);
                        }
                    }
                } else if (actName == actions[2]) {
                    // Trying to move left
                    if (toPos.second == fromPos.second &&
                            toPos.first == fromPos.first - 1) {
                        if (obs) {
                            SetTransitionProbability(s1, ja, s1, 1.0);
                        } else {
                            SetTransitionProbability(s1, ja, s2, 1.0);
                        }
                    }
                } else if (actName == actions[3]) {
                    // Trying to move right
                    if (toPos.second == fromPos.second &&
                            toPos.first == fromPos.first + 1) {
                        if (obs) {
                            SetTransitionProbability(s1, ja, s1, 1.0);
                        } else {
                            SetTransitionProbability(s1, ja, s2, 1.0);
                        }
                    }
                }
                
                /*
                if(p > 0.0)
                {
                    //compute full s2 index
                    vector< Index >& full_s2 = s2_vec;
                    full_s2.insert(full_s2.end(), ja_vec.begin(), ja_vec.end());
                    Index fs2I = IndexTools::IndividualToJointIndices(full_s2, 
                            _m_nrPerStateFeatureVec);
                    SetTransitionProbability(s1, ja, fs2I, p);
                }
            }
            */
        }
    }
}


void PathNetwork::ConstructObservations()
{
    /// add observations:
    Index agentIndex=0;
    //for(Index agentIndex=0; agentIndex < m_nrAgents; agentIndex++)
    //{
        // Observations are just which position the agent is at (individual)
        size_t nrObservationsThisAgent = GetNrStates();
//         _m_nrObservations.push_back(nrObservationsThisAgent);
//         _m_observationVecs.push_back( vector<ObservationDiscrete>() );
        for(Index obsI=0; obsI < nrObservationsThisAgent; obsI++)
        {
            std::stringstream ss;
            //ss << "Ag" <<agentIndex << ":" << whatObs;
            ss << "state" << obsI;
            std::string name = ss.str();
            ss.str("");
            ss << "Observation " << obsI << " of agent " << agentIndex << ": " << name;
            std::string descr = ss.str();
//             ObservationDiscrete od_temp = ObservationDiscrete(obsI,name,descr);
//             _m_observationVecs[agentIndex].push_back( od_temp );
            AddObservation(agentIndex,name,descr);
        }
    //}
}

void PathNetwork::FillObservationModel()
{
    for (Index o1 = 0; o1 < GetNrStates(); o1++) {
        for(Index ja = 0; ja < GetNrJointActions(); ja++) {
            for(Index s1 = 0; s1 < GetNrStates(); s1++) {
                double prob = GetTransitionProbability(s1, ja, o1);
                SetObservationProbability(ja, s1, o1, prob);
                /*for(Index jo=0; jo<GetNrJointObservations();jo++) 
                {
                    vector< Index > ja_vec = JointToIndividualActionIndices(ja);
                    vector< Index > s1_vec = GetStateVector(s1);
                    vector< Index > jo_vec = 
                        JointToIndividualObservationIndices(jo);

                    double prob = 1.0;//ComputeObservationProb(ja_vec, s1_vec, jo_vec);
                    
                    SetObservationProbability(ja, s1, jo, prob);
                }*/
            }
        }
    }
}


void PathNetwork::FillRewardModel()
{
    for (Index ja = 0; ja < GetNrJointActions(); ja++) {
        std::pair<int, int> link = m_actionVulMapping[ja].second;
        std::string cveID = m_actionVulMapping[ja].first;
        TopologyWrapper::Link &actionLink = NetworkSimulator::getInstance().m_top->getLinks()[link];
        TopologyWrapper::Link::VulInstance *vul = nullptr;
        if (cveID != "do nothing") {
            for (std::vector<TopologyWrapper::Link::VulInstance>::iterator vi = actionLink.m_vulnerabilities.begin();
                    vi != actionLink.m_vulnerabilities.end(); ++vi) {
                if (vi->m_vul.cveID == cveID) {
                    vul = &(*vi);
                    break;
                }
            }
        }
        //TopologyWrapper::Vulnerability &vul = m_top->getVulnerabilities()[m_actionVulMapping[ja].first];
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
                SetReward(s1, ja, -10000.0);
                continue;
            }
            for (Index s2 = 0; s2 < GetNrStates(); ++s2) {
                TopologyWrapper::Node &node2 = NetworkSimulator::getInstance().m_top->getNodes()[m_stateNodeMapping[s2]];
                double p;
                if (vul->m_vul.impact == -1.0) {
                    p = 0.5;
                } else {
                    p = vul->m_vul.impact / MAX_IMPACT_SCORE;
                }
                if (node2.id == m_goalNode && node1.id != m_goalNode) {
                    SetReward(s1, ja, s2, 1000.0 * p);
                } else {
                    SetReward(s1, ja, s2, -100.0 * (1.0 - p));
                }
            }

            

            /*
            if (link.first == node.id && node.id != m_goalNode) {
                if (link.second == m_goalNode) {
                    SetReward(s1, ja, 1000);
                    SetReward()
                } else {
                    SetReward(s1, ja, -20);
                }
            } else {
                SetReward(s1, ja, -10000);
            }
            bool foundVul = false;
            double r = -5;
            for (std::set<int>::iterator l = node.m_links.begin();
                    l != node.m_links.end(); ++l) {
                int otherId = *l;
                Link &link = m_links[std::make_pair(node.id, otherId)];
                for (std::vector<Link::VulInstance>::iterator v = link.m_vulnerabilities.begin();
                        v != link.m_vulnerabilities.end(); ++v) {
                    if (vul.cveID == v->m_vul.cveID)
                        foundVul = true;
                    break;
                }
                if (foundVul) {
                    if (m_nodes[otherId].id == 3)
                        r = 100;
                    else if (m_nodes[otherId].id == 7)
                        r = -17;
                    break;
                }
            }
            SetReward(s1, ja, r);
            */
        }
        /*
        for (Index ja = 0; ja < GetNrJointActions(); ja++) {
            for (Index s2 = 0; s2 < GetNrStates(); s2++) {
                sizePair toPos = GetPosOfIndex(s2);
                double r;// = ComputeReward(s2);
                bool badPos = false;
                for(vector<sizePair>::const_iterator it = _m_badPos.begin();
                        it < _m_badPos.end(); it++) {
                    if (toPos == *it) {
                        // Bad rewards
                        badPos = true;
                        break;
                    }
                }
                if (badPos) {
                    r = _m_badPosReward;
                } else if (toPos == _m_goalPos) {
                    r = _m_goalReward;
                } else {
                    r = -5;
                }
                SetReward(s1, ja, s2, r);
            }
        }*/
    }
}


