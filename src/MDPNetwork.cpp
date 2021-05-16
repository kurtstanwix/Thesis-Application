#include "MDPNetwork.h"


#include "NullPlanner.h"

#include "MDPPolicyIteration.h"
#include "JointPolicyPureVector.h"

#include <sstream>

MDPNetwork::MDPNetwork(const sf::Vector2f &windowSize,
        const MDPGUIConfig &config,
        const std::vector<std::string> &defenderActions, int nodeWidth)
    :
        TopologyWrapper(windowSize, config, defenderActions, nodeWidth),
        m_nrAgents(1),
        m_startNode(8)
{
    SetNrAgents(m_nrAgents);
    
    // Number of states (one per node position in this case)
    // size_t nrS = m_nodes.size();
    PLOGI << "Number of states: " << m_nodes.size();
    
    
    Index stateI = 0;
    for (std::map<int, Node>::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it) {
        std::stringstream ss;
        ss << "S_node" << it->second.id;
        m_nodeStateMapping[it->second.id] = stateI;
        m_stateNodeMapping[stateI++] = it->second.id;
        this->AddState(ss.str());
    }
    
    
    // Initial State Distribution
    std::vector<double> isd(m_nodes.size(), 0.0);
    isd.at(m_startNode) = 1.0;
    StateDistributionVector *isdv = new StateDistributionVector(isd);
    this->SetISD(isdv);
    
    SetStatesInitialized(true);
    SetDiscount(0.1);
    
    // add actions:
    ConstructActions();
    // add joint actions
    size_t testNRJA = ConstructJointActions();
    
    SetActionsInitialized(true);
    
    // add the transition model
    CreateNewTransitionModel();
    FillTransitionModel();
    
    // add observation model
    //if(DEBUG_PFF) cout << ">>>Adding Observation model..."<<endl;
    //CreateNewObservationModel();
    //FillObservationModel();
    //MultiAgentDecisionProcessDiscrete::SetInitialized(true);

    // add rewards
    CreateNewRewardModel();
    FillRewardModel();
    
    
    
    
    
    
    
    
    
    // Move out of constructor
    
    int horizon = MAXHORIZON;
    PLOGI << "Making NullPlanner";
    PlanningUnitDecPOMDPDiscrete *np = new NullPlanner(horizon, this);
    /*JointPolicyPureVector p(np);
    p.SetIndex(0);
    
    PLOGI << "Get JOINT POLICY:";
    p.Print();
    PLOGI << "Making PolicyIteration";*/
    MDPPolicyIteration *mdpSolver = new MDPPolicyIteration(*np);
    
    Timing Time;
    //start planning
    Time.Start("Plan");
    mdpSolver->Plan(); // calls PlanSlow() on MDPPolicyIteration and MDPPolicyIterationGPU objects

    Time.Stop("Plan");
    PLOGI << "...done.";
    
    
    std::vector<int> mpolicy = mdpSolver->GetPolicy();
    //size_t nrS = GetNrStates();
    
    PLOGI << "POLICY:";
    int i = 0;
    for (std::vector<int>::iterator it = mpolicy.begin(); it != mpolicy.end();
            ++it) {
        Vulnerability &vul = m_vulnerabilities[m_actionVulMapping[*it]];
        Node &node = m_nodes[i];
        bool foundVul = false;
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
        }
        //PLOGI << "Node " << i;// << " exists: " << (m_nodes.find(i) != m_nodes.end());
        PLOGI << "Node " << m_nodes[i].id << " go " << m_vulnerabilities[m_actionVulMapping[*it]].cveID;
        i++;
    }
    PLOGI << "Get POLICY:";
    for (std::vector<int>::iterator it = mpolicy.begin(); it != mpolicy.end();
            ++it) {
        PLOGI << *it;
    }
    
    /*
    PLOGI << "Get JOINT POLICY:";
    PLOGI << np->GetJointPolicy()->SoftPrint();
    PLOGI << "Get JOINT POLICY PURE:";
    PLOGI << np->GetJointPolicyPureVector()->SoftPrint();
     * */
}

/* Actions for this example are simply the "vulnerabilities" of this system
 * moving up, down, left, right */
void MDPNetwork::ConstructActions()
{
    int jointActionI = 0;
    for (Index agentIndex = 0; agentIndex < m_nrAgents; agentIndex++) {
//         _m_nrActions.push_back(nrActionsThisAgent);
//         _m_actionVecs.push_back( vector<ActionDiscrete>() );
        int actionI = 0;
        for (std::map<std::string, Vulnerability>::iterator it = m_vulnerabilities.begin();
                it != m_vulnerabilities.end(); ++it) {
            std::stringstream ss;
            ss << "Action " << actionI++ << " of agent " << agentIndex 
                << ": move in the " << it->second.cweName << " direction";
            std::string descr = ss.str();
            m_vulActionMapping[it->second.cveID] = jointActionI;
            m_actionVulMapping[jointActionI++] = it->second.cveID;

//             ActionDiscrete ad_temp =   ActionDiscrete(actionI,name,descr);
//             _m_actionVecs[agentIndex].push_back( ad_temp );
            AddAction(agentIndex,it->second.cweName,descr);
        }
    }
}


std::string MDPNetwork::SoftPrintBriefDescription(
        size_t nrAgents, size_t nrActions, size_t width, size_t height)
{
    std::stringstream ss;
    ss << "TestGrid_" << nrAgents << 
        "_" << nrActions <<
        "_" << width <<
        "x" << height;
    return ss.str();
}

std::string MDPNetwork::SoftPrintDescription(
        size_t nrAgents, size_t nrActions, size_t width, size_t height)
{
    std::stringstream ss;
    ss << "The fully observable test grid problem with" << nrAgents << 
        " Agents, " << nrActions << " actions, " << width << " width and " 
        << height << " height";
    return ss.str();
}

void MDPNetwork::FillTransitionModel()
{
    // CURRENTLY ONLY SUPPORTS ONE AGENT
    // NEED TO FLESH OUT THE PROBLEM TO MODEL MORE
    // add transitions:
    for (Index ja = 0; ja < GetNrJointActions(); ja++) {
        Vulnerability &vul = m_vulnerabilities[m_actionVulMapping[ja]];
        const Action* act = GetAction(0, ja);
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
            //State* fromState = GetState(s1);
            Node &node = m_nodes[m_stateNodeMapping[s1]];
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



void MDPNetwork::FillRewardModel()
{
    for (Index s1 = 0; s1 < GetNrStates(); s1++) {
        Node &node = m_nodes[m_stateNodeMapping[s1]];
        for (Index ja = 0; ja < GetNrJointActions(); ja++) {
            Vulnerability &vul = m_vulnerabilities[m_actionVulMapping[ja]];
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
                        r = -13;
                    break;
                }
            }
            SetReward(s1, ja, r);
        }/*
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


