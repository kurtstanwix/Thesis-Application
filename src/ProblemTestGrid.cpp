#include "ProblemTestGrid.h"

using namespace std;

#define DEBUG_CJA 1
#define DEBUG_CA 1
#define DEBUG_PFF 1
#define DEBUG_CO 1


string actions[] = {"UP", "DOWN", "LEFT", "RIGHT"};

//Default constructor
ProblemTestGrid::ProblemTestGrid(
        vector<sizePair> agentStartPos,
        sizePair gridSize, vector<sizePair> obstaclePos,
        sizePair goalPos, double goalReward,
        vector<sizePair> badPos, double badPosReward)
    :
        _m_nrAgents(agentStartPos.size())
        ,_m_nrActions(sizeof actions / sizeof actions[0])
        ,_m_width(gridSize.first)
        ,_m_height(gridSize.second)
        ,_m_goalPos(goalPos)
        ,_m_goalReward(goalReward)
        ,_m_badPos(badPos)
        ,_m_badPosReward(badPosReward)
        ,_m_obsPos(obstaclePos)
        ,DecPOMDPDiscrete(
                SoftPrintBriefDescription(_m_nrAgents, _m_nrActions, _m_width, _m_height),
                SoftPrintDescription(_m_nrAgents, _m_nrActions, _m_width, _m_height),
                SoftPrintBriefDescription(_m_nrAgents, _m_nrActions, _m_width, _m_height)
                )
{
    SetNrAgents(_m_nrAgents);
    
    
    //size_t nrS = pow(_m_nrStatesPerAgent, _m_nrAgents);
    vector<Index> agentStartStates = vector<Index>();
    _m_nrStatesPerAgent = vector<size_t>(_m_nrAgents, _m_width * _m_height);
    // Number of states
    size_t nrS = 1;
    for (Index i = 0; i < _m_nrAgents; i++) {
        nrS *= _m_nrStatesPerAgent.at(i);
        // Convert the start positions into their individual state numbers
        agentStartStates.push_back(GetIndexOfPos(agentStartPos.at(i)));
    }
    
    cout << "nrS " << nrS << endl;
    
    // Corresponds to the joint state where each agent is at the position
    // described by startPos
    Index startJointState = IndexTools::IndividualToJointIndices(
            agentStartStates, _m_nrStatesPerAgent);
    cout << "startJointState " << startJointState << endl;
    
    // Create the actual states
    for(Index sI = 0; sI < nrS; sI++)
    {
        vector<Index> s_vec = GetStateVector(sI);
        stringstream ss;
        ss << "S_";
        Index t1;
        cout << "sI " << sI;
        for(Index t1 = 0; t1 < s_vec.size(); t1++)
        {
            cout << ", " << "s_vec[" << t1 << "]:" << s_vec.at(t1);
            sizePair pos = GetPosOfIndex(s_vec.at(t1));
            ss << "a" << t1 << "x" << pos.first << "y" << pos.second;
        }
        cout << endl;
        this->AddState(ss.str());
        if(DEBUG_PFF)
        {
            cout << "added " << this->GetState(sI)->SoftPrintBrief()<<endl;
        }
    }
    // Initial State Distribution
    vector<double> isd(nrS, 0.0);
    isd.at(startJointState) = 1.0;
    StateDistributionVector *isdv=new StateDistributionVector(isd);
    this->SetISD(isdv);
    
    SetStatesInitialized(true);
    SetDiscount(0.1);
    
    // add actions:
    ConstructActions();
    if(DEBUG_CJA) cout << ">>>Creating joint actions and set..."<<endl;
    // add joint actions
    size_t testNRJA = ConstructJointActions();
    if(DEBUG_CA) cout << "testNRJA="<<testNRJA<<endl;    

    SetActionsInitialized(true);
    
    // add observations:
    //ConstructObservations();
    //size_t testNRJO = ConstructJointObservations();
    //if(DEBUG_CO) cout << "testNRJO="<<testNRJO<<endl;

    //SetObservationsInitialized(true);
    
    // add the transition model
    if(DEBUG_PFF) cout << ">>>Adding Transition model..."<<endl;
    CreateNewTransitionModel();
    FillTransitionModel();
    
    // add observation model
    if(DEBUG_PFF) cout << ">>>Adding Observation model..."<<endl;
    //CreateNewObservationModel();
    //FillObservationModel();
    //MultiAgentDecisionProcessDiscrete::SetInitialized(true);

    // add rewards
    CreateNewRewardModel();
    FillRewardModel();
    if(DEBUG_PFF)     cout << "Model created..."<<endl; 
    //DecPOMDPDiscrete::SetInitialized(true);
    
}


void ProblemTestGrid::ConstructActions()
{
    for(Index agentIndex=0; agentIndex < _m_nrAgents; agentIndex++)
    {
        size_t nrActionsThisAgent = _m_nrActions;
//         _m_nrActions.push_back(nrActionsThisAgent);
//         _m_actionVecs.push_back( vector<ActionDiscrete>() );
        for(Index actionI=0; actionI < nrActionsThisAgent; actionI++)
        {
            stringstream ss;
            //ss << "Ag" <<agentIndex << ":House" << actionI;
            ss << actions[actionI];
            string name = ss.str();
            ss.str("");
            ss << "Action " << actionI << " of agent " << agentIndex 
                << ": move in the " << actions[actionI] << " direction";
            string descr = ss.str();

//             ActionDiscrete ad_temp =   ActionDiscrete(actionI,name,descr);
//             _m_actionVecs[agentIndex].push_back( ad_temp );
            AddAction(agentIndex,name,descr);
        }
    }
}

void ProblemTestGrid::ConstructObservations()
{
    /// add observations:
    for(Index agentIndex=0; agentIndex < _m_nrAgents; agentIndex++)
    {
        // Observations are just which position the agent is at (individual)
        size_t nrObservationsThisAgent = _m_nrStatesPerAgent.at(agentIndex);
//         _m_nrObservations.push_back(nrObservationsThisAgent);
//         _m_observationVecs.push_back( vector<ObservationDiscrete>() );
        for(Index obsI=0; obsI < nrObservationsThisAgent; obsI++)
        {
            sizePair pos = GetPosOfIndex(obsI);
            stringstream ss;
            //ss << "Ag" <<agentIndex << ":" << whatObs;
            ss << "x" << pos.first << "y" << pos.second;
            string name = ss.str();
            ss.str("");
            ss << "Observation " << obsI << " of agent " << agentIndex << ": " << name;
            string descr = ss.str();
//             ObservationDiscrete od_temp = ObservationDiscrete(obsI,name,descr);
//             _m_observationVecs[agentIndex].push_back( od_temp );
            AddObservation(agentIndex,name,descr);
        }
    }
}


std::string ProblemTestGrid::SoftPrintBriefDescription(
        size_t nrAgents, size_t nrActions, size_t width, size_t height)
{
    stringstream ss;
    ss << "TestGrid_" << nrAgents << 
        "_" << nrActions <<
        "_" << width <<
        "x" << height;
    return ss.str();
}

std::string ProblemTestGrid::SoftPrintDescription(
        size_t nrAgents, size_t nrActions, size_t width, size_t height)
{
    stringstream ss;
    ss << "The fully observable test grid problem with" << nrAgents << 
        " Agents, " << nrActions << " actions, " << width << " width and " 
        << height << " height";
    return ss.str();
}

void ProblemTestGrid::FillTransitionModel()
{
    // CURRENTLY ONLY SUPPORTS ONE AGENT
    // NEED TO FLESH OUT THE PROBLEM TO MODEL MORE
    // add transitions:
    for(Index ja=0; ja<GetNrJointActions(); ja++)
    {
        const Action* act = GetAction(0, ja);
        for(Index s1=0; s1<GetNrStates();s1++) 
        {
#if DEBUG_CTM
            cout << "Transitions from s1="<<s1<<endl;
#endif
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
                */
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
                */
            }
        }
    }
}


void ProblemTestGrid::FillObservationModel()
{
    for(Index  ja=0; ja< GetNrJointActions(); ja++)
    {
        for(Index s1=0; s1<GetNrStates();s1++)
        {
            SetObservationProbability(ja, s1, s1, 1.0);
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

void ProblemTestGrid::FillRewardModel()
{    

    for(Index s1=0; s1<GetNrStates();s1++) 
    {
        for(Index ja=0; ja<GetNrJointActions(); ja++)
        {
            for(Index s2=0; s2<GetNrStates();s2++) 
            {
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
        }
    }
}
/*
double ProblemTestGrid::ComputeObservationProb(
        const std::vector< Index>& ja,
        const std::vector< Index>& s1,
        const std::vector< Index>& jo
                ) const
{
    double p_jo = 1.0;
    for(Index agI=0; agI < ja.size(); agI++)
    {
        double p_o_thisAgent = 0.0;
        Index hI, FL;
        if(_m_includePositions)
        {
            hI = s1.at(_m_nrHouses + agI);
            if(hI == _m_nrHouses) // agent at start position
                FL = 0;
            else
                FL =  s1.at(hI);
        }
        else
        {
            hI=ja.at(agI);    //the position (house) of agI
            FL = s1.at(hI);   //the firelevel at that house
        }
        //we compute P(FLAMES)
        double pFlames = 0.0;
        switch(FL)
        {
            case(0): //no fire
                pFlames = 0.2; // 0.2 prob. of incorrectly observing
                break;
            case(1):
                pFlames = 0.5;
                break;
            default:
                pFlames = 0.8;
        }
        double pNoFlames = 1.0-pFlames;                 
        observation_t obsAgI = (observation_t) jo.at(agI);
        switch(obsAgI)
        {
            case(FLAMES):
                p_o_thisAgent = pFlames;
                break;
            case(NOFLAMES):
                p_o_thisAgent = pNoFlames;
                break;
        }
        p_jo *= p_o_thisAgent;
    }
    return p_jo;
}
/*
double ProblemTestGrid::ComputeTransitionProb(
                const std::vector< Index>& s1,
                const std::vector< Index>& ja,
                const std::vector< Index>& s2
                ) const
{
    double p = 1.0;
    for(Index hI=0; hI < s1.size(); hI++)
    {
        Index curLevel = s1.at(hI);
        Index nextLevel = s2.at(hI);
        Index nrAgentsAtLocation = 0;
        for(Index aI=0; aI < ja.size(); aI++)
            if(ja.at(aI) == hI)
                nrAgentsAtLocation++;
#if 0 && DEBUG_CTM
        cout << "hI="<<hI<<" #agents="<<nrAgentsAtLocation << " ";
#endif
        //this is dependent on s1 right?:
        bool neighborIsBurning = isNeighborBurning(s1, hI);

        Index sameLevel = curLevel;
        Index higherLevel = min((size_t)sameLevel+1, _m_nrFireLevels-1);
        Index lowerLevel = (curLevel==0) ? 0 : (curLevel-1);
        double p2=0.0;// the prob. factor of ThisHouseFirelevel;
        switch(nrAgentsAtLocation)
        {
            case(0): 
            {
                //this is kind of strange: when a house is not burning, but
                //its neigbhor is, it will increase its FL with p=0.8
                //but when it is already burning (and its neighbor is not), it 
                //increase with p=0.6...

                //fire is likely to increase
                if(neighborIsBurning)
                {
                    if(nextLevel == sameLevel)
                        p2+=0.2;
                    if(nextLevel == higherLevel)
                        p2+=0.8;
                }
                else if (curLevel == 0) //fire won't get ignited
                {
                    if(0 == nextLevel)
                        p2=1.0;
                    else //not possible so we can quit...
                        p2=0.0;
                }
                else //normal burning house
                {
                    if(nextLevel == sameLevel)
                        p2+=0.6;
                    if(nextLevel == higherLevel)
                        p2+=0.4;
                }
                break;
            }
            case(1): 
            {
                //fire is likely to decrease
                if(neighborIsBurning)
                {
                    if(nextLevel == sameLevel)
                        p2+=0.4;
                    if(nextLevel == lowerLevel) 
                        p2+=0.6; //.6 prob of extuinguishing 1 fl
                }
                else if (curLevel == 0) //fire won't get ignited
                {
                    if(0 == nextLevel)
                        p2=1.0;
                    else //not possible so we can quit...
                        p2=0.0;
                }
                else //normal burning house
                {
                    if(nextLevel == sameLevel)
                        p2+=0.0;
                    if(nextLevel == lowerLevel)
                        p2+=1.0;
                }
                break;
            }
            default: 
            {
                //more than 1 agent: fire is extinguished
                if(0 == nextLevel)
                    p2=1.0;
                else //not possible so we can quit...
                    p2=0.0;
            }

            
        }
#if 0 && DEBUG_CTM
        cout << "p=" << p << ", p2=" << p2;
#endif
        p *= p2;
#if 0 && DEBUG_CTM
        cout << ", new p=" << p << " - ";
#endif

    }
#if DEBUG_CTM
    cout << "returning p=" << p << endl;
#endif
    return p;
}
*/