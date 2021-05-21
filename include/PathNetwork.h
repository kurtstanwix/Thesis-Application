#ifndef _PATHNETWORK_H
#define _PATHNETWORK_H

#include "TopologyWrapper.h"

/* the include directives */
#include "Globals.h"
#include "DecPOMDPDiscrete.h"
#include "IndexTools.h"
#include "JointPolicyDiscrete.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "MDPPolicyIteration.h"
#include "SimulationDecPOMDPDiscrete.h"

#include "SimulationUnit.h"

// Simulation unit to calculate the path an attacker will take and to simulate those actions
class PathNetwork : public DecPOMDPDiscrete, public SimulationUnit
{
private:
    int m_nrAgents;
    int m_startNode;
    int m_goalNode;
    Index m_johI;
    PlanningUnitDecPOMDPDiscrete *m_bfs;
    MDPPolicyIteration *m_mdpSolver;
    
    std::map<Index, int> m_stateNodeMapping;
    std::map<int, Index> m_nodeStateMapping;
    std::map<Index, std::pair<std::string, std::pair<int, int>>> m_actionVulMapping;
    std::map<std::pair<std::string, std::pair<int, int>>, Index> m_vulActionMapping;
    
    void ConstructActions();
    ///Fills the transition model with the  problem transitions.
    void FillTransitionModel();
    void ConstructObservations();
    ///Fills the observation model with the  problem obs. probs.
    void FillObservationModel();
    ///Fills the reward model with the  problem rewards.
    void FillRewardModel();
protected:
    static std::string SoftPrintBriefDescription(
        size_t nrAgents, size_t nrActions, size_t width, size_t height);
    static std::string SoftPrintDescription(
        size_t nrAgents, size_t nrActions, size_t width, size_t height);
    
public:
    
    PathNetwork(int startNodeId, int goalNodeId, const sf::Vector2f &windowSize);
    ~PathNetwork()
    {
        delete m_bfs;
        delete m_mdpSolver;
    }

    void reset();
    void PlanMDP();
    std::vector<int> GetPolicy();
    boost::shared_ptr<JointPolicyDiscrete>  GetJointPolicy();
    std::list<std::pair<std::string, std::pair<int, int>>> GetPath();
    void ShowPolicy();
    void TakeAction();

    void setStartNode(int nodeId);
};




#endif