#ifndef _MDPNETWORK_H
#define _MDPNETWORK_H

#include "TopologyWrapper.h"

/* the include directives */
#include "Globals.h"
#include "DecPOMDPDiscrete.h"
#include "IndexTools.h"

class MDPNetwork : public TopologyWrapper, public DecPOMDPDiscrete
{
private:
    int m_nrAgents;
    int m_startNode;
    
    std::map<Index, int> m_stateNodeMapping;
    std::map<int, Index> m_nodeStateMapping;
    std::map<Index, std::string> m_actionVulMapping;
    std::map<std::string, Index> m_vulActionMapping;
    
    void ConstructActions();
    ///Fills the transition model with the  problem transitions.
    void FillTransitionModel();
    ///Fills the observation model with the  problem obs. probs.
    //void FillObservationModel();
    ///Fills the reward model with the  problem rewards.
    void FillRewardModel();
protected:
    static std::string SoftPrintBriefDescription(
        size_t nrAgents, size_t nrActions, size_t width, size_t height);
    static std::string SoftPrintDescription(
        size_t nrAgents, size_t nrActions, size_t width, size_t height);
    
public:
    
    MDPNetwork(const sf::Vector2f &windowSize,
            const std::string &fileName,
            const std::vector<std::string> &defenderActions,
            int nodeWidth = 100);
};



#endif