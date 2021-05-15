
/* Only include this header file once. */
#ifndef _PROBLEMTESTGRID_H_
#define _PROBLEMTESTGRID_H_ 1

/* the include directives */
#include "Globals.h"
#include "DecPOMDPDiscrete.h"
#include "IndexTools.h"

typedef std::pair<size_t, size_t> sizePair;

class ProblemTestGrid : public DecPOMDPDiscrete
{
    private: 
        //enum observation_t { FLAMES, NOFLAMES };
        size_t _m_nrAgents;
        size_t _m_nrActions;
        size_t _m_width;
        size_t _m_height;
        
        //size_t _m_nrBadPos;
        sizePair _m_goalPos;
        double _m_goalReward;
        std::vector<sizePair> _m_badPos;
        double _m_badPosReward;
        std::vector<sizePair> _m_obsPos;
        //bool _m_includePositions;

        //size_t _m_nrStateFeatures;
        //vector that stores the number of values per state feature.
        std::vector<size_t> _m_nrStatesPerAgent;

        //size_t _m_nrJointFirelevels;
        //vector that stores the number of values per state feature.
        //std::vector<size_t> _m_nrFLs_vec;
        
        ///Construct all the Actions and actionSets (the vector _m_actionVecs).
        void ConstructActions();
        ///Construct all the observations and observation sets.
        void ConstructObservations();
        ///Fills the transition model with the  problem transitions.
        void FillTransitionModel();
        ///Fills the observation model with the  problem obs. probs.
        void FillObservationModel();
        ///Fills the reward model with the  problem rewards.
        void FillRewardModel();
    protected:
        static std::string SoftPrintBriefDescription(
            size_t nrAgents, size_t nrActions, size_t width, size_t height);
        static std::string SoftPrintDescription(
            size_t nrAgents, size_t nrActions, size_t width, size_t height);
        std::vector<Index> GetStateVector(Index sI) const
        {
            return IndexTools::JointToIndividualIndices
                (sI, _m_nrStatesPerAgent);
        }
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        ProblemTestGrid(
                std::vector<sizePair> agentStartPos,
                sizePair gridSize, std::vector<sizePair> obstaclePos,
                sizePair goalPos, double goalReward,
                std::vector<sizePair> badPos, double badPosReward);
        
        sizePair GetPosOfIndex(Index i) { return std::make_pair(i % _m_width, i / _m_width); }
        Index GetIndexOfPos(sizePair pos) { return pos.first + pos.second * _m_width; }
};

#endif