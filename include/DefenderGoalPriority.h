#ifndef _DEFENDERGOALPRIORITY_H
#define _DEFENDERGOALPRIORITY_H

#include "SimulationUnit.h"

class DefenderGoalPriority : public SimulationUnit
{
private:
    //std::random_device m_random;
public:
    void reset();
    void TakeAction();
};







#endif