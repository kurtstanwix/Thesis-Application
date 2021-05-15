#ifndef _DEFENDERATTACKERPRIORITY_H
#define _DEFENDERATTACKERPRIORITY_H

#include <random>

#include "SimulationUnit.h"

class DefenderAttackerPriority : public SimulationUnit
{
private:
    //std::random_device m_random;

public:
    void reset();
    void TakeAction();
};







#endif