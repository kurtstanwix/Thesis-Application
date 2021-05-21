#ifndef _SIMULATIONUNIT_H
#define _SIMULATIONUNIT_H

class SimulationUnit;

#include "NetworkSimulator.h"

class SimulationUnit
{
protected:
    void TakeAction(const SimAction &action);
public:
    virtual void reset() = 0;
    virtual void TakeAction() = 0;
};




#endif