#ifndef _SIMULATIONUNIT_H
#define _SIMULATIONUNIT_H

class SimulationUnit
{
public:
    virtual void reset() = 0;
    virtual void TakeAction() = 0;
};




#endif