#ifndef _MDPGUICONFIG_H
#define _MDPGUICONFIG_H 1

#include "plog/Log.h"
#include "SFML/Graphics.hpp"

/*union ConfigProperty
{
    sf::Color color,
}*/

enum DefenderPolicy {
    goal = 0,
    attack = 1,
    exploitability = 2,
    impact = 3
};

class MDPGUIConfig
{
private:
    
public:
    sf::Color backgroundColor;
    sf::Color nodeColor;
    sf::Color linkColor;
    sf::Color linkNewPathColor;
    sf::Color linkOldPathColor;
    sf::Color attackerMoveColor;
    sf::Color defenderMoveColor;
    sf::Color goalColor;
    int width;
    int height;
    std::string networkFile;
    int startNodeId;
    int goalNodeId;
    DefenderPolicy defenderPolicy;
    plog::Severity logLevel;
    bool GUI;
    int numSimulations;
    
    MDPGUIConfig();
    
    static MDPGUIConfig LoadConfig();
    static MDPGUIConfig LoadConfig(const std::string &fileName);
};

#endif