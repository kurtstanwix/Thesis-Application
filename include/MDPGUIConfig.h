#ifndef _MDPGUICONFIG_H
#define _MDPGUICONFIG_H 1

#include "SFML/Graphics.hpp"

/*union ConfigProperty
{
    sf::Color color,
}*/

enum DefenderPolicy {
    goal = 0,
    attack = 1,
    exploiatbility = 2
};

class MDPGUIConfig
{
private:
    
public:
    sf::Color backgroundColor;
    sf::Color serverColor;
    sf::Color databaseColor;
    sf::Color clientColor;
    int width;
    int height;
    std::string networkFile;
    DefenderPolicy defenderPolicy;
    
    MDPGUIConfig();
    
    static MDPGUIConfig LoadConfig();
    static MDPGUIConfig LoadConfig(const std::string &fileName);
};

#endif