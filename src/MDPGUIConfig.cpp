#include "MDPGUIConfig.h"

#include <fstream>

#include "Utility.h"

#include <nlohmann/json.hpp>

#define DEFAULT_WIDTH 1200
#define DEFAULT_HEIGHT 1200

#define DEFAULT_BACKGROUND_COLOR sf::Color::Black
#define DEFAULT_NODE_COLOR sf::Color::Yellow
#define DEFAULT_LINK_COLOR sf::Color(160, 0, 255)
#define DEFAULT_LINK_NEW_PATH_COLOR sf::Color::White
#define DEFAULT_LINK_OLD_PATH_COLOR sf::Color(160, 160, 160)
#define DEFAULT_ATTACKER_MOVE_COLOR sf::Color::Red
#define DEFAULT_DEFENDER_MOVE_COLOR sf::Color::Green
#define DEFAULT_GOAL_COLOR sf::Color::Green

#define DEFAULT_NETWORK_FILE RESOURCE("MDPExample1.json")
#define DEFAULT_START_NODE_ID 9
#define DEFAULT_GOAL_NODE_ID 3

#define DEFAULT_DEFENDER_POLICY goal

#define DEFAULT_GUI true
#define DEFAULT_NUM_SIMULATIONS 1

#define DEFAULT_LOG_LEVEL plog::info

using json = nlohmann::json;

// Converts an array of either [r,g,b] or [r,g,b,a] values into the corresponding rgb colour.
// Alpha defaults to opaque if no given a value
sf::Color colorFromJSON(json &color)
{
    if (color.size() == 3)
        return sf::Color(color[0], color[1], color[2]);
    if (color.size() == 4) /* Has an alpha channel */
        return sf::Color(color[0], color[1], color[2], color[3]);
    PLOGW << "Invalid Color: " << color.dump();
    return sf::Color::Transparent;
}

MDPGUIConfig::MDPGUIConfig()
    :
        width(DEFAULT_WIDTH),
        height(DEFAULT_HEIGHT),
        backgroundColor(DEFAULT_BACKGROUND_COLOR),
        nodeColor(DEFAULT_NODE_COLOR),
        linkColor(DEFAULT_LINK_COLOR),
        linkNewPathColor(DEFAULT_LINK_NEW_PATH_COLOR),
        linkOldPathColor(DEFAULT_LINK_OLD_PATH_COLOR),
        attackerMoveColor(DEFAULT_ATTACKER_MOVE_COLOR),
        defenderMoveColor(DEFAULT_DEFENDER_MOVE_COLOR),
        goalColor(DEFAULT_GOAL_COLOR),
        networkFile(DEFAULT_NETWORK_FILE),
        startNodeId(DEFAULT_START_NODE_ID),
        goalNodeId(DEFAULT_GOAL_NODE_ID),
        defenderPolicy(DEFAULT_DEFENDER_POLICY),
        logLevel(DEFAULT_LOG_LEVEL),
        GUI(DEFAULT_GUI),
        numSimulations(DEFAULT_NUM_SIMULATIONS)
{}

// Load the default configuration values
MDPGUIConfig MDPGUIConfig::LoadConfig()
{
    return MDPGUIConfig();
}

// Load the values from a provided configuration file
MDPGUIConfig MDPGUIConfig::LoadConfig(const std::string &fileName)
{
    MDPGUIConfig result;
    
    std::ifstream i(fileName);
    json j;
    i >> j;
    
    if (j.contains("width")) {
        result.width = j["width"];
        PLOGV << "Loaded width=" << result.width;
    } else {
        PLOGV << "No width";
    }
    
    if (j.contains("height")) {
        result.height = j["height"];
        PLOGV << "Loaded height=" << result.height;
    } else {
        PLOGV << "No height";
    }
    
    sf::Color tempColor;
    if (j.contains("backgroundColor")) {
        tempColor = colorFromJSON(j["backgroundColor"]);
        if (tempColor != sf::Color::Transparent)
            result.backgroundColor = tempColor;
        PLOGV << "Loaded backgroundColor=" << result.backgroundColor;
    } else {
        PLOGV << "No backgroundColor";
    }
    
    if (j.contains("nodeColor")) {
        tempColor = colorFromJSON(j["nodeColor"]);
        if (tempColor != sf::Color::Transparent)
            result.nodeColor = tempColor;
        PLOGV << "Loaded nodeColor=" << result.nodeColor;
    } else {
        PLOGV << "No nodeColor";
    }
    
    if (j.contains("linkColor")) {
        tempColor = colorFromJSON(j["linkColor"]);
        if (tempColor != sf::Color::Transparent)
            result.linkColor = tempColor;
        PLOGV << "Loaded linkColor=" << result.linkColor;
    } else {
        PLOGV << "No linkColor";
    }
    
    if (j.contains("linkNewPathColor")) {
        tempColor = colorFromJSON(j["linkNewPathColor"]);
        if (tempColor != sf::Color::Transparent)
            result.linkNewPathColor = tempColor;
        PLOGV << "Loaded linkNewPathColor=" << result.linkNewPathColor;
    } else {
        PLOGV << "No linkNewPathColor";
    }
    
    if (j.contains("linkOldPathColor")) {
        tempColor = colorFromJSON(j["linkOldPathColor"]);
        if (tempColor != sf::Color::Transparent)
            result.linkOldPathColor = tempColor;
        PLOGV << "Loaded linkOldPathColor=" << result.linkOldPathColor;
    } else {
        PLOGV << "No linkOldPathColor";
    }
    
    if (j.contains("attackerMoveColor")) {
        tempColor = colorFromJSON(j["attackerMoveColor"]);
        if (tempColor != sf::Color::Transparent)
            result.attackerMoveColor = tempColor;
        PLOGV << "Loaded attackerMoveColor=" << result.attackerMoveColor;
    } else {
        PLOGV << "No attackerMoveColor";
    }
    
    if (j.contains("defenderMoveColor")) {
        tempColor = colorFromJSON(j["defenderMoveColor"]);
        if (tempColor != sf::Color::Transparent)
            result.defenderMoveColor = tempColor;
        PLOGV << "Loaded defenderMoveColor=" << result.defenderMoveColor;
    } else {
        PLOGV << "No defenderMoveColor";
    }
    
    if (j.contains("goalColor")) {
        tempColor = colorFromJSON(j["goalColor"]);
        if (tempColor != sf::Color::Transparent)
            result.goalColor = tempColor;
        PLOGV << "Loaded goalColor=" << result.goalColor;
    } else {
        PLOGV << "No goalColor";
    }
    
    if (j.contains("networkFile")) {
        result.networkFile = RESOURCE(j["networkFile"]);
        PLOGV << "Loaded networkFile=" << result.networkFile;
    } else {
        PLOGV << "No networkFile";
    }
    
    if (j.contains("startNodeId")) {
        result.startNodeId = j["startNodeId"];
        PLOGV << "Loaded startNodeId=" << result.startNodeId;
    } else {
        PLOGV << "No startNodeId";
    }
    
    if (j.contains("goalNodeId")) {
        result.goalNodeId = j["goalNodeId"];
        PLOGV << "Loaded goalNodeId=" << result.goalNodeId;
    } else {
        PLOGV << "No goalNodeId";
    }
    
    if (j.contains("defenderPolicy")) {
        result.defenderPolicy = j["defenderPolicy"];
        PLOGV << "Loaded defenderPolicy=" << result.defenderPolicy;
    } else {
        PLOGV << "No defenderPolicy";
    }
    
    if (j.contains("logLevel")) {
        result.logLevel = j["logLevel"];
        PLOGV << "Loaded logLevel=" << result.logLevel;
    } else {
        PLOGV << "No logLevel";
    }
    
    if (j.contains("GUI")) {
        result.GUI = j["GUI"];
        PLOGV << "Loaded GUI=" << result.GUI;
    } else {
        PLOGV << "No GUI";
    }
    
    if (j.contains("numSimulations")) {
        result.numSimulations = j["numSimulations"];
        PLOGV << "Loaded numSimulations=" << result.numSimulations;
    } else {
        PLOGV << "No numSimulations";
    }
    
    return result;
}
