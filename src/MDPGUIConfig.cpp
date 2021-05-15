#include "MDPGUIConfig.h"

#include <fstream>

#include "Utility.h"

#include "plog/Log.h"
#include <nlohmann/json.hpp>

#define DEFAULT_WIDTH 1200
#define DEFAULT_HEIGHT 1200

#define DEFAULT_BACKGROUND_COLOR sf::Color::Black
#define DEFAULT_SERVER_COLOR sf::Color::Yellow
#define DEFAULT_DATABASE_COLOR sf::Color(160, 0, 255)
#define DEFAULT_CLIENT_COLOR sf::Color::Green

#define DEFAULT_NETWORK_FILE RESOURCE("MDPExample1.json")

#define DEFAULT_DEFENDER_POLICY goal

using json = nlohmann::json;

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
        serverColor(DEFAULT_SERVER_COLOR),
        databaseColor(DEFAULT_DATABASE_COLOR),
        clientColor(DEFAULT_CLIENT_COLOR),
        networkFile(DEFAULT_NETWORK_FILE),
        defenderPolicy(DEFAULT_DEFENDER_POLICY)
{}

MDPGUIConfig MDPGUIConfig::LoadConfig()
{
    return MDPGUIConfig();
}

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
    
    if (j.contains("serverColor")) {
        tempColor = colorFromJSON(j["serverColor"]);
        if (tempColor != sf::Color::Transparent)
            result.serverColor = tempColor;
        PLOGV << "Loaded serverColor=" << result.serverColor;
    } else {
        PLOGV << "No serverColor";
    }
    
    if (j.contains("databaseColor")) {
        tempColor = colorFromJSON(j["databaseColor"]);
        if (tempColor != sf::Color::Transparent)
            result.databaseColor = tempColor;
        PLOGV << "Loaded databaseColor=" << result.databaseColor;
    } else {
        PLOGV << "No databaseColor";
    }
    
    if (j.contains("clientColor")) {
        tempColor = colorFromJSON(j["clientColor"]);
        if (tempColor != sf::Color::Transparent)
            result.clientColor = tempColor;
        PLOGV << "Loaded clientColor=" << result.clientColor;
    } else {
        PLOGV << "No clientColor";
    }
    
    if (j.contains("networkFile")) {
        result.networkFile = RESOURCE(j["networkFile"]);
        PLOGV << "Loaded networkFile=" << result.networkFile;
    } else {
        PLOGV << "No networkFile";
    }
    
    if (j.contains("defenderPolicy")) {
        result.defenderPolicy = j["defenderPolicy"];
        PLOGV << "Loaded defenderPolicy=" << result.defenderPolicy;
    } else {
        PLOGV << "No defenderPolicy";
    }
    
    /*
    json nodes = j["nodes"];
    std::map<int, std::set<int>> nodeLinks;
    for (json::const_iterator it = nodes.begin(); it != nodes.end(); it++) {
        if (nodeLinks.insert(std::pair<int, std::set<int>>(
                (*it)["id"],
                (*it)["links"])).second == false) {
            // This was a duplicate node, bad file
            return nullptr;
        }
        //(*it)["id"] << std::endl;
    }
    
    
    for (std::map<int, std::set<int>>::iterator it = nodeLinks.begin();
            it != nodeLinks.end(); it++) {
        std::stringstream ss;
        ss << "id: " << it->first << ", links: [";
        for (std::set<int>::iterator iit = it->second.begin();
                iit != it->second.end(); iit++) {
            ss << *iit << ", ";
        }
        PLOGD << ss.str() << "]";
        //(*it)["id"] << std::endl;
    }
    
    PLOGD << "SIZE: " << nodeLinks.size();
    
    NetworkTopology *test = new NetworkTopology(nodeLinks, nodeWidth,
            windowSize, layout);
    //NetworkTopology *test = new NetworkTopology(numNodes, nodeWidth, windowSize);
    for (std::list<std::reference_wrapper<Link>>::iterator it = test->m_links.begin();
            it != test->m_links.end(); it++) {
        PLOGD << "Link: " << it->get();
    }*/
    return result;
}
