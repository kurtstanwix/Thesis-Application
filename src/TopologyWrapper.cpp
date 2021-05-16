#include "TopologyWrapper.h"


#include <fstream>

#include "plog/Log.h"
#include <nlohmann/json.hpp>

#include "Utility.h"

using json = nlohmann::json;



std::string TopologyWrapper::Link::VulInstance::getString() const
{
    std::stringstream ss;
    ss << "- ";
    if (m_highlighted)
        ss << "`";
    ss << m_vul.cveID;
    ss << "\n   CVSS: ";
    ss << m_vul.cvss;
    ss << "\n   State: ";
    switch (m_state) {
        case patched:
            ss << "Patched";
            break;
        case compromised:
            ss << "Compromised";
            break;
        case vulnerable:
            /* FALL THROUGH */
        default:
            ss << "Vulnerable";
            break;
    }
    if (m_highlighted)
        ss << "`";
    return ss.str();
}

std::string TopologyWrapper::Link::getVulString(vulnerability_state state) const
{
    std::string vulString;
    for (std::vector<VulInstance>::const_iterator v = m_vulnerabilities.begin();
            v != m_vulnerabilities.end(); ++v) {
        if (v->m_state != state) {
            continue;
        }
        vulString.append(v->getString());
        if (std::next(v) != m_vulnerabilities.end())
            vulString.append("\n");
    }
    return vulString;
}

void TopologyWrapper::updateLinkInfo(const Link &link)
{
    m_netWindow.m_nettop->setLinkInfoParameter(link.m_endIds.first, link.m_endIds.second,
            "Vulnerable", link.getVulString(vulnerable), true);
    m_netWindow.m_nettop->setLinkInfoParameter(link.m_endIds.first, link.m_endIds.second,
            "Compromised", link.getVulString(compromised), true);
    m_netWindow.m_nettop->setLinkInfoParameter(link.m_endIds.first, link.m_endIds.second,
            "Patched", link.getVulString(patched), true);
}



TopologyWrapper::TopologyWrapper(const sf::Vector2f &windowSize,
        const MDPGUIConfig &config,
        const std::vector<std::string> &defenderActions, int nodeWidth)
    :
        m_defenderActions(defenderActions)
{
    if (!m_netWindow.init(windowSize, config.networkFile, nodeWidth) ||
            !init(windowSize, config, defenderActions, nodeWidth)) {
        /* Could handle or prompt another file name */
        PLOGF << "Bad file, exiting";
        exit(1);
    }
    
    
    /*
    
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
    }
    return test;*/
}

TopologyWrapper* TopologyWrapper::init(const sf::Vector2f &windowSize,
        const MDPGUIConfig &config,
        const std::vector<std::string> &defenderActions, int nodeWidth)
{
    std::ifstream i(config.networkFile);
    json j;
    i >> j;
    
    /* Load in the vulnerabilities present in the system over all links */
    if (!j.contains("systemVulnerabilities")) {
        PLOGE << "Does not contain required field \"systemVulnerabilities\"";
        return nullptr;
    }
    PLOGI << "Vulnerabilities:";
    json sysVulnerabilities = j["systemVulnerabilities"];
    for (json::const_iterator v = sysVulnerabilities.begin();
            v != sysVulnerabilities.end(); v++) {
        if (!v->contains("cveID")) {
            PLOGE << "Does not contain required field \"cveID\"";
            return nullptr;
        }
        PLOGI << " CVE ID: " << (*v)["cveID"];
        Vulnerability &vul = m_vulnerabilities[(*v)["cveID"]];
        vul.cveID = (*v)["cveID"];
        vul.cweName = "";
        if (v->contains("cweName"))
            vul.cweName = (*v)["cweName"];
        
        vul.cvss = -1;
        if (v->contains("cvss"))
            vul.cvss = (*v)["cvss"];
        
        vul.base = -1;
        if (v->contains("base"))
            vul.base = (*v)["base"];
        
        vul.impact = -1;
        if (v->contains("impact"))
            vul.impact = (*v)["impact"];
        
        vul.exploitability = -1;
        if (v->contains("exploitability"))
            vul.exploitability = (*v)["exploitability"];
    }
    
    /* Load in all the nodes, and each outgoing link, along with the
     * vulnerability instances on these links */
    if (!j.contains("nodes")) {
        PLOGE << "Contains no nodes";
        return nullptr;
    }
    PLOGI << "Nodes:";
    json nodes = j["nodes"];
    for (json::const_iterator n = nodes.begin(); n != nodes.end(); n++) {
        /* Create the node based on the saved object */
        int nodeID = (*n)["id"];
        Node &node = m_nodes[nodeID];
        node.id = nodeID;
        /* Need to add info for each node */
        m_netWindow.m_nettop->setNodeInfoTitle(nodeID,
                "Node " + std::to_string(nodeID));
        ///////////////
        if (!n->contains("links"))
            continue;
        PLOGI << "  Links:";
        json links = (*n)["links"];
        for (json::const_iterator li = links.begin(); li != links.end();
                li++) {
            int otherNodeID = (*li)["otherNodeID"];
            PLOGI << "   ID: " << otherNodeID;
            node.m_links.insert(otherNodeID);
            Link &link = m_links[{nodeID, otherNodeID}];
            link.m_endIds = {nodeID, otherNodeID};
            m_netWindow.m_nettop->setLinkInfoTitle(nodeID, otherNodeID,
                    "Link " + std::to_string(nodeID) + "->" +
                    std::to_string(otherNodeID));
            
            m_netWindow.m_nettop->setLinkInfoParameter(nodeID, otherNodeID,
                    "Vulnerabilities", "");
            if (li->contains("vulnerabilities")) {
                json vulnerabilities = (*li)["vulnerabilities"];
                PLOGI << "   Vulnerabilities:";
                for (json::const_iterator v = vulnerabilities.begin();
                        v != vulnerabilities.end(); v++) {
                    if (!(v->contains("cveID"))) {
                        PLOGE << "Vulnerability on link " << link.m_endIds << " does not contain required field \"cveID\"";
                        return nullptr;
                    }
                    std::string cveID = (*v)["cveID"];
                    PLOGI << "    cveID: \"" << cveID << "\"";
                    link.m_vulnerabilities.emplace_back(m_vulnerabilities[cveID]);
                    Link::VulInstance &vul = link.m_vulnerabilities.back();

                    if (v->contains("state"))
                        vul.m_state = (*v)["state"];
                    else
                        vul.m_state = vulnerable;
                }
            }
            updateLinkInfo(link);
        }
        
        m_netWindow.m_nettop->setNodeInfoParameter(nodeID, "Num Links",
                std::to_string(node.m_links.size()));
    }
    m_netWindow.m_nettop->setNodeInfoParameter(0, "Test", "Testing");

    // Set up network colouring from config
    std::vector<int> nodeIds = m_netWindow.m_nettop->getNodeIds();
    for (std::vector<int>::iterator id1 = nodeIds.begin(); id1 != nodeIds.end(); ++id1) {
        m_netWindow.m_nettop->setNodeColor(*id1, config.nodeColor);
        m_netWindow.m_nettop->setNodeInfoColor(*id1,
                sf::Color(rand() % 255, rand() % 255, rand() % 255));
        std::vector<int> linkIds;
        m_netWindow.m_nettop->getNodesOut(*id1, linkIds);
        for (std::vector<int>::iterator id2 = linkIds.begin(); id2 != linkIds.end(); ++id2) {
            m_netWindow.m_nettop->setLinkColor(*id1, *id2, config.linkColor);
            m_netWindow.m_nettop->setLinkInfoColor(*id1, *id2,
                    sf::Color(rand() % 255, rand() % 255, rand() % 255));
        }
    }

    m_netWindow.m_nettop->setColor(config.backgroundColor);

    return this;
}

void TopologyWrapper::update(sf::Event *event, const sf::Vector2f &windowSize,
        bool &clickedOn)
{
    m_netWindow.update(event, windowSize, clickedOn);
}
void TopologyWrapper::render(sf::RenderWindow& window, const sf::Vector2f &windowSize)
{
    m_netWindow.render(window, windowSize);
}
