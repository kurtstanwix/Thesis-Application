#ifndef _TOPOLOGY_WRAPPER_H
#define _TOPOLOGY_WRAPPER_H

#include "NetworkWindow.h"

enum vulnerability_state
{
    vulnerable = 0,
    compromised = 1,
    patched = 2
};

#define MAX_CVSS_SCORE 10.0
#define MAX_IMPACT_SCORE 6.0
#define MAX_EXPLOITABILITY_SCORE 3.9

class TopologyWrapper : public Renderable
{
public:
    struct Node;
    struct Link;
    struct Vulnerability;
protected:
    std::map<int, Node> m_nodes;
    std::map<std::pair<int, int>, Link> m_links;
    std::map<std::string, Vulnerability> m_vulnerabilities;
    
    std::vector<std::string> m_defenderActions;

    TopologyWrapper* init(const sf::Vector2f &windowSize,
        const std::string &fileName,
        const std::vector<std::string> &defenderActions, int nodeWidth);
public:
    /* Classes for managing the state of the POMDP system */
    struct Node
    {
        int id;
        std::set<int> m_links;
    };

    struct Link
    {
        struct VulInstance
        {
            Vulnerability &m_vul;
            vulnerability_state m_state;
            bool m_highlighted;
            VulInstance() = delete;
            VulInstance(Vulnerability &vul)
                :
                    m_vul(vul),
                    m_state(vulnerable),
                    m_highlighted(false)
            {};
            std::string getString() const;
        };
        
        std::pair<int, int> m_endIds;
        std::vector<VulInstance> m_vulnerabilities;
        std::string getVulString(vulnerability_state state) const;
    };
    
    struct Vulnerability
    {
        std::string cveID;
        std::string cweName;
        float cvss;
        float base;
        float impact;
        float exploitability;
    };
    
    NetworkWindow m_netWindow;
    
    TopologyWrapper() = delete;
    
    TopologyWrapper(const sf::Vector2f &windowSize,
            const std::string &fileName,
            const std::vector<std::string> &defenderActions,
            int nodeWidth = 100);
    
    void updateLinkInfo(const Link &link);

    std::map<int, Node>& getNodes()
    {
        return m_nodes;
    }

    std::map<std::pair<int, int>, Link>& getLinks()
    {
        return m_links;
    }

    std::map<std::string, Vulnerability>& getVulnerabilities()
    {
        return m_vulnerabilities;
    }



    /* Renderable interface */
    void update(sf::Event *event, const sf::Vector2f &windowSize,
            bool &clickedOn);
    void render(sf::RenderWindow& window, const sf::Vector2f &windowSize);
    bool contains(float x, float y) { return false; };
protected:
    void streamOut(std::ostream& os) const {};
};

#endif