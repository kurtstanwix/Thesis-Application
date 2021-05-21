#include "NetworkSimulator.h"

// Take the next action in the simulation
void NetworkSimulator::nextAction()
{
    if (m_finished) {
        if (m_currSimulation < m_numSimulations) {
            reset();
        }
    }
    if (!m_finished) {
        if (m_toMove == attacker) {
            m_attacker->reset();

            // Remove old move highlighting
            for (std::list<SimAction>::iterator it = m_currPath.begin(); it != m_currPath.end(); ++it) {
                if (m_top->m_netWindow.m_nettop->getLinkColor(
                        it->second.first, it->second.second) == m_config.linkNewPathColor) {
                    m_top->m_netWindow.m_nettop->setLinkColor(
                            it->second.first, it->second.second, m_config.linkColor);
                }
            }

            m_attacker->TakeAction();

            // Set the already travelled path colour
            for (SimActionHistory::iterator it = m_attackerActions.begin();
                    it != m_attackerActions.end(); ++it) {
                if (it->second == fail) {
                    continue;
                }
                TopologyWrapper::Link &link = m_top->getLinks().at(it->first.second);
                m_top->m_netWindow.m_nettop->setLinkColor(link.m_endIds.first,
                        link.m_endIds.second, m_config.linkOldPathColor);
                for (std::vector<TopologyWrapper::Link::VulInstance>::iterator itt = link.m_vulnerabilities.begin();
                        itt != link.m_vulnerabilities.end(); ++itt) {
                    if (itt->m_vul.cveID == it->first.first) {
                        itt->m_highlighted = false; // Only want the current move highlighted
                    }
                }
                m_top->updateLinkInfo(link);
            }

            std::pair<SimAction, ActionResult> &action = m_attackerActions.back();
            if (action.second == success) {
                // Highlight the link and update the vulnerability state
                TopologyWrapper::Link &link = m_top->getLinks().at(action.first.second);
                m_top->m_netWindow.m_nettop->setLinkColor(link.m_endIds.first,
                        link.m_endIds.second, m_config.attackerMoveColor);
                for (std::vector<TopologyWrapper::Link::VulInstance>::iterator it = link.m_vulnerabilities.begin();
                        it != link.m_vulnerabilities.end(); ++it) {
                    if (it->m_vul.cveID == action.first.first) {
                        it->m_highlighted = true;
                        it->m_state = vulnerability_state::compromised;
                    }
                }
                m_top->updateLinkInfo(link);
                m_top->m_netWindow.m_nettop->setNodeColor(
                        action.first.second.first, m_config.linkOldPathColor);
                m_top->m_netWindow.m_nettop->setNodeColor(
                        action.first.second.second, m_config.attackerMoveColor);
            }

            m_toMove = defender;
            for (std::list<SimAction>::iterator it =  m_currPath.begin(); it != m_currPath.end(); ++it) {
                m_top->m_netWindow.m_nettop->setLinkColor(
                        it->second.first, it->second.second, m_config.linkNewPathColor);
            }
        } else {
            // Remove highlighting of previous defender moves
            for (SimActionHistory::iterator it = 
                    m_defenderActions.begin(); it != m_defenderActions.end(); ++it) {
                if (it->second == fail) {
                    continue;
                }
                TopologyWrapper::Link &link = m_top->getLinks().at(it->first.second);
                if (m_top->m_netWindow.m_nettop->getLinkColor(
                        link.m_endIds.first, link.m_endIds.second) == m_config.defenderMoveColor) {
                    m_top->m_netWindow.m_nettop->setLinkColor(
                            link.m_endIds.first, link.m_endIds.second, m_config.linkColor);
                }
                for (std::vector<TopologyWrapper::Link::VulInstance>::iterator itt = link.m_vulnerabilities.begin();
                        itt != link.m_vulnerabilities.end(); ++itt) {
                    if (itt->m_vul.cveID == it->first.first) {
                        itt->m_highlighted = false; // Only want the current move highlighted
                        break;
                    }
                }
                m_top->updateLinkInfo(link);
            }
            m_defender->TakeAction();
            std::pair<SimAction, ActionResult> &action = m_defenderActions.back();
            if (action.second == success) {
                m_top->m_netWindow.m_nettop->setLinkColor(
                        action.first.second.first, action.first.second.second, m_config.defenderMoveColor);
            }
            m_toMove = attacker;
        }

        // Determine who won and print result
        if (m_finished || m_currNode == m_goalNode) {
            if (m_currNode == m_goalNode) {
                m_finished = true;
                m_simResult = attackerWin;
            } else {
                m_simResult = defenderWin;
            }
            printActions(m_attackerActions, m_defenderActions, m_simResult);
            saveHistory();
        }
    }
}

// Prints out the simulation  actions and result based on the parameters
void NetworkSimulator::printActions(const SimActionHistory &attackerActions,
        const SimActionHistory &defenderActions, const SimResult &result)
{
    std::cout << "Attacker Actions | Defender Actions" << std::endl;
    SimActionHistory::const_iterator da = defenderActions.begin();
    for (SimActionHistory::const_iterator aa = attackerActions.begin();
            aa != attackerActions.end(); ++aa) {
        std::cout << " Exploit " << aa->first.first << " on link " << aa->first.second.first << "->" << aa->first.second.second << ": "
                << (aa->second == fail ? "FAILED" : "SUCCEEDED");
        std::cout << " | ";
        if (da != defenderActions.end()) {
            std::cout << " Patch " << da->first.first << " on link " << da->first.second.first << "->" << da->first.second.second << ": "
                    << (da->second == fail ? "FAILED" : "SUCCEEDED");
            ++da;
        }
        std::cout << std::endl;
    }
    std::string winner;
    switch (result) {
        case attackerWin:
            winner = "Attacker";
            break;
        case defenderWin:
            winner = "Defender";
            break;
        default:
            winner = "Undefined";
    }
    std::cout << " Result: " << winner << " wins" << std::endl;
}

// Run a single simulation until completion, if there are anymore to to run
void NetworkSimulator::runSimulation()
{
    if (m_finished) {
        if (m_currSimulation < m_numSimulations) {
            reset();
        }
    }
    while (!m_finished) {
        nextAction();
    }
    saveHistory();
}

// Runs all remaining simulations until completion
void NetworkSimulator::runAllSimulations()
{
    while (m_currSimulation < m_numSimulations) {
        runSimulation();
    }
}

// Store the current simulation moves and actions and reset them
void NetworkSimulator::saveHistory()
{
    if (!m_attackerActions.empty() || !m_defenderActions.empty()) {
        m_history.emplace_back(std::make_pair(m_attackerActions, m_defenderActions), m_simResult);
        m_attackerActions.clear();
        m_defenderActions.clear();
    }
}

// Print the history of all simulations that have been saved
void NetworkSimulator::printCurrentHistory() {
    int i = 1;
    int attackerWins = 0;
    int defenderWins = 0;
    for (std::list<std::pair<std::pair<SimActionHistory, SimActionHistory>, SimResult>>::iterator it = m_history.begin();
            it != m_history.end(); ++it) {
        std::cout << "Simulation " << i++ << ":" << std::endl;
        printActions(it->first.first, it->first.second, it->second);
        if (it->second == attackerWin)
            attackerWins++;
        else if (it->second == defenderWin)
            defenderWins++;
    }
    std::cout << "Attacker Wins: " << attackerWins << std::endl;
    std::cout << "Defender Wins: " << defenderWins << std::endl;
}