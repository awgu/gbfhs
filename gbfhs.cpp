/**
 * @file gbfhs.cpp
 * @brief GBFHS implementation for n-pancake problem.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug The current implementation assumes unit costs.
 */

/* includes */
#include "gbfhs.h"

#include <random>
#include <limits.h>
#include <assert.h>

/* used for randomness */
std::random_device rd;
std::mt19937 gen(rd());

/**
 * @brief Checks if the given node is expandable in the given direction.
 * 
 * A node is expandable iff f_D(node) <= fLim and g_D(node) < gLim_D.
 * 
 * @param node Node to check.
 * @param dir Direction to check.
 * @param fLim Lower bound on the optimal solution cost.
 * @param gLim_D Upper bound on the g-value of nodes to explore in direction D.
 * @return True if the node is expandable; false otherwise.
 */
bool is_expandable(const Node &node, Direction dir, int fLim, int gLim_D) {
    assert(dir == Direction::F || dir == Direction::B);
    int f_D = 0;
    int g_D = 0;
    if (dir == Direction::F) {
        assert(node.g_F >= 0 && node.h_F >= 0);
        g_D = node.g_F;
        f_D = node.g_F + node.h_F;
    } else if (dir == Direction::B) {
        assert(node.g_B >= 0 && node.h_B >= 0);
        g_D = node.g_B;
        f_D = node.g_B + node.h_B;
    } else {
        throw std::runtime_error("invalid direction");
    }
    return f_D <= fLim && g_D < gLim_D;
}

/**
 * @brief Divides the value of gLSum among gLim_F and gLim_B.
 * 
 * The function modifies gLim_F and gLim_B in-place.
 * 
 * @param gLSum Smallest value of gLim_F + gLim_B such that all solutions with
 * a cost of fLim have been found by the end of the iteration.
 * @param gLim_F Reference to gLim_F, an upper bound on the g-value of nodes
 * to explore in the forward direction.
 * @param gLim_B Reference to gLim_B, an upper bound on the g-value of nodes
 * to explore in the backward direction.
 * @return Void.
 * @post Updated gLim_F >= original gLim_F and updated gLim_B >= original
 * gLim_B;
 * @post gLim_F + gLim_B == gLSum.
 */
void split(int gLSum, int &gLim_F, int &gLim_B) {
    int old_gLim_F = gLim_F;
    int old_gLim_B = gLim_B;
    /* divide excess among gLim_F and gLim_B */
    int excess = gLSum - gLim_F - gLim_B;
    int ratio = 2;
    int gLim_F_delta = excess / ratio;
    gLim_F += gLim_F_delta;
    gLim_B += excess - gLim_F_delta;
    assert(gLim_B + gLim_F == gLSum);
    assert(gLim_F >= old_gLim_F);
    assert(gLim_B >= old_gLim_B);
}

/**
 * @brief Pick a uniform random node from the two given forward and backward
 * expandable sets.
 * 
 * @param expandable_F Subset of open_F that is forward expandable.
 * @param expandable_B Subset of open_B that is backward expandable.
 * @return Uniform random node chosen from the two expandable sets.
 */
Node pick(const NodeSet &expandable_F, const NodeSet &expandable_B) {
    std::uniform_int_distribution<> dist(0, expandable_F.size() + expandable_B.size() - 1);
    int random_index = dist(gen);
    auto it = expandable_F.begin();
    if (random_index >= static_cast<int>(expandable_F.size())) {
        random_index -= expandable_F.size();    
        it = expandable_B.begin();
    }
    assert(random_index >= 0);
    std::advance(it, random_index);
    return *it;   
}

/**
 * @brief Expands the current level.
 * 
 * @param gLim_F Upper bound on the g-value of nodes to explore in the forward
 * direction.
 * @param gLim_B Upper bound on the g-value of nodes to explore in the
 * backward direction.
 * @param fLim Lower bound on the optimal solution cost.
 * @param best Lowest solution cost so far.
 * @param gap_x x for the GAP-x heuristic.
 * @param nodes_expanded Number of nodes expanded so far.
 * @param open_F Forward open set.
 * @param open_B Backward open set.
 * @param closed_F Forward closed set.
 * @param closed_B Backward closed set.
 * @return Void.
 */
void expand_level(int gLim_F, int gLim_B, int fLim, int &best, int gap_x, int &nodes_expanded,
    NodeSet &open_F, NodeSet &open_B, NodeSet &closed_F, NodeSet &closed_B) {
    NodeSet expandable_F;
    NodeSet expandable_B;
    for (const Node &node : open_F) {
        if (is_expandable(node, Direction::F, fLim, gLim_F)) {
            expandable_F.emplace(node);
        }
    }
    for (const Node &node : open_B) {
        if (is_expandable(node, Direction::B, fLim, gLim_B)) {
            expandable_B.emplace(node);
        }
    }
    while (!expandable_F.empty() || !expandable_B.empty()) {
        Node node = pick(expandable_F, expandable_B);
        Direction dir = node.dir;
        assert(dir == Direction::F || dir == Direction::B);
        if (dir == Direction::F) {
            assert(node.g_F >= 0 && node.g_F < INT_MAX);
            assert(expandable_F.count(node) > 0 && open_F.count(node) > 0);
            expandable_F.erase(node);
            open_F.erase(node);
            assert(closed_F.count(node) == 0);
            closed_F.insert(node);
            NodeVector successors = expand(node, gap_x, nodes_expanded);
            for (Node &s_node : successors) {
                /* get the successor node's old g_F value if it exists */
                assert(!(open_F.count(s_node) > 0 && closed_F.count(s_node) > 0));
                int old_s_node_g_F = INT_MAX;
                if (open_F.count(s_node) > 0) {
                    old_s_node_g_F = (*open_F.find(s_node)).g_F;
                } else if (closed_F.count(s_node) > 0) {
                    old_s_node_g_F = (*closed_F.find(s_node)).g_F;
                }

                /* continue if node visits s_node via a suboptimal path */
                // if (... && g_F(node) + cost(node, s_node) >= g_F(s_node))
                if ((open_F.count(s_node) > 0 || closed_F.count(s_node) > 0) &&
                    (node.g_F + 1 >= old_s_node_g_F)) {  // assumes unit cost
                    continue;
                }

                /* node visits s_node via a better path */
                // if (s_node in open_F U closed_F)
                if (open_F.count(s_node) > 0 || closed_F.count(s_node) > 0) {
                    open_F.erase(s_node);
                    closed_F.erase(s_node);
                }
                // g_F(s_node) <- g_F(node) + cost(node, s_node)
                s_node.g_F = node.g_F + 1;
                assert(open_F.count(s_node) == 0);
                open_F.insert(s_node);
                if (is_expandable(s_node, Direction::F, fLim, gLim_F)) {
                    if (expandable_F.count(s_node) > 0 && (*expandable_F.find(s_node)).g_F > s_node.g_F) {
                        expandable_F.erase(s_node);
                    }
                    expandable_F.insert(s_node);
                }

                /* collision */
                if (open_B.count(s_node) > 0) {
                    assert(s_node.g_B == -1);
                    s_node.g_B = (*open_B.find(s_node)).g_B;
                    best = std::min(best, s_node.g_F + s_node.g_B);
                    if (best <= fLim) {
                        return;
                    }
                }
            }
        } 
        else if (dir == Direction::B) {  // symmetric
            assert(node.g_B >= 0 && node.g_B < INT_MAX);
            assert(expandable_B.count(node) > 0 && open_B.count(node) > 0);
            expandable_B.erase(node);
            open_B.erase(node);
            assert(closed_B.count(node) == 0);
            closed_B.insert(node);
            NodeVector successors = expand(node, gap_x, nodes_expanded);
            for (Node &s_node : successors) {
                /* get the successor node's old g_B value if it exists */
                assert(!(open_B.count(s_node) > 0 && closed_B.count(s_node) > 0));
                int old_s_node_g_B = INT_MAX;
                if (open_B.count(s_node) > 0) {
                    old_s_node_g_B = (*open_B.find(s_node)).g_B;
                } else if (closed_B.count(s_node) > 0) {
                    old_s_node_g_B = (*closed_B.find(s_node)).g_B;
                }

                /* continue if node visits s_node via a suboptimal path */
                // if (... && g_B(node) + cost(node, s_node) >= g_B(s_node))
                if ((open_B.count(s_node) > 0 || closed_B.count(s_node) > 0) &&
                    (node.g_B + 1 >= old_s_node_g_B)) {  // assumes unit cost
                    continue;
                }

                /* node visits s_node via a better path */
                // if (s_node in open_B U closed_B)
                if (open_B.count(s_node) > 0 || closed_B.count(s_node) > 0) {
                    open_B.erase(s_node);
                    closed_B.erase(s_node);
                }
                // g_B(s_node) <- g_B(node) + cost(node, s_node)
                s_node.g_B = node.g_B + 1;
                assert(open_B.count(s_node) == 0);
                open_B.insert(s_node);
                if (is_expandable(s_node, Direction::B, fLim, gLim_B)) {
                    if (expandable_B.count(s_node) > 0 && (*expandable_B.find(s_node)).g_B > s_node.g_B) {
                        expandable_B.erase(s_node);
                    }
                    expandable_B.insert(s_node);
                }

                /* collision */
                if (open_F.count(s_node) > 0) {
                    assert(s_node.g_F == -1);
                    s_node.g_F = (*open_F.find(s_node)).g_F;
                    best = std::min(best, s_node.g_B + s_node.g_F);
                    if (best <= fLim) {
                        return;
                    }
                }
            }
        }
    }
}

/**
 * @brief Runs the GBFHS algorithm with the given initial and goal states.
 * 
 * @param initial_state Initial state.
 * @param goal_state Goal state.
 * @param eps Integer representing the minimum-cost operator in the domain,
 * i.e. the cheapest edge in the state space.
 * @param gap_x x for the GAP-x heuristic.
 * @param nodes_expanded To be set to the number of nodes expanded.
 * @return Optimal cost.
 */
int gbfhs(const std::vector<int> &initial_state, const std::vector<int> &goal_state, int eps, int gap_x, int &nodes_expanded) {
    if (is_solved(initial_state, goal_state)) {
        return 0;
    }
    int best = INT_MAX;  // unsolvable
    NodeSet open_F;
    NodeSet open_B;
    NodeSet closed_F;
    NodeSet closed_B;
    open_F.emplace(initial_state, 0, INT_MAX, h(initial_state, Direction::F, gap_x), -1, Direction::F);
    open_B.emplace(goal_state, INT_MAX, 0, -1, 0, Direction::B);
    nodes_expanded = 0;
    int fLim = std::max(std::max(h(initial_state, Direction::F, gap_x), h(goal_state, Direction::B, gap_x)), eps);
    int gLim_F = 0;
    int gLim_B = 0;
    while (!open_F.empty() && !open_B.empty()) {
        if (best == fLim) {
            return best;
        }
        int gLSum = fLim - eps + 1;
        split(gLSum, gLim_F, gLim_B);
        expand_level(gLim_F, gLim_B, fLim, best, gap_x, nodes_expanded, open_F, open_B, closed_F, closed_B);
        if (best == fLim) {
            return best;
        }
        std::cout << "nodes expanded: " << nodes_expanded << std::endl;
        fLim++;
    }
    return best;
}

