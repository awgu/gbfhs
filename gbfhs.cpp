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
 * @param gap_x x for the GAP-x heuristic.
 * @param fLim Lower bound on the optimal solution cost.
 * @param gLim_D Upper bound on the g-value of nodes to explore in direction D.
 * @param g_D Map from nodes to costs.
 * @return True if the node is expandable; false otherwise.
 */
bool is_expandable(const Node &node, Direction dir, int gap_x, int fLim, int gLim_D, const NodeIntMap &g_D) {
    assert(dir == Direction::F || dir == Direction::B);
    int g_D_ = g_D.find(node)->second;
    int h_D = h(node.s, dir, gap_x);
    int f_D = g_D_ + h_D;
    return f_D <= fLim && g_D_ < gLim_D;
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
    if (excess == 1) { // after first iter, excess should always == 1
        if (gLim_F < gLim_B) {
            gLim_F++;
        } else {
            gLim_B++;
        }
    } else {
        int ratio = 2;
        int gLim_F_delta = excess / ratio;
        gLim_F += gLim_F_delta;
        gLim_B += excess - gLim_F_delta;
    }

    /* check split constraints */
    assert(gLim_F >= old_gLim_F);
    assert(gLim_B >= old_gLim_B);
    assert(gLim_B + gLim_F == gLSum);
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
 * @param g_F Map from nodes to forward costs.
 * @param g_B Map from nodes to backward costs.
 * @return Void.
 */
void expand_level(int gLim_F, int gLim_B, int fLim, int &best, int gap_x, int &nodes_expanded,
    NodeSet &open_F, NodeSet &open_B, NodeSet &closed_F, NodeSet &closed_B, NodeIntMap &g_F, NodeIntMap &g_B) {
    /* construct expandable sets */
    NodeSet expandable_F;  // subset of open_F
    NodeSet expandable_B;  // subset of open_B
    for (const Node &node : open_F) {
        if (is_expandable(node, Direction::F, gap_x, fLim, gLim_F, g_F)) {
            expandable_F.emplace(node);
        }
    }
    for (const Node &node : open_B) {
        if (is_expandable(node, Direction::B, gap_x, fLim, gLim_B, g_B)) {
            expandable_B.emplace(node);
        }
    }

    /* main loop */
    while (!expandable_F.empty() || !expandable_B.empty()) {
        Node node = pick(expandable_F, expandable_B);
        Direction dir = node.dir;
        
        /* generalize to D == F or D == B */
        NodeSet &open_D = (dir == Direction::F) ? open_F : open_B;
        NodeSet &open_D_opp = (dir == Direction::F) ? open_B : open_F;
        NodeSet &closed_D = (dir == Direction::F) ? closed_F : closed_B;
        NodeSet &expandable_D = (dir == Direction::F) ? expandable_F : expandable_B;
        NodeIntMap &g_D = (dir == Direction::F) ? g_F : g_B;
        NodeIntMap &g_D_opp = (dir == Direction::F) ? g_B : g_F;
        int gLim_D = (dir == Direction::F) ? gLim_F : gLim_B;

        /* mark node as closed */
        assert(closed_D.count(node) == 0);
        expandable_D.erase(node);
        open_D.erase(node);
        closed_D.insert(node);

        /* iterate over successor nodes */
        NodeVector successors = expand(node, gap_x, nodes_expanded);
        for (Node &s_node : successors) {
            /* continue if node visits s_node via a suboptimal path */
            bool already_seen = open_D.count(s_node) > 0 || closed_D.count(s_node) > 0;
            if (already_seen) {
                assert(g_D.count(node) > 0 && g_D.count(s_node) > 0);
                bool suboptimal_cost = g_D[node] + 1 >= g_D[s_node];  // assumes unit cost
                if (suboptimal_cost) {
                    continue;
                }
            }

            /* node visits s_node via a cheaper path */
            open_D.erase(s_node);
            closed_D.erase(s_node);

            if (g_D.count(s_node) > 0) {
                assert(g_D[s_node] > g_D[node] + 1);
            }
            g_D[s_node] = g_D[node] + 1;  // assumes unit cost
            open_D.insert(s_node);
            if (is_expandable(s_node, dir, gap_x, fLim, gLim_D, g_D)) {
                expandable_D.insert(s_node);
            }

            /* check for collision */
            if (open_D_opp.count(s_node) > 0) {
                assert(g_D.count(s_node) > 0 && g_D_opp.count(s_node) > 0);
                best = std::min(best, g_D[s_node] + g_D_opp[s_node]);
                if (best <= fLim) {
                    return;
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
    nodes_expanded = 0;

    /* initialize node sets */
    NodeSet open_F, open_B, closed_F, closed_B;
    open_F.emplace(initial_state, Direction::F);
    open_B.emplace(goal_state, Direction::B);

    /* initialize cost data structures */
    NodeIntMap g_F, g_B;
    g_F.emplace(std::make_pair(Node { initial_state, Direction::F }, 0));
    g_B.emplace(std::make_pair(Node { goal_state, Direction::B }, 0));    

    /* initialize limits */
    int fLim = std::max(std::max(h(initial_state, Direction::F, gap_x), h(goal_state, Direction::B, gap_x)), eps);
    int gLim_F = 0;
    int gLim_B = 0;

    /* main loop */
    while (!open_F.empty() && !open_B.empty()) {
        if (best == fLim) {
            return best;
        }
        int gLSum = fLim - eps + 1;
        split(gLSum, gLim_F, gLim_B);
        expand_level(gLim_F, gLim_B, fLim, best, gap_x, nodes_expanded, open_F, open_B, closed_F, closed_B, g_F, g_B);
        if (best == fLim) {
            return best;
        }
        // std::cout << "nodes expanded: " << nodes_expanded << std::endl;
        fLim++;
    }
    return best;
}

