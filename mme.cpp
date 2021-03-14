/**
 * @file mme.cpp
 * @brief MMe implementation for n-pancake problem.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug 
 */

#include "mme.h"

#include <limits.h>
#include <assert.h>

/**
 * @brief Return the priority of the given node in the given direction.
 * 
 * The priority of node n in direction D is given by:
 *     pr_D(n) := max(f_D(n), 2g_D(n) + eps)
 * 
 * @param node Node to get the priority of.
 * @param eps Minimum cost operator on the node.
 * @param dir Direction.
 * @param gap_x x for the GAP-x heuristic.
 * @param g_D Map from nodes to cost in direction dir.
 * @return Priority of the node.
 */
int pr(const Node &node, int eps, Direction dir, int gap_x, const NodeIntMap &g_D) {
    assert(dir == Direction::F || dir == Direction::B);
    assert(g_D.count(node) > 0);
    int g_D_ = g_D.find(node)->second;
    int h_D = h(node.s, dir, gap_x);
    int f_D = g_D_ + h_D;
    return std::max(f_D, 2 * g_D_ + eps);
}

/**
 * @brief Scans over the open set for the given direction to collect prmin_D,
 * fmin_D, and gmin_D and return the optimal node to expand.
 *
 * The optimal node to expand is the node n with pr_D(n) == prmin_D and
 * minimal g_D(n) in the case of ties.
 * 
 * @param open_D Open set to scan.
 * @param eps Minimum cost operator.
 * @param dir Direction of the open set to scan.
 * @param gap_X x for the GAP-x heuristic.
 * @param prmin_D (output) Minimum priority on open_D.
 * @param fmin_D (output) Minumum f on open_D.
 * @param gmin_D (output) Minimum g on open_D.
 * @param g_D Map from nodes to costs in direction dir.
 * @return Node with pr_D(n) == prmin_D and minimal g_D(n).
 */
Node scan(const NodeSet &open_D, int eps, Direction dir, int gap_x, int &prmin_D, int &fmin_D, int &gmin_D, const NodeIntMap &g_D) {
    assert(!open_D.empty());
    assert(dir == Direction::F || dir == Direction::B);

    prmin_D = INT_MAX;
    fmin_D = INT_MAX;
    gmin_D = INT_MAX;

    /* node n with pr_D(n) == prmin_D and minimal g_D(n) for ties */
    const Node *opt_node = &(*open_D.begin());
    int g_D_ = INT_MAX;

    for (const Node &node : open_D) {
        int pr_D = pr(node, eps, dir, gap_x, g_D);
        assert(g_D.count(node) > 0);

        /* strictly smaller priority */
        if (pr_D < prmin_D) {
            opt_node = &node;
            prmin_D = pr_D;    
            g_D_ = g_D.find(node)->second;
        }
        /* equal priority but strictly smaller g_D */
        else if (pr_D == prmin_D) {
            if (g_D.count(node) > 0 && g_D.find(node)->second < g_D_) {
                opt_node = &node;
                g_D_ = g_D.find(node)->second;
            }
        }

        fmin_D = std::min(fmin_D, g_D.find(node)->second + h(node.s, dir, gap_x));
        gmin_D = std::min(gmin_D, g_D.find(node)->second);
    }

    assert(opt_node != nullptr);
    return *opt_node;
}

/**
 * @brief Runs the MMe algorithm with the given initial and goal state.
 * 
 * @param initial_state Initial state.
 * @param goal_state Goal state.
 * @param eps Integer representing the minimum-cost operator in the domain,
 * i.e. the cheapest edge in the state space.
 * @param gap_x x for the GAP-x heuristic.
 * @param nodes_expanded (output) Number of nodes expanded.
 * @return Optimal cost.
 */
int mme(const std::vector<int> &initial_state, const std::vector<int> &goal_state, int eps, int gap_x, int &nodes_expanded) {
    int U = INT_MAX;  // unsolvable
    nodes_expanded = 0;

    /* initialize node sets */
    NodeSet open_F, open_B, closed_F, closed_B;
    open_F.emplace(initial_state, Direction::F);
    open_B.emplace(goal_state, Direction::B);

    /* initialize cost data structures */
    NodeIntMap g_F, g_B;
    g_F.emplace(std::make_pair(Node { initial_state, Direction::F }, 0));
    g_B.emplace(std::make_pair(Node { goal_state, Direction::B }, 0));    

    /* main loop */
    while (!open_F.empty() && !open_B.empty()) {
        int fmin_F, fmin_B, gmin_F, gmin_B, prmin_F, prmin_B;

        Node node_F = scan(open_F, eps, Direction::F, gap_x, prmin_F, fmin_F, gmin_F, g_F);
        Node node_B = scan(open_B, eps, Direction::B, gap_x, prmin_B, fmin_B, gmin_B, g_B);
        int C = std::min(prmin_F, prmin_B);
        if (U <= std::max(std::max(C, fmin_F), std::max(fmin_B, gmin_F + gmin_B + eps))) {
            return U;
        }

        Node &node = (C == prmin_F) ? node_F : node_B;
        NodeSet &open_D = (C == prmin_F) ? open_F : open_B;
        NodeSet &open_D_opp = (C == prmin_F) ? open_B : open_F;
        NodeSet &closed_D = (C == prmin_F) ? closed_F : closed_B;
        NodeIntMap &g_D = (C == prmin_F) ? g_F : g_B;
        NodeIntMap &g_D_opp = (C == prmin_F) ? g_B : g_F;

        /* mark node as closed */
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

            /* collision */
            if (open_D_opp.count(s_node) > 0) {
                assert(g_D.count(s_node) > 0 && g_D_opp.count(s_node) > 0);
                U = std::min(U, g_D[s_node] + g_D_opp[s_node]);
            }
        }
    }
    return U;
}