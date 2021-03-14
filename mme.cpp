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
 * @return Priority of the node.
 */
int pr(const Node &node, int eps, Direction dir) {
    assert(dir == Direction::F || dir == Direction::B);
    int f_D, g_D;
    if (dir == Direction::F) {
        f_D = node.g_F + node.h_F;
        g_D = node.g_F;
    } else if (dir == Direction::B) {
        f_D = node.g_B + node.h_B;
        g_D = node.g_B;
    } else {
        throw std::runtime_error("invalid direction");
    }
    return std::max(f_D, 2 * g_D + eps);
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
 * @param prmin_D Minimum priority on open_D.
 * @param fmin_D Minumum f on open_D.
 * @param gmin_D Minimum g on open_D.
 * @return Node with pr_D(n) == prmin_D and minimal g_D(n).
 */
Node scan(const NodeSet &open_D, int eps, Direction dir, int &prmin_D, int &fmin_D, int &gmin_D) {
    assert(!open_D.empty());
    assert(dir == Direction::F || dir == Direction::B);
    prmin_D = INT_MAX;
    fmin_D = INT_MAX;
    gmin_D = INT_MAX;

    /* node n with pr_D(n) == prmin_D and minimal g_D(n) for ties */
    const Node *opt_node = &(*open_D.begin());
    int g_D = INT_MAX;

    for (const Node &node : open_D) {
        int pr_D = pr(node, eps, dir);
        /* strictly smaller priority */
        if (pr_D < prmin_D) {
            opt_node = &node;
            prmin_D = pr_D;
            g_D = (dir == Direction::F) ? node.g_F : node.g_B;
        }
        /* equal priority but strictly smaller g_D */
        else if (pr_D == prmin_D) {
            if (dir == Direction::F && node.g_F < g_D) {
                opt_node = &node;
                g_D = node.g_F;    
            } else if (dir == Direction::B && node.g_B < g_D) {
                opt_node = &node;
                g_D = node.g_B;
            }
        }

        if (dir == Direction::F) {
            fmin_D = std::min(fmin_D, node.g_F + node.h_F);
            gmin_D = std::min(gmin_D, node.g_F);
        } else if (dir == Direction::B) {
            fmin_D = std::min(fmin_D, node.g_B + node.h_B);
            gmin_D = std::min(gmin_D, node.g_B);
        }
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
 * @param nodes_expanded To be set to the number of nodes expanded.
 * @return Optimal cost.
 */
int mme(const std::vector<int> &initial_state, const std::vector<int> &goal_state, int eps, int gap_x, int &nodes_expanded) {
    int U = INT_MAX;  // unsolvable
    NodeSet open_F;
    NodeSet open_B;
    NodeSet closed_F;
    NodeSet closed_B;
    open_F.emplace(initial_state, 0, 0, h(initial_state, Direction::F, gap_x), h(initial_state, Direction::B, gap_x), Direction::F);
    open_B.emplace(goal_state, 0, 0, 0, 0, Direction::B);
    nodes_expanded = 0;
    while (!open_F.empty() && !open_B.empty()) {
        int fmin_F, fmin_B, gmin_F, gmin_B, prmin_F, prmin_B;

        Node node_F = scan(open_F, eps, Direction::F, prmin_F, fmin_F, gmin_F);
        Node node_B = scan(open_B, eps, Direction::B, prmin_B, fmin_B, gmin_B);
        int C = std::min(prmin_F, prmin_B);
        if (U <= std::max(std::max(C, fmin_F), std::max(fmin_B, gmin_F + gmin_B + eps))) {
            return U;
        }
        if (C == prmin_F) {
            /* expand in the forward direction */
            Node &node = node_F;
            open_F.erase(node);
            closed_F.insert(node);
            NodeVector successors = expand(node, gap_x, nodes_expanded);
            for (Node &s_node : successors) {
                int old_s_node_g_F = INT_MAX;
                assert(!(open_F.count(s_node) > 0 && closed_F.count(s_node) > 0));
                if (open_F.count(s_node) > 0) {
                    old_s_node_g_F = (*open_F.find(s_node)).g_F;
                } else if (closed_F.count(s_node) > 0) {
                    old_s_node_g_F = (*closed_F.find(s_node)).g_F;
                }
                // if (... && g_F(node) + cost(node, s_node) >= g_F(s_node))
                if ((open_F.count(s_node) > 0 || closed_F.count(s_node) > 0) &&
                    (node.g_F + 1 >= old_s_node_g_F)) {  // assumes unit cost
                    continue;
                }
                // if (s_node in open_F U closed_F)
                if (open_F.count(s_node) > 0 || closed_F.count(s_node) > 0) {
                    open_F.erase(s_node);
                    closed_F.erase(s_node);
                }
                // g_F(s_node) <- g_F(node) + cost(node, s_node)
                s_node.g_F = node.g_F + 1;  // g_F does not affect hash/equality
                open_F.insert(s_node);
                if (open_B.count(s_node) > 0) {  // collision
                    s_node.g_B = (*open_B.find(s_node)).g_B;
                    U = std::min(U, s_node.g_F + s_node.g_B);
                }
            }
        } else {  // C == prmin_B
            /* expand in the backward direction */
            Node &node = node_B;
            open_B.erase(node);
            closed_B.insert(node);
            NodeVector successors = expand(node, gap_x, nodes_expanded);
            for (Node &s_node : successors) {
                int old_s_node_g_B = INT_MAX;
                assert(!(open_B.count(s_node) > 0 && closed_B.count(s_node) > 0));
                if (open_B.count(s_node) > 0) {
                    old_s_node_g_B = (*open_B.find(s_node)).g_B;
                } else if (closed_B.count(s_node) > 0) {
                    old_s_node_g_B = (*closed_B.find(s_node)).g_B;
                }
                // if (... && g_B(node) + cost(node, s_node) >= g_B(s_node))
                if ((open_B.count(s_node) > 0 || closed_B.count(s_node) > 0) &&
                    (node.g_B + 1 >= old_s_node_g_B)) {  // assumes unit cost
                    continue;
                }
                // if (s_node in open_B U closed_B)
                if (open_B.count(s_node) > 0 || closed_B.count(s_node) > 0) {
                    open_B.erase(s_node);
                    closed_B.erase(s_node);
                }
                // g_B(s_node) <- g_B(node) + cost(node, s_node)
                s_node.g_B = node.g_B + 1;  // g_B does not affect hash/equality
                open_B.insert(s_node);
                if (open_F.count(s_node) > 0) {  // collision
                    s_node.g_F = (*open_F.find(s_node)).g_F;
                    U = std::min(U, s_node.g_B + s_node.g_F);
                }
            }
        }
    }
    return U;
}