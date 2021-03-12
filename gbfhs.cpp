/**
 * @file gbfhs.cpp
 * @brief GBFHS implementation for n-pancake problem.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug The current implementation assumes unit costs.
 */

/* includes */
#include "gbfhs.h"

#include <unordered_set>
#include <random>

#include <assert.h>
#include <limits.h>

/** @brief x value for GAP-x heuristic */
#define GAP_X (10)

/* typedef for convenience */
typedef std::unordered_set<Node,NodeHash,NodeEqual> NodeSet;

/* global variables */
int nodes_expanded = 0;

/* used for randomness */
std::random_device rd;
std::mt19937 gen(rd());

/**
 * @brief Prints the contents of the given node.
 * 
 * The output format looks like:
 * Node:
 * s: ...
 * g_F: ... g_B: ... h_F: ... h_B: ...
 * dir: ...
 * 
 * @param node Node to print.
 * @return Void.
 */
void print_node(const Node &node) {
    std::cout << "Node:" << std::endl << "s: ";
    for (int i : node.s) {
        std::cout << i << " ";
    }
    std::cout << "\ng_F: " << node.g_F << " g_B: " << node.g_B << " h_F: " << node.h_F << " h_B: " << node.h_B;
    if (node.dir == Direction::F) {
        std::cout << "\ndir: F" << std::endl;
    } else if (node.dir == Direction::B) {
        std::cout << "\ndir: B" << std::endl;
    }
}

/**
 * @brief Checks if the given state is solved by comparing against the goal
 * state.
 * 
 * @param s State to check.
 * @param g Goal state.
 * @return True if the state is solved; false otherwise.
 */
bool is_solved(const std::vector<int> &s, const std::vector<int> &g) {
    if (s.size() != g.size()) {
        return false;
    }
    int s_size = s.size();
    for (int i = 0; i < s_size; ++i) {
        if (s[i] != g[i]) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Computes the heuristic for the given state and direction.
 * 
 * The current implementation is the GAP-x heuristic for both the forward and
 * backward directions.
 * 
 * @param s Vector representing the pancake stack.
 * @param dir F for forward; B for backward.
 * @return Heuristic value.
 */
int h(const std::vector<int> &s, __attribute__((unused)) Direction dir) {
    int n = s.size() - 1;
    /* GAP-x heuristic */
    int gap = 0;
    for (int i = GAP_X; i < n; ++i) {
        if (abs(s[i] - s[i+1]) > 1) {
            gap++;
        }
    }
    return gap;
}

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
    int f_D, g_D;
    if (dir == Direction::F) {
        g_D = node.g_F;
        f_D = node.g_F + node.h_F;
    } else if (dir == Direction::B) {
        g_D = node.g_B;
        f_D = node.g_B + node.h_B;
    } else {
        return false;  // should never get here
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
    gLim_B = gLSum / 2;
    gLim_F = gLSum - gLim_B;
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
    std::advance(it, random_index);
    return *it;
}

/**
 * @brief Performs a k-flip on the given pancake stack.
 * 
 * @param s Pancake stack ordered from top to bottom.
 * @param k Index to flip.
 * @return Pancake stack after the flip.
 * @pre k is in [1, n - 1] where n is the number of pancakes.
 */
std::vector<int> flip(const std::vector<int> &s, int k) {
    int n = s.size() - 1;
    assert(k >= 1 && k < n);
    std::vector<int> s_flip(s.begin(), s.end());
    int i = 0;
    int j = k;
    while (i < j) {
        std::swap(s_flip[i++], s_flip[j--]);
    }
    return s_flip;
}

/**
 * @brief Expands the given node.
 * 
 * @param node Node representing a state.
 * @return Set of all states one k-flip away from the given state.
 */
NodeSet expand(const Node &node) {
    nodes_expanded++;
    NodeSet successors;
    int n = node.s.size() - 1;
    if (node.dir == Direction::F) {
        for (int k = 1; k < n; ++k) {
            std::vector<int> s_flip = flip(node.s, k);
            // increment g_F (assumes unit cost)
            successors.emplace(s_flip, node.g_F + 1, node.g_B, h(s_flip, Direction::F), h(s_flip, Direction::B), Direction::F);
        }
    } else if (node.dir == Direction::B) {
        for (int k = 1; k < n; ++k) {
            std::vector<int> s_flip = flip(node.s, k);
            // increment g_B (assumes unit cost)
            successors.emplace(s_flip, node.g_F, node.g_B + 1, h(s_flip, Direction::F), h(s_flip, Direction::B), Direction::B);
        }
    }
    return successors;
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
 * @return Void.
 */
void expand_level(int gLim_F, int gLim_B, int fLim, int &best, 
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
    while (expandable_F.size() > 0 || expandable_B.size() > 0) {
        Node node = pick(expandable_F, expandable_B);
        Direction dir = node.dir;
        if (dir == Direction::F) {
            expandable_F.erase(node);
            open_F.erase(node);
            closed_F.insert(node);
            NodeSet successors = expand(node);
            for (const Node &s_node : successors) {
                // if (... && g_F(node) + cost(node, s_node) >= g_F(s_node))
                if ((open_F.count(s_node) > 0 || closed_F.count(s_node) > 0) &&
                    (node.g_F + 1 >= s_node.g_F)) {
                    continue;
                }
                // if (s_node in open_F U closed_F)
                if (open_F.count(s_node) > 0 || closed_F.count(s_node) > 0) {
                    open_F.erase(s_node);
                    closed_F.erase(s_node);
                }
                // g_F(s_node) <- g_F(node) + cost(node, s_node)
                const_cast<Node &>(s_node).g_F = node.g_F + 1;  // g_F does not affect hash/equality
                open_F.insert(s_node);
                if (is_expandable(s_node, Direction::F, fLim, gLim_F)) {
                    expandable_F.insert(s_node);
                }
                if (open_B.count(s_node) > 0) {  // collision
                    const_cast<Node &>(s_node).g_B = std::max(s_node.g_B, (*open_B.find(s_node)).g_B);
                    best = std::min(best, s_node.g_F + s_node.g_B);
                    if (best <= fLim) {
                        return;
                    }
                }
            }
        } 
        else if (dir == Direction::B) {  // symmetric
            expandable_B.erase(node);
            open_B.erase(node);
            closed_B.insert(node);
            NodeSet successors = expand(node);
            for (const Node &s_node : successors) {
                // if (... && g_B(node) + cost(node, s_node) >= g_B(s_node))
                if ((open_B.count(s_node) > 0 || closed_B.count(s_node) > 0) &&
                    (node.g_F + 1 >= s_node.g_F)) {
                    continue;
                }
                // if (s_node in open_B U closed_B)
                if (open_B.count(s_node) > 0 || closed_B.count(s_node) > 0) {
                    open_B.erase(s_node);
                    closed_B.erase(s_node);
                }
                // g_B(s_node) <- g_B(node) + cost(node, s_node)
                const_cast<Node &>(s_node).g_B = node.g_B + 1;  // g_B does not affect hash/equality
                open_B.insert(s_node);
                if (is_expandable(s_node, Direction::B, fLim, gLim_B)) {
                    expandable_B.insert(s_node);
                }
                if (open_F.count(s_node) > 0) {  // collision
                    const_cast<Node &>(s_node).g_F = std::max(s_node.g_F, (*open_F.find(s_node)).g_F);
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
 */
int gbfhs(const std::vector<int> &initial_state, const std::vector<int> &goal_state, int eps) {
    if (is_solved(initial_state, goal_state)) {
        return 0;
    }
    int best = INT_MAX;  // unsolvable
    NodeSet open_F;
    NodeSet open_B;
    NodeSet closed_F;
    NodeSet closed_B;
    open_F.emplace(initial_state, 0, 0, h(initial_state, Direction::F), h(initial_state, Direction::B), Direction::F);
    open_B.emplace(goal_state, 0, 0, 0, 0, Direction::B);
    nodes_expanded = 0;
    int fLim = std::max(std::max(h(initial_state, Direction::F), h(goal_state, Direction::B)), eps);
    int gLim_F, gLim_B;
    while (!open_F.empty() && !open_B.empty()) {
        if (best == fLim) {
            // std::cout << "nodes expanded: " << nodes_expanded << std::endl;
            return best;
        }
        int gLSum = fLim - eps + 1;
        split(gLSum, gLim_F, gLim_B);
        expand_level(gLim_F, gLim_B, fLim, best, open_F, open_B, closed_F, closed_B);
        if (best == fLim) {
            // std::cout << "nodes expanded: " << nodes_expanded << std::endl;
            return best;
        }
        fLim++;
    }
    // std::cout << "nodes expanded: " << nodes_expanded << std::endl;
    return best;
}








