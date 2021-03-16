/**
 * @file a_star.cpp
 * @brief A* implementation for the n-pancake problem.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug Incomplete.
 */

#include "a_star.h"

#include <queue>
#include <limits.h>

/**
 * @brief Node used in A*.
 * 
 * This is made separate from the Node struct defined in pancake.h since the
 * nodes in A* must track their own g and h values for storage in a priority
 * queue.
 */
struct AStarNode {
    /** @brief state representing a pancake stack */
    std::vector<int> s;
    /** @brief cost of path so far */
    int g;
    /** @brief heuristic cost */
    int h;

    /**
     * @brief Constructor.
     */
    AStarNode(const std::vector<int> &s, int g, int h)
        : s(s), g(g), h(h)
    {}
};

/**
 * @brief Comparison function for A* nodes.
 */
struct AStarNodeCompare {
    /**
     * @brief A node with lower f = g + h has higher priority.
     */
    bool operator()(const AStarNode &node1, const AStarNode &node2) {
        return node1.g + node1.h > node2.g + node2.h;
    }
};

/* typedef for convenience */
typedef std::priority_queue<AStarNode,std::vector<AStarNode>,AStarNodeCompare> PQ;

/**
 * @brief Runs the A* algorithm with the given initial and goal state.
 * 
 * @param initial_state Initial state.
 * @param goal_state Goal state.
 * @param eps Integer representing the minimum-cost operator in the domain,
 * i.e. the cheapest edge in the state space.
 * @param gap_x x for the GAP-x heuristic.
 * @param nodes_expanded To be set to the number of nodes expanded.
 * @return Optimal cost.
 */
int a_star(const std::vector<int> &initial_state, const std::vector<int> &goal_state, int gap_x, int &nodes_expanded) {

    const int n = initial_state.size() - 1;
    int cost = 0;
    PQ pq;
    pq.emplace(initial_state, 0, h(initial_state, Direction::F, gap_x));
    while (!pq.empty()) {
        nodes_expanded++;
        AStarNode node = pq.top();
        pq.pop();
        cost = node.g;
        if (is_solved(node.s, goal_state)) {
            return cost;
        }
        for (int i = 1; i < n; i++)
        {
            std::vector<int> child_i = flip(node.s,i);
            pq.emplace(child_i,node.g+1,h(child_i, Direction::F, gap_x));
        }
        /** TODO: expand node with highest priority */
    }

    return cost;
}