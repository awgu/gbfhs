/**
 * @file astar.cpp
 * @brief A* implementation for the n-puzzle problem.
 * 
 * There is a repeated code between this file and puzzle.cpp, the reason being
 * that this A* implementation prefers each node to store its g and h values
 * to be used in the priority queue priority computation (whereas previously
 * the g values were stored separately and the h values were computed on the
 * fly).
 * 
 * @author Andrew Gu (andrewg2)
 * @bug No known bugs.
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

/**
 * @brief Hash function for AStarNode.
 */
struct AStarNodeHash {
    /**
     * @brief Hash a node based only on its state s.
     */
    std::size_t operator()(const AStarNode &node) const {
        std::size_t seed = node.s.size();
        for (int i : node.s) {
            seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

/**
 * @brief Equality function for AStarNode.
 */
struct AStarNodeEqual {
    /**
     * @brief Two nodes are equal iff their states s are the same.
     */
    bool operator()(const AStarNode &node1, const AStarNode &node2) const {
        if (node1.s.size() != node2.s.size()) {
            return false;
        }
        int s_size = node1.s.size();
        for (int i = 0; i < s_size; ++i) {
            if (node1.s[i] != node2.s[i]) {
                return false;
            }
        }
        return true;
    }
};

/* typedef for convenience */
typedef std::priority_queue<AStarNode,std::vector<AStarNode>,AStarNodeCompare> PQ;
typedef std::vector<AStarNode> AStarNodeVector;

/**
 * @brief Expands the given node and returns its successors.
 * 
 * @param node Node to expand.
 * @param gs Goal state.
 * @param discount Used for degrading the heuristic.
 * @param nodes_expanded (output) Number of nodes expanded so far.
 * @return Vector of successor nodes.
 */
AStarNodeVector expand(const AStarNode &node, const std::vector<int> &gs, int discount, int &nodes_expanded) {
    nodes_expanded++;
    int row, col;
    get_pos(node.s, 0, row, col);
    AStarNodeVector successors;
    if (is_valid_up(row)) {
        std::vector<int> up_successor = make_move(node.s, Move::Up);
        successors.emplace_back(up_successor, node.g + 1, h(up_successor, gs, discount));
    }
    if (is_valid_down(row)) {
        std::vector<int> down_successor = make_move(node.s, Move::Down);
        successors.emplace_back(down_successor, node.g + 1, h(down_successor, gs, discount));
    }
    if (is_valid_left(col)) {
        std::vector<int> left_successor = make_move(node.s, Move::Left);
        successors.emplace_back(left_successor, node.g + 1, h(left_successor, gs, discount));
    }
    if (is_valid_right(col)) {
        std::vector<int> right_successor = make_move(node.s, Move::Right);
        successors.emplace_back(right_successor, node.g + 1, h(right_successor, gs, discount));
    }
    return successors;
}


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
int astar(const std::vector<int> &initial_state, const std::vector<int> &goal_state, int discount, int &nodes_expanded) {
    std::unordered_set<AStarNode,AStarNodeHash,AStarNodeEqual> visited;

    PQ pq;
    pq.emplace(initial_state, 0, h(initial_state, goal_state, discount));
    while (!pq.empty()) {
        AStarNode node = pq.top();
        pq.pop();
        visited.insert(node);
        if (is_solved(node.s, goal_state)) {
            return node.g;
        }
        
        AStarNodeVector successors = expand(node, goal_state, discount, nodes_expanded);
        for (const AStarNode & s_node : successors) {
            if (visited.count(s_node) == 0) {
                pq.push(s_node);
            }
        }
    }

    return INT_MAX;  // unsolvable
}