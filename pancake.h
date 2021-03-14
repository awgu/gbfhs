/**
 * @file pancake.h
 * @brief Struct definitions and function prototypes relating to the n-pancake
 * problem.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug No known bugs.
 */

#pragma once

#include <vector>
#include <unordered_set>
#include <memory>
#include <iostream>

/** @brief x value for GAP-x heuristic */
// #define GAP_X (10)

/**
 * @brief Forward or backward direction.
 */
enum Direction {
    F, B
};

/**
 * @brief Node used in the search algorithm.
 */
struct Node {
    /** @brief state representing a pancake stack */
    std::vector<int> s;
    /** @brief cost of path from initial state; 0 if unused */
    int g_F;
    /** @brief cost of path from goal state; 0 if unused */
    int g_B;
    /** @brief heuristic cost to goal state */
    int h_F;
    /** @brief heuristic cost to initial state */
    int h_B;
    /** @brief direction that the node is visited from */
    Direction dir;

    /**
     * @brief Constructor.
     */
    Node(const std::vector<int> &s, int g_F, int g_B, int h_F, int h_B, Direction dir)
        : s(s), g_F(g_F), g_B(g_B), h_F(h_F), h_B(h_B), dir(dir)
    {}
};

/**
 * @brief Hash function for Node.
 */
struct NodeHash {
    /**
     * @brief Hash a node based only on its state s.
     */
    std::size_t operator()(const Node &node) const {
        std::size_t seed = node.s.size();
        for (int i : node.s) {
            seed ^= i + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

/**
 * @brief Equality function for Node.
 */
struct NodeEqual {
    /**
     * @brief Two nodes are equal iff their states s are the same.
     */
    bool operator()(const Node &node1, const Node &node2) const {
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
typedef std::unordered_set<Node,NodeHash,NodeEqual> NodeSet;
typedef std::vector<Node> NodeVector;

/* exported function prototypes */
void print_node(const Node &node);
void print_vector(const std::vector<int> &v);

bool is_solved(const std::vector<int> &s, const std::vector<int> &g);
std::vector<int> flip(const std::vector<int> &s, int k);
int h(const std::vector<int> &s, __attribute__((unused)) Direction dir, int gap_x);
NodeVector expand(const Node &node, int gap_x, int &nodes_expanded);