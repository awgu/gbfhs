/**
 * @file pancake.cpp
 * @brief Implementation for functions relating to the n-pancake problem.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug No known bugs.
 */

#include "pancake.h"

#include <assert.h>
#include <limits.h>

/**
 * @brief Prints the contents of the given node.
 * 
 * The output format looks like:
 * Node:
 * s: ...
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
    if (node.dir == Direction::F) {
        std::cout << "\ndir: F" << std::endl;
    } else if (node.dir == Direction::B) {
        std::cout << "\ndir: B" << std::endl;
    }
}

/**
 * @brief Print the contents of a vector.
 * 
 * @param v Vector to print.
 * @return Void.
 */
void print_vector(const std::vector<int> &v) {
    int v_size = v.size();
    for (int i = 0; i < v_size; ++i) {
        std::cout << v[i] << " ";
    }
    std::cout << std::endl;
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
 * @brief Computes the heuristic for the given state and direction.
 * 
 * The current implementation is the GAP-x heuristic for both the forward and
 * backward directions.
 * 
 * @param s Vector representing the pancake stack.
 * @param dir F for forward; B for backward.
 * @param gap_x x for the GAP-x heuristic.
 * @return Heuristic value.
 */
int h(const std::vector<int> &s, Direction dir, int gap_x) {
    /* blind heuristic for backward direction */
    if (dir == Direction::B) {
        return 0;
    }
    int n = s.size() - 1;
    /* GAP-x heuristic */
    int gap = 0;
    for (int i = gap_x; i < n; ++i) {
        if (abs(s[i] - s[i+1]) > 1) {
            gap++;
        }
    }
    return gap;
}

/**
 * @brief Expands the given node.
 * 
 * @param node Node representing a state.
 * @param gap_x x for the GAP-x heuristic.
 * @param nodes_expanded (output) Number of nodes expanded so far.
 * @return Vector of all states one k-flip away from the given state.
 */
NodeVector expand(const Node &node, int gap_x, int &nodes_expanded) {
    nodes_expanded++;
    NodeVector successors;
    int n = node.s.size() - 1;
    if (node.dir == Direction::F) {
        for (int k = 1; k < n; ++k) {
            std::vector<int> s_flip = flip(node.s, k);
            // increment g_F (assumes unit cost)
            successors.emplace_back(s_flip, Direction::F);
        }
    } else if (node.dir == Direction::B) {
        for (int k = 1; k < n; ++k) {
            std::vector<int> s_flip = flip(node.s, k);
            // increment g_B (assumes unit cost)
            successors.emplace_back(s_flip, Direction::B);
        }
    }
    return successors;
}