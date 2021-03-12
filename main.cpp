/**
 * @file main.cpp
 * @brief Main function for experiments.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug No known bugs.
 */

#include "gbfhs.h"

#include <random>
#include <algorithm>

/* number of iterations to average over */
#define NUM_ITERS (50)

/* set by each invokation to gbfhs() */
extern int nodes_expanded;

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
 * @brief Main function.
 */
int main() {
    std::srand(15780);  // set seed

    int total_nodes_expanded = 0;

    /* 10-pancake problem */
    for (int i = 0; i < NUM_ITERS; ++i) {
        /* random initial state */
        std::vector<int> initial_state = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
        std::random_shuffle(initial_state.begin(), initial_state.end());
        initial_state.push_back(11);  // plate is always last
        // print_vector(initial_state);

        std::vector<int> goal_state = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

        int optimal_cost = gbfhs(initial_state, goal_state, 1);
        std::cout << "optimal_cost: " << optimal_cost << std::endl;
        std::cout << std::endl;

        total_nodes_expanded += nodes_expanded;
    }

    std::cout << "average nodes expanded: " << total_nodes_expanded / NUM_ITERS << std::endl;

    return 0;
}