/**
 * @file main.cpp
 * @brief Main function for experiments.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug No known bugs.
 */

#include "gbfhs.h"
#include "mme.h"

#include <random>
#include <algorithm>
#include <assert.h>
#include <fstream>

/* number of iterations to average over */
#define NUM_ITERS (50)

/**
 * @brief Main function.
 */
int main() {
    std::srand(15780);  // set seed

    int eps = 1;

    int gbfhs_nodes_expanded = 0;
    int mme_nodes_expanded = 0;
    std::ofstream gbfhs_out;
    std::ofstream mme_out;
    gbfhs_out.open("experiments/gbfhs_gap_50.txt", std::ofstream::trunc);
    mme_out.open("experiments/mme_gap_50.txt", std::ofstream::trunc);

    /* 10-pancake problem */
    for (int gap_x = 0; gap_x <= 10; ++gap_x) {
        gbfhs_nodes_expanded = 0;
        mme_nodes_expanded = 0;
        std::cout << "GAP-" << gap_x << std::endl;
        for (int i = 0; i < NUM_ITERS; ++i) {
            /* random initial state */
            std::vector<int> initial_state = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
            std::random_shuffle(initial_state.begin(), initial_state.end());
            initial_state.push_back(11);  // plate is always last
            // print_vector(initial_state);

            std::vector<int> goal_state = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

            int nodes_expanded = 0;
            int gbfhs_opt = gbfhs(initial_state, goal_state, eps, gap_x, nodes_expanded);
            (void)gbfhs_opt;
            gbfhs_nodes_expanded += nodes_expanded;
            gbfhs_out << nodes_expanded << std::endl;
            std::cout << "GBFHS opt: " << gbfhs_opt << std::endl;
            std::cout << "nodes expanded: " << nodes_expanded << std::endl;

            nodes_expanded = 0;
            int mme_opt = mme(initial_state, goal_state, eps, gap_x, nodes_expanded);
            mme_nodes_expanded += nodes_expanded;
            mme_out << nodes_expanded << std::endl;
            std::cout << "MMe opt: " << mme_opt << std::endl;
            std::cout << "nodes expanded: " << nodes_expanded << std::endl;

            if (gbfhs_opt != mme_opt) {
                std::cout << "GBFHS optimal_cost: " << gbfhs_opt << std::endl;
                std::cout << "MMe optimal cost: " << mme_opt << std::endl;
                print_vector(initial_state);
                exit(-1);
            }
            
        }
        std::cout << "GBFHS avg nodes expanded: " << gbfhs_nodes_expanded / NUM_ITERS << std::endl;
        std::cout << "MMe avg nodes expanded: " << mme_nodes_expanded / NUM_ITERS << std::endl;
        std::cout << std::endl;

        gbfhs_out << std::endl;
        mme_out << std::endl;
    }

    gbfhs_out.close();
    mme_out.close();

    // // std::vector<int> initial_state = { 9, 7, 6, 10, 8, 3, 2, 1, 5, 4, 11 };
    // // std::vector<int> initial_state = { 9, 1, 8, 4, 2, 6, 7, 5, 3, 10, 11 };
    // // std::vector<int> initial_state = { 1, 8, 7, 2, 6, 10, 4, 5, 9, 3, 11 };
    // std::vector<int> initial_state = { 3, 10, 5, 4, 1, 7, 6, 2, 9, 8, 11 };
    // std::vector<int> goal_state = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
    // int nodes_expanded = 0;
    // int gap_x = 0;
    // int i = 0;
    // while (true) {
    //     int out = gbfhs(initial_state, goal_state, 1, gap_x, nodes_expanded);
    //     if (out == 10) {
    //         std::cout << i++ << " " << out << std::endl;
    //     }
    //     // std::cout << i << " " << mme(initial_state, goal_state, 1, gap_x, nodes_expanded) << std::endl;
    // }
    
    return 0;
}