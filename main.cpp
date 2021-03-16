/**
 * @file main.cpp
 * @brief Main function for experiments.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug No known bugs.
 */

#include "gbfhs.h"
#include "mme.h"
#include "astar.h"

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
    int discount = 5;

    int gbfhs_nodes_expanded = 0;
    int mme_nodes_expanded = 0;
    int astar_nodes_expanded = 0;
    std::ofstream gbfhs_out;
    std::ofstream mme_out;
    std::ofstream astar_out;
    gbfhs_out.open("experiments/gbfhs_8puzzle_50_" + std::to_string(discount) + ".txt", std::ofstream::trunc);
    mme_out.open("experiments/mme_8puzzle_50_" + std::to_string(discount) + ".txt", std::ofstream::trunc);
    astar_out.open("experiments/astar_8puzzle_50_" + std::to_string(discount) + ".txt", std::ofstream::trunc);

    /* 10-pancake problem */
    gbfhs_nodes_expanded = 0;
    mme_nodes_expanded = 0;
    astar_nodes_expanded = 0;
    for (int i = 0; i < NUM_ITERS; ++i) {
        /* random initial state */
        std::vector<int> initial_state;
        std::vector<int> goal_state;
        for (int i = 0; i < BOARD_DIM * BOARD_DIM; ++i) {
            initial_state.push_back(i);
            goal_state.push_back(i);
        }
        while (true) {
            std::random_shuffle(initial_state.begin(), initial_state.end());
            if (get_num_inversions(initial_state) % 2 == 0) {
                break;
            }
        }
        
        int nodes_expanded = 0;
        int gbfhs_opt = gbfhs(initial_state, goal_state, eps, discount, nodes_expanded);
        gbfhs_nodes_expanded += nodes_expanded;
        gbfhs_out << nodes_expanded << std::endl;
        std::cout << "GBFHS opt: " << gbfhs_opt << std::endl;
        std::cout << "nodes expanded: " << nodes_expanded << std::endl;

        nodes_expanded = 0;
        int mme_opt = mme(initial_state, goal_state, eps, discount, nodes_expanded);
        mme_nodes_expanded += nodes_expanded;
        mme_out << nodes_expanded << std::endl;
        std::cout << "MMe opt: " << mme_opt << std::endl;
        std::cout << "nodes expanded: " << nodes_expanded << std::endl;

        nodes_expanded = 0;
        int astar_opt = astar(initial_state, goal_state, discount, nodes_expanded);
        astar_nodes_expanded += nodes_expanded;
        astar_out << nodes_expanded << std::endl;
        std::cout << "A* opt: " << astar_opt << std::endl;
        std::cout << "nodes expanded: " << nodes_expanded << std::endl;

        if (gbfhs_opt != mme_opt) {
            std::cout << "GBFHS optimal_cost: " << gbfhs_opt << std::endl;
            std::cout << "MMe optimal cost: " << mme_opt << std::endl;
            print_puzzle(initial_state);
            exit(-1);
        }
        
    }
    std::cout << "GBFHS avg nodes expanded: " << gbfhs_nodes_expanded / NUM_ITERS << std::endl;
    std::cout << "MMe avg nodes expanded: " << mme_nodes_expanded / NUM_ITERS << std::endl;
    std::cout << "A* avg nodes expanded: " << astar_nodes_expanded / NUM_ITERS << std::endl;
    std::cout << std::endl;

    gbfhs_out << std::endl;
    mme_out << std::endl;
    astar_out << std::endl;

    gbfhs_out.close();
    mme_out.close();
    astar_out.close();
    
    return 0;
}