/**
 * @file astar.h
 * @brief 
 * 
 * @author Andrew Gu (andrewg2)
 * @bug No known bugs.
 */

#pragma once

#include "puzzle.h"

/* exported function prototypes */
int astar(const std::vector<int> &initial_state, const std::vector<int> &goal_state, int gap_x, int &nodes_expanded);