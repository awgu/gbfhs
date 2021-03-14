/**
 * @file gbfhs.h
 * @brief Function interface for GBFHS.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug No known bugs.
 */

#pragma once

#include "pancake.h"

/* exported function prototypes */
int gbfhs(const std::vector<int> &initial_state, const std::vector<int> &goal_state, int eps, int gap_x, int &nodes_expanded);