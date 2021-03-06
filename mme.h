/**
 * @file mme.h
 * @brief Struct definitions and function interface for MMe.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug No known bugs.
 */

#pragma once

#include "puzzle.h"

/* exported function prototypes */
int mme(const std::vector<int> &initial_state, const std::vector<int> &goal_state, int eps, int gap_x, int &nodes_expanded);