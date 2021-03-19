/**
 * @file puzzle.cpp
 * @brief Implementation of functions pertaining to the n-puzzle problem.
 * 
 * @author Andrew Gu (andrewg2)
 * @bug No known bugs.
 */

#include "puzzle.h"

#include <assert.h>

/**
 * @brief Prints the n-puzzle.
 * 
 * @param puzzle Row-major representation of the n-puzzle.
 * @return Void.
 */
void print_puzzle(const std::vector<int> &puzzle) {
    assert(puzzle.size() == BOARD_DIM * BOARD_DIM);
    for (int i = 0; i < BOARD_DIM; ++i) {
        for (int j = 0; j < BOARD_DIM; ++j) {
            std::cout << puzzle[i * BOARD_DIM + j] << " ";
        }
        std::cout << std::endl;
    }
}

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
    print_puzzle(node.s);
    if (node.dir == Direction::F) {
        std::cout << "\ndir: F" << std::endl;
    } else if (node.dir == Direction::B) {
        std::cout << "\ndir: B" << std::endl;
    }
}

/**
 * @brief Checks if the given puzzle is solved by comparing it against the
 * given goal state.
 * 
 * @param s Puzzle state to check.
 * @param g Goal state to check against.
 * @return True if the puzzle matches the goal; false otherwise.
 */
bool is_solved(const std::vector<int> &s, const std::vector<int> &g) {
    assert(s.size() == g.size());
    int s_size = s.size();
    assert(s_size == BOARD_DIM * BOARD_DIM);
    for (int i = 0; i < s_size; ++i) {
        if (s[i] != g[i]) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Gets the row corresponding to the given index based on the board
 * dimension.
 * 
 * @param i Index.
 * @return Row corresponding to i.
 */
int inline index_to_row(int i) {
    return i / BOARD_DIM;
}

/**
 * @brief Gets the column corresponding to the given index based on the board
 * dimension.
 * 
 * @param i Index.
 * @return Column corresponding to i.
 */
int inline index_to_col(int i) {
    return i % BOARD_DIM;
}

/**
 * @brief Gets the index corresponding to the given row and column based on
 * the board dimension.
 * 
 * @param row Row.
 * @param col Column.
 * @return Index corresponding to (row, col).
 */
int inline row_col_to_index(int row, int col) {
    return row * BOARD_DIM + col;
}

/**
 * @brief Gets the position of the square in the puzzle with the given value.
 * 
 * @param s Puzzle state.
 * @param val Value to look for.
 * @param row (output) Row of the empty square.
 * @param col (output) Column of the empty square.
 * @return True on success; false on failure.
 */
bool get_pos(const std::vector<int> &s, int val, int &row, int &col) {
    int s_size = s.size();
    assert(s_size == BOARD_DIM * BOARD_DIM);
    for (int i = 0; i < s_size; ++i) {
        if (s[i] == val) {
            row = index_to_row(i);
            col = index_to_col(i);
            return true;
        }
    }
    return false;
}

/**
 * @brief Checks if 'up' is a valid move.
 * 
 * @param row Row of the empty square.
 * @return True if 'up' is a valid move; false otherwise.
 */
bool is_valid_up(int row) {
    return row > 0;
}

/**
 * @brief Checks if 'down' is a valid move.
 * 
 * @param row Row of the empty square.
 * @return True if 'down' is a valid move; false otherwise.
 */
bool is_valid_down(int row) {
    return row < BOARD_DIM - 1;
}

/**
 * @brief Checks if 'left' is a valid move.
 * 
 * @param col Column of the empty square.
 * @return True if 'left' is a valid move; false otherwise.
 */
bool is_valid_left(int col) {
    return col > 0;
}

/**
 * @brief Checks if 'left' is a valid move.
 * 
 * @param col Column of the empty square.
 * @return True if 'left' is a valid move; false otherwise.
 */
bool is_valid_right(int col) {
    return col < BOARD_DIM - 1;
}

/**
 * @brief Performs the move on the given state.
 * 
 * @param s Puzzle state.
 * @param move Up, down, left, or right.
 * @return Vector representing the puzzle state after the move.
 */
std::vector<int> make_move(const std::vector<int> &s, Move move) {
    int row, col;
    get_pos(s, 0, row, col);
    int index = row_col_to_index(row, col);
    int swap_index = index;
    std::vector<int> s_copy(s.begin(), s.end());
    switch (move) {
        case Move::Up:
            assert(is_valid_up(row));
            swap_index = row_col_to_index(row - 1, col);
            break;
        case Move::Down:
            assert(is_valid_down(row));
            swap_index = row_col_to_index(row + 1, col);
            break;
        case Move::Left:
            assert(is_valid_left(col));
            swap_index = row_col_to_index(row, col - 1);
            break;
        case Move::Right:
            assert(is_valid_right(col));
            swap_index = row_col_to_index(row, col + 1);
            break;
        default:
            throw std::runtime_error("invalid move type");
    }
    std::swap(s_copy[index], s_copy[swap_index]);
    return s_copy;
}

/**
 * @brief Computes the l1 distance.
 * 
 * @param s_row Puzzle state row coordinate.
 * @param s_col Puzzle state column coordinate.
 * @param g_row Goal state row coordinate.
 * @param g_col Goal state column coordinate.
 * @return l1 distance between (s_row, s_col) and (g_row, g_col).
 */
int inline get_l1_dist(int s_row, int s_col, int g_row, int g_col) {
    return abs(s_row - g_row) + abs(s_col - g_col);
}

/**
 * @brief Computes the Manhattan distance between the given state and goal
 * state.
 * 
 * @param s Puzzle state.
 * @param g Goal state.
 * @param discount Normal h is multiplied by 1 / discount.
 * @return Heuristic value.
 */
int h(const std::vector<int> &s, const std::vector<int> &g, int discount) {
    assert(s.size() == g.size());
    assert(s.size() == BOARD_DIM * BOARD_DIM);
    int s_size = s.size();
    /* brute force */
    int h = 0;
    for (int i = std::max(1, discount); i < s_size; ++i) {
        int s_row, s_col, g_row, g_col;
        get_pos(s, i, s_row, s_col);
        get_pos(g, i, g_row, g_col);
        h += get_l1_dist(s_row, s_col, g_row, g_col);
    }
    return h;
}

/**
 * @brief Expands the given node and returns its successors.
 * 
 * @param node Node to expand.
 * @param nodes_expanded (output) Number of nodes expanded so far.
 * @return Vector of successor nodes.
 */
NodeVector expand(const Node &node, int &nodes_expanded) {
    nodes_expanded++;
    int row, col;
    get_pos(node.s, 0, row, col);
    NodeVector successors;
    if (is_valid_up(row)) {
        successors.emplace_back(make_move(node.s, Move::Up), node.dir);
    }
    if (is_valid_down(row)) {
        successors.emplace_back(make_move(node.s, Move::Down), node.dir);
    }
    if (is_valid_left(col)) {
        successors.emplace_back(make_move(node.s, Move::Left), node.dir);
    }
    if (is_valid_right(col)) {
        successors.emplace_back(make_move(node.s, Move::Right), node.dir);
    }
    return successors;
}

/**
 * @brief Gets the number of inversions in the given state.
 * 
 * 
 * @param s Puzzle state.
 * @return Number of inversions.
 */
int get_num_inversions(const std::vector<int> &s) {
    int num_inversions = 0;
    for (int i = 0; i < BOARD_DIM * BOARD_DIM - 1; ++i) {
        for (int j = i + 1; j < BOARD_DIM * BOARD_DIM; ++j) {
            if (s[i] != 0 && s[j] != 0 && s[i] > s[j]) {
                num_inversions++;
            }
        }
    }
    return num_inversions;
}