#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <nao_behavior_tree/node.h>

bool navigate_left();

bool navigate_right();

bool navigate_up();

bool navigate_down();

void set_node_state(STATE state);

void reset_overwritten();

void reset_node_state();

void print_node_info();

#endif
