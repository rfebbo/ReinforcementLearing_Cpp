#pragma once
#include "controller.h"
#include "params.h"
#include <vector>

class Replay : public controller {
public:
  void init();
  int get_action() { return actions[curr_action++]; }
  void set_replay(std::vector<int> actions) { this->actions = actions; }
  void reset() { curr_action = 0; }
  double get_time() { return curr_action * TIMESTEP; }

private:
  long curr_action;
  std::vector<int> actions;
};