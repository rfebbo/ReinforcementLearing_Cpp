#pragma once
#include <iostream>
#include <map>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

#include "body.h"
#include "params.h"

/*equations of motion adopted from
 * https://coneural.org/florian/papers/05_cart_pole.pdf*/

class Env {
public:
  Env(env_params e);
  ~Env();
  void reset_env();
  void step(double force);
  void step(int action);
  long get_state();
  double get_reward();
  long get_num_states() { return num_states; }
  int get_num_actions() { return num_actions; }
  bool is_done() { return done; }
  double get_force() { return force; }
  double get_time() { return time; }
  body *pole_body;
  body *cart_body;
  const env_params ep;

private:
  void init();
  long num_states;
  int num_actions;

  double time;  /*running time of the episode*/
  double force; /*force applied at the current step*/
  bool done;
};
