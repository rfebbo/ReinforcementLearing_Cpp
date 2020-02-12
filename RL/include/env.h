#pragma once
#include <iostream>
#include <map>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

#include "body.h"
#include "cart.h"
#include "params.h"
#include "pole.h"

class Env {
public:
  void reset_averages();
  void new_episode();
  void init(double r, long e);
  void step();
  bool is_done() { return done; }
  double get_force() { return force; }
  double get_time() { return time; }
  double get_avg_force() { return avg_force; }
  double get_avg_time_alive() { return avg_time_alive; }
  double get_avg_num_rand_actions() { return avg_rand_actions; }
  double get_max_time_alive() { return max_time_alive; }
  pole *pole_body;
  cart *cart_body;

private:
  void reset_env();
  std::vector<double> P; /*state transistion probability*/
  std::vector<double> Q; /*state value*/
  // vector<long> R;    /*reward*/
  long num_states;
  long num_actions;

  /*per episode variables*/
  std::vector<double> current_run; /*force applied at each timestep*/
  bool done;
  double force;              /*force applied at current step*/
  double total_force;        /*force applied over the whole episode*/
  double total_rand_actions; /*number of random actions taken in this episode*/
  long num_steps;            /*number of steps in the episode*/
  double time;               /*running time of the episode*/

  double explore;   /*chance to take a random action*/
  double reduction; /*amount to reduce explore by after an episode*/
  long tot_eps;     /*total number of episodes*/

  /*best run*/
  std::vector<double> best_run;
  double max_time_alive; /*maximum recorded lifespan*/

  /*cumulative moving average vars*/
  double avg_force;
  double avg_time_alive;
  double avg_rand_actions;
  long eps_elapsed; /*episodes elapsed since last avg_reset*/
};
