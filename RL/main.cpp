#include <iostream>
#include <map>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

#include "RL.h"
#include "body.h"
#include "controller.h"
#include "env.h"
#include "params.h"
#include "replay.h"
#include <inttypes.h>

using namespace std;

class Euler : public controller {};

int main() {
  body_params cp; /*sim params for cart*/
  body_params pp; /*sim params for pole*/
  env_params ep;  /*sim params for environment*/
  rl_agent_params rlap;

  cp.mass = 20;
  cp.start_position = 0;
  cp.start_velocity = 0;
  cp.num_positions = 135;
  cp.num_velocities = 135;
  cp.end_position_1 = -1;
  cp.end_position_2 = 1;
  cp.end_velocity_1 = -1;
  cp.end_velocity_2 = 1;

  pp.mass = 10;
  pp.start_position = 0.01;
  pp.start_velocity = 0;
  pp.num_positions = 135;
  pp.num_velocities = 135;
  pp.end_position_1 = -M_PI_4;
  pp.end_position_2 = M_PI_4;
  pp.end_velocity_1 = -1;
  pp.end_velocity_2 = 1;

  ep.input_force = 250;
  ep.pole_length = 2;
  ep.c = cp;
  ep.p = pp;

  rlap.num_episodes = 10000;
  rlap.explore_start = 0.8;
  rlap.explore_end = 0.0;
  rlap.discount = 0.9;
  rlap.learning_rate = 0.2;

  Env e(ep);
  RL agent(e.get_num_states(), e.get_num_actions(), rlap);
  Replay replay_agent;
  FILE *f;
  f = fopen("test", "w");

  for (int i = 0; i < rlap.num_episodes; i++) {

    while (!e.is_done()) {
      long state_1 = e.get_state();
      int action = agent.get_action(state_1);

      e.step(action);

      agent.update_q(state_1, e.get_state(), action, e.get_reward(),
                     e.is_done());
    }

    e.reset_env();

    agent.new_episode();

    if ((i + 1) % 100 == 0) {
      printf("Episode %i: avg: rand_actions: %3.2lf, action: %4.2lf, Time "
             "alive: %2.2lf CPU time: %.2lf ns\n",
             i + 1, agent.get_avg_num_rand_actions(), agent.get_avg_action(),
             agent.get_avg_time_alive(), agent.get_avg_cpu_time());
      agent.reset_averages();
    }
  }

  replay_agent.set_replay(agent.get_best());
  replay_agent.reset();
  agent.print_params(f);
  e.pole_body->print_params(f);
  e.cart_body->print_params(f);
  printf("GET_ACTION CPU TIME: %i ms\n", agent.get_cpu_time());
  // std::cout << "time: " << agent.get_cpu_time();
  fprintf(f, "TIME: %lf\n", agent.get_best().size() * TIMESTEP);
  fprintf(f, "SIMULATION BEGIN\n");
  while (!e.is_done()) {
    double pole_position = e.pole_body->get_position();
    double cart_position = e.cart_body->get_position();
    double pole_velocity = e.pole_body->get_velocity();
    double cart_velocity = e.cart_body->get_velocity();
    fprintf(f, "%lf %lf %lf %lf %lf %lf\n", e.get_time(), pole_position,
            cart_position, pole_velocity, cart_velocity, e.get_force());

    int action = replay_agent.get_action();

    e.step(action);
  }
}