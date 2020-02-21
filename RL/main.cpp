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

long run_agent(env_params ep, rl_agent_params rlap, int run_num);

int main() {
  body_params cp; /*sim params for cart*/
  body_params pp; /*sim params for pole*/
  env_params ep;  /*sim params for environment*/
  rl_agent_params rlap;

  cp.r_type = R_Type::ENDS;
  cp.mass = 20;
  cp.start_position = 0;
  cp.start_velocity = 0;
  cp.num_positions = 35;
  cp.num_velocities = 35;
  cp.end_position_1 = -1;
  cp.end_position_2 = 1;
  cp.end_velocity_1 = -1;
  cp.end_velocity_2 = 1;

  pp.r_type = R_Type::ENDS;
  pp.mass = 10;
  pp.start_position = 0.01;
  pp.start_velocity = 0;
  pp.num_positions = 35;
  pp.num_velocities = 35;
  pp.end_position_1 = -M_PI_4;
  pp.end_position_2 = M_PI_4;
  pp.end_velocity_1 = -1;
  pp.end_velocity_2 = 1;

  ep.input_force = 250;
  ep.pole_length = 2;
  ep.c = cp;
  ep.p = pp;

  rlap.num_episodes = 1000;
  rlap.explore_start = 0.8;
  rlap.explore_end = 0.0;
  rlap.discount = 0.9;
  rlap.learning_rate = 0.2;

  FILE *f;
  f = fopen("./output5_Rtype_ENDS/state_res_vs_total_actions", "w");

  int run_num = 0;
  int start = 1;
  int end = 101;
  int res = 10;
  int total_runs = pow((start - end + 1) / res, 4);
  for (long i = start; i <= end; i += res) {
    for (long j = start; j <= end; j += res) {
      for (long k = start; k <= end; k += res) {
        for (long l = start; l <= end; l += res) {
          ep.p.num_positions = i;
          ep.p.num_velocities = j;
          ep.c.num_positions = k;
          ep.c.num_velocities = l;
          long total_actions = run_agent(ep, rlap, run_num);
          fprintf(f, "%i %i %i %i %ld\n", i, j, k, l, total_actions);
          run_num++;
          printf("finished run %i out of %i (%.2lf%%).\n", run_num, total_runs,
                 100.0 * (static_cast<double>(run_num) / total_runs));
        }
      }
    }
  }

  fclose(f);
}

long run_agent(env_params ep, rl_agent_params rlap, int run_num) {

  Env e = Env(ep);
  RL agent = RL(e.get_num_states(), e.get_num_actions(), rlap);
  Replay replay_agent = Replay();

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

    // if ((i + 1) % 100 == 0) {
    //   printf("Episode %i: avg: rand_actions: %.2lf, action: %.2lf, Time "
    //          "alive: %.2lf\n",
    //          i + 1, agent.get_avg_num_rand_actions(), agent.get_avg_action(),
    //          agent.get_avg_time_alive());
    //   agent.reset_averages();
    // }
  }

  FILE *f;
  std::string out_file_name;
  out_file_name = "./output5_Rtype_ENDS/RUN_";
  out_file_name.append(to_string(
      10000000 * (agent.get_q_size() /
                  static_cast<double>(agent.get_total_actions_taken()))));
  out_file_name.append("_");
  out_file_name.append(to_string(ep.p.num_positions));
  out_file_name.append("_");
  out_file_name.append(to_string(ep.p.num_velocities));
  out_file_name.append("_");
  out_file_name.append(to_string(ep.c.num_positions));
  out_file_name.append("_");
  out_file_name.append(to_string(ep.p.num_velocities));
  f = fopen(out_file_name.c_str(), "w");
  if (f == NULL) {
    fprintf(stderr, "UNABLE TO OPEN FILE\n");
  }

  replay_agent.set_replay(agent.get_best());
  replay_agent.reset();
  agent.print_params(f);
  e.pole_body->print_params(f);
  e.cart_body->print_params(f);
  fprintf(f, "GET_ACTION AVG CPU TIME: %.2lf ns\n",
          agent.get_tot_avg_cpu_time());
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

  fclose(f);

  return agent.get_total_actions_taken();
}