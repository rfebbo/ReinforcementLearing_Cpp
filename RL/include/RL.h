#pragma once
#include "controller.h"
#include "params.h"
#include <chrono>
#include <string>
#include <vector>

class RL : public controller {
public:
  RL(long n_states, int n_actions, rl_agent_params rlap);
  ~RL() {}
  void new_episode();
  void reset_averages();
  int get_action(long state);
  std::vector<int> get_best() { return best_run; }
  void update_q(long prev_state, long cur_state, long prev_action, long reward,
                bool done);
  double get_time() { return time; }
  double get_avg_cpu_time() { return avg_cpu_time; }
  double get_avg_action() { return avg_action; }
  double get_avg_time_alive() { return avg_time_alive; }
  double get_avg_num_rand_actions() { return avg_rand_actions; }
  double get_max_time_alive() { return max_time_alive; }
  double get_tot_avg_cpu_time() {
    return total_cpu_time.count() / static_cast<double>(total_actions_taken);
  }
  long get_total_actions_taken() { return total_actions_taken; }
  size_t get_q_size() { return Q.size(); }
  void print_params(FILE *f);
  const rl_agent_params p;

private:
  void init();
  std::vector<double> Q; /*state value (s,a)*/
  long num_states{0};
  int num_actions{0};
  std::chrono::nanoseconds total_cpu_time;
  long total_eps_elapsed{0};
  long total_actions_taken{0};

  /*per episode variables*/
  std::vector<int> current_run; /*force applied at each timestep*/
  double total_action{0};       /*force applied over the whole episode*/
  double total_rand_actions{
      0};            /*number of random actions taken in this episode*/
  long num_steps{0}; /*number of steps in the episode*/
  double time{0};    /*running time of the episode*/
  std::chrono::nanoseconds cpu_time;

  double explore{0};   /*chance to take a random action*/
  double reduction{0}; /*amount to reduce explore by after an episode*/
  long tot_eps{0};     /*total number of episodes*/

  /*best run*/
  std::vector<int> best_run;
  double max_time_alive{0}; /*maximum recorded lifespan*/

  /*cumulative moving average vars*/
  double avg_cpu_time{0};
  double avg_action{0};
  double avg_time_alive{0};
  double avg_rand_actions{0};
  long eps_elapsed{0}; /*episodes elapsed since last avg_reset*/
};
