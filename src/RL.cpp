#include "RL.h"

using namespace std::chrono;

RL::RL(long n_states, int n_actions, rl_agent_params rlap)
    : num_states(n_states), num_actions(n_actions), p(rlap) {
  this->init();
}

void RL::init() {
  explore = p.explore_start;

  this->reduction = (p.explore_start - p.explore_end) / p.num_episodes;
  // printf("Q size: %i\n", num_states * num_actions);

  for (long i = 0; i < num_states * num_actions; i++) {
    this->Q.push_back(((rand() / static_cast<double>(RAND_MAX))));
    // printf("Q %i %f\n", i, Q.back());
  }

  cpu_time = nanoseconds::zero();
  total_cpu_time = nanoseconds::zero();
  total_eps_elapsed = 0;
  total_actions_taken = 0;
  max_time_alive = 0;
  min_reward = 0;
  min_reward_set = false;
}

void RL::new_episode() {
  /*compute averages*/
  eps_elapsed++;
  total_eps_elapsed++;

  avg_cpu_time += ((cpu_time.count() / num_steps) - avg_cpu_time) / eps_elapsed;
  avg_action += ((total_action / time) - avg_action) / eps_elapsed;
  avg_time_alive += (time - avg_time_alive) / eps_elapsed;
  avg_rand_actions +=
      ((total_rand_actions / time) - avg_rand_actions) / eps_elapsed;

  /*save best time*/
  if (time > max_time_alive) {
    max_time_alive = time;
    best_run = current_run;
  }

  /*decrement explore*/
  if (explore > p.explore_end)
    explore -= reduction;

  /*increment totals*/
  // total_actions_taken += num_steps;
  total_cpu_time += cpu_time;

  /*reset vars for next episode*/
  time = 0;
  num_steps = 0;
  cpu_time = nanoseconds::zero();
  total_action = 0;
  total_rand_actions = 0;
  current_run.clear();
}

void RL::reset_averages() {
  avg_cpu_time = 0;
  avg_action = 0;
  avg_rand_actions = 0;
  avg_time_alive = 0;
  eps_elapsed = 0;
}

int RL::get_action(long state) {

  auto start = steady_clock::now();
  int action = -1;
  total_actions_taken++;
  bool rand_action = false;
  double randn = rand() / static_cast<double>(RAND_MAX);

  if (randn < explore) {
    total_rand_actions++;
    rand_action = true;
  }

  bool found_actual_max_q = false;
  double actual_max_q;
  for (int i = 0; i < num_actions; i++) {
    int idx = state * num_actions + i;
    if (!found_actual_max_q || Q[idx] > actual_max_q) {
      actual_max_q = Q[idx];
      action = i;
      found_actual_max_q = true;
    }
  }

  if (rand_action) {
    action = rand() % 3;
  }

  num_steps++;
  total_action += action;
  time += TIMESTEP;
  current_run.push_back(action);

  auto end = steady_clock::now();

  auto duration = duration_cast<nanoseconds>(end - start);
  cpu_time += duration;

  return action;
}

void RL::update_q(long prev_state, long cur_state, long prev_action,
                  double reward, bool done) {

  bool found_actual_max_q = false;
  double max_q;
  for (int i = 0; i < num_actions; i++) {
    int idx = cur_state * num_actions + i;
    if (!found_actual_max_q || Q[idx] > max_q) {
      max_q = Q[idx];
      found_actual_max_q = true;
    }
  }

  // if (!done) {
  double delta = p.reward_incentive * reward;
  delta += p.discount * max_q;
  delta -= Q[prev_state * num_actions + prev_action];
  delta *= p.learning_rate;

  Q[prev_state * num_actions + prev_action] += delta;

  // } else {
  //   Q[prev_state * num_actions + prev_action] = p.reward_incentive * reward;
  // }

  if (!min_reward_set)
    min_reward = reward;
  else if (min_reward > reward)
    min_reward = reward;

  // normalize Q values
  // double sum = 0;
  // for (int i = 0; i < num_actions; i++) {
  //   sum += Q[prev_state * num_actions + i];
  // }

  // for (int i = 0; i < num_actions; i++) {
  //   Q[prev_state * num_actions + i] /= sum;
  // }
  // sum = 0;
  // for (int i = 0; i < num_actions; i++) {
  //   sum += Q[prev_state * num_actions + i];
  // }
  // if (sum > 1.000001)
  //   printf("normalize failed\n");
}

void RL::print_params(FILE *f) {

  fprintf(f,
          "NUM_EPISODES %i\n"
          "EXPLORE_START %lf\n"
          "EXPLORE_END %lf\n"
          "DISCOUNT %lf\n"
          "LEARNING_RATE %lf\n"
          "TOTAL ACTIONS TAKEN %i\n"
          "Q size(%i): %.2lf MB\n\n",
          this->p.num_episodes, this->p.explore_start, this->p.explore_end,
          this->p.discount, this->p.learning_rate, this->total_actions_taken,
          Q.size(), (static_cast<double>(Q.size()) * sizeof(double)) / 1048576);
}

void RL::print_Q(FILE *f) {
  for (long i = 0; i < num_states; i++) {
    fprintf(f, "%lf %lf %lf\n", Q[i * num_actions], Q[i * num_actions + 1],
            Q[i * num_actions + 2]);
  }
}