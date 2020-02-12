#include "env.h"

using namespace std;

void Env::reset_env() {
  total_force = 0;
  total_rand_actions = 0;
  cart_body->reset();
  pole_body->reset();
  current_run.clear();
  done = false;
  num_steps = 0;
  force = 0;
  time = 0;
}

void Env::new_episode() {
  eps_elapsed++;

  avg_force += (total_force - avg_force) / eps_elapsed;
  avg_time_alive += (time - avg_time_alive) / eps_elapsed;
  avg_rand_actions += (total_rand_actions - avg_rand_actions) / eps_elapsed;

  if (time > max_time_alive) {
    max_time_alive = time;
    best_run = current_run;
  }

  if (explore > EXPLORE_END)
    explore -= reduction;

  reset_env();
}

void Env::reset_averages() {
  avg_force = 0;
  avg_rand_actions = 0;
  avg_time_alive = 0;
  eps_elapsed = 0;
}

void Env::init(double r, long e) {
  // srand(time(NULL));
  srand(0);
  pole_body = new pole(R_Type::DISTANCE);
  cart_body = new cart(R_Type::ENDS);
  num_states = pole_body->get_num_states() * cart_body->get_num_states();
  num_actions = 3;
  eps_elapsed = 0;
  max_time_alive = 0;
  reduction = r;
  tot_eps = e;
  explore = EXPLORE_START;
  reset_env();
  reset_averages();

  for (long i = 0; i < pow(num_states, 2) * num_actions; i++) {
    P.push_back(rand() / static_cast<double>(RAND_MAX));
    // printf("Q %i %f\n", i, Q.back());
  }

  for (long i = 0; i < num_states * num_actions; i++) {
    Q.push_back(rand() / static_cast<double>(RAND_MAX));
    // printf("Q %i %f\n", i, Q.back());
  }

  printf("Q size: %.2lf MB\n",
         (static_cast<double>(Q.size()) * sizeof(double)) / 1048576);

  printf("P size: %.2lf MB\n",
         (static_cast<double>(P.size()) * sizeof(double)) / 1048576);
  // for (long i = 0; i < pole_body->positions.size(); i++) {
  //   R.push_back(pole_body->get_R(i));
  //   // printf("pole position: %i R: %i\n", i, pole_body->get_R(i));
  // }

  // for (long i = 0; i < cart_body->positions.size(); i++) {
  //   R.push_back(cart_body->get_R(i));
  //   // printf("cart position: %i R: %i\n", i, cart_body->get_R(i));
  // }
}

void Env::step() {

  long action = -1;
  bool rand_action = false;
  double randn = rand() / static_cast<double>(RAND_MAX);
  if (randn < explore) {
    total_rand_actions++;
    rand_action = true;
  }

  double input = 0.0;
  long state = pole_body->get_state() * cart_body->get_num_states() +
               cart_body->get_state();

  vector<long> next_states;
  next_states.resize(num_actions);

  for (long i = 0; i < num_actions; i++) {
    double max_p = -1;
    int found_P = 0;
    for (long j = 0; j < num_states; j++) {
      long idx = state * num_states * num_actions + j * num_actions + i;
      if (!found_P || P[idx] > max_p) {
        max_p = P[idx];
        next_states[i] = j; /*'j' is the most likely
                         next state given 'state' and action 'i'*/
      }
    }
  }

  /*find the max next state*/

  long m_q_found = 0;
  double max_q = -10;
  for (long i = 0; i < next_states.size(); i++) {
    if (Q[next_states[i] * num_actions + i] > max_q || m_q_found == 0) {
      // printf("%lf\n", Q.at(next_states[i] * num_actions + i));
      action = i;
      m_q_found = 1;
      max_q = Q[next_states[i] * num_actions + i];
    }
  }

  // printf("action : %i\n", action);

  if (rand_action) {
    action = rand() % 3;
  }

  if (action == 0)
    input = -INPUT_FORCE;
  else if (action == 1)
    input = 0;
  else if (action == 2)
    input = INPUT_FORCE;
  else
    fprintf(stderr, "INVALID ACTION\n");

  // force += (1 / TAU) * (input - force) * TIMESTEP;
  force = input;

  // printf("force %lf\n", force);

  double p_a; /*pole acceleration*/
  double c_a; /*cart acceleration*/

  p_a = G * sin(pole_body->get_position());
  p_a += cos(pole_body->get_position()) *
         ((-force - pole_body->get_mass() * pole_body->length *
                        pow(pole_body->get_velocity(), 2) *
                        sin(pole_body->get_position())) /
          (cart_body->get_mass() + pole_body->get_mass()));
  p_a /= pole_body->length *
         ((4 / 3) -
          (pole_body->get_mass() * pow(cos(pole_body->get_position()), 2)) /
              (cart_body->get_mass() + pole_body->get_mass()));

  c_a = force;
  c_a += pole_body->get_mass() * pole_body->length *
         ((pow(pole_body->get_velocity(), 2) * sin(pole_body->get_position())) -
          (p_a * cos(pole_body->get_position())));
  c_a /= (cart_body->get_mass() + pole_body->get_mass());

  pole_body->set_acceleration(p_a);
  cart_body->set_acceleration(c_a);

  done |= pole_body->update();
  done |= cart_body->update();

  long new_state = pole_body->get_state() * cart_body->get_num_states() +
                   cart_body->get_state();

  long predicted_idx = state * num_states * num_actions +
                       next_states[action] * num_actions + action;
  long actual_idx =
      state * num_states * num_actions + new_state * num_actions + action;

  if (predicted_idx == actual_idx) {
    P[predicted_idx] += PROB_RATE;
    // if (P[predicted_idx] > 1)
    //   P[predicted_idx] = 1;
  } else {
    P[predicted_idx] -= PROB_RATE;
    // if (P[predicted_idx] < 0)
    //   P[predicted_idx] = 0;

    P[actual_idx] += PROB_RATE;
    // if (P[actual_idx] > 1)
    //   P[actual_idx] = 1;
  }

  long reward = cart_body->get_R() + pole_body->get_R();

  bool found_actual_max_q = false;
  double actual_max_q;
  for (int i = 0; i < num_actions; i++) {
    int idx = new_state * num_actions + i;
    if (!found_actual_max_q || Q[idx] > actual_max_q)
      actual_max_q = Q[idx];
  }

  if (!done) {
    double delta = LEARNING_RATE * (reward + DISCOUNT * actual_max_q -
                                    Q[state * num_actions + action]);

    Q[state * num_actions + action] += delta;

    // fast_sigmoid(Q[state * num_actions + action]);
  } else {
    Q[state * num_actions + action] = reward;
  }

  num_steps++;
  time += TIMESTEP;
  total_force += force;
  current_run.push_back(force);
}