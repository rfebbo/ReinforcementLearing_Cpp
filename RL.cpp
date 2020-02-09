#include <iostream>
#include <map>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

/*equations of motion adopted from
 * https://coneural.org/florian/papers/05_cart_pole.pdf*/

using namespace std;

/*Hyperparameters*/
#define NUM_EPISODES 5000
#define EXPLORE_START 0.8
#define EXPLORE_END 0.0
#define PROB_RATE 0.1
#define DISCOUNT 0.4
#define LEARNING_RATE 0.1

/*Discretization Parameters*/
#define NUM_ANGLES 61
#define END_ANGLE_1 -M_PI_4
#define END_ANGLE_2 M_PI_4

#define NUM_POSITIONS 11
#define END_POSITION_1 -1
#define END_POSITION_2 1

#define NUM_VELOCITIES 5
#define VEL_END_1 -10
#define VEL_END_2 10

/*Simulation Parameters*/
#define INPUT_FORCE 250
#define TIMESTEP 0.01
#define TAU 1
#define G 9.81
/*--POLE*/
#define POLE_START 0.01 // radians, 0 = straight up
#define POLE_START_VEL 0.0
#define POLE_MASS 10
#define POLE_LENGTH 2
/*--CART*/
#define CART_START 0.0
#define CART_START_VEL 0.0
#define CART_MASS 20

void fast_sigmoid(double &value) {
  if (value < 0)
    value = (value / (1 - value));
  else
    value = (value / (1 + value));
}

enum class R_Type { ENDS, DISTANCE }; // reinforcement type

class body {
public:
  body(double mass, double start_position, double end_position_1,
       double end_position_2, double start_velocity, double end_velocity_1,
       double end_velocity_2, string name, long num_positions,
       long num_velocities, R_Type r = R_Type::ENDS);
  ~body(){};
  void init();
  R_Type r;
  long get_R(long i);
  long get_R(double p);
  long get_R();
  long get_state();
  long get_num_states();
  void set_acceleration(double a) { acceleration = a; }
  bool update();
  void reset();
  double get_velocity() { return velocity; }
  double get_position() { return position; }
  double get_mass() { return mass; }

protected:
  /*instantaneous values*/
  double acceleration; /*inst accel*/
  double velocity;     /*inst vel*/
  double position;     /*inst position*/

  string name;
  double mass;

  /*Discretization values*/
  long mid_point;

  vector<double> positions;
  vector<double> velocities;

  double start_velocity;
  double end_velocity_1;
  double end_velocity_2;

  double start_position;
  double end_position_1;
  double end_position_2;
  long num_positions;
  long num_velocities;
};

body::body(double mass, double start_position, double end_position_1,
           double end_position_2, double start_velocity, double end_velocity_1,
           double end_velocity_2, string name, long num_positions,
           long num_velocities, R_Type r)
    : mass(mass), start_position(start_position),
      end_position_1(end_position_1), end_position_2(end_position_2),
      start_velocity(start_velocity), end_velocity_1(end_velocity_1),
      end_velocity_2(end_velocity_2), name(name), num_positions(num_positions),
      num_velocities(num_velocities) {
  this->r = r;
  this->init();
}

bool body::update() {
  velocity += acceleration * TIMESTEP;
  position += velocity * TIMESTEP;

  if (position < end_position_1 || position > end_position_2)
    return true;

  if (position < end_velocity_1 || position > end_velocity_2)
    return true;
  return false;
}

void body::init() {
  mid_point = num_positions / 2;
  double current_position = end_position_1;
  double position_delta =
      abs((end_position_1 - end_position_2)) / (num_positions - 1);

  for (long i = 0; i < num_positions; i++) {
    // printf("%s position %i = %f\n", name.c_str(), i, current_position);
    positions.push_back(current_position);
    current_position += position_delta;
  }

  position = start_position;

  double current_velocity = end_velocity_1;
  double velocity_delta =
      abs((end_velocity_1 - end_velocity_2)) / (num_velocities - 1);

  for (long i = 0; i < num_velocities; i++) {
    // printf("%s velocity %i = %f\n", name.c_str(), i, current_velocity);
    velocities.push_back(current_velocity);
    current_velocity += velocity_delta;
  }

  velocity = start_velocity;
}

long body::get_R(long i) {
  switch (r) {
  case R_Type::ENDS: {
    if (i == positions.size() - 1 || i == 0)
      return -1;
    return 0;
    break;
  }
  case R_Type::DISTANCE: {
    if (i == mid_point)
      return 0;

    long distance = i - mid_point;

    return -(distance * distance);
    break;
  }
  default:
    break;
  }

  fprintf(stderr, "INVALID R_Type\n");
  return 0;
}

long body::get_R(double p) {
  double min_distance;
  long min_distance_idx = -1;
  for (long i = 0; i < positions.size(); i++) {
    double distance = abs(p - positions[i]);

    if (min_distance_idx == -1 || distance < min_distance) {
      min_distance_idx = i;
      min_distance = distance;
    }
  }

  return get_R(min_distance_idx);
}

long body::get_R() { return get_R(this->position); }

long body::get_state() {
  double min_distance;
  long min_vel_distance_idx = -1;
  long min_pos_distance_idx = -1;

  for (long i = 0; i < positions.size(); i++) {
    double distance = abs(position - positions[i]);

    if (min_pos_distance_idx == -1 || distance < min_distance) {
      min_pos_distance_idx = i;
      min_distance = distance;
    }
  }

  for (long i = 0; i < velocities.size(); i++) {
    double distance = abs(velocity - velocities[i]);

    if (min_vel_distance_idx == -1 || distance < min_distance) {
      min_vel_distance_idx = i;
      min_distance = distance;
    }
  }

  return min_pos_distance_idx * velocities.size() + min_vel_distance_idx;
}

long body::get_num_states() { return velocities.size() * positions.size(); }

void body::reset() {
  position = start_position;
  velocity = 0;
  acceleration = 0;
}

class pole : public body {
public:
  pole(R_Type r = R_Type::ENDS, double mass = POLE_MASS,
       double length = POLE_LENGTH, double start_position = POLE_START,
       double end_position_1 = END_ANGLE_1, double end_position_2 = END_ANGLE_2,
       double start_velocity = POLE_START_VEL,
       double end_velocity_1 = VEL_END_1, double end_velocity_2 = VEL_END_2,
       string name = "POLE", long num_positions = NUM_ANGLES,
       long num_velocities = NUM_VELOCITIES)
      : body{mass,
             start_position,
             end_position_1,
             end_position_2,
             start_velocity,
             end_velocity_1,
             end_velocity_2,
             name,
             num_positions,
             num_velocities,
             r},
        length{length} {}

  double length;

private:
};

class cart : public body {
public:
  cart(R_Type r = R_Type::ENDS, double mass = CART_MASS,
       double start_position = CART_START,
       double end_position_1 = END_POSITION_1,
       double end_position_2 = END_POSITION_2,
       double start_velocity = POLE_START_VEL,
       double end_velocity_1 = VEL_END_1, double end_velocity_2 = VEL_END_2,
       string name = "CART", long num_positions = NUM_POSITIONS,
       long num_velocities = NUM_VELOCITIES)
      : body{mass,
             start_position,
             end_position_1,
             end_position_2,
             start_velocity,
             end_velocity_1,
             end_velocity_2,
             name,
             num_positions,
             num_velocities,
             r} {}

private:
};

class Env {
public:
  void reset();
  void init(double r, long e);
  void step();
  bool is_done() { return done; }
  double get_force() { return force; }
  double get_time() { return time; }
  double get_avg_force(long e) { return (avg_force * tot_eps) / e; }
  double get_avg_time_alive(long e) { return (avg_time_alive * tot_eps) / e; }
  double get_avg_num_rand_actions(long e) {
    return ((avg_num_rand_actions * tot_eps) / e);
  }
  pole *pole_body;
  cart *cart_body;

private:
  vector<double> P; /*state transistion probability*/
  vector<double> Q; /*state value*/
  // vector<long> R;    /*reward*/
  long num_states;
  long num_actions;

  bool done;
  double force;
  double avg_time_alive;
  double avg_force;
  double explore;
  long num_steps;
  double avg_num_rand_actions;
  long tot_eps;
  double reduction;
  double time;
};

void Env::reset() {
  cart_body->reset();
  pole_body->reset();
  done = false;
  // avg_time_alive = 0;
  // avg_force = 0;
  // num_rand_actions = 0;
  num_steps = 0;
  force = 0;
  time = 0;
}

void Env::init(double r, long e) {
  // srand(time(NULL));
  srand(0);
  pole_body = new pole(R_Type::DISTANCE);
  cart_body = new cart(R_Type::ENDS);
  num_states = pole_body->get_num_states() * cart_body->get_num_states();
  num_actions = 3;
  reduction = r;
  tot_eps = e;
  explore = EXPLORE_START;
  reset();

  for (long i = 0; i < pow(num_states, 2) * num_actions; i++) {
    P.push_back(rand() / static_cast<double>(RAND_MAX));
    // printf("Q %i %f\n", i, Q.back());
  }

  for (long i = 0; i < num_states * num_actions; i++) {
    Q.push_back(rand() / static_cast<double>(RAND_MAX));
    // printf("Q %i %f\n", i, Q.back());
  }

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
  time += TIMESTEP;

  long action = -1;
  double randn = rand() / static_cast<double>(RAND_MAX);
  if (randn < explore) {
    avg_num_rand_actions += 1 / static_cast<double>(tot_eps);
    action = -2;
  }

  double input = 0.0;
  long state = pole_body->get_state() * cart_body->get_num_states() +
               cart_body->get_state();

  vector<long> next_states;
  next_states.resize(num_actions);

  for (long i = 0; i < num_actions; i++) {
    double max_p = -1;
    for (long j = 0; j < num_states; j++) {
      long idx = state * num_states * num_actions + j * num_actions + i;
      if (P[idx] > max_p) {
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

  if (action == -2) {
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

  force += (1 / TAU) * (input - force) * TIMESTEP;

  num_steps++;
  avg_force += force / tot_eps;
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

  if (!done) {
    double delta = LEARNING_RATE * (reward + DISCOUNT * max_q -
                                    Q[state * num_actions + action]);

    Q[state * num_actions + action] += delta;

    // fast_sigmoid(Q[state * num_actions + action]);
  } else {
    Q[state * num_actions + action] = reward;
  }

  avg_time_alive += TIMESTEP / tot_eps;

  if (explore > EXPLORE_END)
    explore -= reduction;
}

int main() {
  Env e;
  FILE *f;
  f = fopen("test", "w");

  // long num_eps = NUM_EPISODES;
  double reduction = (EXPLORE_START - EXPLORE_END) / NUM_EPISODES;
  printf("reduction: %lf\n", reduction);
  e.init(reduction, NUM_EPISODES);
  for (long i = 0; i < NUM_EPISODES; i++) {

    while (!e.is_done()) {
      // double cart_position = e.cart_body->get_position();
      // double pole_position = e.pole_body->get_position();
      // printf("%f %f\n", pole_position, cart_position);
      e.step();
    }

    if ((i + 1) % 100 == 0) {
      printf("Episode %i done. rand_actions: %.2lf avg force: %.2lf Time "
             "alive: %.2lf\n",
             i + 1, e.get_avg_num_rand_actions(i), e.get_avg_force(i),
             e.get_avg_time_alive(i));
    }
    e.reset();
  }

  while (!e.is_done()) {
    double cart_position = e.cart_body->get_position();
    double pole_position = e.pole_body->get_position();
    double cart_velocity = e.cart_body->get_velocity();
    double pole_velocity = e.pole_body->get_velocity();
    fprintf(f, "%f %f %f %f %f\n", e.get_time(), pole_position, cart_position,
            pole_velocity, cart_velocity);
    e.step();
  }
  // printf("Episode %i done. Time alive: %lf\n", 101, e.get_time_alive());

  // for (long i = 0; i < 100; i++) {
  //   // double input = 5 * cos(i * TIMESTEP * 0.5);
  //   double cart_position = e.cart_body->get_position();
  //   double pole_position = e.pole_body->get_position();
  //   printf("%f %f\n", pole_position, cart_position);
  //   e.step();
  // }
}