#include <iostream>
#include <map>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

#include "body.h"
#include "cart.h"
#include "env.h"
#include "params.h"
#include "pole.h"

/*equations of motion adopted from
 * https://coneural.org/florian/papers/05_cart_pole.pdf*/

using namespace std;

void fast_sigmoid(double &value) {
  if (value < 0)
    value = (value / (1 - value));
  else
    value = (value / (1 + value));
}

class controller {};

class RL : public controller {};

class Replay : public controller {};

class Euler : public controller {};

int main() {
  Env e;
  FILE *f;
  f = fopen("test", "w");

  double reduction = (EXPLORE_START - EXPLORE_END) / NUM_EPISODES;
  printf("reduction: %lf\n", reduction);
  e.init(reduction, NUM_EPISODES);
  for (long i = 0; i < NUM_EPISODES; i++) {

    while (!e.is_done()) {
      e.step();
    }

    e.new_episode();

    if ((i + 1) % 10 == 0) {
      printf("Episode %i done. avg: rand_actions: %3.2lf, force: %4.2lf, Time "
             "alive: %2.2lf\n",
             i + 1, e.get_avg_num_rand_actions(), e.get_avg_force(),
             e.get_avg_time_alive());
      e.reset_averages();
    }
  }

  while (!e.is_done()) {
    double cart_position = e.cart_body->get_position();
    double pole_position = e.pole_body->get_position();
    double cart_velocity = e.cart_body->get_velocity();
    double pole_velocity = e.pole_body->get_velocity();
    double force = e.get_force();
    fprintf(f, "%lf %lf %lf %lf %lf %lf\n", e.get_time(), pole_position,
            cart_position, pole_velocity, cart_velocity, force);
    e.step();
  }

  printf("max time alive: %lf\n", e.get_max_time_alive());
}