#include "env.h"

using namespace std;

Env::Env(env_params ep) : ep(ep) { this->init(); }

Env::~Env() {
  delete pole_body;
  delete cart_body;
}

void Env::reset_env() {
  cart_body->reset();
  pole_body->reset();
  done = false;
  force = 0;
  time = 0;
}

void Env::init() {
  // srand(time(NULL));
  srand(0);
  pole_body = new body(ep.p, "POLE", R_Type::DISTANCE);
  cart_body = new body(ep.c, "CART", R_Type::DISTANCE);
  num_states = pole_body->get_num_states() * cart_body->get_num_states();
  num_actions = 3;
  reset_env();
}

void Env::step(double input) {

  double p_a; /*pole acceleration*/
  double c_a; /*cart acceleration*/

  p_a = G * sin(pole_body->get_position());
  p_a += cos(pole_body->get_position()) *
         ((-input - pole_body->get_mass() * ep.pole_length *
                        pow(pole_body->get_velocity(), 2) *
                        sin(pole_body->get_position())) /
          (cart_body->get_mass() + pole_body->get_mass()));
  p_a /= ep.pole_length *
         ((4 / 3) -
          (pole_body->get_mass() * pow(cos(pole_body->get_position()), 2)) /
              (cart_body->get_mass() + pole_body->get_mass()));

  c_a = input;
  c_a += pole_body->get_mass() * ep.pole_length *
         ((pow(pole_body->get_velocity(), 2) * sin(pole_body->get_position())) -
          (p_a * cos(pole_body->get_position())));
  c_a /= (cart_body->get_mass() + pole_body->get_mass());

  pole_body->set_acceleration(p_a);
  cart_body->set_acceleration(c_a);

  done |= pole_body->update();
  done |= cart_body->update();

  time += TIMESTEP;
  force = input;
}

void Env::step(int action) {
  double new_force;

  if (action == 0)
    new_force = -ep.input_force;
  else if (action == 1)
    new_force = 0;
  else if (action == 2)
    new_force = ep.input_force;
  else
    fprintf(stderr, "INVALID ACTION\n");

  this->step(new_force);
}

long Env::get_state() {
  return pole_body->get_state() * cart_body->get_num_states() +
         cart_body->get_state();
}

long Env::get_reward() { return cart_body->get_R() + pole_body->get_R(); }