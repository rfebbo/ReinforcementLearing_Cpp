#pragma once
#include <stdio.h>
#include <string>
#include <vector>

#include "params.h"
#include <math.h>

class body {
public:
  body(body_params bp, std::string name);
  ~body(){};
  void init();
  double get_R(double p);
  double get_R();
  long get_state();
  long get_num_states();
  void set_acceleration(double a) { acceleration = a; }
  bool update();
  void reset();
  double get_velocity() { return velocity; }
  double get_position() { return position; }
  double get_mass() { return bp.mass; }
  double get_position_delta() { return position_delta; }
  double get_velocity_delta() { return velocity_delta; }
  void print_params(FILE *f);

protected:
  /*instantaneous values*/
  double acceleration; /*inst accel*/
  double velocity;     /*inst vel*/
  double position;     /*inst position*/

  /*Discretization values*/
  long mid_point;
  double position_delta{0};
  double velocity_delta{0};

  std::vector<double> positions;
  std::vector<double> velocities;

  std::string name;

  double max_sqrd;

  const body_params bp;
};