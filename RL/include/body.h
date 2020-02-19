#pragma once
#include <stdio.h>
#include <string>
#include <vector>

#include "params.h"
#include <math.h>

enum class R_Type { ENDS, DISTANCE }; // reinforcement type

class body {
public:
  body(body_params bp, std::string name, R_Type r = R_Type::ENDS);
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
  double position_delta;
  double velocity_delta;

  std::vector<double> positions;
  std::vector<double> velocities;

  std::string name;

  const body_params bp;
};