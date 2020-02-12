#pragma once
#include <stdio.h>
#include <string>
#include <vector>

#include "params.h"
#include <math.h>

enum class R_Type { ENDS, DISTANCE }; // reinforcement type

class body {
public:
  body(double mass, double start_position, double end_position_1,
       double end_position_2, double start_velocity, double end_velocity_1,
       double end_velocity_2, std::string name, long num_positions,
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

  std::string name;
  double mass;

  /*Discretization values*/
  long mid_point;

  std::vector<double> positions;
  std::vector<double> velocities;

  double start_velocity;
  double end_velocity_1;
  double end_velocity_2;

  double start_position;
  double end_position_1;
  double end_position_2;
  long num_positions;
  long num_velocities;
};