#pragma once

#define G 9.81
#define TIMESTEP 0.01

enum class R_Type { ENDS, DISTANCE, POSITIVE_DISTANCE }; // reinforcement type

struct rl_agent_params {
  /*Hyperparameters*/
  int num_episodes;
  double explore_start;
  double explore_end;
  double discount;
  double learning_rate;
  double reward_incentive;
};

struct body_params {
  R_Type r_type;
  /*simulation parameters*/
  double mass;
  double start_position;
  double start_velocity;

  /*Discretization Parameters*/
  long num_positions;
  long num_velocities;

  double end_position_1; /*defines the start of the range of positions*/
  double end_position_2; /*defines the end of the range of positions*/

  double end_velocity_1; /*defines the start of the range of velocities*/
  double end_velocity_2; /*defines the end of the range of velocities*/
};

struct env_params {

  /*Simulation Parameters*/
  double input_force;
  double pole_length;

  body_params c;
  body_params p;
};