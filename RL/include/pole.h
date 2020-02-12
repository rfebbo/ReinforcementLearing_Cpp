#pragma once
#include "body.h"
#include "params.h"

class pole : public body {
public:
  pole(R_Type r = R_Type::ENDS, double mass = POLE_MASS,
       double length = POLE_LENGTH, double start_position = POLE_START,
       double end_position_1 = END_ANGLE_1, double end_position_2 = END_ANGLE_2,
       double start_velocity = POLE_START_VEL,
       double end_velocity_1 = VEL_END_1, double end_velocity_2 = VEL_END_2,
       std::string name = "POLE", long num_positions = NUM_ANGLES,
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