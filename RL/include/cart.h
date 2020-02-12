#pragma once
#include "body.h"
#include "params.h"

class cart : public body {
public:
  cart(R_Type r = R_Type::ENDS, double mass = CART_MASS,
       double start_position = CART_START,
       double end_position_1 = END_POSITION_1,
       double end_position_2 = END_POSITION_2,
       double start_velocity = POLE_START_VEL,
       double end_velocity_1 = VEL_END_1, double end_velocity_2 = VEL_END_2,
       std::string name = "CART", long num_positions = NUM_POSITIONS,
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