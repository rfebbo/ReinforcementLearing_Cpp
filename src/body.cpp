#include "body.h"

using namespace std;

body::body(body_params bp, string name) : bp(bp), name(name) { this->init(); }

bool body::update() {
  velocity += acceleration * TIMESTEP;
  position += velocity * TIMESTEP;

  if (position < bp.end_position_1 || position > bp.end_position_2)
    return true;

  if (position < bp.end_velocity_1 || position > bp.end_velocity_2)
    return true;
  return false;
}

void body::init() {
  mid_point = bp.num_positions / 2;
  max_sqrd = pow(bp.end_position_2, 2);

  double current_position = bp.end_position_1;
  position_delta =
      abs((bp.end_position_1 - bp.end_position_2)) / (bp.num_positions - 1);

  for (long i = 0; i < bp.num_positions; i++) {
    // printf("%s position %i = %f\n", name.c_str(), i, current_position);
    positions.push_back(current_position);
    current_position += position_delta;
  }

  position = bp.start_position;

  double current_velocity = bp.end_velocity_1;
  velocity_delta =
      abs((bp.end_velocity_1 - bp.end_velocity_2)) / (bp.num_velocities - 1);

  for (long i = 0; i < bp.num_velocities; i++) {
    // printf("%s velocity %i = %f\n", name.c_str(), i, current_velocity);
    velocities.push_back(current_velocity);
    current_velocity += velocity_delta;
  }

  velocity = bp.start_velocity;
}

double body::get_R(double p) {
  switch (bp.r_type) {
  case R_Type::ENDS: {
    if (p >= bp.end_position_2 || p <= bp.end_position_1)
      return -1;
    return 0;
    break;
  }
  case R_Type::DISTANCE: {
    if (p == positions[mid_point])
      return 0;

    double distance = p - positions[mid_point];

    return -(distance * distance);
    break;
  }
  case R_Type::POSITIVE_DISTANCE: {
    double rv = (max_sqrd - pow(p, 2)) / max_sqrd;
    return rv;
    break;
  }
  default:
    break;
  }

  fprintf(stderr, "INVALID R_Type\n");
  return 0;
}

// long body::get_R(double p) {
//   double min_distance;
//   long min_distance_idx = -1;
//   for (long i = 0; i < positions.size(); i++) {
//     double distance = abs(p - positions[i]);

//     if (min_distance_idx == -1 || distance < min_distance) {
//       min_distance_idx = i;
//       min_distance = distance;
//     }
//   }

//   return get_R(min_distance_idx);
// }

double body::get_R() { return get_R(this->position); }

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
  position = bp.start_position;
  velocity = 0;
  acceleration = 0;
}

void body::print_params(FILE *f) {
  fprintf(f,
          "%s:\n"
          "POSITION START %lf\n"
          "POSITION DELTA %lf(%i)\n"
          "POSITION_END_1 %lf\n"
          "POSITION_END_2 %lf\n"
          "VELOCITY START %lf\n"
          "VELOCITY DELTA %lf(%i)\n"
          "VELOCITY_END_1 %lf\n"
          "VELOCITY_END_2 %lf\n\n",
          name.c_str(), this->bp.start_position, this->position_delta,
          this->bp.num_positions, this->bp.end_position_1,
          this->bp.end_position_2, this->bp.start_velocity,
          this->velocity_delta, this->bp.num_velocities,
          this->bp.end_velocity_1, this->bp.end_velocity_2);
}