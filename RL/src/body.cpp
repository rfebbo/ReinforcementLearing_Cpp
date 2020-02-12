#include "body.h"

using namespace std;

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
