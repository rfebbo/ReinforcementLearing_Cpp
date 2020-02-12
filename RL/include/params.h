#pragma once
/*Hyperparameters*/
#define NUM_EPISODES 200
#define EXPLORE_START 0.9
#define EXPLORE_END 0.0
#define PROB_RATE 0.1
#define DISCOUNT 0.8
#define LEARNING_RATE 0.2

/*Discretization Parameters*/
#define NUM_ANGLES 21
#define END_ANGLE_1 -M_PI_4
#define END_ANGLE_2 M_PI_4

#define NUM_POSITIONS 11
#define END_POSITION_1 -1
#define END_POSITION_2 1

#define NUM_VELOCITIES 7
#define VEL_END_1 -1
#define VEL_END_2 1

/*Simulation Parameters*/
#define INPUT_FORCE 250
#define TIMESTEP 0.01
#define TAU 1
#define G 9.81
/*--POLE*/
#define POLE_START 0.01 // radians, 0 = straight up
#define POLE_START_VEL 0.0
#define POLE_MASS 10
#define POLE_LENGTH 2
/*--CART*/
#define CART_START 0.0
#define CART_START_VEL 0.0
#define CART_MASS 20