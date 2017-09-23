#ifndef MOVER_H
#define MOVER_H

#define SPEED_OF_SOUND 343 // meters/second

#include "Arduino.h"
#include "Stepper.h"

class mover {
    int steps_;
    int pin1_;
    int pin2_;
    int pin3_;
    int pin4_;
    double max_block_time_; // milliseconds
    int max_steps_per_move_;
    double degrees_per_step_; // minimum turn distance
    double rpms_;
  public:
    Stepper stepper_;
    double move_toward(double); // takes angle in degrees relative to current facing, returns degrees moved
    double get_min_turn_angle();
    int set_rpms();
    mover(int steps, int pin1, int pin2, int pin3, int pin4, int rpms, double max_block_time);
};

#endif

