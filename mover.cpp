#include "mover.h"

double mover_sgn (double x)
{
  return x < 1e-5 ? -1 : x > 1e-5;
}

mover::mover(int steps, int pin1, int pin2, int pin3, int pin4, int rpms, double max_block_time) : steps_(steps), pin1_(pin1), pin2_(pin2), pin3_(pin3), pin4_(pin4), rpms_(rpms), max_block_time_(max_block_time), stepper_(Stepper(steps_, pin1_, pin2_, pin3_, pin4_))
{
  stepper_.setSpeed(rpms_);
  degrees_per_step_ = 360.0 / steps_;
  max_steps_per_move_ = (int)floor(max_block_time * rpms_ * steps_ / 60000.0); // steps_/rotation * rotations/min * 1 min / 60 s * 1 s / 1000 ms = steps / ms
  //  Serial.println("RPMS" + (String)rpms_);
}

double mover::get_min_turn_angle()
{
  return degrees_per_step_;
}

double mover::move_toward(double angle)
{
  //  Serial.println("angle" + (String)angle);
  //  Serial.println("degrees_per_step_" + (String)degrees_per_step_);
  int steps_to_move = (int)floor(angle / degrees_per_step_);
  //  Serial.println("Before" + (String)steps_to_move);
  double time_to_move = steps_to_move * 1 / (rpms_ * steps_ / .06); // 1/(steps_/rotation * rotations/min * 1min/60s * 1s/1000ms) = ms/step
  steps_to_move = (int)(mover_sgn(steps_to_move) * min(abs(steps_to_move), abs(max_steps_per_move_))); // account for negative angles
  //  Serial.println("After" + (String)steps_to_move);
  //  Serial.println("max_steps_per_move_" + (String)max_steps_per_move_);
  stepper_.step(steps_to_move);
  return steps_to_move * degrees_per_step_;
}

