#include "PID.h"
#include <iostream>
#include <vector>
#include <numeric>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  // PID coefficients
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  // PID errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  // Cross track errors (cte)
  prev_cte = 0.0;
  best_error = 0.0;
  total_error = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cross tracker error (cte).
   */
  // P controller error
  p_error = cte;

  // I controller error
  i_error += cte;
  
  // D controller error
  d_error  = cte - prev_cte;
  prev_cte = cte;
  
  // Steering command angles from PID controller
  // steer_value = -Kp * cte - Kd * d_error - Ki * int_cte
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  // Sum of the PID controller errors
  return p_error * Kp + i_error * Ki + d_error * Kd;
//  total_error = Kp * p_error + Ki * i_error + Kd * d_error;
//  return total_error;
}
