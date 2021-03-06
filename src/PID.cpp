#include "PID.h"
#include <vector>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;
  cte_last = 0.0;
  initialized = false;
  twiddle_max = 300;
  twiddle_counter = 0;
  twiddle_past_skip = false;
  twiddle_skip_first_max = 100;
  twiddle_skip_counter = 0;
  twiddle_error = 0;
  twiddle_best_error = 100000000000.0;

  // Start with first values at one given that it expects p[0] += dp[0]
  twiddle_p_values = {1.43433, 14.7539, 0.00620688};
  twiddle_dp_values = {1.0, 1.0, 0.1};
  Kp = twiddle_p_values[0];
  Kd = twiddle_p_values[1];
  Ki = twiddle_p_values[2];
  twiddle_done = false;

  twiddle_current_index = 0;

  twiddle_back_calc = false;

  current_sum = 0.0;

  use_i = false;
  use_d = false;
}

void PID::UpdateError(double cte) {
  if (!initialized) {
    cte_last = cte;
    initialized = true;
  }
  p_error = cte;
  double diff_cte;
  if (use_d) {
    d_error = cte - cte_last;
    cte_last = cte;
  } else {
    d_error = 0.0;
  }
  double int_cte;
  if (use_i) {
    i_error += cte;
  } else {
    i_error = 0;
  }
}

double PID::TotalError() {
  return - Kp * p_error - Kd * d_error - Ki * i_error;
}
