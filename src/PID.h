#ifndef PID_H
#define PID_H
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * The previous cte value.
  */
  double cte_last;
  bool initialized;
  int twiddle_max;
  int twiddle_counter;

  bool twiddle_past_skip;
  int twiddle_skip_first_max;
  int twiddle_skip_counter;

  double twiddle_error;
  double twiddle_best_error;

  std::vector<double> twiddle_p_values;
  std::vector<double> twiddle_dp_values;

  bool twiddle_done;

  int twiddle_current_index;

  bool twiddle_back_calc;

  double current_sum;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
