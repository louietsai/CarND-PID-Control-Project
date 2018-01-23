#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double twiddle_error_sum;
  double twiddle_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  double n_cte;
  double prev_cte;
  double diff_cte;
  double int_cte;

  double total_frame;

  double param[3];
  double dparam[3];
  double dparam_change_pattern[3];

  int param_index;
  int change_pattern_index;
  double best_err;
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
  void Init(double Kp, double Kd, double Ki);

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
