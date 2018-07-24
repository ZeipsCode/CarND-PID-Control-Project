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
  * Twiddle variables
  */

  std::vector<double> dp;
  int step, parameter;
  // number of steps to allow changes to settle, then to evaluate error
  int train_steps, eval_steps;
  double total_error, best_error;
  bool used_addition, used_subtraction, use_twiddle;

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

  /*
  * Convenience function for adding amount (dp) to a PID controller parameter based on index
  */
  void Update_params(int index, double value);
};

#endif /* PID_H */