#ifndef PID_H
#define PID_H

class PID {
public:
  /*
   * Tunabble constants
   */
   const double twiddle_tolerance = .2;
   const int pid_iterations = 16;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double sq_error;
  double prior_sq_error;
  double twiddle_best_err;

  /*
  * Coefficients
  */ 
//  double Kp;
//  double Ki;
//  double Kd;
  static const int coeff_count = 3;
  double twiddle_deltas[coeff_count];
  double coefficients[coeff_count];

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
  void UpdateCarState(double cte, double speed);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Calculate steering angle
   */
  double SteerAngle();

  enum twiddle_st {chill, first_pid, reverse_param_pid};

  enum pid_st {init, add_err};

  /*
   * internal state
   */
  twiddle_st twiddle_state;
  pid_st pid_state;

  int iter_count;

  int twiddle_current_param;
  bool twiddle_pre_pid;
  bool twiddle_mode = true;
  double current_speed;
  bool debug = true;

  bool ShouldChill();

 private:
  void Twiddle();
  void NextTwiddleParam();
  void InitDeltas();
};



#endif /* PID_H */
