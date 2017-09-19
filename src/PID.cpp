#include "PID.h"
#include <iostream>
#include <cmath>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  iter_count = 0;
  twiddle_state = chill;
  pid_state = init;
  current_speed = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  if(twiddle_mode){
    twiddle_deltas[0] = Kp;
    twiddle_deltas[2] = Ki;
    twiddle_deltas[1] = Kd;
    for(int i = 0; i != coeff_count; i++){
      coefficients[i] = pow(10., -i-1);
    }
  }else{
    coefficients[0] = Kp;
    coefficients[2] = Ki;
    coefficients[1] = Kd;
    InitDeltas();
  }
  p_error = 0.;
  d_error = 0.;
  i_error = 0;
  sq_error = 0;
  prior_sq_error = 0;
  twiddle_state = ShouldChill() ? chill : first_pid;
  pid_state = init;
  twiddle_pre_pid = true;
  twiddle_current_param = 0;
  twiddle_best_err = -1;
}

void PID::UpdateCarState(double cte, double speed) {
  current_speed = speed;
  switch(pid_state){
    case init:
      if(twiddle_mode){
        p_error = 0;
        d_error = 0;
              i_error = 0;
        prior_sq_error = sq_error;
        sq_error = 0;
      }
      iter_count = 0;
      pid_state = add_err;
      if(debug) cout << "PID-INIT: " << coefficients[0] << " " << coefficients[1] << " " << coefficients[2] << endl;
    case add_err:
      d_error = p_error - cte;
      i_error += cte;
      p_error = cte;
      if(iter_count > pid_iterations/2)
        sq_error += cte * cte;
      if(++iter_count == pid_iterations){
        pid_state = init;
      }
      break;
  }

}
bool PID::ShouldChill(){
  double coeff_sum = 0.;
  for(int i = 0; i != coeff_count; i++){
    coeff_sum += fabs((long double) twiddle_deltas[i]);
  }
  bool result = coeff_sum <= twiddle_tolerance;
  if(result && iter_count == pid_iterations){
    result = sq_error <= prior_sq_error;
  }
  return result;
}

double PID::TotalError() {
  double result = twiddle_best_err;
  if(iter_count > pid_iterations/2){
    result = sq_error / (iter_count - pid_iterations/2);
  }
  if (debug) cout << "Total Err: " << result << endl;
  return result;
}

void PID::NextTwiddleParam() {
  twiddle_current_param++;
  if (twiddle_current_param == coeff_count) {
    twiddle_current_param = 0;
    twiddle_state = ShouldChill() ? chill : first_pid;
  } else {
    twiddle_state = first_pid;
  }
  if(twiddle_state == first_pid){
    if(debug) cout << "p[" << twiddle_current_param << "] += dp[" << twiddle_current_param << "]" << endl;
    coefficients[twiddle_current_param] +=  twiddle_deltas[twiddle_current_param];
  }
}

void PID::InitDeltas() {
  for (int i = 0; i != coeff_count; i++) {
    if (coefficients[twiddle_current_param] != 0.) {
      twiddle_deltas[i] = pow(
          10,
          (int) (log10(fabs((float) (coefficients[twiddle_current_param])))));
    } else {
      twiddle_deltas[i] = .0001;
    }
  }
}

void PID::Twiddle() {
//  if(current_speed < 10.) return;
  switch (twiddle_state) {
    case (chill):
      if (ShouldChill()) {
        break;
      } else {
        // init twiddler
        twiddle_current_param = 0;
        InitDeltas();
        twiddle_pre_pid = true;
        twiddle_state = first_pid;
        twiddle_best_err = -1;
      }
    case (first_pid):
      if (twiddle_pre_pid) {
        if (pid_state == init) {
          if(debug){
            cout << twiddle_state << twiddle_pre_pid << " ";
            cout << "Best Err: " << twiddle_best_err << endl;
          }
          if (twiddle_best_err < 0 || TotalError() < twiddle_best_err)
            twiddle_best_err = TotalError();
          if(debug) cout << "p[" << twiddle_current_param << "] += dp[" << twiddle_current_param << "]" << endl;
          coefficients[twiddle_current_param] +=  twiddle_deltas[twiddle_current_param];
          twiddle_pre_pid = false;
        }
      } else {
        if (pid_state == init) {
          if(debug){
            cout << twiddle_state << twiddle_pre_pid << " ";
            cout << "Best Err: " << twiddle_best_err << endl;
          }
          if (TotalError() < twiddle_best_err) {
            twiddle_best_err = TotalError();
            if(debug) cout << "dp[" << twiddle_current_param << "] *= 1.1" << endl;
            twiddle_deltas[twiddle_current_param] *= 1.1;
            NextTwiddleParam();

          } else {
            if(debug) cout << "p[" << twiddle_current_param << "] -= 2 * dp[" << twiddle_current_param << "]" << endl;
            coefficients[twiddle_current_param] -= 2. * twiddle_deltas[twiddle_current_param];
            twiddle_state = reverse_param_pid;
          }
        }
      }
    break;
    case (reverse_param_pid):
        if (pid_state == init) {
          if(debug){
            cout << twiddle_state << twiddle_pre_pid << " ";
            cout << "Best Err: " << twiddle_best_err << endl;
          }
          if (TotalError() < twiddle_best_err) {
            twiddle_best_err = TotalError();
            if(debug) cout << "dp[" << twiddle_current_param << "] *= 1.1" << endl;
            twiddle_deltas[twiddle_current_param] *= 1.1;
          } else {
            if(debug){
              cout << "p[" << twiddle_current_param << "] += dp[" << twiddle_current_param << "]" << endl;
              cout << "dp[" << twiddle_current_param << "] *= 0.9" << endl;
            }
            coefficients[twiddle_current_param] += twiddle_deltas[twiddle_current_param];
            twiddle_deltas[twiddle_current_param] *= .9;
          }
          NextTwiddleParam();
        }
  }
}

double PID::SteerAngle(){
  double result = -(coefficients[0] * p_error + coefficients[1] * d_error + coefficients[2] * i_error);
  if(fabs((float)result) > 1.){
    result = result > 0 ? 1. : -1.;
  }
  if(twiddle_mode) Twiddle();
  return result;
}
