#ifndef PID_H
#define PID_H
#include <vector>
#include <numeric>
#include <deque>

class PID {
 public:
  /**
   * Constructor
   */ 
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  double UpdateError(double cte);


  void Twiddle(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  double x;
  double y;
  double orientation;
  double length;
  double steering_noise;
  double distance_noise;
  double steering_drift;

  double prev_cte = 0;
  double int_cte = 0;
  double best_cte = 990;
  std::vector<double> dp ;

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  std::deque<double> errorVec;
  std::vector<double> p;
  double cur_average_cte;
};

#endif  // PID_H
