#ifndef PID_H
#define PID_H
#include <iostream>
#include <vector>
#include <math.h>

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
  * Need to keep track
  */
  double diff_cte;
  double previous_cte;
  double total_cte;


  /*
  * Twiddle variables
  */
  int frames;
  bool finished;
  bool restart;
  int nvar;
  int step;
  double err;
  double best_err;
  double threshold;
  double dt;
  double t_previous;
  //commented out - turn of twiddle for evaluation
  //std::vector<double> p{0, 0, 0};
  //std::vector<double> dp{1, 1, 1};
  std::vector<double> p{0.25, 3.0, 0.00001};
  std::vector<double> dp{0, 0, 0};
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
  void Init();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Return steering angle.
  */
  double CalcSteer(double cte, double speed);

  /*
  * Update PID parameters
  */
  void Twiddle();

};

#endif /* PID_H */
