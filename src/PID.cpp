#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

    //add somehing
    x = 0.0;
    y = 0.0;
    orientation = 0.0;
    Kp = Kp_;
    Kd = Kd_;
    Ki = Ki_;
    dp = {0.5, 0.01, 1.0};
    p = { Kp, Ki, Kd};

}

double PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

    if(errorVec.size() < 50)
        errorVec.push_back(cte);
    else
        errorVec.pop_front();
    cur_average_cte = std::accumulate(errorVec.begin(), errorVec.end(), 0)/errorVec.size();

    double diff_cte = cte - prev_cte;
    int_cte+=cte;
    prev_cte = cte;
    double steering = -Kp*cte - Kd* diff_cte  -Ki*int_cte;
    return steering;



}


void PID::Twiddle(double cte)
{
    //need some flags help the flow
    //twiddle addest the PID parameters in the beginning of 1000 datapoint of cte data
    static bool flag_change_parameter = true;
    static bool flag_compare_error = false;
    static int i = 0;

    double sum_dp = std::accumulate(dp.begin(), dp.end(), 0);

    int p_size = p.size();

    if(sum_dp > 0.2 && i <= 200 )
    {
    
     if(flag_change_parameter)
     {
        i %= p_size;
        p[i] += dp[i];
        // flag_change_parameter = false;
     }   
     //get updated error from new parameteres
     //cte should be the next updated average cte after the parametere change
     if(flag_compare_error)
     {
        if(cte < best_cte )
        {      
            best_cte = cte;
            dp[i]*= 1.1;
            // flag_compare_error = false;
            // flag_change_parameter = true;
        }
        else
        {
            p[i]-=2*dp[i];
            //get updated error for new parameters
            //set flag, need to get updated cte average
            if(cte < best_cte)
            {
                best_cte = cte;
                dp[i] *=1.1;

            }
            else
            {
                p[i] +=dp[i];
                dp[i] *=0.9;

            }

        }
     }
     flag_compare_error = true;
           


   }
   i++;

   Kp = p[0];
   Ki = p[1];
   Kd = p[2];


}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
    
  return int_cte;  // TODO: Add your total error calc here!
}
