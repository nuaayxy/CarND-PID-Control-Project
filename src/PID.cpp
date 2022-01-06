#include "PID.h"
#include "thread"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

    Kp = Kp_;
    Kd = Kd_;
    Ki = Ki_;
    dp = {0.05, 0.001, 1.0};
    p = { Kp, Ki, Kd};
    twiddleThread = std::thread(&PID::Twiddle, this);

    

}

double PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

    if(errorVec.size() < 10)
    {
        errorVec.push_back(cte);
        // std::cout<<"push back " <<std::endl;
    }
    else
    {
        errorVec.pop_front();
        // std::cout<<"pop front " <<std::endl;
    }
    cur_average_cte = std::accumulate(errorVec.begin(), errorVec.end(), cur_average_cte);
    cur_average_cte /= 10.0; 
    std::cout<<cur_average_cte<<std::endl;

    double diff_cte = cte - prev_cte;
    int_cte+=cte;
    prev_cte = cte;
    double steering = -Kp*cte - Kd* diff_cte  -Ki*int_cte;
    return steering;



}


void PID::Twiddle()
{
    //need some flags help the flow
    //twiddle addest the PID parameters in the beginning of 1000 datapoint of cte data
    static int i = 0;

    double sum_dp = std::accumulate(dp.begin(), dp.end(), sum_dp);

    int p_size = p.size();

    //only twiddle in the beginning and once paramters change small we stop in a optimal point
     while(sum_dp > 0.002 )
    {
        if(start)
        {
        i %= p_size;
        p[i] += dp[i];

        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
        std::this_thread::sleep_for( std::chrono::milliseconds( 150 ) );
        // flag_change_parameter = false;
     //get updated error from new parameteres
     //cte should be the next updated average cte after the parametere change

        if(GetError() < best_cte )
        {      
            best_cte = GetError();
            dp[i]*= 1.3;


        }
        else
        {
            p[i]-=2*dp[i];
            //get updated error for new parameters
            //set flag, need to get updated cte average
            std::this_thread::sleep_for( std::chrono::milliseconds( 150 ) );
            if(GetError() < best_cte)
            {
                best_cte = GetError();
                dp[i] *=1.3;

            }
            else
            {
                p[i] +=dp[i];
                dp[i] *=0.7;
                Kp = p[0];
                Ki = p[1];
                Kd = p[2];

            }

        }
        i++;
     }

   
   
   sum_dp = 0;
   sum_dp = std::accumulate(dp.begin(), dp.end(), sum_dp);



    }
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
    
  return int_cte;  // TODO: Add your total error calc here!
}

double PID::GetError() {
  /**
   * TODO: get lastest averaged error over 10 samples
   */
    
  return cur_average_cte;  // TODO: Add your total error calc here!
}

  std::vector<double> PID::GetPID()
  {
      return p;

  }