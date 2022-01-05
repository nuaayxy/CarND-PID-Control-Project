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

}

double PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

    double diff_cte = cte - prev_cte;
    int_cte+=cte;
    prev_cte = cte;
    double steering = -Kp*cte - Kd* diff_cte  -Ki*int_cte;

    /*


    x_trajectory = []
    y_trajectory = []
    err = 0
    prev_cte = robot.y
    int_cte = 0
    for i in range(2 * n):
        cte = robot.y
        diff_cte = cte - prev_cte
        int_cte += cte
        prev_cte = cte
        steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        if i >= n:
            err += cte ** 2
    return x_trajectory, y_trajectory, err / n

    //twiddle addest the PID parameters
    # Don't forget to call `make_robot` before every call of `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)
    # TODO: twiddle loop here
    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    */
//    double dp[3] = {1.0, 1.0, 1.0};
//    double sum_dp = 
//    static int count = 1000;
//    if(sum_dp > 0.2)
//    {
//        count++;
//        Kd += dKd;
//        Kp += dKp;
//        Ki += dKi;
//        if( cte < best_cte)
//         {
//             best_cte = cte;
//             dKp*= 1.1;
//             dKd*= 1.1;
//             dKi*= 1.1;
//         }
//         else
//         {
//          Kd-=2*dKd;
//          Kp-=2*dKp;


//         }



//    }

    return steering;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
//    x_trajectory = []
//    y_trajectory = []
//    err = 0
//    prev_cte = robot.y
//    int_cte = 0
//    for i in range(2 * n):
//        cte = robot.y
//        diff_cte = cte - prev_cte
//        int_cte += cte
//        prev_cte = cte
//        steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte
//        robot.move(steer, speed)
//        x_trajectory.append(robot.x)
//        y_trajectory.append(robot.y)
//        if i >= n:
//            err += cte ** 2
//    return x_trajectory, y_trajectory, err / n
    
  return int_cte;  // TODO: Add your total error calc here!
}
