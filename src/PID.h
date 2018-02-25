#ifndef PID_H
#define PID_H


#include <ctime>
#include <cstdio>
#include "Eigen/Dense"
#include <iostream>
#include <cmath> 

using Eigen::VectorXd;

class PID {
public:
    /*
    * Errors
    */
    double p_error;
    double i_error;
    double d_error;

    double p_error_speed;
    double d_error_speed;

    /*
    * Coefficients
    */ 
    double Kp;
    double Ki;
    double Kd;

    double Kp2;
    double Kd2;

    /*
        Others
    */ 

    
    double previous_cte;

    double previous_err_speed;
    bool is_initialized_;

    /*
        Twiddle
    */ 

    VectorXd K; 
    VectorXd dK;

    double sum_dp;

    int control_K; // Control which K parameters will be changed
    int control_stage; // Control which stage of the Twiddle Algorithm will be processed

    double cte_avg; // CTE Average
    double best_err;

    int N_it; // Number of iterations to calcule the Average CTE
    int counter;
    double previous_cte_avg;
    double current_cte_avg;
    int loop_number; // NUmber of loops that calcule the CTE Average

    double sum_dK;

    // Time
    double previous_time;

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

    void Init_PD(double Kp2, double Kd2);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte, double dt);

    void UpdateErrorSpeed(double err_speed, double dt) ;

    /*
    * Calculate the total PID error.
    */
    double TotalError();

    double TotalError_throttle();

    void MyCounter(double cte);


    /*
        Twiddle.
    */
    void Twiddle();
};

#endif /* PID_H */
