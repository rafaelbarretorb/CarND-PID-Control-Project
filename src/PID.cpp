#include "PID.h"


using namespace std;
using Eigen::VectorXd;


/*
* TODO: Complete the PID class.
*/

PID::PID() 
{
	//previous_cte = 0;
	is_initialized_ = false;
	previous_cte_avg = 0;
	VectorXd dK(3);
	VectorXd K(3);
	control_K = 0;
	control_stage = 0;
	cte_avg = 0;
	current_cte_avg = 0;
	previous_cte = 0;
}

PID::~PID() {}

// Steering PID Control
void PID::Init(double Kp, double Kd, double Ki) 
{
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

// Speed PD Control
void PID::Init_PD(double Kp2, double Kd2) 
{
	this->Kp2 = Kp2;
	this->Kd2 = Kd2;
}

void PID::UpdateError(double cte, double dt) 
{
	/*
		Update the PID error variables given cross track error.
	*/


	if (!is_initialized_)
	{
		d_error = 0;
		i_error = 0;
		is_initialized_ = true;
	}
	else
	{
		// Time difference
		//dt = double(clock() - previous_time)/CLOCKS_PER_SEC;
		cout << "dt: " << dt  << endl;
		
		// differential error

		d_error = (cte - previous_cte);///dt;
		cout << "d_error: " << d_error  << endl;

		  // Integral error
	  	if (fabs(cte) < 0.01 || fabs(cte) > 1)
	  		i_error = 0;
	  	else
			i_error += cte*dt;
		cout << "i_error: " << i_error  << endl;
		
	}

	// Proportional Error
  	p_error = cte;

	previous_cte = cte;


}

void PID::MyCounter(double cte)
{
	cte_avg += cte;

	if (counter == (N_it - 1))
	{
		loop_number++;
		if (loop_number >= 2)
			previous_cte_avg = current_cte_avg;

		current_cte_avg = cte_avg/N_it;
		cte_avg = 0;

	}

	
}

void PID::UpdateErrorSpeed(double err_speed, double dt) 
{
	/*
		Update the PID error variables given cross track error.
	*/

	if (is_initialized_ == true && loop_number > 10) // Avoid dt = 0
	{
		// differential error
		d_error_speed = (err_speed - previous_err_speed)/dt;
		cout << "d_error_speed: " << d_error_speed  << endl;
	}
	else
		d_error_speed = 0;

	// Proportional Error
  	p_error_speed = err_speed;

	previous_err_speed = err_speed;
}


double PID::TotalError() 
{
	double totalError = p_error * Kp + d_error * Kd + i_error * Ki;
	if (totalError > 1)
		totalError = 1;
	if (totalError <-1)
		totalError = -1;
	return -totalError;
}

double PID::TotalError_throttle()
{
	double totalError_throttle = p_error_speed * Kp2 + d_error_speed * Kd2;

	if (totalError_throttle > 0.2)
		totalError_throttle = 0.2;

	if (totalError_throttle < -0.2)
		totalError_throttle = -0.2;

	return totalError_throttle;

}

void PID::Twiddle()
{
	// target parameters p = [Kp , Kd, Ki]
	K = VectorXd(3);
	K << Kp, Kd, Ki;

	// vector of potential changes of the parameters K
	

	if (control_stage == 3)
	{
		control_stage = 0;
		control_K++;

		if (control_K == 3)
			control_K = 0;
			//previous_cte_avg =0; 
	}

	int i = control_K;

	if (previous_cte_avg == 0)
		best_err = current_cte_avg;
	else
	{ 
		if (control_stage == 0)
			K(i) += dK(i);

		if (control_stage == 1)
		{
			if (current_cte_avg < previous_cte_avg)
			{
				best_err = current_cte_avg;
				K(i) *= 1.1;
			}
			else
				K(i) -= 2 * dK(i);

		}	

		if (control_stage == 2)
		{
			if (current_cte_avg < previous_cte_avg)
			{
				best_err = current_cte_avg;
				dK(i) *= 1.1;
			}
			else
			{
				K(i) += dK(i);
				dK(i) *= 0.9;
			}
		}

		control_stage++;
		Init(K(0),K(1),K(2));

		// set Kp = 1 always
		//Init(1,K(1),K(2));

	}

	sum_dK = dK(0) + dK(1)+ dK(2);

}