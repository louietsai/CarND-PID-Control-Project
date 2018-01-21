#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
        Kp = 0;
        Ki = 0;
        Kd = 0;
        prev_cte = 0;
        diff_cte = 0;
        int_cte = 0;
        p_error= 0;
        i_error = 0;
        d_error = 0;
}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_, double Ki_) {
        std::cout << " into PID::Init" << std::endl;
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
        return;
}

void PID::UpdateError(double cte) {
        if (prev_cte !=0){
                diff_cte = cte - prev_cte;
        }
        prev_cte = cte;
        int_cte += cte;
        p_error = Kp * cte;
        d_error = Kd * diff_cte;
        i_error = Ki * int_cte;
	return;

}

double PID::TotalError() {

	double total_error= -p_error - d_error - i_error;

	return total_error;

}
