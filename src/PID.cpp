#include "PID.h"
#include <iostream>
#include <math.h>
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
        twiddle_error_sum = 0;
        twiddle_error = 0;
        n_cte = 0;
        total_frame = 0;
        //param[] = {0.12, 3.0, 0.004};
        param_index = 0;
        change_pattern_index = 0;
        best_err = 0;
}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_, double Ki_) {
        std::cout << " into PID::Init" << " ,Kp_:"<<Kp_<<" ,Kd_:"<<Kd_<<" ,Ki_:"<<Ki_<< std::endl;
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
        n_cte += 1;
        twiddle_error_sum += pow(cte,2);
        twiddle_error = twiddle_error_sum/n_cte;
        //std::cout << "  pid:update error , twiddle_error: "<<twiddle_error << " number of CTE :"<< n_cte << std::endl;
	return;

}

double PID::TotalError() {

	double total_error= -p_error - d_error - i_error;

	return total_error;

}
