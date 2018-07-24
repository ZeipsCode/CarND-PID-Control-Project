#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    p_error = d_error = i_error = 0.0;
    dp = {0.1 * Kp,0.1 * Kd,0.1 * Ki};
    step = 1;

    // Twiddling parameters
    use_twiddle = true;
        
    parameter = 0;  
    n_settle_steps = 100;
    n_eval_steps = 300;
    total_error = 0;
    best_error = 1000; // just some random high number for initialization
    used_addition = false; // twiddle parameter
    used_subtraction = false; // twiddle parameter
}

void PID::UpdateError(double cte) {
    if (step == 1) {

        p_error = cte;
    }

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    
    if (step % (n_settle_steps + n_eval_steps) > n_settle_steps){
        total_error += pow(cte,2);
    }

    // start twiddling regularly
    //  instead of loop try at every update step

    if (use_twiddle && step % (n_settle_steps + n_eval_steps) == 0){
        
        cout << "best error: " << best_error << endl;
        if (total_error < best_error) {
            best_error = total_error;
    
            if (step !=  n_settle_steps + n_eval_steps) {
                dp[parameter] *= 1.1;            
            }
            // choose next parameter to update
            parameter = (parameter + 1) % 3;
            used_addition = used_subtraction = false; // set parameters to false, so they can be used again
        }
        if (!used_addition && !used_subtraction) {
            Update_params(parameter, dp[parameter]);
            used_addition = true;
        }
        else if (used_addition && !used_subtraction) {
            Update_params(parameter, -2 * dp[parameter]);     
            used_subtraction = true;         
        }
        else {
            Update_params(parameter, dp[parameter]);      
            dp[parameter] *= 0.9;
            
            parameter = (parameter + 1) % 3;
            used_addition = used_subtraction = false;
        }
        total_error = 0;
        cout << "new PID parameters" << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;        
    }
    step++;
}

double PID::TotalError() {
    return 0.0;
}

void PID::Update_params(int index, double value) {
    if (index == 0) {
        Kp += value;
    }
    else if (index == 1) {
        Kd += value;
    }
    else if (index == 2) {
        Ki += value;
    }   
}
