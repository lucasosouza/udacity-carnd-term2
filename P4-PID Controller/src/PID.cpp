#include "PID.h"
#include "helper_functions.h"
#include <iostream>
#include <vector>

using namespace std;

/*
* TODO: Complete the PID class.
*/
    
PID::PID() {}

PID::~PID() {}

void PID::Twiddle() {

    // check termination
    if ((dp[0] + dp[1] + dp[2]) < threshold) {
    //if (fabs(best_err) <= 400) {
        finished = true;
        cout << "Selected Params" << p[0] << "***" << p[1] << "***"  << p[2] << endl;
        return;
    }

    // if not finished, update dp
    if (!finished) {
        // first try with + dp
        if (step == 0) {
            p[nvar] += dp[nvar]; 
            step += 1;
            //cout << "Step0" << nvar << p[nvar] << endl;
            cout << "Params" << p[0] << "***" << p[1] << "***"  << p[2] << endl;
            cout << "Updates" << dp[0] << "***" << dp[1]<< "***"  << dp[2] << endl;
            return;
        }
        // then try with - dp
        if (step == 1) {
            if (err < best_err) {
                dp[nvar] *= 1.1;
                best_err = err;
                // move to next variable if it works
                step = 0;
                nvar = (nvar+1) % 3;
            } else {
                p[nvar] -= 2*dp[nvar];
                step +=1 ;
                cout << "Params" << p[0] << "***" << p[1] << "***"  << p[2] << endl;
                cout << "Updates" << dp[0] << "***" << dp[1]<< "***"  << dp[2] << endl;
                return;
                //cout << "Step1" << nvar << p[nvar] << endl;
            }
        } 
        // if nothing works, restore values and lower dp
        if (step == 2 ) {
            if (err < best_err) {
                dp[nvar] *= 1.1;
                best_err = err;
            } else {
                p[nvar] += dp[nvar];
                dp[nvar] *= .9;
            }
            // move to next variable anyway, already tried + and -
            step = 0;
            nvar = (nvar+1) % 3;
            //cout << "Step2" << nvar << p[nvar] << endl;
        }
        // call twiddle again
        Twiddle();

    }


}


void PID::Init() {

    // initialize remaining variables
    total_cte = 0;
    previous_cte = false;
    
    // twidlle variables
    frames = 0;
    finished = false;
    nvar = 0;
    step = 0;
    best_err = 0;
    err = 0;
    threshold = 0.001;
    dt = 0;
    t_previous = 0;

}

double PID::CalcSteer(double cte, double speed) {

    // update total error
    err += (cte * cte);

    //update frame count
    frames = (frames+1) % 3200;

    //update total
    //only after 200 frames, wait for the car to accelerate to a stable spped
    if (frames>200) {
        total_cte += cte;
    }

    // calculate difference and update previous
    if (!previous_cte) {
        diff_cte = 0;
    } else {
        // calculate delta time, in milliseconds
        dt = (clock() - t_previous)/CLOCKS_PER_SEC*1000;\
        // differential gain
        // fixing dt as 1 to ensure reviewer can reproduce exact same results
        diff_cte = (cte - previous_cte)/1;
    }
    previous_cte = cte;

    // calculate errors
    p_error = p[0] * cte;
    d_error = p[1] * diff_cte;
    i_error = p[2] * total_cte;

    // calculate steer
    double steer = truncate(-p_error -d_error -i_error);

    // call twiddle to update parameters
    // stop when it finishes or the car stops
    if (frames==0){
        // init best_err in first iteration
        if (best_err == 0) {
            best_err = err;
        }
        cout << "Total Error: " << err << endl;
        cout << "Improvement: " << best_err - err << endl;
        // run twiddle to update dp
        Twiddle();
        // set flag to restart
        restart = true;
        // reinitialize cte and err cumulators
        err = 0;
        total_cte = 0;
        previous_cte = false;
        t_previous = 0;

    }

    // store last time recorded
    t_previous = clock();

    return steer;

}
