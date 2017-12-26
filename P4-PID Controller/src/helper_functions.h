#include <iostream>
#include <math.h> 

double truncate(double n) {    
    if (n > 1.0) {
        return 1.0;
    } 
    if (n < -1.0) {
        return -1.0;
    }
    return n;
};



