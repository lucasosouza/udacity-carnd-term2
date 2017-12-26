normalization.cpp

#include <iostream>
#include <math.h>
using namespace std;


double NormalizeAngleA(double phi)
{
  return atan2(sin(phi), cos(phi));
}

double NormalizeAngleB(double phi)
{
  double M_PI = 3.14;
  return fmod(phi, M_PI);
}

double NormalizeAngleC(double phi)
{
    double M_PI = 3.14;
    while (phi> M_PI) phi -= 2.*M_PI;
    while (phi<-M_PI) phi += 2.*M_PI;
    return phi;
}

int main() {

    double a = 123.90;
    double b = -78.90;
    
    cout << NormalizeAngleA(a) << endl;
    cout << NormalizeAngleA(b) << endl;

    cout << NormalizeAngleB(a) << endl;
    cout << NormalizeAngleB(b) << endl;

    cout << NormalizeAngleC(a) << endl;
    cout << NormalizeAngleC(b) << endl;

    return 0;
}

/* results


-1.76371
2.78141
1.44
-0.4
-1.7
2.74

/*