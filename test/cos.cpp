#include <iostream>
#include <math.h>
using namespace std;

int main(){
    int a=54, b=68, C=1;
    double c = sqrt(pow(b, 2) + pow(a, 2) - 2*b*a * cos(C));

    cout << cos(C) << endl;
    cout << c << endl;

    return 0;
}
