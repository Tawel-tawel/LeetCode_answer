#include<iostream>
#include<iomanip>
#include<cmath>
using namespace std;

int main()
{
    double a, b, c,p;
    cin>>a>>b>>c;

    p = (a+b+c)/double(2);
    p = p*(p-a)*(p-b)*(p-c);

    cout<<fixed<<setprecision(1)<<sqrt(p);
    return 0;


}