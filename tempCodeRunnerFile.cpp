#include<iostream>
using namespace std;

int main()
{
    float num={};
    char ch='0';
    cin>>num;

    ch = ch + (num-int(num))*10;
    cout<<ch<<'.';
    ch='0';
    ch = ch + (int(num)%10);
    cout<<ch;
    ch='0';
    ch = ch + ((int(num)%100)/10);
    cout<<ch;
    ch='0';
    ch = ch + (int(num)/100);
    cout<<ch;

    return 0;
}