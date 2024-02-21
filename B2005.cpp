#include<iostream>
using namespace std;

int main()
{
    char flag={};

    cin>>flag;
    for (int i = 1; i < 6; i++)
    {
        if(i <= 3)
        {
            for (int j = 1; j <= 3-i;++j)
                cout<<' ';
            for (int j = 1; j <= 2*i-1;++j)
                cout<<flag;
            cout<<endl;
        }
        else
        {
            
        }
    }
    return 0;
}