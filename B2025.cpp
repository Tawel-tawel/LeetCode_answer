#include<iostream>
using namespace std;

int main()
{
    for (int i = 1; i < 6; i++)
    {
        if(i <= 3)
        {
            for (int j = 1; j <= 3-i;++j)
                cout<<' ';
            for (int j = 1; j <= 2*i-1;++j)
                cout<<'*';
            cout<<endl;
        }
        else
        {
            for (int j = 1; j <= i-3;++j)
                cout<<' ';
            for (int j = 1; j <= 11-2*i;++j)
                cout<<'*';
            cout<<endl;
        }
    }
    return 0;
}