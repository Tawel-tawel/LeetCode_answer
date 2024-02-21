#include<iostream>
using namespace std;

int main()
{
    string str;
    int i = 0;
    cin>>str;

    while (str[i] != 0)
    {
        if(str[i]>='a' && str[i]<='z')
          str[i] = str[i]+'A'-'a';
        cout<<str[i];
        ++i;
    }
    return 0;
    
}