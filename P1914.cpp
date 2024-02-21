#include<iostream>
using namespace std;

int main()
{
    string str;
    int n,size;
    cin>>n;
    cin.get();
    cin>>str;

    size = str.size();
    for (int i = 0; i <size;++i)
    {
        if((str[i]+n)>'z')
        {
            cout<<char((str[i]+n)%'z'+'a'-1);
        }
        else
        {
            cout<<char(str[i]+n);
        }
    }
    return 0;


}