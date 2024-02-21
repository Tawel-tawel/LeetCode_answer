#include<iostream>
using namespace std;

int main()
{
    string ch;
    cin>>ch;
    int len = ch.size();
    for(int i=len-1;i>=0;i--)
	cout<<ch[i];

    return 0;
}