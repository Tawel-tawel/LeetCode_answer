#include<iostream>
using namespace std;

int main()
{
    int n, k,left_flag=0, right_flag=0, box;
    int *arry=nullptr;

    //ÊäÈë
    cin>>n>>k;
    cin.get();
    arry =new int[n]{};
    for (int i= 0; i < n;++i)
    {
        cin>>arry[i];
    }
    
    return 0;
}