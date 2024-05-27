// 编写一个函数来查找字符串数组中的最长公共前缀。

// 如果不存在公共前缀，返回空字符串 ""。
#include <iostream>
#include <string>
#include <vector>

using namespace std;

class Solution {
public:
    string longestCommonPrefix(vector<string>& strs) {
        sort(strs.begin(), strs.end());
        string &s1=strs.front();
        string &s2=strs.back();
        int i=0;
        while(i<s1.size()&&i<s2.size()&&s1[i]==s2[i]){
            ++i;
        }
        return s1.substr(0,i);
        
    }
};