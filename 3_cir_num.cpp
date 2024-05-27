// 给你一个整数 x ，如果 x 是一个回文整数，返回 true ；否则，返回 false 。

// 回文数
// 是指正序（从左向右）和倒序（从右向左）读都是一样的整数。

// 例如，121 是回文，而 123 不是。
#include <iostream>
#include <string>
using namespace std;
class Solution {
public:
    bool isPalindrome(int x) {
        if(x < 0)
            return false;
        string str_1 = to_string(x);
        string str_2 = str_1;
        reverse(str_2.begin(), str_2.end());
        if(str_1 == str_2)
            return true;
        else
            return false;

    }
};