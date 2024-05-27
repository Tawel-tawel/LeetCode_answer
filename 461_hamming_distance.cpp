// 两个整数之间的 汉明距离 指的是这两个数字对应二进制位不同的位置的数目。

// 给你两个整数 x 和 y，计算并返回它们之间的汉明距离

class Solution {
public:
    int hammingDistance(int x, int y) {
        int res = 0;
        while (x!= 0 || y!= 0) {
            // 计算 x 和 y 的最后一位不同的位置
            res += (x & 1) ^ (y & 1);
            // 右移一位
            x >>= 1;
            y >>= 1;
        }
        return res;

    }
};
