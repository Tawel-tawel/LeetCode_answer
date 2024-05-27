// 颠倒给定的 32 位无符号整数的二进制位。
class Solution {
public:
    uint32_t reverseBits(uint32_t n) {
        uint32_t res = 0;
        for (int i = 0; i < 32; i++) {
            // 左移一位，并将 n 的最低位添加到 res 中
            res <<= 1;
            // 将 n 的最低位添加到 res 中
            res |= (n & 1);
            // 右移一位，并将 n 的最低位移出
            n >>= 1;
        }
        return res;
        
    }
};