#include <iostream>
using namespace std;

bool judge_num_is_full(int num = 0)
{
    int judge_full_num[10] = {}, flag_judge_full_num = 0;

    if (num > 0 && num * 3 < 1000)
    {
        for (int i = num; i <= num * 3; i = i + num)
        {
            if (++judge_full_num[i % 10] > 1)
            {
                flag_judge_full_num = 1;
                break;
            }
            if (++judge_full_num[(i % 100) / 10] > 1)
            {
                flag_judge_full_num = 1;
                break;
            }
            if (++judge_full_num[i / 100] > 1)
            {
                flag_judge_full_num = 1;
                break;
            }
        }
        for (int i = 1; i < 10; ++i)
            if (judge_full_num[i] == 0)
                flag_judge_full_num = 1;

        if (flag_judge_full_num == 1)
            return 0;
        else
            return 1;
    }
    else
        return 0;
}

int main()
{
    int box[334] = {};

    for (int i = 1; i <= 333; ++i)
    {
        if (judge_num_is_full(i))
            cout << i << ' ' << i * 2 << ' ' << i * 3 << endl;
    }
    return 0;
}