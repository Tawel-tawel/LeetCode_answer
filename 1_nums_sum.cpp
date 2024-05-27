// 给定一个整数数组 nums 和一个整数目标值 target，请你在该数组中找出 和为目标值 target  的那 两个 整数，并返回它们的数组下标。

// 你可以假设每种输入只会对应一个答案。但是，数组中同一个元素在答案里不能重复出现。

// 你可以按任意顺序返回答案。
class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {

        vector<int> res;
        unordered_map<int, int> hash_map;
        for(int i=0; i<nums.size(); i++){
            int complement = target - nums[i];
            if(hash_map.find(complement)!= hash_map.end()){
                res.push_back(hash_map[complement]);
                res.push_back(i);
                return res;
            }
            hash_map[nums[i]] = i;
        }
        return res;
        
    }
};