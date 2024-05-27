#include <iostream>
#include <vector>
// #include <opencv2/opencv.hpp>
// using namespace cv;
using namespace std;

struct MapNode {
    public:
    int x;
    int y;
    int cost_f;
    int cost_g;
    int cost_h;
    std::vector<int> parent;

    MapNode() : x(0), y(0), cost_f(0), cost_g(0), cost_h(0), parent({0, 0}) {}
    // MapNode 结构体的定义
};

int main() {
    std::vector<MapNode> open_list;

    // 添加元素到 open_list 中

    // 获取 open_list 中元素的数量
    int num_nodes = open_list.size();

    // 输出元素数量
    cout << "Number of nodes in open_list: " << num_nodes << endl;

    return 0;
}